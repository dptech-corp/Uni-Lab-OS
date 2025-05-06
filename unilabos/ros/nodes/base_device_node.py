import json
import threading
import time
import traceback
import uuid
from typing import get_type_hints, TypeVar, Generic, Dict, Any, Type, TypedDict, Optional

from concurrent.futures import ThreadPoolExecutor
import asyncio

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.action.server import ServerGoalHandle
from rclpy.client import Client
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.service import Service
from unilabos_msgs.action import SendCmd
from unilabos_msgs.srv._serial_command import SerialCommand_Request, SerialCommand_Response

from unilabos.resources.graphio import convert_resources_to_type, convert_resources_from_type, resource_ulab_to_plr, \
    initialize_resources
from unilabos.ros.msgs.message_converter import (
    convert_to_ros_msg,
    convert_from_ros_msg,
    convert_from_ros_msg_with_mapping,
    convert_to_ros_msg_with_mapping, ros_action_to_json_schema,
)
from unilabos_msgs.srv import ResourceAdd, ResourceGet, ResourceDelete, ResourceUpdate, ResourceList, \
    SerialCommand  # type: ignore
from unilabos_msgs.msg import Resource  # type: ignore

from unilabos.ros.nodes.resource_tracker import DeviceNodeResourceTracker
from unilabos.ros.x.rclpyx import get_event_loop
from unilabos.ros.utils.driver_creator import ProtocolNodeCreator, PyLabRobotCreator, DeviceClassCreator
from unilabos.utils.async_util import run_async_func
from unilabos.utils.log import info, debug, warning, error, critical, logger
from unilabos.utils.type_check import get_type_class, TypeEncoder

T = TypeVar("T")


# 在线设备注册表
registered_devices: Dict[str, "DeviceInfoType"] = {}


# 实现同时记录自定义日志和ROS2日志的适配器
class ROSLoggerAdapter:
    """同时向自定义日志和ROS2日志发送消息的适配器"""

    @property
    def identifier(self):
        return f"{self.namespace}"

    def __init__(self, ros_logger, namespace):
        """
        初始化日志适配器

        Args:
            ros_logger: ROS2日志记录器
            namespace: 命名空间
        """
        self.ros_logger = ros_logger
        self.namespace = namespace
        self.level_2_logger_func = {
            "info": info,
            "debug": debug,
            "warning": warning,
            "error": error,
            "critical": critical,
        }

    def _log(self, level, msg, *args, **kwargs):
        """实际执行日志记录的内部方法"""
        # 添加前缀，使日志更易识别
        msg = f"[{self.identifier}] {msg}"
        # 向ROS2日志发送消息（标准库logging不支持stack_level参数）
        ros_log_func = getattr(self.ros_logger, "debug")  # 默认发送debug，这样不会显示在控制台
        ros_log_func(msg)
        self.level_2_logger_func[level](msg, *args, stack_level=1, **kwargs)

    def debug(self, msg, *args, **kwargs):
        """记录DEBUG级别日志"""
        self._log("debug", msg, *args, **kwargs)

    def info(self, msg, *args, **kwargs):
        """记录INFO级别日志"""
        self._log("info", msg, *args, **kwargs)

    def warning(self, msg, *args, **kwargs):
        """记录WARNING级别日志"""
        self._log("warning", msg, *args, **kwargs)

    def error(self, msg, *args, **kwargs):
        """记录ERROR级别日志"""
        self._log("error", msg, *args, **kwargs)

    def critical(self, msg, *args, **kwargs):
        """记录CRITICAL级别日志"""
        self._log("critical", msg, *args, **kwargs)


def init_wrapper(
    self,
    device_id: str,
    driver_class: type[T],
    device_config: Dict[str, Any],
    status_types: Dict[str, Any],
    action_value_mappings: Dict[str, Any],
    hardware_interface: Dict[str, Any],
    print_publish: bool,
    children: Optional[list] = None,
    driver_params: Optional[Dict[str, Any]] = None,
    driver_is_ros: bool = False,
    *args,
    **kwargs,
):
    """初始化设备节点的包装函数，和ROS2DeviceNode初始化保持一致"""
    if driver_params is None:
        driver_params = kwargs.copy()
    if children is None:
        children = []
    kwargs["device_id"] = device_id
    kwargs["driver_class"] = driver_class
    kwargs["device_config"] = device_config
    kwargs["driver_params"] = driver_params
    kwargs["status_types"] = status_types
    kwargs["action_value_mappings"] = action_value_mappings
    kwargs["hardware_interface"] = hardware_interface
    kwargs["children"] = children
    kwargs["print_publish"] = print_publish
    kwargs["driver_is_ros"] = driver_is_ros
    super(type(self), self).__init__(*args, **kwargs)


class PropertyPublisher:
    def __init__(
        self,
        node: "BaseROS2DeviceNode",
        name: str,
        get_method,
        msg_type,
        initial_period: float = 5.0,
        print_publish=True,
    ):
        self.node = node
        self.name = name
        self.msg_type = msg_type
        self.get_method = get_method
        self.timer_period = initial_period
        self.print_publish = print_publish

        self._value = None
        self.publisher_ = node.create_publisher(msg_type, f"{name}", 10)
        self.timer = node.create_timer(self.timer_period, self.publish_property)
        self.__loop = get_event_loop()
        str_msg_type = str(msg_type)[8:-2]
        self.node.lab_logger().debug(f"发布属性: {name}, 类型: {str_msg_type}, 周期: {initial_period}秒")

    def get_property(self):
        if asyncio.iscoroutinefunction(self.get_method):
            # 如果是异步函数，运行事件循环并等待结果
            self.node.get_logger().debug(f"【PropertyPublisher.get_property】获取异步属性: {self.name}")
            loop = self.__loop
            if loop:
                future = asyncio.run_coroutine_threadsafe(self.get_method(), loop)
                self._value = future.result()
                return self._value
            else:
                self.node.get_logger().error(f"【PropertyPublisher.get_property】事件循环未初始化")
                return None
        else:
            # 如果是同步函数，直接调用并返回结果
            self.node.get_logger().debug(f"【PropertyPublisher.get_property】获取同步属性: {self.name}")
            self._value = self.get_method()
            return self._value

    async def get_property_async(self):
        try:
            # 获取异步属性值
            self.node.get_logger().debug(f"【PropertyPublisher.get_property_async】异步获取属性: {self.name}")
            self._value = await self.get_method()
        except Exception as e:
            self.node.get_logger().error(f"【PropertyPublisher.get_property_async】获取异步属性出错: {str(e)}")

    def publish_property(self):
        try:
            self.node.get_logger().debug(f"【PropertyPublisher.publish_property】开始发布属性: {self.name}")
            value = self.get_property()
            if self.print_publish:
                self.node.get_logger().info(f"【PropertyPublisher.publish_property】发布 {self.msg_type}: {value}")
            if value is not None:
                msg = convert_to_ros_msg(self.msg_type, value)
                self.publisher_.publish(msg)
                self.node.get_logger().debug(f"【PropertyPublisher.publish_property】属性 {self.name} 发布成功")
        except Exception as e:
            traceback.print_exc()
            self.node.get_logger().error(f"【PropertyPublisher.publish_property】发布属性出错: {str(e)}")

    def change_frequency(self, period):
        # 动态改变定时器频率
        self.timer_period = period
        self.node.get_logger().info(
            f"【PropertyPublisher.change_frequency】修改 {self.name} 定时器周期为: {self.timer_period} 秒"
        )

        # 重置定时器
        self.timer.cancel()
        self.timer = self.node.create_timer(self.timer_period, self.publish_property)


class BaseROS2DeviceNode(Node, Generic[T]):
    """
    ROS2设备节点基类

    这个类提供了ROS2设备节点的基本功能，包括属性发布、动作服务等。
    通过泛型参数T来指定具体的设备类型。
    """

    @property
    def identifier(self):
        return f"{self.namespace}/{self.device_id}"

    node_name: str
    namespace: str
    # TODO 要删除，添加时间相关的属性，避免动态添加属性的警告
    time_spent = 0.0
    time_remaining = 0.0
    create_action_server = True

    def __init__(
        self,
        driver_instance: T,
        device_id: str,
        status_types: Dict[str, Any],
        action_value_mappings: Dict[str, Any],
        hardware_interface: Dict[str, Any],
        print_publish=True,
        resource_tracker: Optional["DeviceNodeResourceTracker"] = None,
    ):
        """
        初始化ROS2设备节点

        Args:
            driver_instance: 设备实例
            device_id: 设备标识符
            status_types: 需要发布的状态和传感器信息
            action_value_mappings: 设备动作
            hardware_interface: 硬件接口配置
            print_publish: 是否打印发布信息
        """
        self.driver_instance = driver_instance
        self.device_id = device_id
        self.uuid = str(uuid.uuid4())
        self.publish_high_frequency = False
        self.callback_group = ReentrantCallbackGroup()
        self.resource_tracker = resource_tracker

        # 初始化ROS节点
        self.node_name = f'{device_id.split("/")[-1]}'
        self.namespace = f"/devices/{device_id}"
        Node.__init__(self, self.node_name, namespace=self.namespace)  # type: ignore
        if self.resource_tracker is None:
            self.lab_logger().critical("资源跟踪器未初始化，请检查")

        # 创建自定义日志记录器
        self._lab_logger = ROSLoggerAdapter(self.get_logger(), self.namespace)

        self._action_servers: Dict[str, ActionServer] = {}
        self._property_publishers = {}
        self._status_types = status_types
        self._action_value_mappings = action_value_mappings
        self._hardware_interface = hardware_interface
        self._print_publish = print_publish

        # 创建属性发布者
        for attr_name, msg_type in self._status_types.items():
            if isinstance(attr_name, (int, float)):
                if "param" in msg_type.keys():
                    pass
                else:
                    for k, v in msg_type.items():
                        self.create_ros_publisher(k, v, initial_period=5.0)
            else:
                self.create_ros_publisher(attr_name, msg_type)

        # 创建动作服务
        if self.create_action_server:
            for action_name, action_value_mapping in self._action_value_mappings.items():
                self.create_ros_action_server(action_name, action_value_mapping)

        # 创建线程池执行器
        self._executor = ThreadPoolExecutor(max_workers=max(len(action_value_mappings), 1), thread_name_prefix=f"ROSDevice{self.device_id}")

        # 创建资源管理客户端
        self._resource_clients: Dict[str, Client] = {
            "resource_add": self.create_client(ResourceAdd, "/resources/add"),
            "resource_get": self.create_client(ResourceGet, "/resources/get"),
            "resource_delete": self.create_client(ResourceDelete, "/resources/delete"),
            "resource_update": self.create_client(ResourceUpdate, "/resources/update"),
            "resource_list": self.create_client(ResourceList, "/resources/list"),
        }

        def query_host_name_cb(req, res):
            self.register_device()
            self.lab_logger().info("Host要求重新注册当前节点")
            res.response = ""
            return res

        def append_resource(req: SerialCommand_Request, res: SerialCommand_Response):
            # 物料传输到对应的node节点
            rclient = self.create_client(ResourceAdd, "/resources/add")
            rclient.wait_for_service()
            request = ResourceAdd.Request()
            command_json = json.loads(req.command)
            namespace = command_json["namespace"]
            bind_parent_id = command_json["bind_parent_id"]
            edge_device_id = command_json["edge_device_id"]
            location = command_json["bind_location"]
            other_calling_param = command_json["other_calling_param"]
            resources = command_json["resource"]
            initialize_full = other_calling_param.pop("initialize_full", False)
            # 本地拿到这个物料，可能需要先做初始化?
            if isinstance(resources, list):
                if initialize_full:
                    resources = initialize_resources(resources)
                request.resources = [convert_to_ros_msg(Resource, resource) for resource in resources]
            else:
                if initialize_full:
                    resources = initialize_resources([resources])
                request.resources = [convert_to_ros_msg(Resource, resources)]
            response = rclient.call(request)
            # 应该先add_resource了
            res.response = "OK"
            # 接下来该根据bind_parent_id进行assign了，目前只有plr可以进行assign，不然没有办法输入到物料系统中
            resource = self.resource_tracker.figure_resource({"name": bind_parent_id})
            request.resources = [convert_to_ros_msg(Resource, resources)]

            try:
                from pylabrobot.resources.resource import Resource as ResourcePLR
                from pylabrobot.resources.deck import Deck
                from pylabrobot.resources import Coordinate
                from pylabrobot.resources import OTDeck
                contain_model = not isinstance(resource, Deck)
                if isinstance(resource, ResourcePLR):
                    # resources.list()
                    plr_instance = resource_ulab_to_plr(resources, contain_model)
                    if isinstance(resource, OTDeck) and "slot" in other_calling_param:
                        resource.assign_child_at_slot(plr_instance, **other_calling_param)
                    resource.assign_child_resource(plr_instance, Coordinate(location["x"], location["y"], location["z"]), **other_calling_param)
                # 发送给ResourceMeshManager
                action_client = ActionClient(
                    self, SendCmd, "/devices/resource_mesh_manager/add_resource_mesh", callback_group=self.callback_group
                )
                goal = SendCmd.Goal()
                goal.command = json.dumps({
                    "resources": resources,
                    "bind_parent_id": bind_parent_id,
                })
                future = action_client.send_goal_async(goal, goal_uuid=uuid.uuid4())

                def done_cb(*args):
                    self.lab_logger().info(f"向meshmanager发送新增resource完成")

                future.add_done_callback(done_cb)
            except ImportError:
                self.lab_logger().error("Host请求添加物料时，本环境并不存在pylabrobot")
            except Exception as e:
                self.lab_logger().error("Host请求添加物料时出错")
                self.lab_logger().error(traceback.format_exc())
            return res

        # noinspection PyTypeChecker
        self._service_server: Dict[str, Service] = {
            "query_host_name": self.create_service(
                SerialCommand, f"/srv{self.namespace}/query_host_name", query_host_name_cb, callback_group=self.callback_group
            ),
            "append_resource": self.create_service(
                SerialCommand, f"/srv{self.namespace}/append_resource", append_resource, callback_group=self.callback_group
            ),
        }

        # 向全局在线设备注册表添加设备信息
        self.register_device()
        rclpy.get_global_executor().add_node(self)
        self.lab_logger().debug(f"ROS节点初始化完成")

    def register_device(self):
        """向注册表中注册设备信息"""
        topics_info = self._property_publishers.copy()
        actions_info = self._action_servers.copy()
        # 创建设备信息
        device_info = DeviceInfoType(
            id=self.device_id,
            uuid=self.uuid,
            node_name=self.node_name,
            namespace=self.namespace,
            driver_instance=self.driver_instance,
            status_publishers=topics_info,
            actions=actions_info,
            hardware_interface=self._hardware_interface,
            base_node_instance=self,
        )
        # 加入全局注册表
        registered_devices[self.device_id] = device_info
        from unilabos.config.config import BasicConfig
        if not BasicConfig.is_host_mode:
            sclient = self.create_client(SerialCommand, "/node_info_update")
            # 启动线程执行发送任务
            threading.Thread(
                target=self.send_slave_node_info,
                args=(sclient,),
                daemon=True,
                name=f"ROSDevice{self.device_id}_send_slave_node_info"
            ).start()

    def send_slave_node_info(self, sclient):
        sclient.wait_for_service()
        request = SerialCommand.Request()
        from unilabos.config.config import BasicConfig
        request.command = json.dumps({
            "SYNC_SLAVE_NODE_INFO": {
                "machine_name": BasicConfig.machine_name,
                "type": "slave",
                "edge_device_id": self.device_id
            }}, ensure_ascii=False, cls=TypeEncoder)

        # 发送异步请求并等待结果
        future = sclient.call_async(request)
        response = future.result()

    def lab_logger(self):
        """
        获取实验室自定义日志记录器

        这个日志记录器会同时向ROS2日志和自定义日志发送消息，
        并使用node_name和namespace作为标识。

        Returns:
            日志记录器实例
        """
        return self._lab_logger

    def create_ros_publisher(self, attr_name, msg_type, initial_period=5.0):
        """创建ROS发布者"""

        # 获取属性值的方法
        def get_device_attr():
            try:
                if hasattr(self.driver_instance, f"get_{attr_name}"):
                    return getattr(self.driver_instance, f"get_{attr_name}")()
                else:
                    return getattr(self.driver_instance, attr_name)
            except AttributeError as ex:
                if ex.args[0].startswith(f"AttributeError: '{self.driver_instance.__class__.__name__}' object"):
                    self.lab_logger().error(
                        f"publish error, {str(type(self.driver_instance))[8:-2]} has no attribute '{attr_name}'"
                    )
                else:
                    self.lab_logger().error(
                        f"publish error, when {str(type(self.driver_instance))[8:-2]} getting attribute '{attr_name}'"
                    )
                    self.lab_logger().error(traceback.format_exc())

        self._property_publishers[attr_name] = PropertyPublisher(
            self, attr_name, get_device_attr, msg_type, initial_period, self._print_publish
        )

    def create_ros_action_server(self, action_name, action_value_mapping):
        """创建ROS动作服务器"""
        action_type = action_value_mapping["type"]
        str_action_type = str(action_type)[8:-2]

        self._action_servers[action_name] = ActionServer(
            self,
            action_type,
            action_name,
            execute_callback=self._create_execute_callback(action_name, action_value_mapping),
            callback_group=ReentrantCallbackGroup(),
        )

        self.lab_logger().debug(f"发布动作: {action_name}, 类型: {str_action_type}")

    def _create_execute_callback(self, action_name, action_value_mapping):
        """创建动作执行回调函数"""

        async def execute_callback(goal_handle: ServerGoalHandle):
            self.lab_logger().info(f"执行动作: {action_name}")
            goal = goal_handle.request

            # 从目标消息中提取参数, 并调用对应的方法
            if "sequence" in self._action_value_mappings:
                # 如果一个指令对应函数的连续调用，如启动和等待结果，默认参数应该属于第一个函数调用
                def ACTION(**kwargs):
                    for i, action in enumerate(self._action_value_mappings["sequence"]):
                        if i == 0:
                            self.lab_logger().info(f"执行序列动作第一步: {action}")
                            getattr(self.driver_instance, action)(**kwargs)
                        else:
                            self.lab_logger().info(f"执行序列动作后续步骤: {action}")
                            getattr(self.driver_instance, action)()

                action_paramtypes = get_type_hints(
                    getattr(self.driver_instance, self._action_value_mappings["sequence"][0])
                )
            else:
                ACTION = getattr(self.driver_instance, action_name)
                action_paramtypes = get_type_hints(ACTION)

            action_kwargs = convert_from_ros_msg_with_mapping(goal, action_value_mapping["goal"])
            self.lab_logger().debug(f"接收到原始目标: {action_kwargs}")
            # 向Host查询物料当前状态，如果是host本身的增加物料的请求，则直接跳过
            if action_name != "add_resource_from_outer":
                for k, v in goal.get_fields_and_field_types().items():
                    if v in ["unilabos_msgs/Resource", "sequence<unilabos_msgs/Resource>"]:
                        self.lab_logger().info(f"查询资源状态: Key: {k} Type: {v}")
                        current_resources = []
                        try:
                            if len(action_kwargs[k]) > 1:
                                for i in action_kwargs[k]:
                                    r = ResourceGet.Request()
                                    r.id = i["id"]
                                    r.with_children = True
                                    response = await self._resource_clients["resource_get"].call_async(r)
                                    current_resources.extend(response.resources)
                            else:
                                r = ResourceGet.Request()
                                r.id = action_kwargs[k]["id"] if v == "unilabos_msgs/Resource" else action_kwargs[k][0]["id"]
                                r.with_children = True
                                response = await self._resource_clients["resource_get"].call_async(r)
                                current_resources.extend(response.resources)
                        except Exception:
                            logger.error(f"资源查询失败，默认使用本地资源")
                        # 删除对response.resources的检查，因为它总是存在
                        resources_list = [convert_from_ros_msg(rs) for rs in current_resources]  # type: ignore  # FIXME
                        self.lab_logger().debug(f"资源查询结果: {len(resources_list)} 个资源")
                        type_hint = action_paramtypes[k]
                        final_type = get_type_class(type_hint)
                        # 判断 ACTION 是否需要特殊的物料类型如 pylabrobot.resources.Resource，并做转换
                        final_resource = [convert_resources_to_type([i], final_type)[0] for i in resources_list]
                        action_kwargs[k] = self.resource_tracker.figure_resource(final_resource)

            self.lab_logger().info(f"准备执行: {action_kwargs}, 函数: {ACTION.__name__}")
            time_start = time.time()
            time_overall = 100

            # 将阻塞操作放入线程池执行
            if asyncio.iscoroutinefunction(ACTION):
                try:
                    self.lab_logger().info(f"异步执行动作 {ACTION}")
                    future = ROS2DeviceNode.run_async_func(ACTION, **action_kwargs)
                except Exception as e:
                    self.lab_logger().error(f"创建异步任务失败: {traceback.format_exc()}")
                    raise e
            else:
                self.lab_logger().info(f"同步执行动作 {ACTION}")
                future = self._executor.submit(ACTION, **action_kwargs)

                def _handle_future_exception(fut):
                    try:
                        fut.result()
                    except Exception as e:
                        error(f"同步任务 {ACTION.__name__} 报错了")
                        error(traceback.format_exc())

                future.add_done_callback(_handle_future_exception)

            action_type = action_value_mapping["type"]
            feedback_msg_types = action_type.Feedback.get_fields_and_field_types()
            result_msg_types = action_type.Result.get_fields_and_field_types()

            while not future.done():
                if goal_handle.is_cancel_requested:
                    self.lab_logger().info(f"取消动作: {action_name}")
                    future.cancel()  # 尝试取消线程池中的任务
                    goal_handle.canceled()
                    return action_type.Result()

                self.time_spent = time.time() - time_start
                self.time_remaining = time_overall - self.time_spent

                # 发布反馈
                feedback_values = {}
                for msg_name, attr_name in action_value_mapping["feedback"].items():
                    if hasattr(self.driver_instance, f"get_{attr_name}"):
                        method = getattr(self.driver_instance, f"get_{attr_name}")
                        if not asyncio.iscoroutinefunction(method):
                            feedback_values[msg_name] = method()
                    elif hasattr(self.driver_instance, attr_name):
                        feedback_values[msg_name] = getattr(self.driver_instance, attr_name)

                if self._print_publish:
                    self.lab_logger().info(f"反馈: {feedback_values}")

                feedback_msg = convert_to_ros_msg_with_mapping(
                    ros_msg_type=action_type.Feedback(),
                    obj=feedback_values,
                    value_mapping=action_value_mapping["feedback"],
                )

                goal_handle.publish_feedback(feedback_msg)
                time.sleep(0.5)

            if future.cancelled():
                self.lab_logger().info(f"动作 {action_name} 已取消")
                return action_type.Result()

            self.lab_logger().info(f"动作执行完成: {action_name}")
            del future

            # 向Host更新物料当前状态
            if action_name != "add_resource_from_outer":
                for k, v in goal.get_fields_and_field_types().items():
                    if v not in ["unilabos_msgs/Resource", "sequence<unilabos_msgs/Resource>"]:
                        continue
                    self.lab_logger().info(f"更新资源状态: {k}")
                    r = ResourceUpdate.Request()
                    # 仅当action_kwargs[k]不为None时尝试转换
                    akv = action_kwargs[k]
                    apv = action_paramtypes[k]
                    final_type = get_type_class(apv)
                    if final_type is None:
                        continue
                    try:
                        r.resources = [
                            convert_to_ros_msg(Resource, self.resource_tracker.root_resource(rs))
                            for rs in convert_resources_from_type(akv, final_type)  # type: ignore  # FIXME  # 考虑反查到最大的
                        ]
                        response = await self._resource_clients["resource_update"].call_async(r)
                        self.lab_logger().debug(f"资源更新结果: {response}")
                    except Exception as e:
                        self.lab_logger().error(f"资源更新失败: {e}")
                        self.lab_logger().error(traceback.format_exc())

            # 发布结果
            goal_handle.succeed()
            self.lab_logger().info(f"设置动作成功: {action_name}")

            result_values = {}
            for msg_name, attr_name in action_value_mapping["result"].items():
                if hasattr(self.driver_instance, f"get_{attr_name}"):
                    result_values[msg_name] = getattr(self.driver_instance, f"get_{attr_name}")()
                elif hasattr(self.driver_instance, attr_name):
                    result_values[msg_name] = getattr(self.driver_instance, attr_name)

            result_msg = convert_to_ros_msg_with_mapping(
                ros_msg_type=action_type.Result(), obj=result_values, value_mapping=action_value_mapping["result"]
            )

            for attr_name in result_msg_types.keys():
                if attr_name in ["success", "reached_goal"]:
                    setattr(result_msg, attr_name, True)

            self.lab_logger().info(f"动作 {action_name} 完成并返回结果")
            return result_msg

        return execute_callback

    # 异步上下文管理方法
    async def __aenter__(self):
        """进入异步上下文"""
        self.lab_logger().info(f"进入异步上下文: {self.device_id}")
        if hasattr(self.driver_instance, "__aenter__"):
            await self.driver_instance.__aenter__()  # type: ignore
        self.lab_logger().info(f"异步上下文初始化完成: {self.device_id}")
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """退出异步上下文"""
        self.lab_logger().info(f"退出异步上下文: {self.device_id}")
        if hasattr(self.driver_instance, "__aexit__"):
            await self.driver_instance.__aexit__(exc_type, exc_val, exc_tb)  # type: ignore
        self.lab_logger().info(f"异步上下文清理完成: {self.device_id}")


class DeviceInitError(Exception):
    pass


class ROS2DeviceNode:
    """
    ROS2设备节点类

    这个类封装了设备类实例和ROS2节点的功能，提供ROS2接口。
    它不继承设备类，而是通过代理模式访问设备类的属性和方法。
    """

    # 类变量，用于循环管理
    _loop = None
    _loop_running = False
    _loop_thread = None

    @classmethod
    def get_loop(cls):
        return cls._loop

    @classmethod
    def run_async_func(cls, func, **kwargs):
        return run_async_func(func, loop=cls._loop, **kwargs)

    @property
    def driver_instance(self):
        return self._driver_instance

    @property
    def ros_node_instance(self):
        return self._ros_node

    def __init__(
        self,
        device_id: str,
        driver_class: Type[T],
        device_config: Dict[str, Any],
        driver_params: Dict[str, Any],
        status_types: Dict[str, Any],
        action_value_mappings: Dict[str, Any],
        hardware_interface: Dict[str, Any],
        children: Dict[str, Any],
        print_publish: bool = True,
        driver_is_ros: bool = False,
    ):
        """
        初始化ROS2设备节点

        Args:
            device_id: 设备标识符
            driver_class: 设备类
            device_config: 原始初始化的json
            driver_params: driver初始化的参数
            status_types: 状态类型映射
            action_value_mappings: 动作值映射
            hardware_interface: 硬件接口配置
            children:
            print_publish: 是否打印发布信息
            driver_is_ros:
        """
        # 在初始化时检查循环状态
        if ROS2DeviceNode._loop_running and ROS2DeviceNode._loop_thread is not None:
            pass
        elif ROS2DeviceNode._loop_thread is None:
            self._start_loop()

        # 保存设备类是否支持异步上下文
        self._has_async_context = hasattr(driver_class, "__aenter__") and hasattr(driver_class, "__aexit__")
        self._driver_class = driver_class
        self.device_config = device_config
        self.driver_is_ros = driver_is_ros
        self.resource_tracker = DeviceNodeResourceTracker()

        # use_pylabrobot_creator 使用 cls的包路径检测
        use_pylabrobot_creator = driver_class.__module__.startswith("pylabrobot") or driver_class.__name__ == "DPLiquidHandler"

        # TODO: 要在创建之前预先请求服务器是否有当前id的物料，放到resource_tracker中，让pylabrobot进行创建
        # 创建设备类实例
        if use_pylabrobot_creator:
            self._driver_creator = PyLabRobotCreator(
                driver_class, children=children, resource_tracker=self.resource_tracker
            )
        else:
            from unilabos.ros.nodes.presets.protocol_node import ROS2ProtocolNode

            if self._driver_class is ROS2ProtocolNode:
                self._driver_creator = ProtocolNodeCreator(driver_class, children=children)
            else:
                self._driver_creator = DeviceClassCreator(driver_class)

        if driver_is_ros:
            driver_params["device_id"] = device_id
            driver_params["resource_tracker"] = self.resource_tracker
        self._driver_instance = self._driver_creator.create_instance(driver_params)
        if self._driver_instance is None:
            logger.critical(f"设备实例创建失败 {driver_class}, params: {driver_params}")
            raise DeviceInitError("错误: 设备实例创建失败")

        # 创建ROS2节点
        if driver_is_ros:
            self._ros_node = self._driver_instance  # type: ignore
        else:
            self._ros_node = BaseROS2DeviceNode(
                driver_instance=self._driver_instance,
                device_id=device_id,
                status_types=status_types,
                action_value_mappings=action_value_mappings,
                hardware_interface=hardware_interface,
                print_publish=print_publish,
                resource_tracker=self.resource_tracker,
            )
        self._ros_node: BaseROS2DeviceNode
        self._ros_node.lab_logger().info(f"初始化完成 {self._ros_node.uuid} {self.driver_is_ros}")

    def _start_loop(self):
        def run_event_loop():
            loop = asyncio.new_event_loop()
            ROS2DeviceNode._loop = loop
            asyncio.set_event_loop(loop)
            loop.run_forever()

        ROS2DeviceNode._loop_thread = threading.Thread(target=run_event_loop, daemon=True, name="ROS2DeviceNode")
        ROS2DeviceNode._loop_thread.start()
        logger.info(f"循环线程已启动")


class DeviceInfoType(TypedDict):
    id: str
    uuid: str
    node_name: str
    namespace: str
    driver_instance: Any
    status_publishers: Dict[str, PropertyPublisher]
    actions: Dict[str, ActionServer]
    hardware_interface: Dict[str, Any]
    base_node_instance: BaseROS2DeviceNode
