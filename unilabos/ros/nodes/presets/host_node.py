import copy
import json
import threading
import time
import traceback
import uuid
from typing import Optional, Dict, Any, List, ClassVar, Set

from action_msgs.msg import GoalStatus
from unilabos_msgs.msg import Resource  # type: ignore
from unilabos_msgs.srv import ResourceAdd, ResourceGet, ResourceDelete, ResourceUpdate, ResourceList, SerialCommand  # type: ignore
from rclpy.action import ActionClient, get_action_server_names_and_types_by_node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.service import Service
from unique_identifier_msgs.msg import UUID

from unilabos.registry.registry import lab_registry
from unilabos.resources.registry import add_schema
from unilabos.ros.initialize_device import initialize_device_from_dict
from unilabos.ros.msgs.message_converter import (
    get_msg_type,
    get_ros_type_by_msgname,
    convert_from_ros_msg,
    convert_to_ros_msg,
    msg_converter_manager,
    ros_action_to_json_schema,
)
from unilabos.ros.nodes.base_device_node import BaseROS2DeviceNode, ROS2DeviceNode, DeviceNodeResourceTracker
from unilabos.ros.nodes.presets.controller_node import ControllerNode
from unilabos.utils.type_check import TypeEncoder


class HostNode(BaseROS2DeviceNode):
    """
    主机节点类，负责管理设备、资源和控制器

    作为单例模式实现，确保整个应用中只有一个主机节点实例
    """

    _instance: ClassVar[Optional["HostNode"]] = None
    _ready_event: ClassVar[threading.Event] = threading.Event()

    @classmethod
    def get_instance(cls, timeout=None) -> Optional["HostNode"]:
        if cls._ready_event.wait(timeout):
            return cls._instance
        return None

    def __init__(
        self,
        device_id: str,
        devices_config: Dict[str, Any],
        resources_config: Any,
        physical_setup_graph: Optional[Dict[str, Any]] = None,
        controllers_config: Optional[Dict[str, Any]] = None,
        bridges: Optional[List[Any]] = None,
        discovery_interval: float = 180.0,  # 设备发现间隔，单位为秒
    ):
        """
        初始化主机节点

        Args:
            device_id: 节点名称
            devices_config: 设备配置
            resources_config: 资源配置
            physical_setup_graph: 物理设置图
            controllers_config: 控制器配置
            bridges: 桥接器列表
            discovery_interval: 设备发现间隔（秒），默认5秒
        """
        if self._instance is not None:
            self._instance.lab_logger().critical("[Host Node] HostNode instance already exists.")
        # 初始化Node基类，传递空参数覆盖列表
        BaseROS2DeviceNode.__init__(
            self,
            driver_instance=self,
            device_id=device_id,
            status_types={},
            action_value_mappings={},
            hardware_interface={},
            print_publish=False,
            resource_tracker=DeviceNodeResourceTracker(),  # host node并不是通过initialize 包一层传进来的
        )

        # 设置单例实例
        self.__class__._instance = self

        # 初始化配置
        self.devices_config = devices_config
        self.resources_config = resources_config
        self.physical_setup_graph = physical_setup_graph
        if controllers_config is None:
            controllers_config = {}
        self.controllers_config = controllers_config
        if bridges is None:
            bridges = []
        self.bridges = bridges

        # 创建设备、动作客户端和目标存储
        self.devices_names: Dict[str, str] = {}  # 存储设备名称和命名空间的映射
        self.devices_instances: Dict[str, ROS2DeviceNode] = {}  # 存储设备实例
        self.device_machine_names: Dict[str, str] = {device_id: "本地", }  # 存储设备ID到机器名称的映射
        self._action_clients: Dict[str, ActionClient] = {}  # 用来存储多个ActionClient实例
        self._action_value_mappings: Dict[str, Dict] = (
            {}
        )  # 用来存储多个ActionClient的type, goal, feedback, result的变量名映射关系
        self._goals: Dict[str, Any] = {}  # 用来存储多个目标的状态
        self._online_devices: Set[str] = set()  # 用于跟踪在线设备
        self._last_discovery_time = 0.0  # 上次设备发现的时间
        self._discovery_lock = threading.Lock()  # 设备发现的互斥锁
        self._subscribed_topics = set()  # 用于跟踪已订阅的话题

        # 创建物料增删改查服务（非客户端）
        self._init_host_service()

        self.device_status = {}  # 用来存储设备状态
        self.device_status_timestamps = {}  # 用来存储设备状态最后更新时间

        from unilabos.app.mq import mqtt_client
        for device_config in lab_registry.obtain_registry_device_info():
            mqtt_client.publish_registry(device_config["id"], device_config)

        # 首次发现网络中的设备
        self._discover_devices()

        # 初始化所有本机设备节点，多一次过滤，防止重复初始化
        for device_id, device_config in devices_config.items():
            if device_config.get("type", "device") != "device":
                self.lab_logger().debug(
                    f"[Host Node] Skipping type {device_config['type']} {device_id} already existed, skipping."
                )
                continue
            if device_id not in self.devices_names:
                self.initialize_device(device_id, device_config)
            else:
                self.lab_logger().warning(f"[Host Node] Device {device_id} already existed, skipping.")
        self.update_device_status_subscriptions()
        # TODO: 需要验证 初始化所有控制器节点
        if controllers_config:
            update_rate = controllers_config["controller_manager"]["ros__parameters"]["update_rate"]
            for controller_id, controller_config in controllers_config["controller_manager"]["ros__parameters"][
                "controllers"
            ].items():
                controller_config["update_rate"] = update_rate
                self.initialize_controller(controller_id, controller_config)

        try:
            for bridge in self.bridges:
                if hasattr(bridge, "resource_add"):
                    self.lab_logger().info("[Host Node-Resource] Adding resources to bridge.")
                    bridge.resource_add(add_schema(resources_config))
        except Exception as ex:
            self.lab_logger().error("[Host Node-Resource] 添加物料出错！")
            self.lab_logger().error(traceback.format_exc())

        # 创建定时器，定期发现设备
        self._discovery_timer = self.create_timer(
            discovery_interval, self._discovery_devices_callback, callback_group=ReentrantCallbackGroup()
        )

        self.lab_logger().info("[Host Node] Host node initialized.")
        HostNode._ready_event.set()

    def _send_re_register(self, sclient):
        sclient.wait_for_service()
        request = SerialCommand.Request()
        request.command = ""
        future = sclient.call_async(request)
        response = future.result()

    def _discover_devices(self) -> None:
        """
        发现网络中的设备

        检测ROS2网络中的所有设备节点，并为它们创建ActionClient
        同时检测设备离线情况
        """
        self.lab_logger().debug("[Host Node] Discovering devices in the network...")

        # 获取当前所有设备
        nodes_and_names = self.get_node_names_and_namespaces()

        # 跟踪本次发现的设备，用于检测离线设备
        current_devices = set()

        for device_id, namespace in nodes_and_names:
            if not namespace.startswith("/devices/"):
                continue
            edge_device_id = namespace[9:]
            # 将设备添加到当前设备集合
            device_key = f"{namespace}/{edge_device_id}"  # namespace已经包含device_id了，这里复写一遍
            current_devices.add(device_key)

            # 如果是新设备，记录并创建ActionClient
            if edge_device_id not in self.devices_names:
                self.lab_logger().info(f"[Host Node] Discovered new device: {device_key}")
                self.devices_names[edge_device_id] = namespace
                self._create_action_clients_for_device(device_id, namespace)
                self._online_devices.add(device_key)
                sclient = self.create_client(SerialCommand, f"/srv{namespace}/query_host_name")
                threading.Thread(
                    target=self._send_re_register,
                    args=(sclient,),
                    daemon=True,
                    name=f"ROSDevice{self.device_id}_query_host_name_{namespace}"
                ).start()
            elif device_key not in self._online_devices:
                # 设备重新上线
                self.lab_logger().info(f"[Host Node] Device reconnected: {device_key}")
                self._online_devices.add(device_key)
                sclient = self.create_client(SerialCommand, f"/srv{namespace}/query_host_name")
                threading.Thread(
                    target=self._send_re_register,
                    args=(sclient,),
                    daemon=True,
                    name=f"ROSDevice{self.device_id}_query_host_name_{namespace}"
                ).start()

        # 检测离线设备
        offline_devices = self._online_devices - current_devices
        for device_key in offline_devices:
            self.lab_logger().warning(f"[Host Node] Device offline: {device_key}")
            self._online_devices.discard(device_key)

        # 更新在线设备列表
        self._online_devices = current_devices
        self.lab_logger().debug(f"[Host Node] Total online devices: {len(self._online_devices)}")

    def _discovery_devices_callback(self) -> None:
        """
        设备发现定时器回调函数
        """
        # 使用互斥锁确保同时只有一个发现过程
        if self._discovery_lock.acquire(blocking=False):
            try:
                self._discover_devices()
                # 发现新设备后，更新设备状态订阅
                self.update_device_status_subscriptions()
            finally:
                self._discovery_lock.release()
        else:
            self.lab_logger().debug("[Host Node] Device discovery already in progress, skipping.")

    def _create_action_clients_for_device(self, device_id: str, namespace: str) -> None:
        """
        为设备创建所有必要的ActionClient

        Args:
            device_id: 设备ID
            namespace: 设备命名空间
        """
        for action_id, action_types in get_action_server_names_and_types_by_node(self, device_id, namespace):
            if action_id not in self._action_clients:
                try:
                    action_type = get_ros_type_by_msgname(action_types[0])
                    self._action_clients[action_id] = ActionClient(
                        self, action_type, action_id, callback_group=self.callback_group
                    )
                    self.lab_logger().debug(f"[Host Node] Created ActionClient (Discovery): {action_id}")
                    action_name = action_id[len(namespace) + 1:]
                    edge_device_id = namespace[9:]
                    from unilabos.app.mq import mqtt_client
                    info_with_schema = ros_action_to_json_schema(action_type)
                    mqtt_client.publish_actions(action_name, {
                        "device_id": edge_device_id,
                        "action_name": action_name,
                        "schema": info_with_schema,
                    })
                except Exception as e:
                    self.lab_logger().error(f"[Host Node] Failed to create ActionClient for {action_id}: {str(e)}")

    def initialize_device(self, device_id: str, device_config: Dict[str, Any]) -> None:
        """
        根据配置初始化设备，

        此函数根据提供的设备配置动态导入适当的设备类并创建其实例。
        同时为设备的动作值映射设置动作客户端。

        Args:
            device_id: 设备唯一标识符
            device_config: 设备配置字典，包含类型和其他参数
        """
        self.lab_logger().info(f"[Host Node] Initializing device: {device_id}")

        device_config_copy = copy.deepcopy(device_config)
        d = initialize_device_from_dict(device_id, device_config_copy)
        if d is None:
            return
        # noinspection PyProtectedMember
        self.devices_names[device_id] = d._ros_node.namespace  # 这里不涉及二级device_id
        self.device_machine_names[device_id] = "本地"
        self.devices_instances[device_id] = d
        # noinspection PyProtectedMember
        for action_name, action_value_mapping in d._ros_node._action_value_mappings.items():
            action_id = f"/devices/{device_id}/{action_name}"
            if action_id not in self._action_clients:
                action_type = action_value_mapping["type"]
                self._action_clients[action_id] = ActionClient(self, action_type, action_id)
                self.lab_logger().debug(f"[Host Node] Created ActionClient (Local): {action_id}")  # 子设备再创建用的是Discover发现的
                from unilabos.app.mq import mqtt_client
                info_with_schema = ros_action_to_json_schema(action_type)
                mqtt_client.publish_actions(action_name, {
                    "device_id": device_id,
                    "action_name": action_name,
                    "schema": info_with_schema,
                })
            else:
                self.lab_logger().warning(f"[Host Node] ActionClient {action_id} already exists.")
        device_key = f"{self.devices_names[device_id]}/{device_id}"  # 这里不涉及二级device_id
        # 添加到在线设备列表
        self._online_devices.add(device_key)

    def update_device_status_subscriptions(self) -> None:
        """
        更新设备状态订阅

        扫描所有设备话题，为新的话题创建订阅，确保不会重复订阅
        """
        topic_names_and_types = self.get_topic_names_and_types()
        for topic, types in topic_names_and_types:
            # 检查是否为设备状态话题且未订阅过
            if (
                topic.startswith("/devices/")
                and not types[0].endswith("FeedbackMessage")
                and "_action" not in topic
                and topic not in self._subscribed_topics
            ):

                # 解析设备名和属性名
                parts = topic.split("/")
                if len(parts) >= 4:  # 可能有ProtocolNode，创建更长的设备
                    device_id = "/".join(parts[2:-1])
                    property_name = parts[-1]

                    # 初始化设备状态字典
                    if device_id not in self.device_status:
                        self.device_status[device_id] = {}
                        self.device_status_timestamps[device_id] = {}

                    # 默认初始化属性值为 None
                    self.device_status[device_id][property_name] = None
                    self.device_status_timestamps[device_id][property_name] = 0  # 初始化时间戳

                    # 动态创建订阅
                    try:
                        type_class = msg_converter_manager.search_class(types[0].replace("/", "."))
                        if type_class is None:
                            self.lab_logger().error(f"[Host Node] Invalid type {types[0]} for {topic}")
                        else:
                            self.create_subscription(
                                type_class,
                                topic,
                                lambda msg, d=device_id, p=property_name: self.property_callback(msg, d, p),
                                1,
                                callback_group=ReentrantCallbackGroup(),
                            )
                            # 标记为已订阅
                            self._subscribed_topics.add(topic)
                            self.lab_logger().debug(f"[Host Node] Subscribed to new topic: {topic}")
                    except (NameError, SyntaxError) as e:
                        self.lab_logger().error(f"[Host Node] Failed to create subscription for topic {topic}: {e}")

    """设备相关"""

    def property_callback(self, msg, device_id: str, property_name: str) -> None:
        """
        更新设备状态字典中的属性值，并发送到桥接器。

        Args:
            msg: 接收到的消息
            device_id: 设备ID
            property_name: 属性名称
        """
        # 更新设备状态字典
        if hasattr(msg, "data"):
            bChange = False
            if isinstance(msg.data, (float, int, str)):
                if self.device_status[device_id][property_name] != msg.data:
                    bChange = True
                self.device_status[device_id][property_name] = msg.data
                # 更新时间戳
                self.device_status_timestamps[device_id][property_name] = time.time()
            else:
                self.lab_logger().debug(
                    f"[Host Node] Unsupported data type for {device_id}/{property_name}: {type(msg.data)}"
                )

            # 所有 Bridge 对象都应具有 publish_device_status 方法；都会收到设备状态更新
            if bChange:
                for bridge in self.bridges:
                    if hasattr(bridge, "publish_device_status"):
                        bridge.publish_device_status(self.device_status, device_id, property_name)
                        self.lab_logger().debug(
                            f"[Host Node] Status updated: {device_id}.{property_name} = {msg.data}"
                        )

    def send_goal(
        self, device_id: str, action_name: str, action_kwargs: Dict[str, Any], goal_uuid: Optional[str] = None
    ) -> None:
        """
        向设备发送目标请求

        Args:
            device_id: 设备ID
            action_name: 动作名称
            action_kwargs: 动作参数
            goal_uuid: 目标UUID，如果为None则自动生成
        """
        action_id = f"/devices/{device_id}/{action_name}"
        if action_id not in self._action_clients:
            self.lab_logger().error(f"[Host Node] ActionClient {action_id} not found.")
            return

        action_client: ActionClient = self._action_clients[action_id]

        goal_msg = convert_to_ros_msg(action_client._action_type.Goal(), action_kwargs)

        self.lab_logger().info(f"[Host Node] Sending goal for {action_id}: {goal_msg}")
        action_client.wait_for_server()

        uuid_str = goal_uuid
        if goal_uuid is not None:
            u = uuid.UUID(goal_uuid)
            goal_uuid_obj = UUID(uuid=list(u.bytes))
        else:
            goal_uuid_obj = None

        future = action_client.send_goal_async(
            goal_msg,
            feedback_callback=lambda feedback_msg: self.feedback_callback(action_id, uuid_str, feedback_msg),
            goal_uuid=goal_uuid_obj,
        )
        future.add_done_callback(lambda future: self.goal_response_callback(action_id, uuid_str, future))

    def goal_response_callback(self, action_id: str, uuid_str: Optional[str], future) -> None:
        """目标响应回调"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.lab_logger().warning(f"[Host Node] Goal {action_id} ({uuid_str}) rejected")
            return

        self.lab_logger().info(f"[Host Node] Goal {action_id} ({uuid_str}) accepted")
        if uuid_str:
            self._goals[uuid_str] = goal_handle
            goal_handle.get_result_async().add_done_callback(
                lambda future: self.get_result_callback(action_id, uuid_str, future)
            )

    def feedback_callback(self, action_id: str, uuid_str: Optional[str], feedback_msg) -> None:
        """反馈回调"""
        feedback_data = convert_from_ros_msg(feedback_msg)
        feedback_data.pop("goal_id")
        self.lab_logger().debug(f"[Host Node] Feedback for {action_id} ({uuid_str}): {feedback_data}")

        if uuid_str:
            for bridge in self.bridges:
                if hasattr(bridge, "publish_job_status"):
                    bridge.publish_job_status(feedback_data, uuid_str, "running")

    def get_result_callback(self, action_id: str, uuid_str: Optional[str], future) -> None:
        """获取结果回调"""
        result_msg = future.result().result
        result_data = convert_from_ros_msg(result_msg)
        self.lab_logger().info(f"[Host Node] Result for {action_id} ({uuid_str}): success")
        self.lab_logger().debug(f"[Host Node] Result data: {result_data}")

        if uuid_str:
            for bridge in self.bridges:
                if hasattr(bridge, "publish_job_status"):
                    bridge.publish_job_status(result_data, uuid_str, "success")

    def cancel_goal(self, goal_uuid: str) -> None:
        """取消目标"""
        if goal_uuid in self._goals:
            self.lab_logger().info(f"[Host Node] Cancelling goal {goal_uuid}")
            self._goals[goal_uuid].cancel_goal_async()
        else:
            self.lab_logger().warning(f"[Host Node] Goal {goal_uuid} not found, cannot cancel")

    def get_goal_status(self, uuid_str: str) -> int:
        """获取目标状态"""
        if uuid_str in self._goals:
            g = self._goals[uuid_str]
            status = g.status
            self.lab_logger().debug(f"[Host Node] Goal status for {uuid_str}: {status}")
            return status
        self.lab_logger().warning(f"[Host Node] Goal {uuid_str} not found, status unknown")
        return GoalStatus.STATUS_UNKNOWN

    """Controller Node"""

    def initialize_controller(self, controller_id: str, controller_config: Dict[str, Any]) -> None:
        """
        初始化控制器

        Args:
            controller_id: 控制器ID
            controller_config: 控制器配置
        """
        self.lab_logger().info(f"[Host Node] Initializing controller: {controller_id}")

        class_name = controller_config.pop("type")
        controller_func = globals()[class_name]

        for input_name, input_info in controller_config["inputs"].items():
            controller_config["inputs"][input_name]["type"] = get_msg_type(eval(input_info["type"]))
        for output_name, output_info in controller_config["outputs"].items():
            controller_config["outputs"][output_name]["type"] = get_msg_type(eval(output_info["type"]))

        if controller_config["parameters"] is None:
            controller_config["parameters"] = {}

        controller = ControllerNode(controller_id, controller_func=controller_func, **controller_config)
        self.lab_logger().info(f"[Host Node] Controller {controller_id} created.")
        # rclpy.get_global_executor().add_node(controller)

    """Resource"""

    def _init_host_service(self):
        self._resource_services: Dict[str, Service] = {
            "resource_add": self.create_service(
                ResourceAdd, "/resources/add", self._resource_add_callback, callback_group=ReentrantCallbackGroup()
            ),
            "resource_get": self.create_service(
                ResourceGet, "/resources/get", self._resource_get_callback, callback_group=ReentrantCallbackGroup()
            ),
            "resource_delete": self.create_service(
                ResourceDelete,
                "/resources/delete",
                self._resource_delete_callback,
                callback_group=ReentrantCallbackGroup(),
            ),
            "resource_update": self.create_service(
                ResourceUpdate,
                "/resources/update",
                self._resource_update_callback,
                callback_group=ReentrantCallbackGroup(),
            ),
            "resource_list": self.create_service(
                ResourceList, "/resources/list", self._resource_list_callback, callback_group=ReentrantCallbackGroup()
            ),
            "node_info_update": self.create_service(
                SerialCommand,
                "/node_info_update",
                self._node_info_update_callback,
                callback_group=ReentrantCallbackGroup(),
            ),
        }

    def _node_info_update_callback(self, request, response):
        """
        更新节点信息回调
        """
        self.lab_logger().info(f"[Host Node] Node info update request received: {request}")
        try:
            from unilabos.app.mq import mqtt_client

            info = json.loads(request.command)
            if "SYNC_SLAVE_NODE_INFO" in info:
                info = info["SYNC_SLAVE_NODE_INFO"]
                machine_name = info["machine_name"]
                edge_device_id = info["edge_device_id"]
                self.device_machine_names[edge_device_id] = machine_name
            else:
                registry_config = info["registry_config"]
                for device_config in registry_config:
                    mqtt_client.publish_registry(device_config["id"], device_config)
            self.lab_logger().debug(f"[Host Node] Node info update: {info}")
            response.response = "OK"
        except Exception as e:
            self.lab_logger().error(f"[Host Node] Error updating node info: {e.args}")
            response.response = "ERROR"
        return response

    def _resource_add_callback(self, request, response):
        """
        添加资源回调

        处理添加资源请求，将资源数据传递到桥接器

        Args:
            request: 包含资源数据的请求对象
            response: 响应对象

        Returns:
            响应对象，包含操作结果
        """
        resources = [convert_from_ros_msg(resource) for resource in request.resources]
        self.lab_logger().info(f"[Host Node-Resource] Add request received: {len(resources)} resources")

        success = False
        if len(self.bridges) > 0:
            r = self.bridges[-1].resource_add(add_schema(resources))
            success = bool(r)

        response.success = success
        self.lab_logger().info(f"[Host Node-Resource] Add request completed, success: {success}")
        return response

    def _resource_get_callback(self, request, response):
        """
        获取资源回调

        处理获取资源请求，从桥接器或本地查询资源数据

        Args:
            request: 包含资源ID的请求对象
            response: 响应对象

        Returns:
            响应对象，包含查询到的资源
        """
        self.lab_logger().info(f"[Host Node-Resource] Get request for ID: {request.id}")

        if len(self.bridges) > 0:
            # 云上物料服务，根据 id 查询物料
            try:
                r = self.bridges[-1].resource_get(request.id, request.with_children)["data"]
                self.lab_logger().debug(f"[Host Node-Resource] Retrieved from bridge: {len(r)} resources")
            except Exception as e:
                self.lab_logger().error(f"[Host Node-Resource] Error retrieving from bridge: {str(e)}")
                r = []
        else:
            # 本地物料服务，根据 id 查询物料
            r = [resource for resource in self.resources_config if resource.get("id") == request.id]
            self.lab_logger().debug(f"[Host Node-Resource] Retrieved from local: {len(r)} resources")

        response.resources = [convert_to_ros_msg(Resource, resource) for resource in r]
        return response

    def _resource_delete_callback(self, request, response):
        """
        删除资源回调

        处理删除资源请求，将删除指令传递到桥接器

        Args:
            request: 包含资源ID的请求对象
            response: 响应对象

        Returns:
            响应对象，包含操作结果
        """
        self.lab_logger().info(f"[Host Node-Resource] Delete request for ID: {request.id}")

        success = False
        if len(self.bridges) > 0:
            try:
                r = self.bridges[-1].resource_delete(request.id)
                success = bool(r)
            except Exception as e:
                self.lab_logger().error(f"[Host Node-Resource] Error deleting resource: {str(e)}")

        response.success = success
        self.lab_logger().info(f"[Host Node-Resource] Delete request completed, success: {success}")
        return response

    def _resource_update_callback(self, request, response):
        """
        更新资源回调

        处理更新资源请求，将更新指令传递到桥接器

        Args:
            request: 包含资源数据的请求对象
            response: 响应对象

        Returns:
            响应对象，包含操作结果
        """
        resources = [convert_from_ros_msg(resource) for resource in request.resources]
        self.lab_logger().info(f"[Host Node-Resource] Update request received: {len(resources)} resources")

        success = False
        if len(self.bridges) > 0:
            try:
                r = self.bridges[-1].resource_update(add_schema(resources))
                success = bool(r)
            except Exception as e:
                self.lab_logger().error(f"[Host Node-Resource] Error updating resources: {str(e)}")

        response.success = success
        self.lab_logger().info(f"[Host Node-Resource] Update request completed, success: {success}")
        return response

    def _resource_list_callback(self, request, response):
        """
        列出资源回调

        处理列出资源请求，返回所有可用资源

        Args:
            request: 请求对象
            response: 响应对象

        Returns:
            响应对象，包含资源列表
        """
        self.lab_logger().info(f"[Host Node-Resource] List request received")
        # 这里可以实现返回资源列表的逻辑
        self.lab_logger().debug(f"[Host Node-Resource] List parameters: {request}")
        return response
