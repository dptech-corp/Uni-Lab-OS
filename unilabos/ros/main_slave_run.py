import json
import threading
import time
from typing import Optional, Dict, Any, List

import rclpy
from unilabos_msgs.srv._serial_command import SerialCommand_Response

from unilabos.ros.nodes.presets.resource_mesh_manager import ResourceMeshManager
from unilabos.ros.nodes.resource_tracker import DeviceNodeResourceTracker, ResourceTreeSet
from unilabos.devices.ros_dev.liquid_handler_joint_publisher import LiquidHandlerJointPublisher
from unilabos_msgs.srv import SerialCommand  # type: ignore
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.timer import Timer

from unilabos.registry.registry import lab_registry
from unilabos.ros.initialize_device import initialize_device_from_dict
from unilabos.ros.nodes.presets.host_node import HostNode
from unilabos.utils import logger
from unilabos.config.config import BasicConfig
from unilabos.utils.type_check import TypeEncoder


def exit() -> None:
    """关闭ROS节点和资源"""
    host_instance = HostNode.get_instance()
    if host_instance is not None:
        # 停止发现定时器
        # noinspection PyProtectedMember
        if hasattr(host_instance, "_discovery_timer") and isinstance(host_instance._discovery_timer, Timer):
            # noinspection PyProtectedMember
            host_instance._discovery_timer.cancel()
        for _, device_node in host_instance.devices_instances.items():
            if hasattr(device_node, "destroy_node"):
                device_node.ros_node_instance.destroy_node()
        host_instance.destroy_node()
    rclpy.shutdown()


def main(
    devices_config: ResourceTreeSet,
    resources_config: ResourceTreeSet,
    resources_edge_config: list[dict] = [],
    graph: Optional[Dict[str, Any]] = None,
    controllers_config: Dict[str, Any] = {},
    bridges: List[Any] = [],
    visual: str = "disable",
    resources_mesh_config: dict = {},
    rclpy_init_args: List[str] = ["--log-level", "debug"],
    discovery_interval: float = 15.0,
) -> None:
    """主函数"""

    rclpy.init(args=rclpy_init_args)
    executor = rclpy.__executor = MultiThreadedExecutor()
    # 创建主机节点
    host_node = HostNode(
        "host_node",
        devices_config,
        resources_config,
        resources_edge_config,
        graph,
        controllers_config,
        bridges,
        discovery_interval,
    )

    if visual != "disable":
        from unilabos.ros.nodes.presets.joint_republisher import JointRepublisher

        # 将 ResourceTreeSet 转换为 list 用于 visual 组件
        resources_list = (
            [node.res_content.model_dump(by_alias=True) for node in resources_config.all_nodes]
            if resources_config
            else []
        )
        resource_mesh_manager = ResourceMeshManager(
            resources_mesh_config,
            resources_list,
            resource_tracker=host_node.resource_tracker,
            device_id="resource_mesh_manager",
        )
        joint_republisher = JointRepublisher("joint_republisher", host_node.resource_tracker)
        lh_joint_pub = LiquidHandlerJointPublisher(
            resources_config=resources_list, resource_tracker=host_node.resource_tracker
        )
        executor.add_node(resource_mesh_manager)
        executor.add_node(joint_republisher)
        executor.add_node(lh_joint_pub)

    thread = threading.Thread(target=executor.spin, daemon=True, name="host_executor_thread")
    thread.start()

    while True:
        time.sleep(1)


def slave(
    devices_config: ResourceTreeSet,
    resources_config: ResourceTreeSet,
    resources_edge_config: list = [],
    graph: Optional[Dict[str, Any]] = None,
    controllers_config: Dict[str, Any] = {},
    bridges: List[Any] = [],
    visual: str = "disable",
    resources_mesh_config: dict = {},
    rclpy_init_args: List[str] = ["--log-level", "debug"],
) -> None:
    """从节点函数"""
    if not rclpy.ok():
        rclpy.init(args=rclpy_init_args)
    executor = rclpy.__executor
    if not executor:
        executor = rclpy.__executor = MultiThreadedExecutor()
    devices_instances = {}
    for device_config in devices_config.root_nodes:
        device_id = device_config.res_content.id
        if device_config.res_content.type != "device":
            d = initialize_device_from_dict(device_id, device_config.get_nested_dict())
            devices_instances[device_id] = d
        # 默认初始化
        # if d is not None and isinstance(d, Node):
        #     executor.add_node(d)
        # else:
        #     print(f"Warning: Device {device_id} could not be initialized or is not a valid Node")

    n = Node(f"slaveMachine_{BasicConfig.machine_name}", parameter_overrides=[])
    executor.add_node(n)

    if visual != "disable":
        from unilabos.ros.nodes.presets.joint_republisher import JointRepublisher

        resource_mesh_manager = ResourceMeshManager(
            resources_mesh_config,
            resources_config,  # type: ignore FIXME
            resource_tracker=DeviceNodeResourceTracker(),
            device_id="resource_mesh_manager",
        )
        joint_republisher = JointRepublisher("joint_republisher", DeviceNodeResourceTracker())

        executor.add_node(resource_mesh_manager)
        executor.add_node(joint_republisher)
    thread = threading.Thread(target=executor.spin, daemon=True, name="slave_executor_thread")
    thread.start()

    if not BasicConfig.slave_no_host:
        sclient = n.create_client(SerialCommand, "/node_info_update")
        sclient.wait_for_service()

        request = SerialCommand.Request()
        request.command = json.dumps(
            {
                "machine_name": BasicConfig.machine_name,
                "type": "slave",
                "devices_config": devices_config.dump(),
                "registry_config": lab_registry.obtain_registry_device_info(),
            },
            ensure_ascii=False,
            cls=TypeEncoder,
        )
        response = sclient.call_async(request).result()
        logger.info(f"Slave node info updated.")

        # 使用新的 c2s_update_resource_tree 服务
        rclient = n.create_client(SerialCommand, "/c2s_update_resource_tree")
        rclient.wait_for_service()

        # 序列化 ResourceTreeSet 为 JSON
        if resources_config:
            request = SerialCommand.Request()
            request.command = json.dumps(
                {
                    "data": {
                        "data": resources_config.dump(),
                        "mount_uuid": "",
                        "first_add": True,
                    },
                    "action": "add",
                },
                ensure_ascii=False,
            )
            tree_response: SerialCommand_Response = rclient.call_async(request).result()
            uuid_mapping = json.loads(tree_response.response)
            # 创建反向映射：new_uuid -> old_uuid
            reverse_uuid_mapping = {new_uuid: old_uuid for old_uuid, new_uuid in uuid_mapping.items()}
            for node in resources_config.root_nodes:
                if node.res_content.type == "device":
                    for sub_node in node.children:
                        # 只有二级子设备
                        if sub_node.res_content.type != "device":
                            device_tracker = devices_instances[node.res_content.id].resource_tracker
                            # sub_node.res_content.uuid 已经是新UUID，需要用旧UUID去查找
                            old_uuid = reverse_uuid_mapping.get(sub_node.res_content.uuid)
                            if old_uuid:
                                # 找到旧UUID，使用UUID查找
                                resource_instance = device_tracker.figure_resource({"uuid": old_uuid})
                            else:
                                # 未找到旧UUID，使用name查找
                                resource_instance = device_tracker.figure_resource({"name": sub_node.res_content.name})
                            device_tracker.loop_update_uuid(resource_instance, uuid_mapping)
                else:
                    logger.error("Slave模式不允许新增非设备节点下的物料")
                    continue
            if tree_response:
                logger.info(f"Slave resource tree added. Response: {tree_response.response}")
            else:
                logger.warning("Slave resource tree add response is None")
        else:
            logger.info("No resources to add.")

    while True:
        time.sleep(1)


if __name__ == "__main__":
    main()
