import copy
import json
import os
import threading
from typing import Optional, Dict, Any, List

import rclpy
from unilabos_msgs.msg import Resource  # type: ignore
from unilabos_msgs.srv import ResourceAdd, SerialCommand  # type: ignore
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.timer import Timer

from unilabos.registry.registry import lab_registry
from unilabos.ros.initialize_device import initialize_device_from_dict
from unilabos.ros.msgs.message_converter import (
    convert_to_ros_msg,
)
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
    devices_config: Dict[str, Any] = {},
    resources_config={},
    graph: Optional[Dict[str, Any]] = None,
    controllers_config: Dict[str, Any] = {},
    bridges: List[Any] = [],
    args: List[str] = ["--log-level", "debug"],
    discovery_interval: float = 5.0,
) -> None:
    """主函数"""
    rclpy.init(args=args)
    rclpy.__executor = executor = MultiThreadedExecutor()

    # 创建主机节点
    host_node = HostNode(
        "host_node",
        devices_config,
        resources_config,
        graph,
        controllers_config,
        bridges,
        discovery_interval,
    )

    thread = threading.Thread(target=executor.spin, daemon=True, name="host_executor_thread")
    thread.start()

    while True:
        input()


def slave(
    devices_config: Dict[str, Any] = {},
    resources_config=[],
    graph: Optional[Dict[str, Any]] = None,
    controllers_config: Dict[str, Any] = {},
    bridges: List[Any] = [],
    args: List[str] = ["--log-level", "debug"],
) -> None:
    """从节点函数"""
    rclpy.init(args=args)
    rclpy.__executor = executor = MultiThreadedExecutor()
    devices_config_copy = copy.deepcopy(devices_config)
    for device_id, device_config in devices_config.items():
        d = initialize_device_from_dict(device_id, device_config)
        if d is None:
            continue
        # 默认初始化
        # if d is not None and isinstance(d, Node):
        #     executor.add_node(d)
        # else:
        #     print(f"Warning: Device {device_id} could not be initialized or is not a valid Node")

    n = Node(f"slaveMachine_{BasicConfig.machine_name}", parameter_overrides=[])
    executor.add_node(n)

    thread = threading.Thread(target=executor.spin, daemon=True, name="slave_executor_thread")
    thread.start()

    if not BasicConfig.slave_no_host:
        sclient = n.create_client(SerialCommand, "/node_info_update")
        sclient.wait_for_service()

        request = SerialCommand.Request()
        request.command = json.dumps({
            "machine_name": BasicConfig.machine_name,
            "type": "slave",
            "devices_config": devices_config_copy,
            "registry_config": lab_registry.obtain_registry_device_info()
        }, ensure_ascii=False, cls=TypeEncoder)
        response = sclient.call_async(request).result()
        logger.info(f"Slave node info updated.")

        rclient = n.create_client(ResourceAdd, "/resources/add")
        rclient.wait_for_service()  # FIXME 可能一直等待，加一个参数

        request = ResourceAdd.Request()
        request.resources = [convert_to_ros_msg(Resource, resource) for resource in resources_config]
        response = rclient.call_async(request).result()
        logger.info(f"Slave resource added.")

    while True:
        input()

if __name__ == "__main__":
    main()
