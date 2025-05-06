import copy
from typing import Optional

from unilabos.registry.registry import lab_registry
from unilabos.ros.device_node_wrapper import ros2_device_node
from unilabos.ros.nodes.base_device_node import ROS2DeviceNode, DeviceInitError
from unilabos.utils import logger
from unilabos.utils.import_manager import default_manager


def initialize_device_from_dict(device_id, device_config) -> Optional[ROS2DeviceNode]:
    """Initializes a device based on its configuration.

    This function dynamically imports the appropriate device class and creates an instance of it using the provided device configuration.
    It also sets up action clients for the device based on its action value mappings.

    Args:
        device_id (str): The unique identifier for the device.
        device_config (dict): The configuration dictionary for the device, which includes the class type and other parameters.

    Returns:
        None
    """
    d = None
    original_device_config = copy.deepcopy(device_config)
    device_class_config = device_config["class"]
    if isinstance(device_class_config, str):  # 如果是字符串，则直接去lab_registry中查找，获取class
        if device_class_config not in lab_registry.device_type_registry:
            raise ValueError(f"Device class {device_class_config} not found.")
        device_class_config = device_config["class"] = lab_registry.device_type_registry[device_class_config]["class"]
    else:
        raise ValueError("不再支持class为字典传入，class必须为注册表中已经提供的设备，您可以新增注册表并通过--registry传入")
    if isinstance(device_class_config, dict):
        DEVICE = default_manager.get_class(device_class_config["module"])
        # 不管是ros2的实例，还是python的，都必须包一次，除了HostNode
        DEVICE = ros2_device_node(
            DEVICE,
            status_types=device_class_config.get("status_types", {}),
            device_config=original_device_config,
            action_value_mappings=device_class_config.get("action_value_mappings", {}),
            hardware_interface=device_class_config.get(
                "hardware_interface",
                {"name": "hardware_interface", "write": "send_command", "read": "read_data", "extra_info": []},
            ),
            children=device_config.get("children", {})
        )
        try:
            d = DEVICE(
                device_id=device_id, driver_is_ros=device_class_config["type"] == "ros2", driver_params=device_config.get("config", {})
            )
        except DeviceInitError as ex:
            return d
    else:
        logger.warning(f"initialize device {device_id} failed, provided device_config: {device_config}")
    return d
