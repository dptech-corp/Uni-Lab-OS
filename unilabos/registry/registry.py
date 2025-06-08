import io
import os
import sys
from pathlib import Path
from typing import Any

import yaml

from unilabos.ros.msgs.message_converter import msg_converter_manager, ros_action_to_json_schema
from unilabos.utils import logger
from unilabos.utils.decorator import singleton

DEFAULT_PATHS = [Path(__file__).absolute().parent]


@singleton
class Registry:
    def __init__(self, registry_paths=None):
        self.registry_paths = DEFAULT_PATHS.copy()  # 使用copy避免修改默认值
        if registry_paths:
            self.registry_paths.extend(registry_paths)
        self.ResourceCreateFromOuter = self._replace_type_with_class(
            "ResourceCreateFromOuter", "host_node", f"动作 create_resource_detailed"
        )
        self.ResourceCreateFromOuterEasy = self._replace_type_with_class(
            "ResourceCreateFromOuterEasy", "host_node", f"动作 create_resource"
        )
        self.EmptyIn = self._replace_type_with_class("EmptyIn", "host_node", f"")
        self.device_type_registry = {}
        self.resource_type_registry = {}
        self._setup_called = False  # 跟踪setup是否已调用
        # 其他状态变量
        # self.is_host_mode = False  # 移至BasicConfig中

    def setup(self):
        # 检查是否已调用过setup
        if self._setup_called:
            logger.critical("[UniLab Registry] setup方法已被调用过，不允许多次调用")
            return

        from unilabos.app.web.utils.action_utils import get_yaml_from_goal_type

        self.device_type_registry.update(
            {
                "host_node": {
                    "description": "UniLabOS主机节点",
                    "class": {
                        "module": "unilabos.ros.nodes.presets.host_node",
                        "type": "python",
                        "status_types": {},
                        "action_value_mappings": {
                            "create_resource_detailed": {
                                "type": self.ResourceCreateFromOuter,
                                "goal": {
                                    "resources": "resources",
                                    "device_ids": "device_ids",
                                    "bind_parent_ids": "bind_parent_ids",
                                    "bind_locations": "bind_locations",
                                    "other_calling_params": "other_calling_params",
                                },
                                "feedback": {},
                                "result": {"success": "success"},
                                "schema": ros_action_to_json_schema(self.ResourceCreateFromOuter),
                                "goal_default": yaml.safe_load(
                                    io.StringIO(get_yaml_from_goal_type(self.ResourceCreateFromOuter.Goal))
                                ),
                                "handles": {},
                            },
                            "create_resource": {
                                "type": self.ResourceCreateFromOuterEasy,
                                "goal": {
                                    "res_id": "res_id",
                                    "class_name": "class_name",
                                    "parent": "parent",
                                    "device_id": "device_id",
                                    "bind_locations": "bind_locations",
                                    "liquid_input_slot": "liquid_input_slot[]",
                                    "liquid_type": "liquid_type[]",
                                    "liquid_volume": "liquid_volume[]",
                                    "slot_on_deck": "slot_on_deck",
                                },
                                "feedback": {},
                                "result": {"success": "success"},
                                "schema": ros_action_to_json_schema(self.ResourceCreateFromOuterEasy),
                                "goal_default": yaml.safe_load(
                                    io.StringIO(get_yaml_from_goal_type(self.ResourceCreateFromOuterEasy.Goal))
                                ),
                                "handles": {
                                    "output": [{
                                        "handler_key": "Labware",
                                        "label": "Labware",
                                        "data_type": "resource",
                                        "data_source": "handle",
                                        "data_key": "liquid"
                                    }]
                                },
                            },
                            "test_latency": {
                                "type": self.EmptyIn,
                                "goal": {},
                                "feedback": {},
                                "result": {"latency_ms": "latency_ms", "time_diff_ms": "time_diff_ms"},
                                "schema": ros_action_to_json_schema(self.EmptyIn),
                                "goal_default": {},
                                "handles": {},
                            },
                        },
                    },
                    "icon": "icon_device.webp",
                    "registry_type": "device",
                    "handles": [],
                    "init_param_schema": {},
                    "schema": {"properties": {}, "additionalProperties": False, "type": "object"},
                    "file_path": "/",
                }
            }
        )
        logger.debug(f"[UniLab Registry] ----------Setup----------")
        self.registry_paths = [Path(path).absolute() for path in self.registry_paths]
        for i, path in enumerate(self.registry_paths):
            sys_path = path.parent
            logger.debug(f"[UniLab Registry] Path {i+1}/{len(self.registry_paths)}: {sys_path}")
            sys.path.append(str(sys_path))
            self.load_device_types(path)
            self.load_resource_types(path)
        logger.info("[UniLab Registry] 注册表设置完成")
        # 标记setup已被调用
        self._setup_called = True

    def load_resource_types(self, path: os.PathLike):
        abs_path = Path(path).absolute()
        resource_path = abs_path / "resources"
        files = list(resource_path.glob("*/*.yaml"))
        logger.debug(f"[UniLab Registry] resources: {resource_path.exists()}, total: {len(files)}")
        current_resource_number = len(self.resource_type_registry) + 1
        for i, file in enumerate(files):
            data = yaml.safe_load(open(file, encoding="utf-8"))
            if data:
                # 为每个资源添加文件路径信息
                for resource_id, resource_info in data.items():
                    resource_info["file_path"] = str(file.absolute()).replace("\\", "/")
                    if "description" not in resource_info:
                        resource_info["description"] = ""
                    if "icon" not in resource_info:
                        resource_info["icon"] = ""
                    if "handles" not in resource_info:
                        resource_info["handles"] = []
                    if "init_param_schema" not in resource_info:
                        resource_info["init_param_schema"] = {}
                    resource_info["registry_type"] = "resource"
                self.resource_type_registry.update(data)
                logger.debug(
                    f"[UniLab Registry] Resource-{current_resource_number} File-{i+1}/{len(files)} "
                    + f"Add {list(data.keys())}"
                )
                current_resource_number += 1
            else:
                logger.debug(f"[UniLab Registry] Res File-{i+1}/{len(files)} Not Valid YAML File: {file.absolute()}")

    def _replace_type_with_class(self, type_name: str, device_id: str, field_name: str) -> Any:
        """
        将类型名称替换为实际的类对象

        Args:
            type_name: 类型名称
            device_id: 设备ID，用于错误信息
            field_name: 字段名称，用于错误信息

        Returns:
            找到的类对象或原始字符串

        Raises:
            SystemExit: 如果找不到类型则终止程序
        """
        # 如果类型名为空，跳过替换
        if not type_name or type_name == "":
            logger.warning(f"[UniLab Registry] 设备 {device_id} 的 {field_name} 类型为空，跳过替换")
            return type_name
        if "." in type_name:
            type_class = msg_converter_manager.get_class(type_name)
        else:
            type_class = msg_converter_manager.search_class(type_name)
        if type_class:
            return type_class
        else:
            logger.error(f"[UniLab Registry] 无法找到类型 '{type_name}' 用于设备 {device_id} 的 {field_name}")
            sys.exit(1)

    def load_device_types(self, path: os.PathLike):
        abs_path = Path(path).absolute()
        devices_path = abs_path / "devices"
        device_comms_path = abs_path / "device_comms"
        files = list(devices_path.glob("*.yaml")) + list(device_comms_path.glob("*.yaml"))
        logger.debug(
            f"[UniLab Registry] devices: {devices_path.exists()}, device_comms: {device_comms_path.exists()}, "
            + f"total: {len(files)}"
        )
        current_device_number = len(self.device_type_registry) + 1
        from unilabos.app.web.utils.action_utils import get_yaml_from_goal_type

        for i, file in enumerate(files):
            data = yaml.safe_load(open(file, encoding="utf-8"))
            if data:
                # 在添加到注册表前处理类型替换
                for device_id, device_config in data.items():
                    # 添加文件路径信息 - 使用规范化的完整文件路径
                    device_config["file_path"] = str(file.absolute()).replace("\\", "/")
                    if "description" not in device_config:
                        device_config["description"] = ""
                    if "icon" not in device_config:
                        device_config["icon"] = ""
                    if "handles" not in device_config:
                        device_config["handles"] = []
                    if "init_param_schema" not in device_config:
                        device_config["init_param_schema"] = {}
                    device_config["registry_type"] = "device"
                    if "class" in device_config:
                        # 处理状态类型
                        if "status_types" in device_config["class"]:
                            for status_name, status_type in device_config["class"]["status_types"].items():
                                device_config["class"]["status_types"][status_name] = self._replace_type_with_class(
                                    status_type, device_id, f"状态 {status_name}"
                                )

                        # 处理动作值映射
                        if "action_value_mappings" in device_config["class"]:
                            for action_name, action_config in device_config["class"]["action_value_mappings"].items():
                                if "handles" not in action_config:
                                    action_config["handles"] = []
                                if "type" in action_config:
                                    action_config["type"] = self._replace_type_with_class(
                                        action_config["type"], device_id, f"动作 {action_name}"
                                    )
                                    if action_config["type"] is not None:
                                        action_config["goal_default"] = yaml.safe_load(
                                            io.StringIO(get_yaml_from_goal_type(action_config["type"].Goal))
                                        )
                                        action_config["schema"] = ros_action_to_json_schema(action_config["type"])
                                    else:
                                        logger.warning(
                                            f"[UniLab Registry] 设备 {device_id} 的动作 {action_name} 类型为空，跳过替换"
                                        )

                self.device_type_registry.update(data)

                for device_id in data.keys():
                    logger.debug(
                        f"[UniLab Registry] Device-{current_device_number} File-{i+1}/{len(files)} Add {device_id} "
                        + f"[{data[device_id].get('name', '未命名设备')}]"
                    )
                    current_device_number += 1
            else:
                logger.debug(
                    f"[UniLab Registry] Device File-{i+1}/{len(files)} Not Valid YAML File: {file.absolute()}"
                )

    def obtain_registry_device_info(self):
        devices = []
        for device_id, device_info in self.device_type_registry.items():
            msg = {"id": device_id, **device_info}
            devices.append(msg)
        return devices

    def obtain_registry_resource_info(self):
        resources = []
        for resource_id, resource_info in self.resource_type_registry.items():
            msg = {"id": resource_id, **resource_info}
            resources.append(msg)
        return resources


# 全局单例实例
lab_registry = Registry()


def build_registry(registry_paths=None):
    """
    构建或获取Registry单例实例

    Args:
        registry_paths: 额外的注册表路径列表

    Returns:
        Registry实例
    """
    logger.info("[UniLab Registry] 构建注册表实例")

    # 由于使用了单例，这里不需要重新创建实例
    global lab_registry

    # 如果有额外路径，添加到registry_paths
    if registry_paths:
        current_paths = lab_registry.registry_paths.copy()
        # 检查是否有新路径需要添加
        for path in registry_paths:
            if path not in current_paths:
                lab_registry.registry_paths.append(path)

    # 初始化注册表
    lab_registry.setup()

    return lab_registry
