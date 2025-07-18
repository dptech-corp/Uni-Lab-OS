import copy
import io
import os
import sys
from pathlib import Path
from typing import Any, Dict, List

import yaml

from unilabos.ros.msgs.message_converter import msg_converter_manager, ros_action_to_json_schema, String
from unilabos.utils import logger
from unilabos.utils.decorator import singleton
from unilabos.utils.import_manager import get_enhanced_class_info
from unilabos.utils.type_check import NoAliasDumper

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

    def setup(self, complete_registry=False):
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
                                    "output": [
                                        {
                                            "handler_key": "labware",
                                            "label": "Labware",
                                            "data_type": "resource",
                                            "data_source": "handle",
                                            "data_key": "liquid",
                                        }
                                    ]
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
            self.load_device_types(path, complete_registry)
            self.load_resource_types(path, complete_registry)
        logger.info("[UniLab Registry] 注册表设置完成")
        # 标记setup已被调用
        self._setup_called = True

    def load_resource_types(self, path: os.PathLike, complete_registry: bool):
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
        convert_manager = {  # 将python基本对象转为ros2基本对象
            "str": "String",
            "bool": "Bool",
            "int": "Int64",
            "float": "Float64",
        }
        type_name = convert_manager.get(type_name, type_name)  # 替换为ROS2类型
        if ":" in type_name:
            type_class = msg_converter_manager.get_class(type_name)
        else:
            type_class = msg_converter_manager.search_class(type_name)
        if type_class:
            return type_class
        else:
            logger.error(f"[UniLab Registry] 无法找到类型 '{type_name}' 用于设备 {device_id} 的 {field_name}")
            sys.exit(1)

    def _generate_unilab_json_command_schema(
        self, method_args: List[Dict[str, Any]], method_name: str
    ) -> Dict[str, Any]:
        """
        根据UniLabJsonCommand方法信息生成JSON Schema，暂不支持嵌套类型

        Args:
            method_args: 方法信息字典，包含args等
            method_name: 方法名称

        Returns:
            JSON Schema格式的参数schema
        """
        schema = {
            "description": f"UniLabJsonCommand {method_name} 的参数schema",
            "type": "object",
            "properties": {},
            "required": [],
        }

        for arg_info in method_args:
            param_name = arg_info.get("name", "")
            param_type = arg_info.get("type")
            param_default = arg_info.get("default")
            param_required = arg_info.get("required", True)

            prop_schema = {"description": f"参数: {param_name}"}

            # 根据类型设置schema FIXME 不完整
            if param_type:
                param_type_lower = param_type.lower()
                if param_type_lower in ["str", "string"]:
                    prop_schema["type"] = "string"
                elif param_type_lower in ["int", "integer"]:
                    prop_schema["type"] = "integer"
                elif param_type_lower in ["float", "number"]:
                    prop_schema["type"] = "number"
                elif param_type_lower in ["bool", "boolean"]:
                    prop_schema["type"] = "boolean"
                elif param_type_lower in ["list", "array"]:
                    prop_schema["type"] = "array"
                elif param_type_lower in ["dict", "object"]:
                    prop_schema["type"] = "object"
                else:
                    # 默认为字符串类型
                    prop_schema["type"] = "string"
            else:
                # 如果没有类型信息，默认为字符串
                prop_schema["type"] = "string"

            # 设置默认值
            if param_default is not None:
                prop_schema["default"] = param_default

            schema["properties"][param_name] = prop_schema

            # 如果是必需参数，添加到required列表
            if param_required:
                schema["required"].append(param_name)
        return {
            "title": f"{method_name} 命令参数",
            "description": f"UniLabJsonCommand {method_name} 的参数schema",
            "type": "object",
            "properties": {"goal": schema, "feedback": {}, "result": {}},
            "required": ["goal"],
        }

    def load_device_types(self, path: os.PathLike, complete_registry: bool):
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
            with open(file, encoding="utf-8", mode="r") as f:
                data = yaml.safe_load(io.StringIO(f.read()))
            complete_data = {}
            action_str_type_mapping = {
                "UniLabJsonCommand": "UniLabJsonCommand",
                "UniLabJsonCommandAsync": "UniLabJsonCommandAsync",
            }
            status_str_type_mapping = {}
            if data:
                # 在添加到注册表前处理类型替换
                for device_id, device_config in data.items():
                    # 添加文件路径信息 - 使用规范化的完整文件路径
                    if "description" not in device_config:
                        device_config["description"] = ""
                    if "icon" not in device_config:
                        device_config["icon"] = ""
                    if "handles" not in device_config:
                        device_config["handles"] = []
                    if "init_param_schema" not in device_config:
                        device_config["init_param_schema"] = {}
                    if "class" in device_config:
                        if "status_types" not in device_config["class"]:
                            device_config["class"]["status_types"] = {}
                        if "action_value_mappings" not in device_config["class"]:
                            device_config["class"]["action_value_mappings"] = {}
                        enhanced_info = {}
                        if complete_registry:
                            device_config["class"]["status_types"].clear()
                            enhanced_info = get_enhanced_class_info(device_config["class"]["module"], use_dynamic=True)
                            device_config["class"]["status_types"].update(
                                {k: v["return_type"] for k, v in enhanced_info["status_methods"].items()}
                            )
                        for status_name, status_type in device_config["class"]["status_types"].items():
                            if status_type in ["Any", "None", "Unknown"]:
                                status_type = "String"  # 替换成ROS的String，便于显示
                                device_config["class"]["status_types"][status_name] = status_type
                            target_type = self._replace_type_with_class(status_type, device_id, f"状态 {status_name}")
                            if target_type in [dict, list]:  # 对于嵌套类型返回的对象，暂时处理成字符串，无法直接进行转换
                                target_type = String
                            status_str_type_mapping[status_type] = target_type
                        device_config["class"]["status_types"] = dict(
                            sorted(device_config["class"]["status_types"].items())
                        )
                        if complete_registry:
                            device_config["class"]["action_value_mappings"] = {k:v for k, v in device_config["class"]["action_value_mappings"].items() if not k.startswith("auto-")}
                            # 处理动作值映射
                            device_config["class"]["action_value_mappings"].update(
                                {
                                    f"auto-{k}": {
                                        "type": "UniLabJsonCommandAsync" if v["is_async"] else "UniLabJsonCommand",
                                        "goal": {},
                                        "feedback": {},
                                        "result": {},
                                        "schema": self._generate_unilab_json_command_schema(v["args"], k),
                                        "goal_default": {i["name"]: i["default"] for i in v["args"]},
                                        "handles": [],
                                    }
                                    for k, v in enhanced_info["action_methods"].items()
                                }
                            )
                            device_config["init_param_schema"] = self._generate_unilab_json_command_schema(
                                enhanced_info["init_params"], "__init__"
                            )
                        device_config.pop("schema", None)
                        device_config["class"]["action_value_mappings"] = dict(
                            sorted(device_config["class"]["action_value_mappings"].items())
                        )
                        for action_name, action_config in device_config["class"]["action_value_mappings"].items():
                            if "handles" not in action_config:
                                action_config["handles"] = []
                            if "type" in action_config:
                                action_type_str: str = action_config["type"]
                                # 通过Json发放指令，而不是通过特殊的ros action进行处理
                                if not action_type_str.startswith("UniLabJsonCommand"):
                                    target_type = self._replace_type_with_class(
                                        action_type_str, device_id, f"动作 {action_name}"
                                    )
                                    action_str_type_mapping[action_type_str] = target_type
                                    if target_type is not None:
                                        action_config["goal_default"] = yaml.safe_load(
                                            io.StringIO(get_yaml_from_goal_type(target_type.Goal))
                                        )
                                        action_config["schema"] = ros_action_to_json_schema(target_type)
                                    else:
                                        logger.warning(
                                            f"[UniLab Registry] 设备 {device_id} 的动作 {action_name} 类型为空，跳过替换"
                                        )
                        complete_data[device_id] = copy.deepcopy(dict(sorted(device_config.items())))  # 稍后dump到文件
                        for status_name, status_type in device_config["class"]["status_types"].items():
                            device_config["class"]["status_types"][status_name] = status_str_type_mapping[status_type]
                        for action_name, action_config in device_config["class"]["action_value_mappings"].items():
                            action_config["type"] = action_str_type_mapping[action_config["type"]]
                        for additional_action in ["_execute_driver_command", "_execute_driver_command_async"]:
                            device_config["class"]["action_value_mappings"][additional_action] = {
                                "type": self._replace_type_with_class(
                                    "StrSingleInput", device_id, f"动作 {additional_action}"
                                ),
                                "goal": {"string": "string"},
                                "feedback": {},
                                "result": {},
                                "schema": ros_action_to_json_schema(
                                    self._replace_type_with_class(
                                        "StrSingleInput", device_id, f"动作 {additional_action}"
                                    )
                                ),
                                "goal_default": yaml.safe_load(
                                    io.StringIO(
                                        get_yaml_from_goal_type(
                                            self._replace_type_with_class(
                                                "StrSingleInput", device_id, f"动作 {additional_action}"
                                            ).Goal
                                        )
                                    )
                                ),
                                "handles": [],
                            }
                    if "registry_type" not in device_config:
                        device_config["registry_type"] = "device"
                    device_config["file_path"] = str(file.absolute()).replace("\\", "/")
                    device_config["registry_type"] = "device"
                    logger.debug(
                        f"[UniLab Registry] Device-{current_device_number} File-{i+1}/{len(files)} Add {device_id} "
                        + f"[{data[device_id].get('name', '未命名设备')}]"
                    )
                    current_device_number += 1
                complete_data = dict(sorted(complete_data.items()))
                complete_data = copy.deepcopy(complete_data)
                with open(file, "w", encoding="utf-8") as f:
                    yaml.dump(complete_data, f, allow_unicode=True, default_flow_style=False, Dumper=NoAliasDumper)
                self.device_type_registry.update(data)
            else:
                logger.debug(
                    f"[UniLab Registry] Device File-{i+1}/{len(files)} Not Valid YAML File: {file.absolute()}"
                )

    def obtain_registry_device_info(self):
        devices = []
        for device_id, device_info in self.device_type_registry.items():
            device_info_copy = copy.deepcopy(device_info)
            if "class" in device_info_copy and "action_value_mappings" in device_info_copy["class"]:
                action_mappings = device_info_copy["class"]["action_value_mappings"]
                for action_name, action_config in action_mappings.items():
                    if "schema" in action_config and action_config["schema"]:
                        schema = action_config["schema"]
                        # 确保schema结构存在
                        if (
                            "properties" in schema
                            and "goal" in schema["properties"]
                            and "properties" in schema["properties"]["goal"]
                        ):
                            schema["properties"]["goal"]["properties"] = {
                                "unilabos_device_id": {
                                    "type": "string",
                                    "default": "",
                                    "description": "UniLabOS设备ID，用于指定执行动作的具体设备实例",
                                },
                                **schema["properties"]["goal"]["properties"],
                            }

            msg = {"id": device_id, **device_info_copy}
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


def build_registry(registry_paths=None, complete_registry=False):
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
    lab_registry.setup(complete_registry)

    return lab_registry
