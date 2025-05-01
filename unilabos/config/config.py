#!/usr/bin/env python
# coding=utf-8
# 定义配置变量和加载函数
import traceback
import os
import importlib.util
from unilabos.utils import logger


class BasicConfig:
    ENV = "pro"  # 'test'
    config_path = ""
    is_host_mode = True  # 从registry.py移动过来
    slave_no_host = False  # 是否跳过rclient.wait_for_service()
    machine_name = "undefined"


# MQTT配置
class MQConfig:
    lab_id = ""
    instance_id = ""
    access_key = ""
    secret_key = ""
    group_id = ""
    broker_url = ""
    port = 1883
    ca_content = ""
    cert_content = ""
    key_content = ""

    # 指定
    ca_file = ""  # 相对config.py所在目录的路径
    cert_file = ""  # 相对config.py所在目录的路径
    key_file = ""  # 相对config.py所在目录的路径


# OSS上传配置
class OSSUploadConfig:
    api_host = ""
    authorization = ""
    init_endpoint = ""
    complete_endpoint = ""
    max_retries = 3


# HTTP配置
class HTTPConfig:
    remote_addr = "http://127.0.0.1:48197/api/v1"


# ROS配置
class ROSConfig:
    modules = [
        "std_msgs.msg",
        "geometry_msgs.msg",
        "control_msgs.msg",
        "control_msgs.action",
        "nav2_msgs.action",
        "unilabos_msgs.msg",
        "unilabos_msgs.action",
    ]


def _update_config_from_module(module):
    for name, obj in globals().items():
        if isinstance(obj, type) and name.endswith("Config"):
            if hasattr(module, name) and isinstance(getattr(module, name), type):
                for attr in dir(getattr(module, name)):
                    if not attr.startswith("_"):
                        setattr(obj, attr, getattr(getattr(module, name), attr))
    # 更新OSS认证
    if len(OSSUploadConfig.authorization) == 0:
        OSSUploadConfig.authorization = f"lab {MQConfig.lab_id}"
    # 对 ca_file cert_file key_file 进行初始化
    if len(MQConfig.ca_content) == 0:
        # 需要先判断是否为相对路径
        if MQConfig.ca_file.startswith("."):
            MQConfig.ca_file = os.path.join(BasicConfig.config_path, MQConfig.ca_file)
        if len(MQConfig.ca_file) != 0:
            with open(MQConfig.ca_file, "r", encoding="utf-8") as f:
                MQConfig.ca_content = f.read()
        else:
            logger.warning("Skipping CA file loading, ca_file is empty")
    if len(MQConfig.cert_content) == 0:
        # 需要先判断是否为相对路径
        if MQConfig.cert_file.startswith("."):
            MQConfig.cert_file = os.path.join(BasicConfig.config_path, MQConfig.cert_file)
        if len(MQConfig.ca_file) != 0:
            with open(MQConfig.cert_file, "r", encoding="utf-8") as f:
                MQConfig.cert_content = f.read()
        else:
            logger.warning("Skipping cert file loading, cert_file is empty")
    if len(MQConfig.key_content) == 0:
        # 需要先判断是否为相对路径
        if MQConfig.key_file.startswith("."):
            MQConfig.key_file = os.path.join(BasicConfig.config_path, MQConfig.key_file)
        if len(MQConfig.ca_file) != 0:
            with open(MQConfig.key_file, "r", encoding="utf-8") as f:
                MQConfig.key_content = f.read()
        else:
            logger.warning("Skipping key file loading, key_file is empty")


def _update_config_from_env():
    prefix = "UNILABOS."
    for env_key, env_value in os.environ.items():
        if not env_key.startswith(prefix):
            continue
        try:
            key_path = env_key[len(prefix):]  # Remove UNILAB_ prefix
            class_field = key_path.upper().split(".", 1)
            if len(class_field) != 2:
                logger.warning(f"[ENV] 环境变量格式不正确：{env_key}")
                continue

            class_key, field_key = class_field
            # 遍历 globals 找匹配类（不区分大小写）
            matched_cls = None
            for name, obj in globals().items():
                if name.upper() == class_key and isinstance(obj, type):
                    matched_cls = obj
                    break

            if matched_cls is None:
                logger.warning(f"[ENV] 未找到类：{class_key}")
                continue

            # 查找类属性（不区分大小写）
            matched_field = None
            for attr in dir(matched_cls):
                if attr.upper() == field_key:
                    matched_field = attr
                    break

            if matched_field is None:
                logger.warning(f"[ENV] 类 {matched_cls.__name__} 中未找到字段：{field_key}")
                continue

            current_value = getattr(matched_cls, matched_field)
            attr_type = type(current_value)
            if attr_type == bool:
                value = env_value.lower() in ("true", "1", "yes")
            elif attr_type == int:
                value = int(env_value)
            elif attr_type == float:
                value = float(env_value)
            else:
                value = env_value
            setattr(matched_cls, matched_field, value)
            logger.info(f"[ENV] 设置 {matched_cls.__name__}.{matched_field} = {value}")
        except Exception as e:
            logger.warning(f"[ENV] 解析环境变量 {env_key} 失败: {e}")



def load_config(config_path=None):
    # 如果提供了配置文件路径，从该文件导入配置
    if config_path:
        _update_config_from_env()  # 允许config_path被env设定后读取
        BasicConfig.config_path = os.path.abspath(os.path.dirname(config_path))
        if not os.path.exists(config_path):
            logger.error(f"[ENV] 配置文件 {config_path} 不存在")
            exit(1)

        try:
            module_name = "lab_" + os.path.basename(config_path).replace(".py", "")
            spec = importlib.util.spec_from_file_location(module_name, config_path)
            if spec is None:
                logger.error(f"[ENV] 配置文件 {config_path} 错误")
                return
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)  # type: ignore
            _update_config_from_module(module)
            logger.info(f"[ENV] 配置文件 {config_path} 加载成功")
        except Exception as e:
            logger.error(f"[ENV] 加载配置文件 {config_path} 失败")
            traceback.print_exc()
            exit(1)
    else:
        config_path = os.path.join(os.path.dirname(__file__), "local_config.py")
        load_config(config_path)
