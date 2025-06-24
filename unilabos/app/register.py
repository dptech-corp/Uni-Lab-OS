import argparse
import time

from unilabos.registry.registry import build_registry

from unilabos.app.main import load_config_from_file
from unilabos.utils.log import logger


def register_devices_and_resources(mqtt_client, lab_registry):
    """
    注册设备和资源到 MQTT
    """
    logger.info("[UniLab Register] 开始注册设备和资源...")

    # 注册设备信息
    for device_info in lab_registry.obtain_registry_device_info():
        mqtt_client.publish_registry(device_info["id"], device_info, False)
        logger.debug(f"[UniLab Register] 注册设备: {device_info['id']}")

    # 注册资源信息
    for resource_info in lab_registry.obtain_registry_resource_info():
        mqtt_client.publish_registry(resource_info["id"], resource_info, False)
        logger.debug(f"[UniLab Register] 注册资源: {resource_info['id']}")

    time.sleep(10)

    logger.info("[UniLab Register] 设备和资源注册完成.")


def main():
    """
    命令行入口函数
    """
    parser = argparse.ArgumentParser(description="注册设备和资源到 MQTT")
    parser.add_argument(
        "--registry_path",
        type=str,
        default=None,
        action="append",
        help="注册表路径",
    )
    parser.add_argument(
        "--config",
        type=str,
        default=None,
        help="配置文件路径，支持.py格式的Python配置文件",
    )
    args = parser.parse_args()

    # 构建注册表
    build_registry(args.registry_path)
    load_config_from_file(args.config)

    from unilabos.app.mq import mqtt_client

    # 连接mqtt
    mqtt_client.start()

    from unilabos.registry.registry import lab_registry

    # 注册设备和资源
    register_devices_and_resources(mqtt_client, lab_registry)


if __name__ == "__main__":
    main()