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

    # 注册资源信息 - 使用HTTP方式
    from unilabos.app.web.client import http_client

    resources_to_register = {}
    for resource_info in lab_registry.obtain_registry_resource_info():
        resources_to_register[resource_info["id"]] = resource_info
        logger.debug(f"[UniLab Register] 准备注册资源: {resource_info['id']}")

    if resources_to_register:
        start_time = time.time()
        response = http_client.resource_registry(resources_to_register)
        cost_time = time.time() - start_time
        if response.status_code in [200, 201]:
            logger.info(f"[UniLab Register] 成功通过HTTP注册 {len(resources_to_register)} 个资源 {cost_time}ms")
        else:
            logger.error(f"[UniLab Register] HTTP注册资源失败: {response.status_code}, {response.text} {cost_time}ms")
    logger.info("[UniLab Register] 设备和资源注册完成.")


def main():
    """
    命令行入口函数
    """
    parser = argparse.ArgumentParser(description="注册设备和资源到 MQTT")
    parser.add_argument(
        "--registry",
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
    parser.add_argument(
        "--complete_registry",
        action="store_true",
        default=False,
        help="是否补全注册表",
    )
    args = parser.parse_args()
    load_config_from_file(args.config)
    # 构建注册表
    build_registry(args.registry, args.complete_registry)
    from unilabos.app.mq import mqtt_client

    # 连接mqtt
    mqtt_client.start()

    from unilabos.registry.registry import lab_registry

    # 注册设备和资源
    register_devices_and_resources(mqtt_client, lab_registry)


if __name__ == "__main__":
    main()
