import argparse
import asyncio
import os
import signal
import sys
import json
import time

import yaml
from copy import deepcopy
import threading

# 首先添加项目根目录到路径
current_dir = os.path.dirname(os.path.abspath(__file__))
ilabos_dir = os.path.dirname(os.path.dirname(current_dir))
if ilabos_dir not in sys.path:
    sys.path.append(ilabos_dir)

from unilabos.config.config import load_config, BasicConfig, _update_config_from_env
from unilabos.utils.banner_print import print_status, print_unilab_banner
from unilabos.device_mesh.resource_visalization import ResourceVisualization

def parse_args():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(description="Start Uni-Lab Edge server.")
    parser.add_argument("-g", "--graph", help="Physical setup graph.")
    parser.add_argument("-d", "--devices", help="Devices config file.")
    parser.add_argument("-r", "--resources", help="Resources config file.")
    parser.add_argument("-c", "--controllers", default=None, help="Controllers config file.")
    parser.add_argument(
        "--registry_path",
        type=str,
        default=None,
        action="append",
        help="Path to the registry",
    )
    parser.add_argument(
        "--backend",
        choices=["ros", "simple", "automancer"],
        default="ros",
        help="Choose the backend to run with: 'ros', 'simple', or 'automancer'.",
    )
    parser.add_argument(
        "--app_bridges",
        nargs="+",
        default=["mqtt", "fastapi"],
        help="Bridges to connect to. Now support 'mqtt' and 'fastapi'.",
    )
    parser.add_argument(
        "--without_host",
        action="store_true",
        help="Run the backend as slave (without host).",
    )
    parser.add_argument(
        "--slave_no_host",
        action="store_true",
        help="Slave模式下跳过等待host服务",
    )
    parser.add_argument(
        "--config",
        type=str,
        default=None,
        help="配置文件路径，支持.py格式的Python配置文件",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=8002,
        help="信息页web服务的启动端口",
    )
    parser.add_argument(
        "--open_browser",
        type=bool,
        default=True,
        help="是否在启动时打开信息页",
    )
    parser.add_argument(
        "--visual",
        choices=["rviz", "web","None"],
        default="rviz",
        help="选择可视化工具: 'rviz' 或 'web' 或 'None'，默认'rviz'",
    )
    return parser.parse_args()


def main():
    """主函数"""
    # 解析命令行参数
    args = parse_args()
    args_dict = vars(args)

    # 加载配置文件，优先加载config，然后从env读取
    config_path = args_dict.get("config")
    if config_path is None:
        config_path = os.environ.get("UNILABOS.BASICCONFIG.CONFIG_PATH", None)
    if config_path:
        if not os.path.exists(config_path):
            print_status(f"配置文件 {config_path} 不存在", "error")
        elif not config_path.endswith(".py"):
            print_status(f"配置文件 {config_path} 不是Python文件，必须以.py结尾", "error")
        else:
            load_config(config_path)
    else:
        print_status(f"启动 UniLab-OS时，配置文件参数未正确传入 --config '{config_path}' 尝试本地配置...", "warning")
        load_config(config_path)

    # 设置BasicConfig参数
    BasicConfig.is_host_mode = not args_dict.get("without_host", False)
    BasicConfig.slave_no_host = args_dict.get("slave_no_host", False)
    machine_name = os.popen("hostname").read().strip()
    machine_name = "".join([c if c.isalnum() or c == "_" else "_" for c in machine_name])
    BasicConfig.machine_name = machine_name

    from unilabos.resources.graphio import (
        read_node_link_json,
        read_graphml,
        dict_from_graph,
        dict_to_nested_dict,
        initialize_resources,
    )
    from unilabos.app.mq import mqtt_client
    from unilabos.registry.registry import build_registry
    from unilabos.app.backend import start_backend
    from unilabos.app.web import http_client
    from unilabos.app.web import start_server

    # 显示启动横幅
    print_unilab_banner(args_dict)

    # 注册表
    build_registry(args_dict["registry_path"])


    if args_dict["graph"] is not None:
        import unilabos.resources.graphio as graph_res
        graph_res.physical_setup_graph = (
            read_node_link_json(args_dict["graph"])
            if args_dict["graph"].endswith(".json")
            else read_graphml(args_dict["graph"])
        )
        devices_and_resources = dict_from_graph(graph_res.physical_setup_graph)
        args_dict["resources_config"] = initialize_resources(list(deepcopy(devices_and_resources).values()))
        args_dict["devices_config"] = dict_to_nested_dict(deepcopy(devices_and_resources), devices_only=False)
        # args_dict["resources_config"] = dict_to_tree(devices_and_resources, devices_only=False)

        args_dict["graph"] = graph_res.physical_setup_graph
    else:
        if args_dict["devices"] is None or args_dict["resources"] is None:
            print_status("Either graph or devices and resources must be provided.", "error")
            sys.exit(1)
        args_dict["devices_config"] = json.load(open(args_dict["devices"], encoding="utf-8"))
        args_dict["resources_config"] = initialize_resources(
            list(json.load(open(args_dict["resources"], encoding="utf-8")).values())
        )

    print_status(f"{len(args_dict['resources_config'])} Resources loaded:", "info")
    for i in args_dict["resources_config"]:
        print_status(f"DeviceId: {i['id']}, Class: {i['class']}", "info")

    if args_dict["controllers"] is not None:
        args_dict["controllers_config"] = yaml.safe_load(open(args_dict["controllers"], encoding="utf-8"))
    else:
        args_dict["controllers_config"] = None

    args_dict["bridges"] = []

    if "mqtt" in args_dict["app_bridges"]:
        args_dict["bridges"].append(mqtt_client)
    if "fastapi" in args_dict["app_bridges"]:
        args_dict["bridges"].append(http_client)
    if "mqtt" in args_dict["app_bridges"]:

        def _exit(signum, frame):
            mqtt_client.stop()
            sys.exit(0)

        signal.signal(signal.SIGINT, _exit)
        signal.signal(signal.SIGTERM, _exit)
        mqtt_client.start()

    if args_dict["visual"] != "None":
        if args_dict["visual"] == "rviz":
            enable_rviz=True
        elif args_dict["visual"] == "web":
            enable_rviz=False
        resource_visualization = ResourceVisualization(devices_and_resources, args_dict["resources_config"] ,enable_rviz=enable_rviz)
        devices_config_add = add_resource_mesh_manager_node(resource_visualization.resource_model, args_dict["resources_config"])
        args_dict["devices_config"] = {**args_dict["devices_config"], **devices_config_add}
        start_backend(**args_dict)

        from unilabos.ros.nodes.base_device_node import ROS2DeviceNode
        while ROS2DeviceNode.get_loop() is None:
            time.sleep(0.1)
        asyncio.set_event_loop(ROS2DeviceNode.get_loop())
        resource_visualization.start()
        start_server()
    else:
        start_backend(**args_dict)
        start_server()

def add_resource_mesh_manager_node(
                                    resource_model, 
                                    resource_config ,
                                    mesh_manager_device_id = "resource_mesh_manager",
                                    joint_publisher_device_id = "joint_republisher"):
    mesh_manager_config ={
            "id": mesh_manager_device_id,
            "name": mesh_manager_device_id,
            "children": [],
            "parent": None,
            "type": "device",
            "class": "resource.mesh_manager",
            "position": {
                "x": 620.6111111111111,
                "y": 171,
                "z": 0
            },
            "config": {
                "resource_model": resource_model,
                "resource_config": resource_config
            },
            "data": {
            }
        }
    joint_publisher_config = {
        "id": joint_publisher_device_id,
        "name": joint_publisher_device_id,
        "children": [],
        "parent": None,
        "type": "device",
        "class": "joint_republisher",
        "position": {
            "x": 620.6111111111111,
            "y": 171,
            "z": 0
        },
        "config": {},
        "data": {}
    }
    return {joint_publisher_config["id"]: joint_publisher_config,mesh_manager_config["id"]: mesh_manager_config}
    # return {joint_publisher_config["id"]: joint_publisher_config}

if __name__ == "__main__":
    main()
