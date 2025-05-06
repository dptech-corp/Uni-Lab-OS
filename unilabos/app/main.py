import argparse
import asyncio
import json
import os
import signal
import sys
import threading
import time
from copy import deepcopy

import yaml

# 首先添加项目根目录到路径
current_dir = os.path.dirname(os.path.abspath(__file__))
unilabos_dir = os.path.dirname(os.path.dirname(current_dir))
if unilabos_dir not in sys.path:
    sys.path.append(unilabos_dir)

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
        "--disable_browser",
        action='store_true',
        help="是否在启动时关闭信息页",
    )
    parser.add_argument(
        "--2d_vis",
        action='store_true',
        help="是否在pylabrobot实例启动时，同时启动可视化",
    )
    parser.add_argument(
        "--visual",
        choices=["rviz", "web", "disable"],
        default="disable",
        help="选择可视化工具: rviz, web",
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
    BasicConfig.vis_2d_enable = args_dict["2d_vis"]

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

    devices_and_resources = None
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
    args_dict["resources_mesh_config"] = {}
    # web visiualize 2D
    if args_dict["visual"] != "disable":
        enable_rviz = args_dict["visual"] == "rviz"
        if devices_and_resources is not None:
            resource_visualization = ResourceVisualization(devices_and_resources, args_dict["resources_config"] ,enable_rviz=enable_rviz)
            args_dict["resources_mesh_config"] = resource_visualization.resource_model
            start_backend(**args_dict)
            server_thread = threading.Thread(target=start_server, kwargs=dict(
                open_browser=not args_dict["disable_browser"]
            ))
            server_thread.start()
            asyncio.set_event_loop(asyncio.new_event_loop())
            resource_visualization.start()
            while True:
                time.sleep(1)
        else:
            start_backend(**args_dict)
            start_server(open_browser=not args_dict["disable_browser"])
    else:
        start_backend(**args_dict)
        start_server(open_browser=not args_dict["disable_browser"])


if __name__ == "__main__":
    main()
