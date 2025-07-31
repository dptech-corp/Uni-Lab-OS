import argparse
import asyncio
import os
import shutil
import signal
import sys
import threading
import time
from copy import deepcopy

import yaml

from unilabos.resources.graphio import modify_to_backend_format

# 首先添加项目根目录到路径
current_dir = os.path.dirname(os.path.abspath(__file__))
unilabos_dir = os.path.dirname(os.path.dirname(current_dir))
if unilabos_dir not in sys.path:
    sys.path.append(unilabos_dir)

from unilabos.config.config import load_config, BasicConfig
from unilabos.utils.banner_print import print_status, print_unilab_banner


def load_config_from_file(config_path, override_labid=None):
    if config_path is None:
        config_path = os.environ.get("UNILABOS.BASICCONFIG.CONFIG_PATH", None)
    if config_path:
        if not os.path.exists(config_path):
            print_status(f"配置文件 {config_path} 不存在", "error")
        elif not config_path.endswith(".py"):
            print_status(f"配置文件 {config_path} 不是Python文件，必须以.py结尾", "error")
        else:
            load_config(config_path, override_labid)
    else:
        print_status(f"启动 UniLab-OS时，配置文件参数未正确传入 --config '{config_path}' 尝试本地配置...", "warning")
        load_config(config_path, override_labid)


def convert_argv_dashes_to_underscores(args: argparse.ArgumentParser):
    # easier for user input, easier for dev search code
    option_strings = list(args._option_string_actions.keys())
    for i, arg in enumerate(sys.argv):
        for option_string in option_strings:
            if arg.startswith(option_string):
                new_arg = arg[:2] + arg[2:len(option_string)].replace("-", "_") + arg[len(option_string):]
                sys.argv[i] = new_arg
                break

def parse_args():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(description="Start Uni-Lab Edge server.")
    parser.add_argument("-g", "--graph", help="Physical setup graph.")
    # parser.add_argument("-d", "--devices", help="Devices config file.")
    # parser.add_argument("-r", "--resources", help="Resources config file.")
    parser.add_argument("-c", "--controllers", default=None, help="Controllers config file.")
    parser.add_argument(
        "--registry_path",
        type=str,
        default=None,
        action="append",
        help="Path to the registry",
    )
    parser.add_argument(
        "--working_dir",
        type=str,
        default=None,
        help="Path to the working directory",
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
        "--upload_registry",
        action="store_true",
        help="启动unilab时同时报送注册表信息",
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
        action="store_true",
        help="是否在启动时关闭信息页",
    )
    parser.add_argument(
        "--2d_vis",
        action="store_true",
        help="是否在pylabrobot实例启动时，同时启动可视化",
    )
    parser.add_argument(
        "--visual",
        choices=["rviz", "web", "disable"],
        default="disable",
        help="选择可视化工具: rviz, web",
    )
    parser.add_argument(
        "--labid",
        type=str,
        default="",
        help="实验室唯一ID，也可通过环境变量 UNILABOS.MQCONFIG.LABID 设置或传入--config设置",
    )
    return parser


def main():
    """主函数"""
    # 解析命令行参数
    args = parse_args()
    convert_argv_dashes_to_underscores(args)
    args_dict = vars(args.parse_args())

    # 加载配置文件，优先加载config，然后从env读取
    config_path = args_dict.get("config")
    working_dir = os.path.abspath(os.path.join(os.getcwd(), "unilabos_data"))
    if not config_path and (not os.path.exists(working_dir) or not os.path.exists(os.path.join(working_dir, "local_config.py"))):
        print_status(f"当前未指定config路径，非第一次使用请通过 --config 传入 local_config.py 文件路径", "info")
        print_status(f"您是否为第一次使用？并将当前文件路径 {working_dir} 作为工作目录？ (Y/n)", "info")
        if input() != "n":
            os.makedirs(working_dir, exist_ok=True)
            config_path = os.path.join(working_dir, "local_config.py")
            shutil.copy(os.path.join(os.path.dirname(os.path.dirname(__file__)), "config", "example_config.py"), config_path)
            print_status(f"已创建 local_config.py 路径： {config_path}", "info")
            print_status(f"请在文件夹中配置lab_id，放入下载的CA.crt、lab.crt、lab.key重新启动本程序", "info")
            os._exit(1)
        else:
            os._exit(1)
    else:
        working_dir = args_dict.get("working_dir") or os.path.abspath(os.path.join(os.getcwd(), "unilabos_data"))
        if working_dir:
            if config_path and not os.path.exists(config_path):
                config_path = os.path.join(working_dir, "local_config.py")
                if not os.path.exists(config_path):
                    print_status(f"当前工作目录 {working_dir} 未找到local_config.py，请通过 --config 传入 local_config.py 文件路径", "error")
                    os._exit(1)
            print_status(f"当前工作目录为 {working_dir}", "info")
    # 加载配置文件
    load_config_from_file(config_path, args_dict["labid"])

    # 设置BasicConfig参数
    BasicConfig.working_dir = working_dir
    BasicConfig.is_host_mode = not args_dict.get("without_host", False)
    BasicConfig.slave_no_host = args_dict.get("slave_no_host", False)
    BasicConfig.upload_registry = args_dict.get("upload_registry", False)
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
    if args_dict["graph"] is None:
        request_startup_json = http_client.request_startup_json()
        if not request_startup_json:
            print_status(
                "未指定设备加载文件路径，尝试从HTTP获取失败，请检查网络或者使用-g参数指定设备加载文件路径", "error"
            )
            os._exit(1)
        else:
            print_status("联网获取设备加载文件成功", "info")
        graph, data = read_node_link_json(request_startup_json)
    else:
        if args_dict["graph"].endswith(".json"):
            graph, data = read_node_link_json(args_dict["graph"])
        else:
            graph, data = read_graphml(args_dict["graph"])
    import unilabos.resources.graphio as graph_res

    graph_res.physical_setup_graph = graph
    resource_edge_info = modify_to_backend_format(data["links"])
    devices_and_resources = dict_from_graph(graph_res.physical_setup_graph)
    # args_dict["resources_config"] = initialize_resources(list(deepcopy(devices_and_resources).values()))
    args_dict["resources_config"] = list(devices_and_resources.values())
    args_dict["devices_config"] = dict_to_nested_dict(deepcopy(devices_and_resources), devices_only=False)
    args_dict["graph"] = graph_res.physical_setup_graph

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
    args_dict["resources_edge_config"] = resource_edge_info
    # web visiualize 2D
    if args_dict["visual"] != "disable":
        enable_rviz = args_dict["visual"] == "rviz"
        if devices_and_resources is not None:
            from unilabos.device_mesh.resource_visalization import (
                ResourceVisualization,
            )  # 此处开启后，logger会变更为INFO，有需要请调整

            resource_visualization = ResourceVisualization(
                devices_and_resources, args_dict["resources_config"], enable_rviz=enable_rviz
            )
            args_dict["resources_mesh_config"] = resource_visualization.resource_model
            start_backend(**args_dict)
            server_thread = threading.Thread(
                target=start_server,
                kwargs=dict(
                    open_browser=not args_dict["disable_browser"],
                    port=args_dict["port"],
                ),
            )
            server_thread.start()
            asyncio.set_event_loop(asyncio.new_event_loop())
            resource_visualization.start()
            while True:
                time.sleep(1)
        else:
            start_backend(**args_dict)
            start_server(
                open_browser=not args_dict["disable_browser"],
                port=args_dict["port"],
            )
    else:
        start_backend(**args_dict)
        start_server(
            open_browser=not args_dict["disable_browser"],
            port=args_dict["port"],
        )


if __name__ == "__main__":
    main()
