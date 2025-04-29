"""
Web页面模块

提供系统Web界面的页面定义
"""

import json
import os
import sys
from pathlib import Path
from typing import Dict

from fastapi import APIRouter, HTTPException
from fastapi.responses import HTMLResponse, JSONResponse
from jinja2 import Environment, FileSystemLoader

from unilabos.config.config import BasicConfig
from unilabos.registry.registry import lab_registry
from unilabos.app.mq import mqtt_client
from unilabos.ros.msgs.message_converter import msg_converter_manager
from unilabos.utils.log import error
from unilabos.utils.type_check import TypeEncoder
from unilabos.app.web.utils.device_utils import get_registry_info
from unilabos.app.web.utils.host_utils import get_host_node_info
from unilabos.app.web.utils.ros_utils import get_ros_node_info, update_ros_node_info

# 设置Jinja2模板环境
template_dir = Path(__file__).parent / "templates"
env = Environment(loader=FileSystemLoader(template_dir))


def setup_web_pages(router: APIRouter) -> None:
    """
    设置Web页面路由

    Args:
        router: FastAPI路由器实例
    """
    # 在web服务启动时，尝试初始化ROS节点信息
    update_ros_node_info()

    @router.get("/", response_class=HTMLResponse, summary="Home Page")
    async def home_page() -> str:
        """
        首页，显示所有可用的API路由

        Returns:
            HTMLResponse: 渲染后的HTML页面
        """
        try:
            # 收集所有路由
            routes = []
            for route in router.routes:
                if hasattr(route, "methods") and hasattr(route, "path"):
                    for method in list(getattr(route, "methods", [])):
                        path = getattr(route, "path", "")
                        # 只显示GET方法的路由作为链接
                        if method == "GET":
                            name = getattr(route, "name", "") or path
                            summary = getattr(route, "summary", "") or name
                            routes.append({"method": method, "path": path, "name": name, "summary": summary})

            # 使用模板渲染页面
            template = env.get_template("home.html")
            html = template.render(routes=routes)

            return html
        except Exception as e:
            error(f"生成主页时出错: {str(e)}")
            raise HTTPException(status_code=500, detail=f"Error generating home page: {str(e)}")

    @router.get("/status", response_class=HTMLResponse, summary="System Status")
    async def status_page() -> str:
        """
        状态页面，显示系统状态信息

        Returns:
            HTMLResponse: 渲染后的HTML页面
        """
        try:
            # 准备设备数据
            devices = []
            resources = []
            modules = {"names": [], "classes": [], "displayed_count": 0, "total_count": 0}

            # 获取在线设备信息
            ros_node_info = get_ros_node_info()
            # 获取主机节点信息
            host_node_info = get_host_node_info()
            # 获取Registry路径信息
            registry_info = get_registry_info()

            # 获取已加载的设备
            if lab_registry:
                devices = json.loads(json.dumps(lab_registry.obtain_registry_device_info(), ensure_ascii=False, cls=TypeEncoder))
                # 资源类型
                for resource_id, resource_info in lab_registry.resource_type_registry.items():
                    resources.append(
                        {
                            "id": resource_id,
                            "name": resource_info.get("name", "未命名"),
                            "file_path": resource_info.get("file_path", ""),
                        }
                    )

            # 获取导入的模块
            if msg_converter_manager:
                modules["names"] = msg_converter_manager.list_modules()
                all_classes = [i for i in msg_converter_manager.list_classes() if "." in i]
                modules["total_count"] = len(all_classes)
                modules["classes"] = all_classes

            # 使用模板渲染页面
            template = env.get_template("status.html")
            html = template.render(
                devices=devices,
                resources=resources,
                modules=modules,
                is_host_mode=BasicConfig.is_host_mode,
                registry_info=registry_info,
                ros_node_info=ros_node_info,
                host_node_info=host_node_info,
            )

            return html
        except Exception as e:
            error(f"生成状态页面时出错: {str(e)}")
            raise HTTPException(status_code=500, detail=f"Error generating status page: {str(e)}")

    @router.get("/open-folder", response_class=JSONResponse, summary="Open Local Folder")
    async def open_folder(path: str = "") -> Dict[str, str]:
        """
        打开本地文件夹

        Args:
            path: 要打开的文件夹路径

        Returns:
            JSONResponse: 操作结果

        Raises:
            HTTPException: 如果路径为空或不存在
        """
        if not path:
            return {"status": "error", "message": "Path is empty"}

        try:
            # 规范化路径
            norm_path = os.path.normpath(path)

            # 如果是文件路径，获取其目录
            if os.path.isfile(norm_path):
                norm_path = os.path.dirname(norm_path)

            # 检查路径是否存在
            if not os.path.exists(norm_path):
                return {"status": "error", "message": f"Path does not exist: {norm_path}"}

            # Windows
            if os.name == "nt":
                os.startfile(norm_path)
            # macOS
            elif sys.platform == "darwin":
                os.system(f'open "{norm_path}"')
            # Linux
            else:
                os.system(f'xdg-open "{norm_path}"')

            return {"status": "success", "message": f"Opened folder: {norm_path}"}
        except Exception as e:
            error(f"打开文件夹时出错: {str(e)}")
            return {"status": "error", "message": f"Failed to open folder: {str(e)}"}
