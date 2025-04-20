"""
Web UI 模块

提供了UniLab系统的Web界面功能
"""

from unilabos.app.web.pages import setup_web_pages
from unilabos.app.web.server import setup_server, start_server
from unilabos.app.web.client import http_client
from unilabos.app.web.api import setup_api_routes

__all__ = [
    "setup_web_pages",  # 设置Web页面
    "setup_server",  # 设置服务器
    "start_server",  # 启动服务器
    "http_client",  # HTTP客户端
    "setup_api_routes",  # 设置API路由
]
