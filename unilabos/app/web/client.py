"""
HTTP客户端模块

提供与远程服务器通信的客户端功能，只有host需要用
"""

from typing import List, Dict, Any, Optional

import requests
from unilabos.utils.log import info
from unilabos.config.config import MQConfig, HTTPConfig


class HTTPClient:
    """HTTP客户端，用于与远程服务器通信"""

    def __init__(self, remote_addr: Optional[str] = None, auth: Optional[str] = None) -> None:
        """
        初始化HTTP客户端

        Args:
            remote_addr: 远程服务器地址，如果不提供则从配置中获取
            auth: 授权信息
        """
        self.remote_addr = remote_addr or HTTPConfig.remote_addr
        if auth is not None:
            self.auth = auth
        else:
            self.auth = MQConfig.lab_id
        info(f"HTTPClient 初始化完成: remote_addr={self.remote_addr}")

    def resource_add(self, resources: List[Dict[str, Any]]) -> requests.Response:
        """
        添加资源

        Args:
            resources: 要添加的资源列表

        Returns:
            Response: API响应对象
        """
        response = requests.post(
            f"{self.remote_addr}/lab/resource/",
            json=resources,
            headers={"Authorization": f"lab {self.auth}"},
            timeout=5,
        )
        return response

    def resource_get(self, id: str, with_children: bool = False) -> Dict[str, Any]:
        """
        获取资源

        Args:
            id: 资源ID
            with_children: 是否包含子资源

        Returns:
            Dict: 返回的资源数据
        """
        response = requests.get(
            f"{self.remote_addr}/lab/resource/",
            params={"id": id, "with_children": with_children},
            headers={"Authorization": f"lab {self.auth}"},
            timeout=5,
        )
        return response.json()

    def resource_del(self, id: str) -> requests.Response:
        """
        删除资源

        Args:
            id: 要删除的资源ID

        Returns:
            Response: API响应对象
        """
        response = requests.delete(
            f"{self.remote_addr}/lab/resource/batch_delete/",
            params={"id": id},
            headers={"Authorization": f"lab {self.auth}"},
            timeout=5,
        )
        return response

    def resource_update(self, resources: List[Dict[str, Any]]) -> requests.Response:
        """
        更新资源

        Args:
            resources: 要更新的资源列表

        Returns:
            Response: API响应对象
        """
        response = requests.patch(
            f"{self.remote_addr}/lab/resource/batch_update/",
            json=resources,
            headers={"Authorization": f"lab {self.auth}"},
            timeout=5,
        )
        return response


# 创建默认客户端实例
http_client = HTTPClient()
