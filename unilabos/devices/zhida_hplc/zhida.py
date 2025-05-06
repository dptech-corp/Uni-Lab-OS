#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import json
import base64
import argparse
import sys
import time


class ZhidaClient:
    def __init__(self, host='192.168.1.47', port=5792, timeout=10.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.sock = None

    def connect(self):
        """建立 TCP 连接，并设置超时用于后续 recv/send。"""
        self.sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
        # 确保后续 recv/send 都会在 timeout 秒后抛 socket.timeout
        self.sock.settimeout(self.timeout)

    def close(self):
        """关闭连接。"""
        if self.sock:
            self.sock.close()
            self.sock = None

    def _send_command(self, cmd: dict) -> dict:
        """
        发送一条命令，接收 raw bytes，直到能成功 json.loads。
        """
        if not self.sock:
            raise ConnectionError("Not connected")

        # 1) 发送 JSON 命令
        payload = json.dumps(cmd, ensure_ascii=False).encode('utf-8')
        # 如果服务端需要换行分隔，也可以加上： payload += b'\n'
        self.sock.sendall(payload)

        # 2) 循环 recv，直到能成功解析完整 JSON
        buffer = bytearray()
        start = time.time()
        while True:
            try:
                chunk = self.sock.recv(4096)
                if not chunk:
                    # 对端关闭
                    break
                buffer.extend(chunk)
                # 尝试解码、解析
                text = buffer.decode('utf-8', errors='strict')
                try:
                    return json.loads(text)
                except json.JSONDecodeError:
                    # 继续 recv
                    pass
            except socket.timeout:
                raise TimeoutError("recv() timed out after {:.1f}s".format(self.timeout))
            # 可选：防止死循环，整个循环时长超过 2×timeout 就报错
            if time.time() - start > self.timeout * 2:
                raise TimeoutError("No complete JSON received after {:.1f}s".format(time.time() - start))

        raise ConnectionError("Connection closed before JSON could be parsed")

# @property
# def xxx() -> 类型:
#     return xxxxxx

# def send_command(self, ):
#     self.xxx = dict[xxx]

# 示例响应回复：
# {
#   "result": "RUN",
#   "message": "AcqTime:3.321049min Vial:1"
# }

    @property
    def status(self) -> dict:
        return self._send_command({"command": "getstatus"})["result"]

    # def get_status(self) -> dict:
    #     print(self._send_command({"command": "getstatus"}))
    #     return self._send_command({"command": "getstatus"})

    def get_methods(self) -> dict:
        return self._send_command({"command": "getmethods"})

    def start(self, text) -> dict:
        b64 = base64.b64encode(text.encode('utf-8')).decode('ascii')
        return self._send_command({"command": "start", "message": b64})

    def abort(self) -> dict:
        return self._send_command({"command": "abort"})

"""
a,b,c
1,2,4
2,4,5
"""

client = ZhidaClient()
# 连接
client.connect()
# 获取状态
print(client.status)


# 命令格式：python zhida.py <subcommand> [options]
