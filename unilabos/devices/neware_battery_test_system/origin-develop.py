#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
新华电池测试系统设备类
- 提供TCP通信接口查询电池通道状态
- 支持720个通道（devid 1-7, 8, 86）
- 兼容BTSAPI getchlstatus协议

设备特点：
- TCP连接: 默认127.0.0.1:502
- 通道映射: devid->subdevid->chlid 三级结构
- 状态类型: working/stop/finish/protect/pause/false/unknown
"""

import socket
import xml.etree.ElementTree as ET
import json
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

from pylabrobot.resources import ResourceHolder, Coordinate, create_ordered_items_2d

# ========================
# 基本通信与协议参数
# ========================
BTS_IP = "127.0.0.1"
BTS_PORT = 502
DEVTYPE = "27"
TIMEOUT = 20  # 秒
REQ_END = b"#\r\n"  # 常见实现以 "#\\r\\n" 作为报文结束

# ========================
# 状态与颜色映射（前端可直接使用）
# ========================
STATUS_SET = {"working", "stop", "finish", "protect", "pause", "false"}
STATUS_COLOR = {
    "working": "#22c55e",  # 绿
    "stop":    "#6b7280",  # 灰
    "finish":  "#3b82f6",  # 蓝
    "protect": "#ef4444",  # 红
    "pause":   "#f59e0b",  # 橙
    "false":   "#9ca3af",  # 不存在/无效
    "unknown": "#a855f7",  # 未知
}

ascii_lowercase = 'abcdefghijklmnopqrstuvwxyz'
ascii_uppercase = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ'
LETTERS = ascii_uppercase + ascii_lowercase

@dataclass(frozen=True)
class ChannelKey:
    devid: int
    subdevid: int
    chlid: int

@dataclass
class ChannelStatus:
    state: str           # working/stop/finish/protect/pause/false/unknown
    reservepause: int    # 0/1（是否预约暂停）


# ========================
# 设备主类
# ========================
class NewwareBatteryTestSystem:
    """
    新华电池测试系统设备类
    
    提供电池测试通道状态查询、控制等功能。
    支持720个通道的状态监控和数据导出。
    
    Attributes:
        ip (str): TCP服务器IP地址，默认127.0.0.1
        port (int): TCP端口，默认502
        devtype (str): 设备类型，默认"27"
        timeout (int): 通信超时时间（秒），默认20
    """

    def __init__(self, 
        ip: str = BTS_IP, 
        port: int = BTS_PORT, 
        machine_id: int = 1,
        devtype: str = DEVTYPE, 
        timeout: int = TIMEOUT,
        
        size_x: float = 500.0,
        size_y: float = 500.0,
        size_z: float = 2000.0,
    ):
        """
        初始化新华电池测试系统
        
        Args:
            ip: TCP服务器IP地址
            port: TCP端口
            devtype: 设备类型标识
            timeout: 通信超时时间（秒）
        """
        self.ip = ip
        self.port = port
        self.machine_id = machine_id
        self.devtype = devtype
        self.timeout = timeout
        self._last_status_update = None
        self._cached_status = {}
        
        self.station_resources = create_ordered_items_2d(
            BatteryTestPosition,
            num_items_x=8,
            num_items_y=5,
            dx=10,
            dy=10,
            dz=0,
            item_dx=65,
            item_dy=65
        )
        
        # 初始化通道映射
        self._channels = self._build_channel_map()
        
    @property
    def channel_status(self) -> Dict[int, Dict]:
        """
        获取所有通道状态（按设备ID分组）
        
        这个属性会执行实际的TCP查询并返回格式化的状态数据。
        结果按设备ID分组，包含统计信息和详细状态。
        
        Returns:
            Dict[int, Dict]: 按设备ID分组的通道状态
            {
                devid: {
                    "stats": {"working": n, "stop": n, ...},
                    "subunits": {
                        subdevid: {
                            chlid: {"state": "...", "reservepause": 0/1, "color": "#xxxxxx"}
                        }
                    }
                }
            }
        """
        status_map = self._query_all_channels()
        status_processed = {} if not status_map else self._group_by_devid(status_map)
        
        status_current_machine = status_processed.get(str(self.machine_id), {})
        
        for dev_id in range(5):
            status_row = status_current_machine["subunits"].get(str(dev_id+1), {})
            for subdev_id in range(8):

                r = self.station_resources[f"{LETTERS[dev_id]}{subdev_id+1}"]
                status_channel = status_row.get(str(subdev_id+1), {})
                # 处理每个通道的状态信息
                
                channel_state = {
                    "status": status_channel.get("state", "unknown"),
                    "color": status_channel.get("color", STATUS_COLOR["unknown"]),
                }
                r.load_state(channel_state)
        self._ros_node.update_resources(list(self.station_resources.values()))

        return status_current_machine["stats"]

    @property
    def connection_info(self) -> Dict[str, str]:
        """获取连接信息"""
        return {
            "ip": self.ip,
            "port": str(self.port),
            "devtype": self.devtype,
            "timeout": f"{self.timeout}s"
        }
    
    @property
    def total_channels(self) -> int:
        """获取总通道数"""
        return len(self._channels)
    
    def get_device_summary(self) -> Dict[str, int]:
        """
        获取设备级别的摘要统计
        
        Returns:
            各设备ID的通道数量统计
        """
        summary = {}
        for channel in self._channels:
            devid = channel.devid
            summary[devid] = summary.get(devid, 0) + 1
        return summary

    def export_status_json(self, filepath: str = "bts_status.json") -> bool:
        """
        导出当前状态到JSON文件
        
        Args:
            filepath: 输出文件路径
            
        Returns:
            是否导出成功
        """
        try:
            grouped_status = self.channel_status
            payload = {
                "timestamp": time.time(),
                "device_info": {
                    "ip": self.ip,
                    "port": self.port,
                    "devtype": self.devtype,
                    "total_channels": self.total_channels
                },
                "data": grouped_status,
                "color_mapping": STATUS_COLOR
            }
            
            with open(filepath, "w", encoding="utf-8") as f:
                json.dump(payload, f, ensure_ascii=False, indent=2)
            return True
        except Exception as e:
            print(f"导出JSON失败: {e}")
            return False

    def _build_channel_map(self) -> List[ChannelKey]:
        """构建全量通道映射（720个通道）"""
        channels = []
        
        # devid 1-7: subdevid 1-10, chlid 1-8
        for devid in range(1, 8):
            for sub in range(1, 11):
                for ch in range(1, 9):
                    channels.append(ChannelKey(devid, sub, ch))
        
        # devid 8: subdevid 11-20, chlid 1-8
        for sub in range(11, 21):
            for ch in range(1, 9):
                channels.append(ChannelKey(8, sub, ch))
        
        # devid 86: subdevid 1-10, chlid 1-8
        for sub in range(1, 11):
            for ch in range(1, 9):
                channels.append(ChannelKey(86, sub, ch))
                
        return channels

    def _query_all_channels(self) -> Dict[ChannelKey, ChannelStatus]:
        """执行TCP查询获取所有通道状态"""
        try:
            req_xml = self._build_getchlstatus_xml()
            
            with socket.create_connection((self.ip, self.port), timeout=self.timeout) as sock:
                sock.settimeout(self.timeout)
                sock.sendall(req_xml)
                response = self._recv_until(sock)
                
            return self._parse_getchlstatus_resp(response)
        except Exception as e:
            print(f"查询通道状态失败: {e}")
            return {}

    def _build_getchlstatus_xml(self) -> bytes:
        """构造getchlstatus请求XML"""
        lines = [
            '<?xml version="1.0" encoding="UTF-8" ?>',
            '<bts version="1.0">',
            '<cmd>getchlstatus</cmd>',
            f'<list count="{len(self._channels)}">'
        ]
        
        for c in self._channels:
            lines.append(
                f'<status ip="{self.ip}" devtype="{self.devtype}" '
                f'devid="{c.devid}" subdevid="{c.subdevid}" chlid="{c.chlid}">true</status>'
            )
        
        lines.extend(['</list>', '</bts>'])
        xml_text = "\n".join(lines)
        return xml_text.encode("utf-8") + REQ_END

    def _recv_until(self, sock: socket.socket, end_token: bytes = REQ_END, 
                   alt_close_tag: bytes = b"</bts>") -> bytes:
        """接收TCP响应数据"""
        buf = bytearray()
        while True:
            chunk = sock.recv(8192)
            if not chunk:
                break
            buf.extend(chunk)
            if end_token in buf:
                cut = buf.rfind(end_token)
                return bytes(buf[:cut])
            if alt_close_tag in buf:
                cut = buf.rfind(alt_close_tag) + len(alt_close_tag)
                return bytes(buf[:cut])
        return bytes(buf)

    def _parse_getchlstatus_resp(self, xml_bytes: bytes) -> Dict[ChannelKey, ChannelStatus]:
        """解析getchlstatus_resp响应XML"""
        mapping = {}
        
        try:
            xml_text = xml_bytes.decode("utf-8", errors="ignore").strip()
            root = ET.fromstring(xml_text)
            cmd = root.findtext("cmd", default="").strip()
            
            if cmd != "getchlstatus_resp":
                return mapping
                
            list_node = root.find("list")
            if list_node is None:
                return mapping
                
            for stat in list_node.findall("status"):
                try:
                    devid = int(stat.get("devid", "0"))
                    subdevid = int(stat.get("subdevid", "0"))
                    chlid = int(stat.get("chlid", "0"))
                    reservepause = int(stat.get("reservepause", "0"))
                except ValueError:
                    continue
                    
                state_raw = (stat.text or "").strip().lower()
                state = state_raw if state_raw in STATUS_SET else "unknown"
                
                key = ChannelKey(devid, subdevid, chlid)
                mapping[key] = ChannelStatus(state=state, reservepause=reservepause)
                
        except Exception as e:
            print(f"解析XML响应失败: {e}")
            
        return mapping

    def _group_by_devid(self, status_map: Dict[ChannelKey, ChannelStatus]) -> Dict[int, Dict]:
        """按设备ID分组状态数据"""
        result = {}
        
        for key, val in status_map.items():
            if key.devid not in result:
                result[key.devid] = {
                    "stats": {s: 0 for s in STATUS_SET | {"unknown"}},
                    "subunits": {}
                }
            
            dev = result[key.devid]
            dev["stats"][val.state] = dev["stats"].get(val.state, 0) + 1
            
            subunits = dev["subunits"]
            if key.subdevid not in subunits:
                subunits[key.subdevid] = {}
            
            subunits[key.subdevid][key.chlid] = {
                "state": val.state,
                "reservepause": val.reservepause,
                "color": STATUS_COLOR.get(val.state, STATUS_COLOR["unknown"]),
            }
            
        return result


class BatteryTestPositionState(TypedDict):
    voltage: float  # 电压 (V)
    current: float  # 电流 (A)
    time: float  # 时间 (s)
    
    status: str  # 通道状态
    color: str  # 状态对应颜色


class BatteryTestPosition(ResourceHolder):
    def __init__(
        self,
        name,
        size_x=60,
        size_y=60,
        size_z=60,
        rotation=None,
        category="resource_holder",
        model=None,
        child_location: Coordinate = Coordinate.zero(),
    ):
        super().__init__(name, size_x, size_y, size_z, rotation, category, model, child_location=child_location)
        self._unilabos_state: BatteryTestPositionState = BatteryTestPositionState()
    
    def load_state(self, state: Dict[str, Any]) -> None:
        """格式不变"""
        super().load_state(state)
        self._unilabos_state = state
    #序列化
    
    def serialize_state(self) -> Dict[str, Dict[str, Any]]:
        """格式不变"""
        data = super().serialize_state()
        data.update(self._unilabos_state)  # Container自身的信息，云端物料将保存这一data，本地也通过这里的data进行读写，当前类用来表示这个物料的长宽高大小的属性，而data（state用来表示物料的内容，细节等）
        return data



# ========================
# 示例和测试代码
# ========================
def main():
    """测试和演示设备类的使用"""
    print("=== 新华电池测试系统设备类演示 ===")
    
    # 创建设备实例
    bts = NewwareBatteryTestSystem()
    
    # 测试连接
    print(f"\n1. 连接测试:")
    print(f"   连接信息: {bts.connection_info}")
    if bts.test_connection():
        print("   ✓ TCP连接正常")
    else:
        print("   ✗ TCP连接失败")
        return
    
    # 获取设备摘要
    print(f"\n2. 设备摘要:")
    print(f"   总通道数: {bts.total_channels}")
    summary = bts.get_device_summary()
    for devid, count in summary.items():
        print(f"   设备ID {devid}: {count} 个通道")
    
    # 获取实时状态
    print(f"\n3. 获取通道状态:")
    try:
        bts.print_status_summary()
    except Exception as e:
        print(f"   获取状态失败: {e}")
    
    # 导出JSON
    print(f"\n4. 导出状态数据:")
    if bts.export_status_json("demo_status.json"):
        print("   ✓ 状态数据已导出到 demo_status.json")
    else:
        print("   ✗ 导出失败")


if __name__ == "__main__":
    main()