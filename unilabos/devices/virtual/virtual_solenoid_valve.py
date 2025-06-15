import time
from typing import Union


class VirtualSolenoidValve:
    """
    虚拟电磁阀门 - 简单的开关型阀门，只有开启和关闭两个状态
    """
    def __init__(self, port: str = "VIRTUAL", voltage: float = 12.0, response_time: float = 0.1):
        self.port = port
        self.voltage = voltage
        self.response_time = response_time
        
        # 状态属性
        self._status = "Idle"
        self._valve_state = "Closed"  # "Open" or "Closed"
        self._is_open = False

    @property
    def status(self) -> str:
        return self._status

    @property
    def valve_state(self) -> str:
        return self._valve_state

    @property
    def is_open(self) -> bool:
        return self._is_open

    def get_valve_position(self) -> str:
        """获取阀门位置状态"""
        return "OPEN" if self._is_open else "CLOSED"

    def set_valve_position(self, position: Union[str, bool]):
        """
        设置阀门位置
        
        Args:
            position: "OPEN"/"CLOSED" 或 True/False
        """
        self._status = "Busy"
        
        # 模拟阀门响应时间
        time.sleep(self.response_time)
        
        if isinstance(position, str):
            target_open = position.upper() == "OPEN"
        elif isinstance(position, bool):
            target_open = position
        else:
            self._status = "Error"
            return "Error: Invalid position"
        
        self._is_open = target_open
        self._valve_state = "Open" if target_open else "Closed"
        self._status = "Idle"
        
        return f"Valve {'opened' if target_open else 'closed'}"

    def open(self):
        """打开电磁阀"""
        self._status = "Busy"
        time.sleep(self.response_time)
        
        self._is_open = True
        self._valve_state = "Open"
        self._status = "Idle"
        
        return "Valve opened"

    def close(self):
        """关闭电磁阀"""
        self._status = "Busy"
        time.sleep(self.response_time)
        
        self._is_open = False
        self._valve_state = "Closed"
        self._status = "Idle"
        
        return "Valve closed"

    def set_state(self, command: Union[bool, str]):
        """
        设置阀门状态 - 兼容 SendCmd 类型
    
        Args:
            command: True/False 或 "open"/"close"
        """
        if isinstance(command, bool):
            return self.open() if command else self.close()
        elif isinstance(command, str):
            if command.lower() in ["open", "on", "true", "1"]:
                return self.open()
            elif command.lower() in ["close", "closed", "off", "false", "0"]:
                return self.close()
            else:
                self._status = "Error"
                return "Error: Invalid command"
        else:
            self._status = "Error"
            return "Error: Invalid command type"

    def toggle(self):
        """切换阀门状态"""
        if self._is_open:
            return self.close()
        else:
            return self.open()

    def is_closed(self) -> bool:
        """检查阀门是否关闭"""
        return not self._is_open

    def get_state(self) -> dict:
        """获取阀门完整状态"""
        return {
            "port": self.port,
            "voltage": self.voltage,
            "response_time": self.response_time,
            "is_open": self._is_open,
            "valve_state": self._valve_state,
            "status": self._status,
            "position": self.get_valve_position()
        }

    def reset(self):
        """重置阀门到关闭状态"""
        return self.close()

    def test_cycle(self, cycles: int = 3, delay: float = 1.0):
        """
        测试阀门开关循环
        
        Args:
            cycles: 循环次数
            delay: 每次开关间隔时间(秒)
        """
        results = []
        for i in range(cycles):
            # 打开
            result_open = self.open()
            results.append(f"Cycle {i+1} - Open: {result_open}")
            time.sleep(delay)
            
            # 关闭
            result_close = self.close()
            results.append(f"Cycle {i+1} - Close: {result_close}")
            time.sleep(delay)
        
        return results