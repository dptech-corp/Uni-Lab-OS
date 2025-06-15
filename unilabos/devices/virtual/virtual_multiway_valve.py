import time
from typing import Union


class VirtualMultiwayValve:
    """
    虚拟八通阀门 - 可将一个输入切换到8个输出中的任意一个
    """
    def __init__(self, port: str = "VIRTUAL", positions: int = 8):
        self.port = port
        self.max_positions = positions
        
        # 状态属性
        self._status = "Idle"
        self._valve_state = "Ready"
        self._current_position = 1
        self._target_position = 1

    @property
    def status(self) -> str:
        return self._status

    @property
    def valve_state(self) -> str:
        return self._valve_state

    @property
    def current_position(self) -> int:
        return self._current_position

    @property
    def target_position(self) -> int:
        return self._target_position

    def get_current_position(self) -> int:
        """获取当前阀门位置"""
        return self._current_position

    def set_position(self, command: Union[int, str]):
        """
        设置阀门位置 - 兼容 SendCmd 类型
        
        Args:
            command: 目标位置 (1-8) 或位置字符串
        """
        try:
            # 如果是字符串形式的位置，先转换为数字
            if isinstance(command, str):
                pos = int(command)
            else:
                pos = int(command)
                
            if pos < 1 or pos > self.max_positions:
                raise ValueError(f"Position must be between 1 and {self.max_positions}")
            
            self._status = "Busy"
            self._valve_state = "Moving"
            self._target_position = pos
            
            # 模拟阀门切换时间
            switch_time = abs(self._current_position - pos) * 0.5  # 每个位置0.5秒
            time.sleep(switch_time)
            
            self._current_position = pos
            self._status = "Idle"
            self._valve_state = "Ready"
            
            return f"Position set to {pos}"
            
        except ValueError as e:
            self._status = "Error"
            self._valve_state = "Error"
            return f"Error: {str(e)}"

    def open(self):
        """打开阀门 - 对于多通阀门，相当于设置到位置1"""
        return self.set_position(1)

    def close(self):
        """关闭阀门 - 对于多通阀门，相当于设置到关闭位置"""
        self._status = "Busy"
        self._valve_state = "Closing"
        time.sleep(0.5)
        
        self._current_position = 0  # 0表示关闭状态
        self._status = "Idle"
        self._valve_state = "Closed"
        
        return "Valve closed"

    def get_valve_position(self) -> int:
        """获取阀门位置 - 兼容性方法"""
        return self._current_position

    def is_at_position(self, position: int) -> bool:
        """检查是否在指定位置"""
        return self._current_position == position

    def get_available_positions(self) -> list:
        """获取可用位置列表"""
        return list(range(1, self.max_positions + 1))

    def reset(self):
        """重置阀门到初始位置"""
        return self.set_position(1)

    def get_info(self) -> dict:
        """获取阀门信息"""
        return {
            "port": self.port,
            "max_positions": self.max_positions,
            "current_position": self._current_position,
            "target_position": self._target_position,
            "status": self._status,
            "valve_state": self._valve_state
        }