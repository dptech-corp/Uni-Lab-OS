import time
from typing import Union, Dict, Optional


class VirtualMultiwayValve:
    """
    虚拟九通阀门 - 0号位连接transfer pump，1-8号位连接其他设备
    """
    def __init__(self, port: str = "VIRTUAL", positions: int = 8):
        self.port = port
        self.max_positions = positions  # 1-8号位
        self.total_positions = positions + 1  # 0-8号位，共9个位置
        
        # 状态属性
        self._status = "Idle"
        self._valve_state = "Ready"
        self._current_position = 0  # 默认在0号位（transfer pump位置）
        self._target_position = 0
        
        # 位置映射说明
        self.position_map = {
            0: "transfer_pump",  # 0号位连接转移泵
            1: "port_1",         # 1号位
            2: "port_2",         # 2号位
            3: "port_3",         # 3号位
            4: "port_4",         # 4号位
            5: "port_5",         # 5号位
            6: "port_6",         # 6号位
            7: "port_7",         # 7号位
            8: "port_8"          # 8号位
        }

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

    def get_current_port(self) -> str:
        """获取当前连接的端口名称"""
        return self.position_map.get(self._current_position, "unknown")

    def set_position(self, command: Union[int, str]):
        """
        设置阀门位置 - 支持0-8位置
        
        Args:
            command: 目标位置 (0-8) 或位置字符串
                    0: transfer pump位置
                    1-8: 其他设备位置
        """
        try:
            # 如果是字符串形式的位置，先转换为数字
            if isinstance(command, str):
                pos = int(command)
            else:
                pos = int(command)
                
            if pos < 0 or pos > self.max_positions:
                raise ValueError(f"Position must be between 0 and {self.max_positions}")
            
            self._status = "Busy"
            self._valve_state = "Moving"
            self._target_position = pos
            
            # 模拟阀门切换时间
            switch_time = abs(self._current_position - pos) * 0.5  # 每个位置0.5秒
            time.sleep(switch_time)
            
            self._current_position = pos
            self._status = "Idle"
            self._valve_state = "Ready"
            
            current_port = self.get_current_port()
            return f"Position set to {pos} ({current_port})"
            
        except ValueError as e:
            self._status = "Error"
            self._valve_state = "Error"
            return f"Error: {str(e)}"

    def set_to_pump_position(self):
        """切换到transfer pump位置（0号位）"""
        return self.set_position(0)

    def set_to_port(self, port_number: int):
        """
        切换到指定端口位置
        
        Args:
            port_number: 端口号 (1-8)
        """
        if port_number < 1 or port_number > self.max_positions:
            raise ValueError(f"Port number must be between 1 and {self.max_positions}")
        return self.set_position(port_number)

    def open(self):
        """打开阀门 - 设置到transfer pump位置（0号位）"""
        return self.set_to_pump_position()

    def close(self):
        """关闭阀门 - 对于多通阀门，设置到一个"关闭"状态"""
        self._status = "Busy"
        self._valve_state = "Closing"
        time.sleep(0.5)
        
        # 可以选择保持当前位置或设置特殊关闭状态
        self._status = "Idle"
        self._valve_state = "Closed"
        
        return f"Valve closed at position {self._current_position}"

    def get_valve_position(self) -> int:
        """获取阀门位置 - 兼容性方法"""
        return self._current_position

    def is_at_position(self, position: int) -> bool:
        """检查是否在指定位置"""
        return self._current_position == position

    def is_at_pump_position(self) -> bool:
        """检查是否在transfer pump位置"""
        return self._current_position == 0

    def is_at_port(self, port_number: int) -> bool:
        """检查是否在指定端口位置"""
        return self._current_position == port_number

    def get_available_positions(self) -> list:
        """获取可用位置列表"""
        return list(range(0, self.max_positions + 1))

    def get_available_ports(self) -> Dict[int, str]:
        """获取可用端口映射"""
        return self.position_map.copy()

    def reset(self):
        """重置阀门到transfer pump位置（0号位）"""
        return self.set_position(0)

    def switch_between_pump_and_port(self, port_number: int):
        """
        在transfer pump位置和指定端口之间切换
        
        Args:
            port_number: 目标端口号 (1-8)
        """
        if self._current_position == 0:
            # 当前在pump位置，切换到指定端口
            return self.set_to_port(port_number)
        else:
            # 当前在某个端口，切换到pump位置
            return self.set_to_pump_position()

    def get_flow_path(self) -> str:
        """获取当前流路路径描述"""
        current_port = self.get_current_port()
        if self._current_position == 0:
            return f"Transfer pump connected (position {self._current_position})"
        else:
            return f"Port {self._current_position} connected ({current_port})"

    def get_info(self) -> dict:
        """获取阀门详细信息"""
        return {
            "port": self.port,
            "max_positions": self.max_positions,
            "total_positions": self.total_positions,
            "current_position": self._current_position,
            "current_port": self.get_current_port(),
            "target_position": self._target_position,
            "status": self._status,
            "valve_state": self._valve_state,
            "flow_path": self.get_flow_path(),
            "position_map": self.position_map
        }

    def __str__(self):
        return f"VirtualMultiwayValve(Position: {self._current_position}/{self.max_positions}, Port: {self.get_current_port()}, Status: {self._status})"

    def set_valve_position(self, command: Union[int, str]):
        """
        设置阀门位置 - 兼容pump_protocol调用
        这是set_position的别名方法，用于兼容pump_protocol.py
        
        Args:
            command: 目标位置 (0-8) 或位置字符串
        """
        return self.set_position(command)


# 使用示例
if __name__ == "__main__":
    valve = VirtualMultiwayValve()
    
    print("=== 虚拟九通阀门测试 ===")
    print(f"初始状态: {valve}")
    print(f"当前流路: {valve.get_flow_path()}")
    
    # 切换到试剂瓶1（1号位）
    print(f"\n切换到1号位: {valve.set_position(1)}")
    print(f"当前状态: {valve}")
    
    # 切换到transfer pump位置（0号位）
    print(f"\n切换到pump位置: {valve.set_to_pump_position()}")
    print(f"当前状态: {valve}")
    
    # 切换到试剂瓶2（2号位）
    print(f"\n切换到2号位: {valve.set_to_port(2)}")
    print(f"当前状态: {valve}")
    
    # 显示所有可用位置
    print(f"\n可用位置: {valve.get_available_positions()}")
    print(f"端口映射: {valve.get_available_ports()}")
    
    # 获取详细信息
    print(f"\n详细信息: {valve.get_info()}")