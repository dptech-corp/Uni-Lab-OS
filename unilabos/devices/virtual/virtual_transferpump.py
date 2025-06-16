import asyncio
import time
from enum import Enum
from typing import Union, Optional
import logging


class VirtualPumpMode(Enum):
    Normal = 0
    AccuratePos = 1
    AccuratePosVel = 2


class VirtualPump:
    """虚拟泵类 - 模拟泵的基本功能，无需实际硬件"""
    
    def __init__(self, device_id: str = None, max_volume: float = 25.0, mode: VirtualPumpMode = VirtualPumpMode.Normal, transfer_rate=0):
        self.device_id = device_id or "virtual_pump"
        self.max_volume = max_volume
        self._transfer_rate = transfer_rate
        self.mode = mode
        
        # 状态变量
        self._status = "Idle"
        self._position = 0.0  # 当前柱塞位置 (ml)
        self._max_velocity = 5.0  # 默认最大速度 (ml/s)
        self._current_volume = 0.0  # 当前注射器中的体积

        self.logger = logging.getLogger(f"VirtualPump.{self.device_id}")
        
    async def initialize(self) -> bool:
        """初始化虚拟泵"""
        self.logger.info(f"Initializing virtual pump {self.device_id}")
        self._status = "Idle"
        self._position = 0.0
        self._current_volume = 0.0
        return True
    
    async def cleanup(self) -> bool:
        """清理虚拟泵"""
        self.logger.info(f"Cleaning up virtual pump {self.device_id}")
        self._status = "Idle"
        return True
    
    # 基本属性
    @property
    def status(self) -> str:
        return self._status
    
    @property
    def position(self) -> float:
        """当前柱塞位置 (ml)"""
        return self._position
    
    @property
    def current_volume(self) -> float:
        """当前注射器中的体积 (ml)"""
        return self._current_volume
    
    @property
    def max_velocity(self) -> float:
        return self._max_velocity
    
    @property
    def transfer_rate(self) -> float:
        return self._transfer_rate

    def set_max_velocity(self, velocity: float):
        """设置最大速度 (ml/s)"""
        self._max_velocity = max(0.1, min(50.0, velocity))  # 限制在合理范围内
        self.logger.info(f"Set max velocity to {self._max_velocity} ml/s")
    
    def get_status(self) -> str:
        """获取泵状态"""
        return self._status
    
    async def _simulate_operation(self, duration: float):
        """模拟操作延时"""
        self._status = "Busy"
        await asyncio.sleep(duration)
        self._status = "Idle"
    
    def _calculate_duration(self, volume: float, velocity: float = None) -> float:
        """计算操作持续时间"""
        if velocity is None:
            velocity = self._max_velocity
        return abs(volume) / velocity
    
    # 基本泵操作
    async def set_position(self, position: float, velocity: float = None):
        """
        移动到绝对位置
        
        Args:
            position (float): 目标位置 (ml)
            velocity (float): 移动速度 (ml/s)
        """
        position = max(0, min(self.max_volume, position))  # 限制在有效范围内
        
        volume_to_move = abs(position - self._position)
        duration = self._calculate_duration(volume_to_move, velocity)
        
        self.logger.info(f"Moving to position {position} ml (current: {self._position} ml)")
        
        # 模拟移动过程
        await self._simulate_operation(duration)
        
        self._position = position
        self._current_volume = position  # 假设位置等于体积
        
        self.logger.info(f"Reached position {self._position} ml")
    
    async def pull_plunger(self, volume: float, velocity: float = None):
        """
        拉取柱塞（吸液）
        
        Args:
            volume (float): 要拉取的体积 (ml)
            velocity (float): 拉取速度 (ml/s)
        """
        new_position = min(self.max_volume, self._position + volume)
        actual_volume = new_position - self._position
        
        if actual_volume <= 0:
            self.logger.warning("Cannot pull - already at maximum volume")
            return
        
        duration = self._calculate_duration(actual_volume, velocity)
        
        self.logger.info(f"Pulling {actual_volume} ml (from {self._position} to {new_position})")
        
        await self._simulate_operation(duration)
        
        self._position = new_position
        self._current_volume = new_position
        
        self.logger.info(f"Pulled {actual_volume} ml, current volume: {self._current_volume} ml")
    
    async def push_plunger(self, volume: float, velocity: float = None):
        """
        推出柱塞（排液）
        
        Args:
            volume (float): 要推出的体积 (ml)
            velocity (float): 推出速度 (ml/s)
        """
        new_position = max(0, self._position - volume)
        actual_volume = self._position - new_position
        
        if actual_volume <= 0:
            self.logger.warning("Cannot push - already at minimum volume")
            return
        
        duration = self._calculate_duration(actual_volume, velocity)
        
        self.logger.info(f"Pushing {actual_volume} ml (from {self._position} to {new_position})")
        
        await self._simulate_operation(duration)
        
        self._position = new_position
        self._current_volume = new_position
        
        self.logger.info(f"Pushed {actual_volume} ml, current volume: {self._current_volume} ml")
    
    # 便捷操作方法
    async def aspirate(self, volume: float, velocity: float = None):
        """
        吸液操作
        
        Args:
            volume (float): 吸液体积 (ml)
            velocity (float): 吸液速度 (ml/s)
        """
        await self.pull_plunger(volume, velocity)
    
    async def dispense(self, volume: float, velocity: float = None):
        """
        排液操作
        
        Args:
            volume (float): 排液体积 (ml)
            velocity (float): 排液速度 (ml/s)
        """
        await self.push_plunger(volume, velocity)
    
    async def transfer(self, volume: float, aspirate_velocity: float = None, dispense_velocity: float = None):
        """
        转移操作（先吸后排）
        
        Args:
            volume (float): 转移体积 (ml)
            aspirate_velocity (float): 吸液速度 (ml/s)
            dispense_velocity (float): 排液速度 (ml/s)
        """
        # 吸液
        await self.aspirate(volume, aspirate_velocity)
        
        # 短暂停顿
        await asyncio.sleep(0.1)
        
        # 排液
        await self.dispense(volume, dispense_velocity)
    
    async def empty_syringe(self, velocity: float = None):
        """清空注射器"""
        await self.set_position(0, velocity)
    
    async def fill_syringe(self, velocity: float = None):
        """充满注射器"""
        await self.set_position(self.max_volume, velocity)
    
    async def stop_operation(self):
        """停止当前操作"""
        self._status = "Idle"
        self.logger.info("Operation stopped")
    
    # 状态查询方法
    def get_position(self) -> float:
        """获取当前位置"""
        return self._position
    
    def get_current_volume(self) -> float:
        """获取当前体积"""
        return self._current_volume
    
    def get_remaining_capacity(self) -> float:
        """获取剩余容量"""
        return self.max_volume - self._current_volume
    
    def is_empty(self) -> bool:
        """检查是否为空"""
        return self._current_volume <= 0.01  # 允许小量误差
    
    def is_full(self) -> bool:
        """检查是否已满"""
        return self._current_volume >= (self.max_volume - 0.01)  # 允许小量误差
    
    # 调试和状态信息
    def get_pump_info(self) -> dict:
        """获取泵的详细信息"""
        return {
            "device_id": self.device_id,
            "status": self._status,
            "position": self._position,
            "current_volume": self._current_volume,
            "max_volume": self.max_volume,
            "max_velocity": self._max_velocity,
            "mode": self.mode.name,
            "is_empty": self.is_empty(),
            "is_full": self.is_full(),
            "remaining_capacity": self.get_remaining_capacity()
        }
    
    def __str__(self):
        return f"VirtualPump({self.device_id}: {self._current_volume:.2f}/{self.max_volume} ml, {self._status})"
    
    def __repr__(self):
        return self.__str__()


# 使用示例
async def demo():
    """虚拟泵使用示例"""
    pump = VirtualPump("demo_pump", max_volume=50.0)
    
    await pump.initialize()
    
    print(f"Initial state: {pump}")
    
    # 吸液测试
    await pump.aspirate(10.0, velocity=2.0)
    print(f"After aspirating 10ml: {pump}")
    
    # 排液测试
    await pump.dispense(5.0, velocity=3.0)
    print(f"After dispensing 5ml: {pump}")
    
    # 转移测试
    await pump.transfer(3.0)
    print(f"After transfer 3ml: {pump}")
    
    # 清空测试
    await pump.empty_syringe()
    print(f"After emptying: {pump}")
    
    print("\nPump info:", pump.get_pump_info())


if __name__ == "__main__":
    asyncio.run(demo())