import asyncio
import time
from enum import Enum
from typing import Union, Optional
import logging


class VirtualPumpMode(Enum):
    Normal = 0
    AccuratePos = 1
    AccuratePosVel = 2


class VirtualTransferPump:
    """虚拟转移泵类 - 模拟泵的基本功能，无需实际硬件 🚰"""
    
    def __init__(self, device_id: str = None, config: dict = None, **kwargs):
        """
        初始化虚拟转移泵
        
        Args:
            device_id: 设备ID
            config: 配置字典，包含max_volume, port等参数
            **kwargs: 其他参数，确保兼容性
        """
        self.device_id = device_id or "virtual_transfer_pump"
        
        # 从config或kwargs中获取参数，确保类型正确
        if config:
            self.max_volume = float(config.get('max_volume', 25.0))
            self.port = config.get('port', 'VIRTUAL')
        else:
            self.max_volume = float(kwargs.get('max_volume', 25.0))
            self.port = kwargs.get('port', 'VIRTUAL')
        
        self._transfer_rate = float(kwargs.get('transfer_rate', 0))
        self.mode = kwargs.get('mode', VirtualPumpMode.Normal)
        
        # 状态变量 - 确保都是正确类型
        self._status = "Idle"
        self._position = 0.0  # float
        self._max_velocity = 5.0  # float 
        self._current_volume = 0.0  # float

        # 🚀 新增：快速模式设置 - 大幅缩短执行时间
        self._fast_mode = True  # 是否启用快速模式
        self._fast_move_time = 1.0  # 快速移动时间（秒）
        self._fast_dispense_time = 1.0  # 快速喷射时间（秒）

        self.logger = logging.getLogger(f"VirtualTransferPump.{self.device_id}")
        
        print(f"🚰 === 虚拟转移泵 {self.device_id} 已创建 === ✨")
        print(f"💨 快速模式: {'启用' if self._fast_mode else '禁用'} | 移动时间: {self._fast_move_time}s | 喷射时间: {self._fast_dispense_time}s")
        print(f"📊 最大容量: {self.max_volume}mL | 端口: {self.port}")
    
    async def initialize(self) -> bool:
        """初始化虚拟泵 🚀"""
        self.logger.info(f"🔧 初始化虚拟转移泵 {self.device_id} ✨")
        self._status = "Idle"
        self._position = 0.0
        self._current_volume = 0.0
        self.logger.info(f"✅ 转移泵 {self.device_id} 初始化完成 🚰")
        return True
    
    async def cleanup(self) -> bool:
        """清理虚拟泵 🧹"""
        self.logger.info(f"🧹 清理虚拟转移泵 {self.device_id} 🔚")
        self._status = "Idle"
        self.logger.info(f"✅ 转移泵 {self.device_id} 清理完成 💤")
        return True
    
    # 基本属性
    @property
    def status(self) -> str:
        return self._status
    
    @property
    def position(self) -> float:
        """当前柱塞位置 (ml) 📍"""
        return self._position
    
    @property
    def current_volume(self) -> float:
        """当前注射器中的体积 (ml) 💧"""
        return self._current_volume
    
    @property
    def max_velocity(self) -> float:
        return self._max_velocity
    
    @property
    def transfer_rate(self) -> float:
        return self._transfer_rate

    def set_max_velocity(self, velocity: float):
        """设置最大速度 (ml/s) 🌊"""
        self._max_velocity = max(0.1, min(50.0, velocity))  # 限制在合理范围内
        self.logger.info(f"🌊 设置最大速度为 {self._max_velocity} mL/s")
    
    def get_status(self) -> str:
        """获取泵状态 📋"""
        return self._status
    
    async def _simulate_operation(self, duration: float):
        """模拟操作延时 ⏱️"""
        self._status = "Busy"
        await asyncio.sleep(duration)
        self._status = "Idle"
    
    def _calculate_duration(self, volume: float, velocity: float = None) -> float:
        """
        计算操作持续时间 ⏰
        🚀 快速模式：保留计算逻辑用于日志显示，但实际使用固定的快速时间
        """
        if velocity is None:
            velocity = self._max_velocity
        
        # 📊 计算理论时间（用于日志显示）
        theoretical_duration = abs(volume) / velocity
        
        # 🚀 如果启用快速模式，使用固定的快速时间
        if self._fast_mode:
            # 根据操作类型选择快速时间
            if abs(volume) > 0.1:  # 大于0.1mL的操作
                actual_duration = self._fast_move_time
            else:  # 很小的操作
                actual_duration = 0.5
            
            self.logger.debug(f"⚡ 快速模式: 理论时间 {theoretical_duration:.2f}s → 实际时间 {actual_duration:.2f}s")
            return actual_duration
        else:
            # 正常模式使用理论时间
            return theoretical_duration
    
    def _calculate_display_duration(self, volume: float, velocity: float = None) -> float:
        """
        计算显示用的持续时间（用于日志） 📊
        这个函数返回理论计算时间，用于日志显示
        """
        if velocity is None:
            velocity = self._max_velocity
        return abs(volume) / velocity
    
    # 新的set_position方法 - 专门用于SetPumpPosition动作
    async def set_position(self, position: float, max_velocity: float = None):
        """
        移动到绝对位置 - 专门用于SetPumpPosition动作 🎯
        
        Args:
            position (float): 目标位置 (ml)
            max_velocity (float): 移动速度 (ml/s)
        
        Returns:
            dict: 符合SetPumpPosition.action定义的结果
        """
        try:
            # 验证并转换参数
            target_position = float(position)
            velocity = float(max_velocity) if max_velocity is not None else self._max_velocity
            
            # 限制位置在有效范围内
            target_position = max(0.0, min(float(self.max_volume), target_position))
            
            # 计算移动距离
            volume_to_move = abs(target_position - self._position)
            
            # 📊 计算显示用的时间（用于日志）
            display_duration = self._calculate_display_duration(volume_to_move, velocity)
            
            # ⚡ 计算实际执行时间（快速模式）
            actual_duration = self._calculate_duration(volume_to_move, velocity)
            
            # 🎯 确定操作类型和emoji
            if target_position > self._position:
                operation_type = "吸液"
                operation_emoji = "📥"
            elif target_position < self._position:
                operation_type = "排液"
                operation_emoji = "📤"
            else:
                operation_type = "保持"
                operation_emoji = "📍"
            
            self.logger.info(f"🎯 SET_POSITION: {operation_type} {operation_emoji}")
            self.logger.info(f"  📍 位置: {self._position:.2f}mL → {target_position:.2f}mL (移动 {volume_to_move:.2f}mL)")
            self.logger.info(f"  🌊 速度: {velocity:.2f} mL/s")
            self.logger.info(f"  ⏰ 预计时间: {display_duration:.2f}s")
            
            if self._fast_mode:
                self.logger.info(f"  ⚡ 快速模式: 实际用时 {actual_duration:.2f}s")
            
            # 🚀 模拟移动过程
            if volume_to_move > 0.01:  # 只有当移动距离足够大时才显示进度
                start_position = self._position
                steps = 5 if actual_duration > 0.5 else 2  # 根据实际时间调整步数
                step_duration = actual_duration / steps
                
                self.logger.info(f"🚀 开始{operation_type}... {operation_emoji}")
                
                for i in range(steps + 1):
                    # 计算当前位置和进度
                    progress = (i / steps) * 100 if steps > 0 else 100
                    current_pos = start_position + (target_position - start_position) * (i / steps) if steps > 0 else target_position
                    
                    # 更新状态
                    if i < steps:
                        self._status = f"{operation_type}中"
                        status_emoji = "🔄"
                    else:
                        self._status = "Idle"
                        status_emoji = "✅"
                    
                    self._position = current_pos
                    self._current_volume = current_pos
                    
                    # 显示进度（每25%或最后一步）
                    if i == 0:
                        self.logger.debug(f"  🔄 {operation_type}开始: {progress:.0f}%")
                    elif progress >= 50 and i == steps // 2:
                        self.logger.debug(f"  🔄 {operation_type}进度: {progress:.0f}%")
                    elif i == steps:
                        self.logger.info(f"  ✅ {operation_type}完成: {progress:.0f}% | 当前位置: {current_pos:.2f}mL")
                    
                    # 等待一小步时间
                    if i < steps and step_duration > 0:
                        await asyncio.sleep(step_duration)
            else:
                # 移动距离很小，直接完成
                self._position = target_position
                self._current_volume = target_position
                self.logger.info(f"  📍 微调完成: {target_position:.2f}mL")
        
            # 确保最终位置准确
            self._position = target_position
            self._current_volume = target_position
            self._status = "Idle"
            
            # 📊 最终状态日志
            if volume_to_move > 0.01:
                self.logger.info(f"🎉 SET_POSITION 完成! 📍 最终位置: {self._position:.2f}mL | 💧 当前体积: {self._current_volume:.2f}mL")
            
            # 返回符合action定义的结果
            return {
                "success": True,
                "message": f"✅ 成功移动到位置 {self._position:.2f}mL ({operation_type})",
                "final_position": self._position,
                "final_volume": self._current_volume,
                "operation_type": operation_type
            }
            
        except Exception as e:
            error_msg = f"❌ 设置位置失败: {str(e)}"
            self.logger.error(error_msg)
            return {
                "success": False,
                "message": error_msg,
                "final_position": self._position,
                "final_volume": self._current_volume
            }
    
    # 其他泵操作方法
    async def pull_plunger(self, volume: float, velocity: float = None):
        """
        拉取柱塞（吸液） 📥
        
        Args:
            volume (float): 要拉取的体积 (ml)
            velocity (float): 拉取速度 (ml/s)
        """
        new_position = min(self.max_volume, self._position + volume)
        actual_volume = new_position - self._position
        
        if actual_volume <= 0:
            self.logger.warning("⚠️ 无法吸液 - 已达到最大容量")
            return
        
        display_duration = self._calculate_display_duration(actual_volume, velocity)
        actual_duration = self._calculate_duration(actual_volume, velocity)
        
        self.logger.info(f"📥 开始吸液: {actual_volume:.2f}mL")
        self.logger.info(f"  📍 位置: {self._position:.2f}mL → {new_position:.2f}mL")
        self.logger.info(f"  ⏰ 预计时间: {display_duration:.2f}s")
        
        if self._fast_mode:
            self.logger.info(f"  ⚡ 快速模式: 实际用时 {actual_duration:.2f}s")
        
        await self._simulate_operation(actual_duration)
        
        self._position = new_position
        self._current_volume = new_position
        
        self.logger.info(f"✅ 吸液完成: {actual_volume:.2f}mL | 💧 当前体积: {self._current_volume:.2f}mL")

    async def push_plunger(self, volume: float, velocity: float = None):
        """
        推出柱塞（排液） 📤
        
        Args:
            volume (float): 要推出的体积 (ml)
            velocity (float): 推出速度 (ml/s)
        """
        new_position = max(0, self._position - volume)
        actual_volume = self._position - new_position
        
        if actual_volume <= 0:
            self.logger.warning("⚠️ 无法排液 - 已达到最小容量")
            return
        
        display_duration = self._calculate_display_duration(actual_volume, velocity)
        actual_duration = self._calculate_duration(actual_volume, velocity)
        
        self.logger.info(f"📤 开始排液: {actual_volume:.2f}mL")
        self.logger.info(f"  📍 位置: {self._position:.2f}mL → {new_position:.2f}mL")
        self.logger.info(f"  ⏰ 预计时间: {display_duration:.2f}s")
        
        if self._fast_mode:
            self.logger.info(f"  ⚡ 快速模式: 实际用时 {actual_duration:.2f}s")
        
        await self._simulate_operation(actual_duration)
        
        self._position = new_position
        self._current_volume = new_position
        
        self.logger.info(f"✅ 排液完成: {actual_volume:.2f}mL | 💧 当前体积: {self._current_volume:.2f}mL")

    # 便捷操作方法
    async def aspirate(self, volume: float, velocity: float = None):
        """吸液操作 📥"""
        await self.pull_plunger(volume, velocity)
    
    async def dispense(self, volume: float, velocity: float = None):
        """排液操作 📤"""
        await self.push_plunger(volume, velocity)
    
    async def transfer(self, volume: float, aspirate_velocity: float = None, dispense_velocity: float = None):
        """转移操作（先吸后排） 🔄"""
        self.logger.info(f"🔄 开始转移操作: {volume:.2f}mL")
        
        # 吸液
        await self.aspirate(volume, aspirate_velocity)
        
        # 短暂停顿
        self.logger.debug("⏸️ 短暂停顿...")
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
        return f"VirtualTransferPump({self.device_id}: {self._current_volume:.2f}/{self.max_volume} ml, {self._status})"
    
    def __repr__(self):
        return self.__str__()


# 使用示例
async def demo():
    """虚拟泵使用示例"""
    pump = VirtualTransferPump("demo_pump", {"max_volume": 50.0})
    
    await pump.initialize()
    
    print(f"Initial state: {pump}")
    
    # 测试set_position方法
    result = await pump.set_position(10.0, max_velocity=2.0)
    print(f"Set position result: {result}")
    print(f"After setting position to 10ml: {pump}")
    
    # 吸液测试
    await pump.aspirate(5.0, velocity=2.0)
    print(f"After aspirating 5ml: {pump}")
    
    # 清空测试
    result = await pump.set_position(0.0)
    print(f"Empty result: {result}")
    print(f"After emptying: {pump}")
    
    print("\nPump info:", pump.get_pump_info())


if __name__ == "__main__":
    asyncio.run(demo())
