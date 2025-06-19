import asyncio
import logging
import time as time_module
from typing import Dict, Any, Optional


class VirtualRotavap:
    """Virtual rotary evaporator device - 简化版，只保留核心功能"""

    def __init__(self, device_id: Optional[str] = None, config: Optional[Dict[str, Any]] = None, **kwargs):
        # 处理可能的不同调用方式
        if device_id is None and "id" in kwargs:
            device_id = kwargs.pop("id")
        if config is None and "config" in kwargs:
            config = kwargs.pop("config")

        # 设置默认值
        self.device_id = device_id or "unknown_rotavap"
        self.config = config or {}

        self.logger = logging.getLogger(f"VirtualRotavap.{self.device_id}")
        self.data = {}

        # 从config或kwargs中获取配置参数
        self.port = self.config.get("port") or kwargs.get("port", "VIRTUAL")
        self._max_temp = self.config.get("max_temp") or kwargs.get("max_temp", 180.0)
        self._max_rotation_speed = self.config.get("max_rotation_speed") or kwargs.get("max_rotation_speed", 280.0)

        # 处理其他kwargs参数
        skip_keys = {"port", "max_temp", "max_rotation_speed"}
        for key, value in kwargs.items():
            if key not in skip_keys and not hasattr(self, key):
                setattr(self, key, value)

    async def initialize(self) -> bool:
        """Initialize virtual rotary evaporator"""
        self.logger.info(f"Initializing virtual rotary evaporator {self.device_id}")
        
        # 只保留核心状态
        self.data.update({
            "status": "Idle",
            "rotavap_state": "Ready",  # Ready, Evaporating, Completed, Error
            "current_temp": 25.0,
            "target_temp": 25.0,
            "rotation_speed": 0.0,
            "vacuum_pressure": 1.0,  # 大气压
            "evaporated_volume": 0.0,
            "progress": 0.0,
            "remaining_time": 0.0,
            "message": "Ready for evaporation"
        })
        return True

    async def cleanup(self) -> bool:
        """Cleanup virtual rotary evaporator"""
        self.logger.info(f"Cleaning up virtual rotary evaporator {self.device_id}")
        
        self.data.update({
            "status": "Offline",
            "rotavap_state": "Offline",
            "current_temp": 25.0,
            "rotation_speed": 0.0,
            "vacuum_pressure": 1.0,
            "message": "System offline"
        })
        return True

    async def evaporate(
        self, 
        vessel: str, 
        pressure: float = 0.1, 
        temp: float = 60.0, 
        time: float = 1800.0,  # 30分钟默认
        stir_speed: float = 100.0
    ) -> bool:
        """Execute evaporate action - 简化的蒸发流程"""
        self.logger.info(f"Evaporate: vessel={vessel}, pressure={pressure} bar, temp={temp}°C, time={time}s, rotation={stir_speed} RPM")

        # 验证参数
        if temp > self._max_temp or temp < 10.0:
            error_msg = f"温度 {temp}°C 超出范围 (10-{self._max_temp}°C)"
            self.logger.error(error_msg)
            self.data.update({
                "status": f"Error: {error_msg}",
                "rotavap_state": "Error",
                "message": error_msg
            })
            return False

        if stir_speed > self._max_rotation_speed or stir_speed < 10.0:
            error_msg = f"旋转速度 {stir_speed} RPM 超出范围 (10-{self._max_rotation_speed} RPM)"
            self.logger.error(error_msg)
            self.data.update({
                "status": f"Error: {error_msg}",
                "rotavap_state": "Error",
                "message": error_msg
            })
            return False

        if pressure < 0.01 or pressure > 1.0:
            error_msg = f"真空度 {pressure} bar 超出范围 (0.01-1.0 bar)"
            self.logger.error(error_msg)
            self.data.update({
                "status": f"Error: {error_msg}",
                "rotavap_state": "Error",
                "message": error_msg
            })
            return False

        # 开始蒸发
        self.data.update({
            "status": f"蒸发中: {vessel}",
            "rotavap_state": "Evaporating",
            "current_temp": temp,
            "target_temp": temp,
            "rotation_speed": stir_speed,
            "vacuum_pressure": pressure,
            "remaining_time": time,
            "progress": 0.0,
            "evaporated_volume": 0.0,
            "message": f"Evaporating {vessel} at {temp}°C, {pressure} bar, {stir_speed} RPM"
        })

        try:
            # 蒸发过程 - 实时更新进度
            start_time = time_module.time()
            total_time = time
            
            while True:
                current_time = time_module.time()
                elapsed = current_time - start_time
                remaining = max(0, total_time - elapsed)
                progress = min(100.0, (elapsed / total_time) * 100)
                
                # 模拟蒸发体积
                evaporated_vol = progress * 0.8  # 假设最多蒸发80mL
                
                # 更新状态
                self.data.update({
                    "remaining_time": remaining,
                    "progress": progress,
                    "evaporated_volume": evaporated_vol,
                    "status": f"蒸发中: {vessel} | {temp}°C | {pressure} bar | {progress:.1f}% | 剩余: {remaining:.0f}s",
                    "message": f"Evaporating: {progress:.1f}% complete, {remaining:.0f}s remaining"
                })
                
                # 时间到了，退出循环
                if remaining <= 0:
                    break
                
                # 每秒更新一次
                await asyncio.sleep(1.0)
            
            # 蒸发完成
            final_evaporated = 80.0
            self.data.update({
                "status": f"蒸发完成: {vessel} | 蒸发量: {final_evaporated:.1f}mL",
                "rotavap_state": "Completed",
                "evaporated_volume": final_evaporated,
                "progress": 100.0,
                "remaining_time": 0.0,
                "current_temp": 25.0,  # 冷却下来
                "rotation_speed": 0.0,  # 停止旋转
                "vacuum_pressure": 1.0,  # 恢复大气压
                "message": f"Evaporation completed: {final_evaporated}mL evaporated from {vessel}"
            })

            self.logger.info(f"Evaporation completed: {final_evaporated}mL evaporated from {vessel}")
            return True

        except Exception as e:
            # 出错处理
            self.logger.error(f"Error during evaporation: {str(e)}")
            
            self.data.update({
                "status": f"蒸发错误: {str(e)}",
                "rotavap_state": "Error",
                "current_temp": 25.0,
                "rotation_speed": 0.0,
                "vacuum_pressure": 1.0,
                "message": f"Evaporation failed: {str(e)}"
            })
            return False

    # === 核心状态属性 ===
    @property
    def status(self) -> str:
        return self.data.get("status", "Unknown")

    @property
    def rotavap_state(self) -> str:
        return self.data.get("rotavap_state", "Unknown")

    @property
    def current_temp(self) -> float:
        return self.data.get("current_temp", 25.0)

    @property
    def rotation_speed(self) -> float:
        return self.data.get("rotation_speed", 0.0)

    @property
    def vacuum_pressure(self) -> float:
        return self.data.get("vacuum_pressure", 1.0)

    @property
    def evaporated_volume(self) -> float:
        return self.data.get("evaporated_volume", 0.0)

    @property
    def progress(self) -> float:
        return self.data.get("progress", 0.0)

    @property
    def message(self) -> str:
        return self.data.get("message", "")

    @property
    def max_temp(self) -> float:
        return self._max_temp

    @property
    def max_rotation_speed(self) -> float:
        return self._max_rotation_speed

    @property
    def remaining_time(self) -> float:
        return self.data.get("remaining_time", 0.0)
