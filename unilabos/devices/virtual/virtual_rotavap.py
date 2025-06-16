import asyncio
import logging
from typing import Dict, Any, Optional


class VirtualRotavap:
    """Virtual rotary evaporator device for EvaporateProtocol testing"""

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

        # 添加调试信息
        print(f"=== VirtualRotavap {self.device_id} is being created! ===")
        print(f"=== Config: {self.config} ===")
        print(f"=== Kwargs: {kwargs} ===")

        # 从config或kwargs中获取配置参数
        self.port = self.config.get("port") or kwargs.get("port", "VIRTUAL")
        self._max_temp = self.config.get("max_temp") or kwargs.get("max_temp", 180.0)
        self._max_rotation_speed = self.config.get("max_rotation_speed") or kwargs.get("max_rotation_speed", 280.0)

        # 处理其他kwargs参数，但跳过已知的配置参数
        skip_keys = {"port", "max_temp", "max_rotation_speed"}
        for key, value in kwargs.items():
            if key not in skip_keys and not hasattr(self, key):
                setattr(self, key, value)

    async def initialize(self) -> bool:
        """Initialize virtual rotary evaporator"""
        print(f"=== VirtualRotavap {self.device_id} initialize() called! ===")
        self.logger.info(f"Initializing virtual rotary evaporator {self.device_id}")
        self.data.update(
            {
                "status": "Idle",
                "rotavap_state": "Ready",
                "current_temp": 25.0,
                "target_temp": 25.0,
                "max_temp": self._max_temp,
                "rotation_speed": 0.0,
                "max_rotation_speed": self._max_rotation_speed,
                "vacuum_pressure": 1.0,  # atmospheric pressure
                "evaporated_volume": 0.0,
                "progress": 0.0,
                "message": "",
            }
        )
        return True

    async def cleanup(self) -> bool:
        """Cleanup virtual rotary evaporator"""
        self.logger.info(f"Cleaning up virtual rotary evaporator {self.device_id}")
        return True

    async def evaporate(
        self, vessel: str, pressure: float = 0.5, temp: float = 60.0, time: float = 300.0, stir_speed: float = 100.0
    ) -> bool:
        """Execute evaporate action - matches Evaporate action"""
        self.logger.info(f"Evaporate: vessel={vessel}, pressure={pressure}, temp={temp}, time={time}")

        # 验证参数
        if temp > self._max_temp:
            self.logger.error(f"Temperature {temp} exceeds maximum {self._max_temp}")
            self.data["message"] = f"温度 {temp} 超过最大值 {self._max_temp}"
            return False

        if stir_speed > self._max_rotation_speed:
            self.logger.error(f"Rotation speed {stir_speed} exceeds maximum {self._max_rotation_speed}")
            self.data["message"] = f"旋转速度 {stir_speed} 超过最大值 {self._max_rotation_speed}"
            return False

        if pressure < 0.01 or pressure > 1.0:
            self.logger.error(f"Pressure {pressure} bar is out of valid range (0.01-1.0)")
            self.data["message"] = f"真空度 {pressure} bar 超出有效范围 (0.01-1.0)"
            return False

        # 开始蒸发
        self.data.update(
            {
                "status": "Running",
                "rotavap_state": "Evaporating",
                "target_temp": temp,
                "current_temp": temp,
                "rotation_speed": stir_speed,
                "vacuum_pressure": pressure,
                "vessel": vessel,
                "target_time": time,
                "progress": 0.0,
                "message": f"正在蒸发: {vessel}",
            }
        )

        # 模拟蒸发过程
        simulation_time = min(time / 60.0, 10.0)  # 最多模拟10秒
        for progress in range(0, 101, 10):
            await asyncio.sleep(simulation_time / 10)
            self.data["progress"] = progress
            self.data["evaporated_volume"] = progress * 0.5  # 假设最多蒸发50mL

        # 蒸发完成
        evaporated_vol = 50.0  # 假设蒸发了50mL
        self.data.update(
            {
                "status": "Idle",
                "rotavap_state": "Ready",
                "current_temp": 25.0,
                "target_temp": 25.0,
                "rotation_speed": 0.0,
                "vacuum_pressure": 1.0,
                "evaporated_volume": evaporated_vol,
                "progress": 100.0,
                "message": f"蒸发完成: {evaporated_vol}mL",
            }
        )

        self.logger.info(f"Evaporation completed: {evaporated_vol}mL from {vessel}")
        return True

    # 状态属性
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
    def target_temp(self) -> float:
        return self.data.get("target_temp", 25.0)

    @property
    def max_temp(self) -> float:
        return self.data.get("max_temp", self._max_temp)

    @property
    def rotation_speed(self) -> float:
        return self.data.get("rotation_speed", 0.0)

    @property
    def max_rotation_speed(self) -> float:
        return self.data.get("max_rotation_speed", self._max_rotation_speed)

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
