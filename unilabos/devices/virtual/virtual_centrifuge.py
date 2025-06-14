import asyncio
import logging
from typing import Dict, Any

class VirtualCentrifuge:
    """Virtual centrifuge device for CentrifugeProtocol testing"""
    
    def __init__(self, device_id: str = None, config: Dict[str, Any] = None, **kwargs):
        # 处理可能的不同调用方式
        if device_id is None and 'id' in kwargs:
            device_id = kwargs.pop('id')
        if config is None and 'config' in kwargs:
            config = kwargs.pop('config')
        
        # 设置默认值
        self.device_id = device_id or "unknown_centrifuge"
        self.config = config or {}
        
        self.logger = logging.getLogger(f"VirtualCentrifuge.{self.device_id}")
        self.data = {}
        
        # 添加调试信息
        print(f"=== VirtualCentrifuge {self.device_id} is being created! ===")
        print(f"=== Config: {self.config} ===")
        print(f"=== Kwargs: {kwargs} ===")
        
        # 从config或kwargs中获取配置参数
        self.port = self.config.get('port') or kwargs.get('port', 'VIRTUAL')
        self._max_speed = self.config.get('max_speed') or kwargs.get('max_speed', 15000.0)
        self._max_temp = self.config.get('max_temp') or kwargs.get('max_temp', 40.0)
        self._min_temp = self.config.get('min_temp') or kwargs.get('min_temp', 4.0)
        
        # 处理其他kwargs参数，但跳过已知的配置参数
        skip_keys = {'port', 'max_speed', 'max_temp', 'min_temp'}
        for key, value in kwargs.items():
            if key not in skip_keys and not hasattr(self, key):
                setattr(self, key, value)
    
    async def initialize(self) -> bool:
        """Initialize virtual centrifuge"""
        print(f"=== VirtualCentrifuge {self.device_id} initialize() called! ===")
        self.logger.info(f"Initializing virtual centrifuge {self.device_id}")
        self.data.update({
            "status": "Idle",
            "current_speed": 0.0,
            "target_speed": 0.0,
            "current_temp": 25.0,
            "target_temp": 25.0,
            "max_speed": self._max_speed,
            "max_temp": self._max_temp,
            "min_temp": self._min_temp,
            "centrifuge_state": "Stopped",
            "time_remaining": 0.0,
            "progress": 0.0,
            "message": ""
        })
        return True
    
    async def cleanup(self) -> bool:
        """Cleanup virtual centrifuge"""
        self.logger.info(f"Cleaning up virtual centrifuge {self.device_id}")
        return True
    
    async def centrifuge(self, vessel: str, speed: float, time: float, temp: float = 25.0) -> bool:
        """Execute centrifuge action - matches Centrifuge action"""
        self.logger.info(f"Centrifuge: vessel={vessel}, speed={speed} RPM, time={time}s, temp={temp}°C")
        
        # 验证参数
        if speed > self._max_speed:
            self.logger.error(f"Speed {speed} exceeds maximum {self._max_speed}")
            self.data["message"] = f"速度 {speed} 超过最大值 {self._max_speed}"
            return False
        
        if temp > self._max_temp or temp < self._min_temp:
            self.logger.error(f"Temperature {temp} outside range {self._min_temp}-{self._max_temp}")
            self.data["message"] = f"温度 {temp} 超出范围 {self._min_temp}-{self._max_temp}"
            return False
        
        # 开始离心
        self.data.update({
            "status": "Running",
            "centrifuge_state": "Centrifuging",
            "target_speed": speed,
            "current_speed": speed,
            "target_temp": temp,
            "current_temp": temp,
            "time_remaining": time,
            "vessel": vessel,
            "progress": 0.0,
            "message": f"离心中: {vessel} at {speed} RPM"
        })
        
        # 模拟离心过程
        simulation_time = min(time, 5.0)  # 最多等待5秒用于测试
        await asyncio.sleep(simulation_time)
        
        # 离心完成
        self.data.update({
            "status": "Idle",
            "centrifuge_state": "Stopped",
            "current_speed": 0.0,
            "target_speed": 0.0,
            "time_remaining": 0.0,
            "progress": 100.0,
            "message": f"离心完成: {vessel}"
        })
        
        self.logger.info(f"Centrifuge completed for vessel {vessel}")
        return True
    
    # 状态属性
    @property
    def status(self) -> str:
        return self.data.get("status", "Unknown")
    
    @property
    def current_speed(self) -> float:
        return self.data.get("current_speed", 0.0)
    
    @property
    def target_speed(self) -> float:
        return self.data.get("target_speed", 0.0)
    
    @property
    def current_temp(self) -> float:
        return self.data.get("current_temp", 25.0)
    
    @property
    def target_temp(self) -> float:
        return self.data.get("target_temp", 25.0)
    
    @property
    def max_speed(self) -> float:
        return self.data.get("max_speed", self._max_speed)
    
    @property
    def max_temp(self) -> float:
        return self.data.get("max_temp", self._max_temp)
    
    @property
    def min_temp(self) -> float:
        return self.data.get("min_temp", self._min_temp)
    
    @property
    def centrifuge_state(self) -> str:
        return self.data.get("centrifuge_state", "Unknown")
    
    @property
    def time_remaining(self) -> float:
        return self.data.get("time_remaining", 0.0)
    
    @property
    def progress(self) -> float:
        return self.data.get("progress", 0.0)
    
    @property
    def message(self) -> str:
        return self.data.get("message", "")