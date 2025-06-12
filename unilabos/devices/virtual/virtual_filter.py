import asyncio
import logging
from typing import Dict, Any

class VirtualFilter:
    """Virtual filter device for FilterProtocol testing"""
    
    def __init__(self, device_id: str = None, config: Dict[str, Any] = None, **kwargs):
        # 处理可能的不同调用方式
        if device_id is None and 'id' in kwargs:
            device_id = kwargs.pop('id')
        if config is None and 'config' in kwargs:
            config = kwargs.pop('config')
        
        # 设置默认值
        self.device_id = device_id or "unknown_filter"
        self.config = config or {}
        
        self.logger = logging.getLogger(f"VirtualFilter.{self.device_id}")
        self.data = {}
        
        # 添加调试信息
        print(f"=== VirtualFilter {self.device_id} is being created! ===")
        print(f"=== Config: {self.config} ===")
        print(f"=== Kwargs: {kwargs} ===")
        
        # 从config或kwargs中获取配置参数
        self.port = self.config.get('port') or kwargs.get('port', 'VIRTUAL')
        self._max_temp = self.config.get('max_temp') or kwargs.get('max_temp', 100.0)
        self._max_stir_speed = self.config.get('max_stir_speed') or kwargs.get('max_stir_speed', 1000.0)
        
        # 处理其他kwargs参数，但跳过已知的配置参数
        skip_keys = {'port', 'max_temp', 'max_stir_speed'}
        for key, value in kwargs.items():
            if key not in skip_keys and not hasattr(self, key):
                setattr(self, key, value)
    
    async def initialize(self) -> bool:
        """Initialize virtual filter"""
        print(f"=== VirtualFilter {self.device_id} initialize() called! ===")
        self.logger.info(f"Initializing virtual filter {self.device_id}")
        self.data.update({
            "status": "Idle",
            "filter_state": "Ready",
            "current_temp": 25.0,
            "target_temp": 25.0,
            "max_temp": self._max_temp,
            "stir_speed": 0.0,
            "max_stir_speed": self._max_stir_speed,
            "filtered_volume": 0.0,
            "progress": 0.0,
            "message": ""
        })
        return True
    
    async def cleanup(self) -> bool:
        """Cleanup virtual filter"""
        self.logger.info(f"Cleaning up virtual filter {self.device_id}")
        return True
    
    async def filter_sample(self, vessel: str, filtrate_vessel: str = "", stir: bool = False, 
                           stir_speed: float = 300.0, temp: float = 25.0, 
                           continue_heatchill: bool = False, volume: float = 0.0) -> bool:
        """Execute filter action - matches Filter action"""
        self.logger.info(f"Filter: vessel={vessel}, filtrate_vessel={filtrate_vessel}, stir={stir}, volume={volume}")
        
        # 验证参数
        if temp > self._max_temp:
            self.logger.error(f"Temperature {temp} exceeds maximum {self._max_temp}")
            self.data["message"] = f"温度 {temp} 超过最大值 {self._max_temp}"
            return False
        
        if stir and stir_speed > self._max_stir_speed:
            self.logger.error(f"Stir speed {stir_speed} exceeds maximum {self._max_stir_speed}")
            self.data["message"] = f"搅拌速度 {stir_speed} 超过最大值 {self._max_stir_speed}"
            return False
        
        # 开始过滤
        self.data.update({
            "status": "Running",
            "filter_state": "Filtering",
            "target_temp": temp,
            "current_temp": temp,
            "stir_speed": stir_speed if stir else 0.0,
            "vessel": vessel,
            "filtrate_vessel": filtrate_vessel,
            "target_volume": volume,
            "progress": 0.0,
            "message": f"过滤中: {vessel}"
        })
        
        # 模拟过滤过程
        simulation_time = min(volume / 10.0 if volume > 0 else 5.0, 10.0)
        await asyncio.sleep(simulation_time)
        
        # 过滤完成
        filtered_vol = volume if volume > 0 else 50.0  # 默认过滤量
        self.data.update({
            "status": "Idle",
            "filter_state": "Ready",
            "current_temp": 25.0 if not continue_heatchill else temp,
            "target_temp": 25.0 if not continue_heatchill else temp,
            "stir_speed": 0.0 if not stir else stir_speed,
            "filtered_volume": filtered_vol,
            "progress": 100.0,
            "message": f"过滤完成: {filtered_vol}mL"
        })
        
        self.logger.info(f"Filter completed: {filtered_vol}mL from {vessel}")
        return True
    
    # 状态属性
    @property
    def status(self) -> str:
        return self.data.get("status", "Unknown")
    
    @property
    def filter_state(self) -> str:
        return self.data.get("filter_state", "Unknown")
    
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
    def stir_speed(self) -> float:
        return self.data.get("stir_speed", 0.0)
    
    @property
    def max_stir_speed(self) -> float:
        return self.data.get("max_stir_speed", self._max_stir_speed)
    
    @property
    def filtered_volume(self) -> float:
        return self.data.get("filtered_volume", 0.0)
    
    @property
    def progress(self) -> float:
        return self.data.get("progress", 0.0)
    
    @property
    def message(self) -> str:
        return self.data.get("message", "")