import asyncio
import logging
from typing import Dict, Any

class VirtualHeatChill:
    """Virtual heat chill device for HeatChillProtocol testing"""
    
    def __init__(self, device_id: str = None, config: Dict[str, Any] = None, **kwargs):
        # 处理可能的不同调用方式
        if device_id is None and 'id' in kwargs:
            device_id = kwargs.pop('id')
        if config is None and 'config' in kwargs:
            config = kwargs.pop('config')
        
        # 设置默认值
        self.device_id = device_id or "unknown_heatchill"
        self.config = config or {}
        
        self.logger = logging.getLogger(f"VirtualHeatChill.{self.device_id}")
        self.data = {}
        
        # 添加调试信息
        print(f"=== VirtualHeatChill {self.device_id} is being created! ===")
        print(f"=== Config: {self.config} ===")
        print(f"=== Kwargs: {kwargs} ===")
        
        # 从config或kwargs中获取配置参数
        self.port = self.config.get('port') or kwargs.get('port', 'VIRTUAL')
        self._max_temp = self.config.get('max_temp') or kwargs.get('max_temp', 200.0)
        self._min_temp = self.config.get('min_temp') or kwargs.get('min_temp', -80.0)
        self._max_stir_speed = self.config.get('max_stir_speed') or kwargs.get('max_stir_speed', 1000.0)
        
        # 处理其他kwargs参数，但跳过已知的配置参数
        skip_keys = {'port', 'max_temp', 'min_temp', 'max_stir_speed'}
        for key, value in kwargs.items():
            if key not in skip_keys and not hasattr(self, key):
                setattr(self, key, value)
    
    async def initialize(self) -> bool:
        """Initialize virtual heat chill"""
        print(f"=== VirtualHeatChill {self.device_id} initialize() called! ===")
        self.logger.info(f"Initializing virtual heat chill {self.device_id}")
        self.data.update({
            "status": "Idle"
        })
        return True
    
    async def cleanup(self) -> bool:
        """Cleanup virtual heat chill"""
        self.logger.info(f"Cleaning up virtual heat chill {self.device_id}")
        return True
    
    async def heat_chill(self, vessel: str, temp: float, time: float, stir: bool, 
                        stir_speed: float, purpose: str) -> bool:
        """Execute heat chill action - matches HeatChill action exactly"""
        self.logger.info(f"HeatChill: vessel={vessel}, temp={temp}°C, time={time}s, stir={stir}, stir_speed={stir_speed}, purpose={purpose}")
        
        # 验证参数
        if temp > self._max_temp or temp < self._min_temp:
            self.logger.error(f"Temperature {temp} outside range {self._min_temp}-{self._max_temp}")
            self.data["status"] = f"温度 {temp} 超出范围"
            return False
        
        if stir and stir_speed > self._max_stir_speed:
            self.logger.error(f"Stir speed {stir_speed} exceeds maximum {self._max_stir_speed}")
            self.data["status"] = f"搅拌速度 {stir_speed} 超出范围"
            return False
        
        # 开始加热/冷却
        self.data.update({
            "status": f"加热/冷却中: {vessel} 至 {temp}°C"
        })
        
        # 模拟加热/冷却时间
        simulation_time = min(time, 10.0)  # 最多等待10秒用于测试
        await asyncio.sleep(simulation_time)
        
        # 加热/冷却完成
        self.data["status"] = f"完成: {vessel} 已达到 {temp}°C"
        
        self.logger.info(f"HeatChill completed for vessel {vessel} at {temp}°C")
        return True
    
    async def heat_chill_start(self, vessel: str, temp: float, purpose: str) -> bool:
        """Start heat chill - matches HeatChillStart action exactly"""
        self.logger.info(f"HeatChillStart: vessel={vessel}, temp={temp}°C, purpose={purpose}")
        
        # 验证参数
        if temp > self._max_temp or temp < self._min_temp:
            self.logger.error(f"Temperature {temp} outside range {self._min_temp}-{self._max_temp}")
            self.data["status"] = f"温度 {temp} 超出范围"
            return False
        
        self.data["status"] = f"开始加热/冷却: {vessel} 至 {temp}°C"
        return True
    
    async def heat_chill_stop(self, vessel: str) -> bool:
        """Stop heat chill - matches HeatChillStop action exactly"""
        self.logger.info(f"HeatChillStop: vessel={vessel}")
        
        self.data["status"] = f"停止加热/冷却: {vessel}"
        return True
    
    # 状态属性 - 只保留 action 中定义的 feedback
    @property
    def status(self) -> str:
        return self.data.get("status", "Idle")