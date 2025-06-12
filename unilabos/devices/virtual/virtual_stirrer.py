import asyncio
import logging
from typing import Dict, Any

class VirtualStirrer:
    """Virtual stirrer device for StirProtocol testing"""
    
    def __init__(self, device_id: str = None, config: Dict[str, Any] = None, **kwargs):
        # 处理可能的不同调用方式
        if device_id is None and 'id' in kwargs:
            device_id = kwargs.pop('id')
        if config is None and 'config' in kwargs:
            config = kwargs.pop('config')
        
        # 设置默认值
        self.device_id = device_id or "unknown_stirrer"
        self.config = config or {}
        
        self.logger = logging.getLogger(f"VirtualStirrer.{self.device_id}")
        self.data = {}
        
        # 添加调试信息
        print(f"=== VirtualStirrer {self.device_id} is being created! ===")
        print(f"=== Config: {self.config} ===")
        print(f"=== Kwargs: {kwargs} ===")
        
        # 从config或kwargs中获取配置参数
        self.port = self.config.get('port') or kwargs.get('port', 'VIRTUAL')
        self._max_temp = self.config.get('max_temp') or kwargs.get('max_temp', 100.0)
        self._max_speed = self.config.get('max_speed') or kwargs.get('max_speed', 1000.0)
        
        # 处理其他kwargs参数，但跳过已知的配置参数
        skip_keys = {'port', 'max_temp', 'max_speed'}
        for key, value in kwargs.items():
            if key not in skip_keys and not hasattr(self, key):
                setattr(self, key, value)
    
    async def initialize(self) -> bool:
        """Initialize virtual stirrer"""
        print(f"=== VirtualStirrer {self.device_id} initialize() called! ===")
        self.logger.info(f"Initializing virtual stirrer {self.device_id}")
        self.data.update({
            "status": "Idle"
        })
        return True
    
    async def cleanup(self) -> bool:
        """Cleanup virtual stirrer"""
        self.logger.info(f"Cleaning up virtual stirrer {self.device_id}")
        return True
    
    async def stir(self, stir_time: float, stir_speed: float, settling_time: float) -> bool:
        """Execute stir action - matches Stir action exactly"""
        self.logger.info(f"Stir: speed={stir_speed} RPM, time={stir_time}s, settling={settling_time}s")
        
        # 验证参数
        if stir_speed > self._max_speed:
            self.logger.error(f"Stir speed {stir_speed} exceeds maximum {self._max_speed}")
            self.data["status"] = f"搅拌速度 {stir_speed} 超出范围"
            return False
        
        # 开始搅拌
        self.data["status"] = f"搅拌中: {stir_speed} RPM, {stir_time}s"
        
        # 模拟搅拌时间
        simulation_time = min(stir_time, 10.0)  # 最多等待10秒用于测试
        await asyncio.sleep(simulation_time)
        
        # 搅拌完成，开始沉降
        if settling_time > 0:
            self.data["status"] = f"沉降中: {settling_time}s"
            settling_simulation = min(settling_time, 5.0)  # 最多等待5秒
            await asyncio.sleep(settling_simulation)
        
        # 操作完成
        self.data["status"] = "搅拌完成"
        
        self.logger.info(f"Stir completed: {stir_speed} RPM for {stir_time}s")
        return True
    
    async def start_stir(self, vessel: str, stir_speed: float, purpose: str) -> bool:
        """Start stir action - matches StartStir action exactly"""
        self.logger.info(f"StartStir: vessel={vessel}, speed={stir_speed} RPM, purpose={purpose}")
        
        # 验证参数
        if stir_speed > self._max_speed:
            self.logger.error(f"Stir speed {stir_speed} exceeds maximum {self._max_speed}")
            self.data["status"] = f"搅拌速度 {stir_speed} 超出范围"
            return False
        
        self.data["status"] = f"开始搅拌: {vessel} at {stir_speed} RPM"
        return True
    
    async def stop_stir(self, vessel: str) -> bool:
        """Stop stir action - matches StopStir action exactly"""
        self.logger.info(f"StopStir: vessel={vessel}")
        
        self.data["status"] = f"停止搅拌: {vessel}"
        return True
    
    # 状态属性 - 只保留 action 中定义的 feedback
    @property
    def status(self) -> str:
        return self.data.get("status", "Idle")