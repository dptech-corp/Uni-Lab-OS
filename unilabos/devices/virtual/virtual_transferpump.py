import asyncio
import logging
from typing import Dict, Any, Optional

class VirtualTransferPump:
    """Virtual pump device specifically for Transfer protocol"""
    
    def __init__(self, device_id: str = None, config: Dict[str, Any] = None, **kwargs):
        # 处理可能的不同调用方式
        if device_id is None and 'id' in kwargs:
            device_id = kwargs.pop('id')
        if config is None and 'config' in kwargs:
            config = kwargs.pop('config')
        
        # 设置默认值
        self.device_id = device_id or "unknown_transfer_pump"
        self.config = config or {}
        
        self.logger = logging.getLogger(f"VirtualTransferPump.{self.device_id}")
        self.data = {}
        
        # 添加调试信息
        print(f"=== VirtualTransferPump {self.device_id} is being created! ===")
        print(f"=== Config: {self.config} ===")
        print(f"=== Kwargs: {kwargs} ===")
        
        # 从config或kwargs中获取配置参数
        self.port = self.config.get('port') or kwargs.get('port', 'VIRTUAL')
        self._max_volume = self.config.get('max_volume') or kwargs.get('max_volume', 50.0)
        self._transfer_rate = self.config.get('transfer_rate') or kwargs.get('transfer_rate', 5.0)
        self._current_volume = 0.0
        self.is_running = False
    
    async def initialize(self) -> bool:
        """Initialize virtual transfer pump"""
        print(f"=== VirtualTransferPump {self.device_id} initialize() called! ===")
        self.logger.info(f"Initializing virtual transfer pump {self.device_id}")
        self.data.update({
            "status": "Idle",
            "current_volume": 0.0,
            "max_volume": self._max_volume,
            "transfer_rate": self._transfer_rate,
            "from_vessel": "",
            "to_vessel": "",
            "progress": 0.0,
            "transferred_volume": 0.0,
            "current_status": "Ready"
        })
        return True
    
    async def cleanup(self) -> bool:
        """Cleanup virtual transfer pump"""
        self.logger.info(f"Cleaning up virtual transfer pump {self.device_id}")
        return True
    
    async def transfer(self, from_vessel: str, to_vessel: str, volume: float, 
                      amount: str = "", time: float = 0, viscous: bool = False,
                      rinsing_solvent: str = "", rinsing_volume: float = 0.0,
                      rinsing_repeats: int = 0, solid: bool = False) -> bool:
        """Execute liquid transfer - matches Transfer action"""
        self.logger.info(f"Transfer: {volume}mL from {from_vessel} to {to_vessel}")
        
        # 计算转移时间
        if time > 0:
            transfer_time = time
        else:
            # 如果是粘性液体，降低转移速率
            rate = self._transfer_rate * 0.5 if viscous else self._transfer_rate
            transfer_time = volume / rate
        
        self.data.update({
            "status": "Running",
            "from_vessel": from_vessel,
            "to_vessel": to_vessel,
            "current_status": "Transferring",
            "progress": 0.0,
            "transferred_volume": 0.0
        })
        
        # 模拟转移过程
        steps = 10
        step_time = transfer_time / steps
        step_volume = volume / steps
        
        for i in range(steps):
            await asyncio.sleep(step_time)
            progress = (i + 1) / steps * 100
            transferred = (i + 1) * step_volume
            
            self.data.update({
                "progress": progress,
                "transferred_volume": transferred,
                "current_status": f"Transferring {progress:.1f}%"
            })
            
            self.logger.info(f"Transfer progress: {progress:.1f}% ({transferred:.1f}/{volume}mL)")
        
        # 如果需要冲洗
        if rinsing_solvent and rinsing_volume > 0 and rinsing_repeats > 0:
            self.data["current_status"] = "Rinsing"
            for repeat in range(rinsing_repeats):
                self.logger.info(f"Rinsing cycle {repeat + 1}/{rinsing_repeats} with {rinsing_solvent}")
                await asyncio.sleep(1)  # 模拟冲洗时间
        
        self.data.update({
            "status": "Idle",
            "current_status": "Transfer completed",
            "progress": 100.0,
            "transferred_volume": volume
        })
        
        return True
    
    # 添加所有在virtual_device.yaml中定义的状态属性
    @property
    def status(self) -> str:
        return self.data.get("status", "Unknown")
    
    @property
    def current_volume(self) -> float:
        return self.data.get("current_volume", 0.0)
    
    @property
    def max_volume(self) -> float:
        return self.data.get("max_volume", self._max_volume)
    
    @property
    def transfer_rate(self) -> float:
        return self.data.get("transfer_rate", self._transfer_rate)
    
    @property
    def from_vessel(self) -> str:
        return self.data.get("from_vessel", "")
    
    @property
    def to_vessel(self) -> str:
        return self.data.get("to_vessel", "")
    
    @property
    def progress(self) -> float:
        return self.data.get("progress", 0.0)
    
    @property
    def transferred_volume(self) -> float:
        return self.data.get("transferred_volume", 0.0)
    
    @property
    def current_status(self) -> str:
        return self.data.get("current_status", "Ready")