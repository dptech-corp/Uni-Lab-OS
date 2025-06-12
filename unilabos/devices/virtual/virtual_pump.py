import asyncio
import logging
from typing import Dict, Any, Optional

class VirtualPump:
    """Virtual pump device for transfer and cleaning operations"""
    
    def __init__(self, device_id: str = None, config: Dict[str, Any] = None, **kwargs):
        # 处理可能的不同调用方式
        if device_id is None and 'id' in kwargs:
            device_id = kwargs.pop('id')
        if config is None and 'config' in kwargs:
            config = kwargs.pop('config')
        
        # 设置默认值
        self.device_id = device_id or "unknown_pump"
        self.config = config or {}
        
        self.logger = logging.getLogger(f"VirtualPump.{self.device_id}")
        self.data = {}
        
        # 从config或kwargs中获取配置参数
        self.port = self.config.get('port') or kwargs.get('port', 'VIRTUAL')
        self._max_volume = self.config.get('max_volume') or kwargs.get('max_volume', 50.0)
        self._transfer_rate = self.config.get('transfer_rate') or kwargs.get('transfer_rate', 10.0)
        
        print(f"=== VirtualPump {self.device_id} created with max_volume={self._max_volume}, transfer_rate={self._transfer_rate} ===")
    
    async def initialize(self) -> bool:
        """Initialize virtual pump"""
        self.logger.info(f"Initializing virtual pump {self.device_id}")
        self.data.update({
            "status": "Idle",
            "valve_position": 0,
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
        """Cleanup virtual pump"""
        self.logger.info(f"Cleaning up virtual pump {self.device_id}")
        return True
    
    async def transfer(self, from_vessel: str, to_vessel: str, volume: float,
                      amount: str = "", time: float = 0.0, viscous: bool = False,
                      rinsing_solvent: str = "", rinsing_volume: float = 0.0,
                      rinsing_repeats: int = 0, solid: bool = False) -> bool:
        """Execute transfer operation"""
        self.logger.info(f"Transferring {volume}mL from {from_vessel} to {to_vessel}")
        
        # 计算转移时间
        transfer_time = volume / self._transfer_rate if time == 0 else time
        
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
            current_volume = step_volume * (i + 1)
            
            self.data.update({
                "progress": progress,
                "transferred_volume": current_volume,
                "current_status": f"Transferring: {progress:.1f}%"
            })
            
            self.logger.info(f"Transfer progress: {progress:.1f}%")
        
        self.data.update({
            "status": "Idle",
            "current_status": "Transfer completed",
            "progress": 100.0,
            "transferred_volume": volume
        })
        
        return True
    
    async def clean_vessel(self, vessel: str, solvent: str, volume: float, 
                      temp: float, repeats: int = 1) -> bool:
        """Execute vessel cleaning operation - matches CleanVessel action"""
        self.logger.info(f"Starting vessel cleaning: {vessel} with {solvent} ({volume}mL at {temp}°C, {repeats} repeats)")
        
        # 更新设备状态
        self.data.update({
            "status": "Running",
            "from_vessel": f"flask_{solvent}",
            "to_vessel": vessel,
            "current_status": "Cleaning in progress",
            "progress": 0.0,
            "transferred_volume": 0.0
        })
        
        # 计算清洗时间（基于体积和重复次数）
        # 假设清洗速度为 transfer_rate 的一半（因为需要加载和排放）
        cleaning_rate = self._transfer_rate / 2
        cleaning_time_per_cycle = volume / cleaning_rate
        total_cleaning_time = cleaning_time_per_cycle * repeats
        
        # 模拟清洗过程
        steps_per_repeat = 10  # 每次重复清洗分10个步骤
        total_steps = steps_per_repeat * repeats
        step_time = total_cleaning_time / total_steps
        
        for repeat in range(repeats):
            self.logger.info(f"Starting cleaning cycle {repeat + 1}/{repeats}")
            
            for step in range(steps_per_repeat):
                await asyncio.sleep(step_time)
                
                # 计算当前进度
                current_step = repeat * steps_per_repeat + step + 1
                progress = (current_step / total_steps) * 100
                
                # 计算已处理的体积
                volume_processed = (current_step / total_steps) * volume * repeats
                
                # 更新状态
                self.data.update({
                    "progress": progress,
                    "transferred_volume": volume_processed,
                    "current_status": f"Cleaning cycle {repeat + 1}/{repeats} - Step {step + 1}/{steps_per_repeat} ({progress:.1f}%)"
                })
                
                self.logger.info(f"Cleaning progress: {progress:.1f}% (Cycle {repeat + 1}/{repeats})")
    
        # 清洗完成
        self.data.update({
            "status": "Idle",
            "current_status": "Cleaning completed successfully",
            "progress": 100.0,
            "transferred_volume": volume * repeats,
            "from_vessel": "",
            "to_vessel": ""
        })
        
        self.logger.info(f"Vessel cleaning completed: {vessel}")
        return True
    
    # 状态属性
    @property
    def status(self) -> str:
        return self.data.get("status", "Unknown")
    
    @property
    def valve_position(self) -> int:
        return self.data.get("valve_position", 0)
    
    @property
    def current_volume(self) -> float:
        return self.data.get("current_volume", 0.0)
    
    @property
    def max_volume(self) -> float:
        return self.data.get("max_volume", 0.0)
    
    @property
    def transfer_rate(self) -> float:
        return self.data.get("transfer_rate", 0.0)
    
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