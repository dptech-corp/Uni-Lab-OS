import asyncio
import logging
from typing import Dict, Any, Optional

class VirtualColumn:
    """Virtual column device for RunColumn protocol"""
    
    def __init__(self, device_id: str = None, config: Dict[str, Any] = None, **kwargs):
        # 处理可能的不同调用方式
        if device_id is None and 'id' in kwargs:
            device_id = kwargs.pop('id')
        if config is None and 'config' in kwargs:
            config = kwargs.pop('config')
        
        # 设置默认值
        self.device_id = device_id or "unknown_column"
        self.config = config or {}
        
        self.logger = logging.getLogger(f"VirtualColumn.{self.device_id}")
        self.data = {}
        
        # 从config或kwargs中获取配置参数
        self.port = self.config.get('port') or kwargs.get('port', 'VIRTUAL')
        self._max_flow_rate = self.config.get('max_flow_rate') or kwargs.get('max_flow_rate', 10.0)
        self._column_length = self.config.get('column_length') or kwargs.get('column_length', 25.0)
        self._column_diameter = self.config.get('column_diameter') or kwargs.get('column_diameter', 2.0)
        
        print(f"=== VirtualColumn {self.device_id} created with max_flow_rate={self._max_flow_rate}, length={self._column_length}cm ===")
    
    async def initialize(self) -> bool:
        """Initialize virtual column"""
        self.logger.info(f"Initializing virtual column {self.device_id}")
        self.data.update({
            "status": "Idle",
            "column_state": "Ready",
            "current_flow_rate": 0.0,
            "max_flow_rate": self._max_flow_rate,
            "column_length": self._column_length,
            "column_diameter": self._column_diameter,
            "processed_volume": 0.0,
            "progress": 0.0,
            "current_status": "Ready"
        })
        return True
    
    async def cleanup(self) -> bool:
        """Cleanup virtual column"""
        self.logger.info(f"Cleaning up virtual column {self.device_id}")
        return True
    
    async def run_column(self, from_vessel: str, to_vessel: str, column: str) -> bool:
        """Execute column chromatography run - matches RunColumn action"""
        self.logger.info(f"Running column separation: {from_vessel} -> {to_vessel} using {column}")
        
        # 更新设备状态
        self.data.update({
            "status": "Running",
            "column_state": "Separating",
            "current_status": "Column separation in progress",
            "progress": 0.0,
            "processed_volume": 0.0
        })
        
        # 模拟柱层析分离过程
        # 假设处理时间基于流速和柱子长度
        separation_time = (self._column_length * 2) / self._max_flow_rate  # 简化计算
        
        steps = 20  # 分20个步骤模拟分离过程
        step_time = separation_time / steps
        
        for i in range(steps):
            await asyncio.sleep(step_time)
            
            progress = (i + 1) / steps * 100
            volume_processed = (i + 1) * 5.0  # 假设每步处理5mL
            
            # 更新状态
            self.data.update({
                "progress": progress,
                "processed_volume": volume_processed,
                "current_status": f"Column separation: {progress:.1f}% - Processing {volume_processed:.1f}mL"
            })
            
            self.logger.info(f"Column separation progress: {progress:.1f}%")
        
        # 分离完成
        self.data.update({
            "status": "Idle",
            "column_state": "Ready",
            "current_status": "Column separation completed",
            "progress": 100.0
        })
        
        self.logger.info(f"Column separation completed: {from_vessel} -> {to_vessel}")
        return True
    
    # 状态属性
    @property
    def status(self) -> str:
        return self.data.get("status", "Unknown")
    
    @property
    def column_state(self) -> str:
        return self.data.get("column_state", "Unknown")
    
    @property
    def current_flow_rate(self) -> float:
        return self.data.get("current_flow_rate", 0.0)
    
    @property
    def max_flow_rate(self) -> float:
        return self.data.get("max_flow_rate", 0.0)
    
    @property
    def column_length(self) -> float:
        return self.data.get("column_length", 0.0)
    
    @property
    def column_diameter(self) -> float:
        return self.data.get("column_diameter", 0.0)
    
    @property
    def processed_volume(self) -> float:
        return self.data.get("processed_volume", 0.0)
    
    @property
    def progress(self) -> float:
        return self.data.get("progress", 0.0)
    
    @property
    def current_status(self) -> str:
        return self.data.get("current_status", "Ready")