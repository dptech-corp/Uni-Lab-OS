import asyncio
import logging
import time as time_module
from typing import Dict, Any, Optional


class VirtualFilter:
    """Virtual filter device - 完全按照 Filter.action 规范"""
    
    def __init__(self, device_id: Optional[str] = None, config: Optional[Dict[str, Any]] = None, **kwargs):
        if device_id is None and 'id' in kwargs:
            device_id = kwargs.pop('id')
        if config is None and 'config' in kwargs:
            config = kwargs.pop('config')
        
        self.device_id = device_id or "unknown_filter"
        self.config = config or {}
        self.logger = logging.getLogger(f"VirtualFilter.{self.device_id}")
        self.data = {}
        
        # 从config或kwargs中获取配置参数
        self.port = self.config.get('port') or kwargs.get('port', 'VIRTUAL')
        self._max_temp = self.config.get('max_temp') or kwargs.get('max_temp', 100.0)
        self._max_stir_speed = self.config.get('max_stir_speed') or kwargs.get('max_stir_speed', 1000.0)
        self._max_volume = self.config.get('max_volume') or kwargs.get('max_volume', 500.0)
        
        # 处理其他kwargs参数
        skip_keys = {'port', 'max_temp', 'max_stir_speed', 'max_volume'}
        for key, value in kwargs.items():
            if key not in skip_keys and not hasattr(self, key):
                setattr(self, key, value)
    
    async def initialize(self) -> bool:
        """Initialize virtual filter"""
        self.logger.info(f"Initializing virtual filter {self.device_id}")
        
        # 按照 Filter.action 的 feedback 字段初始化
        self.data.update({
            "status": "Idle",
            "progress": 0.0,           # Filter.action feedback
            "current_temp": 25.0,      # Filter.action feedback
            "filtered_volume": 0.0,    # Filter.action feedback
            "current_status": "Ready for filtration",  # Filter.action feedback
            "message": "Ready for filtration"
        })
        return True
    
    async def cleanup(self) -> bool:
        """Cleanup virtual filter"""
        self.logger.info(f"Cleaning up virtual filter {self.device_id}")
        
        self.data.update({
            "status": "Offline",
            "current_status": "System offline",
            "message": "System offline"
        })
        return True
    
    async def filter(
        self, 
        vessel: str, 
        filtrate_vessel: str = "", 
        stir: bool = False, 
        stir_speed: float = 300.0, 
        temp: float = 25.0, 
        continue_heatchill: bool = False, 
        volume: float = 0.0
    ) -> bool:
        """Execute filter action - 完全按照 Filter.action 参数"""
        
        # 🔧 新增：温度自动调整
        original_temp = temp
        if temp == 0.0:
            temp = 25.0  # 0度自动设置为室温
            self.logger.info(f"温度自动调整: {original_temp}°C → {temp}°C (室温)")
        elif temp < 4.0:
            temp = 4.0   # 小于4度自动设置为4度
            self.logger.info(f"温度自动调整: {original_temp}°C → {temp}°C (最低温度)")
        
        self.logger.info(f"Filter: vessel={vessel}, filtrate_vessel={filtrate_vessel}")
        self.logger.info(f"  stir={stir}, stir_speed={stir_speed}, temp={temp}")
        self.logger.info(f"  continue_heatchill={continue_heatchill}, volume={volume}")
        
        # 验证参数
        if temp > self._max_temp or temp < 4.0:
            error_msg = f"温度 {temp}°C 超出范围 (4-{self._max_temp}°C)"
            self.logger.error(error_msg)
            self.data.update({
                "status": f"Error: {error_msg}",
                "current_status": f"Error: {error_msg}",
                "message": error_msg
            })
            return False
        
        if stir and stir_speed > self._max_stir_speed:
            error_msg = f"搅拌速度 {stir_speed} RPM 超出范围 (0-{self._max_stir_speed} RPM)"
            self.logger.error(error_msg)
            self.data.update({
                "status": f"Error: {error_msg}",
                "current_status": f"Error: {error_msg}",
                "message": error_msg
            })
            return False
        
        if volume > self._max_volume:
            error_msg = f"过滤体积 {volume} mL 超出范围 (0-{self._max_volume} mL)"
            self.logger.error(error_msg)
            self.data.update({
                "status": f"Error: {error_msg}",
                "current_status": f"Error: {error_msg}",
                "message": error_msg
            })
            return False
        
        # 开始过滤
        filter_volume = volume if volume > 0 else 50.0
        
        self.data.update({
            "status": f"过滤中: {vessel}",
            "current_temp": temp,
            "filtered_volume": 0.0,
            "progress": 0.0,
            "current_status": f"Filtering {vessel} → {filtrate_vessel}",
            "message": f"Starting filtration: {vessel} → {filtrate_vessel}"
        })
        
        try:
            # 过滤过程 - 实时更新进度
            start_time = time_module.time()
            # 根据体积和搅拌估算过滤时间
            base_time = filter_volume / 5.0  # 5mL/s 基础速度
            if stir:
                base_time *= 0.8  # 搅拌加速过滤
            if temp > 50.0:
                base_time *= 0.7  # 高温加速过滤
            filter_time = max(base_time, 10.0)  # 最少10秒
            
            while True:
                current_time = time_module.time()
                elapsed = current_time - start_time
                remaining = max(0, filter_time - elapsed)
                progress = min(100.0, (elapsed / filter_time) * 100)
                current_filtered = (progress / 100.0) * filter_volume
                
                # 更新状态 - 按照 Filter.action feedback 字段
                status_msg = f"过滤中: {vessel}"
                if stir:
                    status_msg += f" | 搅拌: {stir_speed} RPM"
                status_msg += f" | {temp}°C | {progress:.1f}% | 已过滤: {current_filtered:.1f}mL"
                
                self.data.update({
                    "progress": progress,               # Filter.action feedback
                    "current_temp": temp,              # Filter.action feedback
                    "filtered_volume": current_filtered, # Filter.action feedback
                    "current_status": f"Filtering: {progress:.1f}% complete", # Filter.action feedback
                    "status": status_msg,
                    "message": f"Filtering: {progress:.1f}% complete, {current_filtered:.1f}mL filtered"
                })
                
                if remaining <= 0:
                    break
                
                await asyncio.sleep(1.0)
            
            # 过滤完成
            final_temp = temp if continue_heatchill else 25.0
            final_status = f"过滤完成: {vessel} | {filter_volume}mL → {filtrate_vessel}"
            if continue_heatchill:
                final_status += " | 继续加热搅拌"
            
            self.data.update({
                "status": final_status,
                "progress": 100.0,                    # Filter.action feedback
                "current_temp": final_temp,           # Filter.action feedback
                "filtered_volume": filter_volume,     # Filter.action feedback
                "current_status": f"Filtration completed: {filter_volume}mL", # Filter.action feedback
                "message": f"Filtration completed: {filter_volume}mL filtered from {vessel}"
            })
            
            self.logger.info(f"Filtration completed: {filter_volume}mL from {vessel} to {filtrate_vessel}")
            return True
            
        except Exception as e:
            self.logger.error(f"Error during filtration: {str(e)}")
            self.data.update({
                "status": f"过滤错误: {str(e)}",
                "current_status": f"Filtration failed: {str(e)}",
                "message": f"Filtration failed: {str(e)}"
            })
            return False
    
    # === 核心状态属性 - 按照 Filter.action feedback 字段 ===
    @property
    def status(self) -> str:
        return self.data.get("status", "Unknown")
    
    @property
    def progress(self) -> float:
        """Filter.action feedback 字段"""
        return self.data.get("progress", 0.0)
    
    @property
    def current_temp(self) -> float:
        """Filter.action feedback 字段"""
        return self.data.get("current_temp", 25.0)
    
    @property
    def filtered_volume(self) -> float:
        """Filter.action feedback 字段"""
        return self.data.get("filtered_volume", 0.0)
    
    @property
    def current_status(self) -> str:
        """Filter.action feedback 字段"""
        return self.data.get("current_status", "")
    
    @property
    def message(self) -> str:
        return self.data.get("message", "")
    
    @property
    def max_temp(self) -> float:
        return self._max_temp
    
    @property
    def max_stir_speed(self) -> float:
        return self._max_stir_speed
    
    @property
    def max_volume(self) -> float:
        return self._max_volume