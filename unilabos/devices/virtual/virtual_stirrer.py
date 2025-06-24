import asyncio
import logging
import time as time_module
from typing import Dict, Any

class VirtualStirrer:
    """Virtual stirrer device for StirProtocol testing - 功能完整版"""
    
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
        
        # 从config或kwargs中获取配置参数
        self.port = self.config.get('port') or kwargs.get('port', 'VIRTUAL')
        self._max_speed = self.config.get('max_speed') or kwargs.get('max_speed', 1500.0)
        self._min_speed = self.config.get('min_speed') or kwargs.get('min_speed', 50.0)
        
        # 处理其他kwargs参数
        skip_keys = {'port', 'max_speed', 'min_speed'}
        for key, value in kwargs.items():
            if key not in skip_keys and not hasattr(self, key):
                setattr(self, key, value)
    
    async def initialize(self) -> bool:
        """Initialize virtual stirrer"""
        self.logger.info(f"Initializing virtual stirrer {self.device_id}")
        
        # 初始化状态信息
        self.data.update({
            "status": "Idle",
            "operation_mode": "Idle",          # 操作模式: Idle, Stirring, Settling, Completed, Error
            "current_vessel": "",              # 当前搅拌的容器
            "current_speed": 0.0,              # 当前搅拌速度
            "is_stirring": False,              # 是否正在搅拌
            "remaining_time": 0.0,             # 剩余时间
        })
        return True
    
    async def cleanup(self) -> bool:
        """Cleanup virtual stirrer"""
        self.logger.info(f"Cleaning up virtual stirrer {self.device_id}")
        self.data.update({
            "status": "Offline",
            "operation_mode": "Offline",
            "current_vessel": "",
            "current_speed": 0.0,
            "is_stirring": False,
            "remaining_time": 0.0,
        })
        return True
    
    async def stir(self, stir_time: float, stir_speed: float, settling_time: float) -> bool:
        """Execute stir action - 定时搅拌 + 沉降"""
        self.logger.info(f"Stir: speed={stir_speed} RPM, time={stir_time}s, settling={settling_time}s")
        
        # 验证参数
        if stir_speed > self._max_speed or stir_speed < self._min_speed:
            error_msg = f"搅拌速度 {stir_speed} RPM 超出范围 ({self._min_speed} - {self._max_speed} RPM)"
            self.logger.error(error_msg)
            self.data.update({
                "status": f"Error: {error_msg}",
                "operation_mode": "Error"
            })
            return False
        
        # === 第一阶段：搅拌 ===
        start_time = time_module.time()
        total_stir_time = stir_time
        
        self.data.update({
            "status": f"搅拌中: {stir_speed} RPM | 剩余: {total_stir_time:.0f}s",
            "operation_mode": "Stirring",
            "current_speed": stir_speed,
            "is_stirring": True,
            "remaining_time": total_stir_time,
        })
        
        # 搅拌过程 - 实时更新剩余时间
        while True:
            current_time = time_module.time()
            elapsed = current_time - start_time
            remaining = max(0, total_stir_time - elapsed)
            
            # 更新状态
            self.data.update({
                "remaining_time": remaining,
                "status": f"搅拌中: {stir_speed} RPM | 剩余: {remaining:.0f}s"
            })
            
            # 搅拌时间到了
            if remaining <= 0:
                break
            
            await asyncio.sleep(1.0)
        
        # === 第二阶段：沉降（如果需要）===
        if settling_time > 0:
            start_settling_time = time_module.time()
            total_settling_time = settling_time
            
            self.data.update({
                "status": f"沉降中: 停止搅拌 | 剩余: {total_settling_time:.0f}s",
                "operation_mode": "Settling",
                "current_speed": 0.0,
                "is_stirring": False,
                "remaining_time": total_settling_time,
            })
            
            # 沉降过程 - 实时更新剩余时间
            while True:
                current_time = time_module.time()
                elapsed = current_time - start_settling_time
                remaining = max(0, total_settling_time - elapsed)
                
                # 更新状态
                self.data.update({
                    "remaining_time": remaining,
                    "status": f"沉降中: 停止搅拌 | 剩余: {remaining:.0f}s"
                })
                
                # 沉降时间到了
                if remaining <= 0:
                    break
                
                await asyncio.sleep(1.0)
        
        # === 操作完成 ===
        settling_info = f" | 沉降: {settling_time:.0f}s" if settling_time > 0 else ""
        self.data.update({
            "status": f"完成: 搅拌 {stir_speed} RPM, {stir_time:.0f}s{settling_info}",
            "operation_mode": "Completed",
            "current_speed": 0.0,
            "is_stirring": False,
            "remaining_time": 0.0,
        })
        
        self.logger.info(f"Stir completed: {stir_speed} RPM for {stir_time}s + settling {settling_time}s")
        return True
    
    async def start_stir(self, vessel: str, stir_speed: float, purpose: str) -> bool:
        """Start stir action - 开始持续搅拌"""
        self.logger.info(f"StartStir: vessel={vessel}, speed={stir_speed} RPM, purpose={purpose}")
        
        # 验证参数
        if stir_speed > self._max_speed or stir_speed < self._min_speed:
            error_msg = f"搅拌速度 {stir_speed} RPM 超出范围 ({self._min_speed} - {self._max_speed} RPM)"
            self.logger.error(error_msg)
            self.data.update({
                "status": f"Error: {error_msg}",
                "operation_mode": "Error"
            })
            return False
        
        self.data.update({
            "status": f"启动: 持续搅拌 {vessel} at {stir_speed} RPM | {purpose}",
            "operation_mode": "Stirring",
            "current_vessel": vessel,
            "current_speed": stir_speed,
            "is_stirring": True,
            "remaining_time": -1.0,  # -1 表示持续运行
        })
        
        return True
    
    async def stop_stir(self, vessel: str) -> bool:
        """Stop stir action - 停止搅拌"""
        self.logger.info(f"StopStir: vessel={vessel}")
        
        current_speed = self.data.get("current_speed", 0.0)
        
        self.data.update({
            "status": f"已停止: {vessel} 搅拌停止 | 之前速度: {current_speed} RPM",
            "operation_mode": "Stopped",
            "current_vessel": "",
            "current_speed": 0.0,
            "is_stirring": False,
            "remaining_time": 0.0,
        })
        
        return True
    
    # 状态属性
    @property
    def status(self) -> str:
        return self.data.get("status", "Idle")
    
    @property
    def operation_mode(self) -> str:
        return self.data.get("operation_mode", "Idle")
    
    @property
    def current_vessel(self) -> str:
        return self.data.get("current_vessel", "")
    
    @property
    def current_speed(self) -> float:
        return self.data.get("current_speed", 0.0)
    
    @property
    def is_stirring(self) -> bool:
        return self.data.get("is_stirring", False)
    
    @property
    def remaining_time(self) -> float:
        return self.data.get("remaining_time", 0.0)