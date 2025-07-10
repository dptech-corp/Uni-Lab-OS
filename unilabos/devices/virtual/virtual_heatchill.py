import asyncio
import logging
import time as time_module  # 重命名time模块，避免与参数冲突
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
        
        # 从config或kwargs中获取配置参数
        self.port = self.config.get('port') or kwargs.get('port', 'VIRTUAL')
        self._max_temp = self.config.get('max_temp') or kwargs.get('max_temp', 200.0)
        self._min_temp = self.config.get('min_temp') or kwargs.get('min_temp', -80.0)
        self._max_stir_speed = self.config.get('max_stir_speed') or kwargs.get('max_stir_speed', 1000.0)
        
        # 处理其他kwargs参数
        skip_keys = {'port', 'max_temp', 'min_temp', 'max_stir_speed'}
        for key, value in kwargs.items():
            if key not in skip_keys and not hasattr(self, key):
                setattr(self, key, value)
    
    async def initialize(self) -> bool:
        """Initialize virtual heat chill"""
        self.logger.info(f"Initializing virtual heat chill {self.device_id}")
        
        # 初始化状态信息
        self.data.update({
            "status": "Idle",
            "operation_mode": "Idle",
            "is_stirring": False,
            "stir_speed": 0.0,
            "remaining_time": 0.0,
        })
        return True
    
    async def cleanup(self) -> bool:
        """Cleanup virtual heat chill"""
        self.logger.info(f"Cleaning up virtual heat chill {self.device_id}")
        self.data.update({
            "status": "Offline",
            "operation_mode": "Offline",
            "is_stirring": False,
            "stir_speed": 0.0,
            "remaining_time": 0.0
        })
        return True
    
    async def heat_chill(self, vessel: str, temp: float, time, stir: bool, 
                        stir_speed: float, purpose: str) -> bool:
        """Execute heat chill action - 🔧 修复：确保参数类型正确"""
        
        # 🔧 关键修复：确保所有参数类型正确
        try:
            temp = float(temp)
            time_value = float(time)  # 强制转换为浮点数
            stir_speed = float(stir_speed)
            stir = bool(stir)
            vessel = str(vessel)
            purpose = str(purpose)
        except (ValueError, TypeError) as e:
            error_msg = f"参数类型转换错误: temp={temp}({type(temp)}), time={time}({type(time)}), error={str(e)}"
            self.logger.error(error_msg)
            self.data.update({
                "status": f"Error: {error_msg}",
                "operation_mode": "Error"
            })
            return False
        
        self.logger.info(f"HeatChill: vessel={vessel}, temp={temp}°C, time={time_value}s, stir={stir}, stir_speed={stir_speed}")
        
        # 验证参数范围
        if temp > self._max_temp or temp < self._min_temp:
            error_msg = f"温度 {temp}°C 超出范围 ({self._min_temp}°C - {self._max_temp}°C)"
            self.logger.error(error_msg)
            self.data.update({
                "status": f"Error: {error_msg}",
                "operation_mode": "Error"
            })
            return False
        
        if stir and stir_speed > self._max_stir_speed:
            error_msg = f"搅拌速度 {stir_speed} RPM 超出最大值 {self._max_stir_speed} RPM"
            self.logger.error(error_msg)
            self.data.update({
                "status": f"Error: {error_msg}",
                "operation_mode": "Error"
            })
            return False
        
        if time_value <= 0:
            error_msg = f"时间 {time_value}s 必须大于0"
            self.logger.error(error_msg)
            self.data.update({
                "status": f"Error: {error_msg}",
                "operation_mode": "Error"
            })
            return False
        
        # 确定操作模式
        if temp > 25.0:
            operation_mode = "Heating"
            status_action = "加热"
        elif temp < 25.0:
            operation_mode = "Cooling"
            status_action = "冷却"
        else:
            operation_mode = "Maintaining"
            status_action = "保温"
        
        # 🔧 修复：使用转换后的时间值
        start_time = time_module.time()
        total_time = time_value  # 使用转换后的浮点数
        
        # 开始操作
        stir_info = f" | 搅拌: {stir_speed} RPM" if stir else ""
        self.data.update({
            "status": f"运行中: {status_action} {vessel} 至 {temp}°C | 剩余: {total_time:.0f}s{stir_info}",
            "operation_mode": operation_mode,
            "is_stirring": stir,
            "stir_speed": stir_speed if stir else 0.0,
            "remaining_time": total_time,
        })
        
        # 在等待过程中每秒更新剩余时间
        while True:
            current_time = time_module.time()
            elapsed = current_time - start_time
            remaining = max(0, total_time - elapsed)
            
            # 更新剩余时间和状态
            self.data.update({
                "remaining_time": remaining,
                "status": f"运行中: {status_action} {vessel} 至 {temp}°C | 剩余: {remaining:.0f}s{stir_info}"
            })
            
            # 如果时间到了，退出循环
            if remaining <= 0:
                break
            
            # 等待1秒后再次检查
            await asyncio.sleep(1.0)
        
        # 操作完成
        final_stir_info = f" | 搅拌: {stir_speed} RPM" if stir else ""
        self.data.update({
            "status": f"完成: {vessel} 已达到 {temp}°C | 用时: {total_time:.0f}s{final_stir_info}",
            "operation_mode": "Completed",
            "remaining_time": 0.0,
            "is_stirring": False,
            "stir_speed": 0.0
        })
        
        self.logger.info(f"HeatChill completed for vessel {vessel} at {temp}°C after {total_time}s")
        return True
    
    async def heat_chill_start(self, vessel: str, temp: float, purpose: str) -> bool:
        """Start continuous heat chill"""
        
        # 🔧 添加类型转换
        try:
            temp = float(temp)
            vessel = str(vessel)
            purpose = str(purpose)
        except (ValueError, TypeError) as e:
            error_msg = f"参数类型转换错误: {str(e)}"
            self.logger.error(error_msg)
            self.data.update({
                "status": f"Error: {error_msg}",
                "operation_mode": "Error"
            })
            return False
        
        self.logger.info(f"HeatChillStart: vessel={vessel}, temp={temp}°C")
        
        # 验证参数
        if temp > self._max_temp or temp < self._min_temp:
            error_msg = f"温度 {temp}°C 超出范围 ({self._min_temp}°C - {self._max_temp}°C)"
            self.logger.error(error_msg)
            self.data.update({
                "status": f"Error: {error_msg}",
                "operation_mode": "Error"
            })
            return False
        
        # 确定操作模式
        if temp > 25.0:
            operation_mode = "Heating"
            status_action = "持续加热"
        elif temp < 25.0:
            operation_mode = "Cooling"
            status_action = "持续冷却"
        else:
            operation_mode = "Maintaining"
            status_action = "恒温保持"
        
        self.data.update({
            "status": f"启动: {status_action} {vessel} 至 {temp}°C | 持续运行",
            "operation_mode": operation_mode,
            "is_stirring": False,
            "stir_speed": 0.0,
            "remaining_time": -1.0,  # -1 表示持续运行
        })
        
        return True
    
    async def heat_chill_stop(self, vessel: str) -> bool:
        """Stop heat chill"""
        
        # 🔧 添加类型转换
        try:
            vessel = str(vessel)
        except (ValueError, TypeError) as e:
            error_msg = f"参数类型转换错误: {str(e)}"
            self.logger.error(error_msg)
            return False
        
        self.logger.info(f"HeatChillStop: vessel={vessel}")
        
        self.data.update({
            "status": f"已停止: {vessel} 温控停止",
            "operation_mode": "Stopped",
            "is_stirring": False,
            "stir_speed": 0.0,
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
    def is_stirring(self) -> bool:
        return self.data.get("is_stirring", False)
    
    @property
    def stir_speed(self) -> float:
        return self.data.get("stir_speed", 0.0)
    
    @property
    def remaining_time(self) -> float:
        return self.data.get("remaining_time", 0.0)