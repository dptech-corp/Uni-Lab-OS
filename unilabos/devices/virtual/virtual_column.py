import asyncio
import logging
from typing import Dict, Any, Optional

class VirtualColumn:
    """Virtual column device for RunColumn protocol 🏛️"""
    
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
        
        print(f"🏛️ === 虚拟色谱柱 {self.device_id} 已创建 === ✨")
        print(f"📏 柱参数: 流速={self._max_flow_rate}mL/min | 长度={self._column_length}cm | 直径={self._column_diameter}cm 🔬")
    
    async def initialize(self) -> bool:
        """Initialize virtual column 🚀"""
        self.logger.info(f"🔧 初始化虚拟色谱柱 {self.device_id} ✨")
        
        self.data.update({
            "status": "Idle",
            "column_state": "Ready", 
            "current_flow_rate": 0.0,
            "max_flow_rate": self._max_flow_rate,
            "column_length": self._column_length,
            "column_diameter": self._column_diameter,
            "processed_volume": 0.0,
            "progress": 0.0,
            "current_status": "Ready for separation"
        })
        
        self.logger.info(f"✅ 色谱柱 {self.device_id} 初始化完成 🏛️")
        self.logger.info(f"📊 设备规格: 最大流速 {self._max_flow_rate}mL/min | 柱长 {self._column_length}cm 📏")
        return True
    
    async def cleanup(self) -> bool:
        """Cleanup virtual column 🧹"""
        self.logger.info(f"🧹 清理虚拟色谱柱 {self.device_id} 🔚")
        
        self.data.update({
            "status": "Offline",
            "column_state": "Offline",
            "current_status": "System offline"
        })
        
        self.logger.info(f"✅ 色谱柱 {self.device_id} 清理完成 💤")
        return True
    
    async def run_column(self, from_vessel: str, to_vessel: str, column: str, **kwargs) -> bool:
        """Execute column chromatography run - matches RunColumn action 🏛️"""
        
        # 提取额外参数
        rf = kwargs.get('rf', '0.3')
        solvent1 = kwargs.get('solvent1', 'ethyl_acetate')
        solvent2 = kwargs.get('solvent2', 'hexane')
        ratio = kwargs.get('ratio', '30:70')
        
        self.logger.info(f"🏛️ 开始柱层析分离: {from_vessel} → {to_vessel} 🚰")
        self.logger.info(f"  🧪 使用色谱柱: {column}")
        self.logger.info(f"  🎯 Rf值: {rf}")
        self.logger.info(f"  🧪 洗脱溶剂: {solvent1}:{solvent2} ({ratio}) 💧")
        
        # 更新设备状态
        self.data.update({
            "status": "Running",
            "column_state": "Separating",
            "current_status": "🏛️ Column separation in progress",
            "progress": 0.0,
            "processed_volume": 0.0,
            "current_from_vessel": from_vessel,
            "current_to_vessel": to_vessel,
            "current_column": column,
            "current_rf": rf,
            "current_solvents": f"{solvent1}:{solvent2} ({ratio})"
        })
        
        # 模拟柱层析分离过程
        # 假设处理时间基于流速和柱子长度
        base_time = (self._column_length * 2) / self._max_flow_rate  # 简化计算
        separation_time = max(base_time, 20.0)  # 最少20秒
        
        self.logger.info(f"⏱️ 预计分离时间: {separation_time:.1f}秒 ⌛")
        self.logger.info(f"📏 柱参数: 长度 {self._column_length}cm | 流速 {self._max_flow_rate}mL/min 🌊")
        
        steps = 20  # 分20个步骤模拟分离过程
        step_time = separation_time / steps
        
        for i in range(steps):
            await asyncio.sleep(step_time)
            
            progress = (i + 1) / steps * 100
            volume_processed = (i + 1) * 5.0  # 假设每步处理5mL
            
            # 不同阶段的状态描述
            if progress <= 25:
                phase = "🌊 样品上柱阶段"
                phase_emoji = "📥"
            elif progress <= 50:
                phase = "🧪 洗脱开始"
                phase_emoji = "💧"
            elif progress <= 75:
                phase = "⚗️ 成分分离中"
                phase_emoji = "🔄"
            else:
                phase = "🎯 收集产物"
                phase_emoji = "📤"
            
            # 更新状态
            status_msg = f"{phase_emoji} {phase}: {progress:.1f}% | 💧 已处理: {volume_processed:.1f}mL"
            
            self.data.update({
                "progress": progress,
                "processed_volume": volume_processed,
                "current_status": status_msg,
                "current_phase": phase
            })
            
            # 进度日志（每25%打印一次）
            if progress >= 25 and (i + 1) % 5 == 0:  # 每5步（25%）打印一次
                self.logger.info(f"📊 分离进度: {progress:.0f}% | {phase} | 💧 {volume_processed:.1f}mL 完成 ✨")
        
        # 分离完成
        final_status = f"✅ 柱层析分离完成: {from_vessel} → {to_vessel} | 💧 共处理 {volume_processed:.1f}mL"
        
        self.data.update({
            "status": "Idle",
            "column_state": "Ready",
            "current_status": final_status,
            "progress": 100.0,
            "final_volume": volume_processed
        })
        
        self.logger.info(f"🎉 柱层析分离完成! ✨")
        self.logger.info(f"📊 分离结果:")
        self.logger.info(f"  🥽 源容器: {from_vessel}")
        self.logger.info(f"  🥽 目标容器: {to_vessel}")
        self.logger.info(f"  🏛️ 使用色谱柱: {column}")
        self.logger.info(f"  💧 处理体积: {volume_processed:.1f}mL")
        self.logger.info(f"  🧪 洗脱条件: {solvent1}:{solvent2} ({ratio})")
        self.logger.info(f"  🎯 Rf值: {rf}")
        self.logger.info(f"  ⏱️ 总耗时: {separation_time:.1f}秒 🏁")
        
        return True
    
    # 状态属性
    @property
    def status(self) -> str:
        return self.data.get("status", "❓ Unknown")
    
    @property
    def column_state(self) -> str:
        return self.data.get("column_state", "❓ Unknown")
    
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
        return self.data.get("current_status", "📋 Ready")
    
    @property
    def current_phase(self) -> str:
        return self.data.get("current_phase", "🏠 待机中")
    
    @property
    def final_volume(self) -> float:
        return self.data.get("final_volume", 0.0)