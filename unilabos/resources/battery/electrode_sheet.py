"""
电极片类定义
Electrode Sheet Resource Classes
"""

from __future__ import annotations
from typing import Any, Dict, Optional
from pylabrobot.resources.resource import Resource


class ElectrodeSheet(Resource):
    """电极片类 - 用于纽扣电池组装"""
    
    def __init__(
        self,
        name: str,
        size_x: float = 12.0,
        size_y: float = 12.0,
        size_z: float = 0.1,
        category: str = "electrode_sheet",
        electrode_type: str = "anode",  # "anode" 负极, "cathode" 正极, "separator" 隔膜
        **kwargs
    ):
        """初始化电极片
        
        Args:
            name: 电极片名称
            size_x: X方向尺寸 (mm)
            size_y: Y方向尺寸 (mm)
            size_z: Z方向尺寸/厚度 (mm)
            category: 类别
            electrode_type: 电极类型
        """
        super().__init__(
            name=name,
            size_x=size_x,
            size_y=size_y,
            size_z=size_z,
            category=category,
            **kwargs
        )
        self._electrode_type = electrode_type
        self._unilabos_state: Dict[str, Any] = {
            "electrode_type": electrode_type,
            "material": "",
            "thickness": size_z,
        }
    
    @property
    def electrode_type(self) -> str:
        """获取电极类型"""
        return self._electrode_type
    
    def load_state(self, state: Dict[str, Any]) -> None:
        """加载状态"""
        super().load_state(state)
        if isinstance(state, dict):
            self._unilabos_state.update(state)
    
    def serialize_state(self) -> Dict[str, Any]:
        """序列化状态"""
        data = super().serialize_state()
        data.update(self._unilabos_state)
        return data



