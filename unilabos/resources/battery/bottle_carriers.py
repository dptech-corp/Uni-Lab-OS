"""
瓶架类定义 - 用于纽扣电池组装工作站
Bottle Carrier Resource Classes
"""

from __future__ import annotations
from pylabrobot.resources import ResourceHolder
from pylabrobot.resources.utils import create_ordered_items_2d
from unilabos.resources.itemized_carrier import ItemizedCarrier


def YIHUA_Electrolyte_12VialCarrier(name: str) -> ItemizedCarrier:
    """依华电解液12瓶架 - 3x4布局
    
    Args:
        name: 瓶架名称
        
    Returns:
        ItemizedCarrier: 包含12个瓶位的瓶架
    """
    sites = create_ordered_items_2d(
        klass=ResourceHolder,
        num_items_x=4,
        num_items_y=3,
        dx=10.0,
        dy=10.0,
        dz=5.0,
        item_dx=70.0,
        item_dy=26.67,
        size_x=60.0,
        size_y=20.0,
        size_z=70.0,
    )
    
    return ItemizedCarrier(
        name=name,
        size_x=300.0,
        size_y=100.0,
        size_z=80.0,
        num_items_x=4,
        num_items_y=3,
        sites=sites,
        category="bottle_carrier",
    )

