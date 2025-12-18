"""
弹夹架类定义 - 用于纽扣电池组装工作站
Magazine Holder Resource Classes
"""

from __future__ import annotations
from typing import List, Optional
from pylabrobot.resources.coordinate import Coordinate
from pylabrobot.resources import ResourceHolder
from pylabrobot.resources.utils import create_ordered_items_2d
from unilabos.resources.itemized_carrier import ItemizedCarrier


def MagazineHolder_4_Cathode(name: str) -> ItemizedCarrier:
    """正极&铝箔弹夹 - 4个洞位 (2x2布局)
    
    Args:
        name: 弹夹名称
        
    Returns:
        ItemizedCarrier: 包含4个槽位的弹夹架
    """
    sites = create_ordered_items_2d(
        klass=ResourceHolder,
        num_items_x=2,
        num_items_y=2,
        dx=10.0,
        dy=10.0,
        dz=0.0,
        item_dx=50.0,
        item_dy=30.0,
        size_x=40.0,
        size_y=25.0,
        size_z=40.0,
    )
    
    return ItemizedCarrier(
        name=name,
        size_x=120.0,
        size_y=80.0,
        size_z=50.0,
        num_items_x=2,
        num_items_y=2,
        sites=sites,
        category="magazine_holder",
    )


def MagazineHolder_6_Cathode(name: str) -> ItemizedCarrier:
    """正极壳&平垫片弹夹 - 6个洞位 (2x3布局)
    
    Args:
        name: 弹夹名称
        
    Returns:
        ItemizedCarrier: 包含6个槽位的弹夹架
    """
    sites = create_ordered_items_2d(
        klass=ResourceHolder,
        num_items_x=3,
        num_items_y=2,
        dx=10.0,
        dy=10.0,
        dz=0.0,
        item_dx=40.0,
        item_dy=30.0,
        size_x=35.0,
        size_y=25.0,
        size_z=40.0,
    )
    
    return ItemizedCarrier(
        name=name,
        size_x=150.0,
        size_y=80.0,
        size_z=50.0,
        num_items_x=3,
        num_items_y=2,
        sites=sites,
        category="magazine_holder",
    )


def MagazineHolder_6_Anode(name: str) -> ItemizedCarrier:
    """负极壳&弹垫片弹夹 - 6个洞位 (2x3布局)
    
    Args:
        name: 弹夹名称
        
    Returns:
        ItemizedCarrier: 包含6个槽位的弹夹架
    """
    sites = create_ordered_items_2d(
        klass=ResourceHolder,
        num_items_x=3,
        num_items_y=2,
        dx=10.0,
        dy=10.0,
        dz=0.0,
        item_dx=40.0,
        item_dy=30.0,
        size_x=35.0,
        size_y=25.0,
        size_z=40.0,
    )
    
    return ItemizedCarrier(
        name=name,
        size_x=150.0,
        size_y=80.0,
        size_z=50.0,
        num_items_x=3,
        num_items_y=2,
        sites=sites,
        category="magazine_holder",
    )


def MagazineHolder_6_Battery(name: str) -> ItemizedCarrier:
    """成品弹夹 - 6个洞位 (3x2布局)
    
    Args:
        name: 弹夹名称
        
    Returns:
        ItemizedCarrier: 包含6个槽位的弹夹架
    """
    sites = create_ordered_items_2d(
        klass=ResourceHolder,
        num_items_x=3,
        num_items_y=2,
        dx=10.0,
        dy=10.0,
        dz=0.0,
        item_dx=33.0,
        item_dy=40.0,
        size_x=30.0,
        size_y=35.0,
        size_z=40.0,
    )
    
    return ItemizedCarrier(
        name=name,
        size_x=120.0,
        size_y=100.0,
        size_z=50.0,
        num_items_x=3,
        num_items_y=2,
        sites=sites,
        category="magazine_holder",
    )

