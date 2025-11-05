from typing import Dict, List, Optional, OrderedDict, Union
import math

from pylabrobot.resources.coordinate import Coordinate
from pylabrobot.resources import Resource, ResourceStack, ItemizedResource
from pylabrobot.resources.carrier import create_homogeneous_resources


class Magazine(ResourceStack):
    """子弹夹洞位类"""

    def __init__(
        self,
        name: str,
        direction: str = 'z',
        resources: Optional[List[Resource]] = None,
        max_sheets: int = 100,
        **kwargs
    ):
        """初始化子弹夹洞位

        Args:
            name: 洞位名称
            direction: 堆叠方向
            resources: 资源列表
            max_sheets: 最大极片数量
        """
        super().__init__(
            name=name,
            direction=direction,
            resources=resources,
        )
        self.max_sheets = max_sheets


class MagazineHolder(ItemizedResource):
    """子弹夹类 - 有多个洞位，每个洞位放多个极片"""

    def __init__(
        self,
        name: str,
        size_x: float,
        size_y: float,
        size_z: float,
        ordered_items: Optional[Dict[str, Magazine]] = None,
        ordering: Optional[OrderedDict[str, str]] = None,
        hole_diameter: float = 14.0,
        hole_depth: float = 10.0,
        max_sheets_per_hole: int = 100,
        cross_section_type: str = "circle",
        category: str = "magazine_holder",
        model: Optional[str] = None,
    ):
        """初始化子弹夹

        Args:
            name: 子弹夹名称
            size_x: 长度 (mm)
            size_y: 宽度 (mm)
            size_z: 高度 (mm)
            hole_diameter: 洞直径 (mm)
            hole_depth: 洞深度 (mm)
            max_sheets_per_hole: 每个洞位最大极片数量
            category: 类别
            model: 型号
        """

        super().__init__(
            name=name,
            size_x=size_x,
            size_y=size_y,
            size_z=size_z,
            ordered_items=ordered_items,
            ordering=ordering,
            category=category,
            model=model,
        )

        # 保存洞位的直径和深度
        self.hole_diameter = hole_diameter
        self.hole_depth = hole_depth
        self.max_sheets_per_hole = max_sheets_per_hole
        self.cross_section_type = cross_section_type

    def serialize(self) -> dict:
        return {
            **super().serialize(),
            "hole_diameter": self.hole_diameter,
            "hole_depth": self.hole_depth,
            "max_sheets_per_hole": self.max_sheets_per_hole,
            "cross_section_type": self.cross_section_type,
        }


def magazine_factory(
    name: str,
    size_x: float,
    size_y: float,
    size_z: float,
    locations: List[Coordinate],
    hole_diameter: float = 14.0,
    hole_depth: float = 10.0,
    max_sheets_per_hole: int = 100,
    category: str = "magazine_holder",
    model: Optional[str] = None,
) -> 'MagazineHolder':
    """工厂函数：创建子弹夹
    
    Args:
        name: 子弹夹名称
        size_x: 长度 (mm)
        size_y: 宽度 (mm)
        size_z: 高度 (mm)
        locations: 洞位坐标列表
        hole_diameter: 洞直径 (mm)
        hole_depth: 洞深度 (mm)
        max_sheets_per_hole: 每个洞位最大极片数量
        category: 类别
        model: 型号
    """
    # 创建洞位
    _sites = create_homogeneous_resources(
        klass=Magazine,
        locations=locations,
        resource_size_x=hole_diameter,
        resource_size_y=hole_diameter,
        name_prefix=name,
        max_sheets=max_sheets_per_hole,
    )
    
    # 生成编号键
    keys = [f"A{i+1}" for i in range(len(locations))]
    sites = dict(zip(keys, _sites.values()))
    
    return MagazineHolder(
        name=name,
        size_x=size_x,
        size_y=size_y,
        size_z=size_z,
        ordered_items=sites,
        hole_diameter=hole_diameter,
        hole_depth=hole_depth,
        max_sheets_per_hole=max_sheets_per_hole,
        category=category,
        model=model,
    )


def MagazineHolder_4(
    name: str,
    size_x: float = 80.0,
    size_y: float = 80.0,
    size_z: float = 10.0,
    hole_diameter: float = 14.0,
    hole_depth: float = 10.0,
    hole_spacing: float = 25.0,
    max_sheets_per_hole: int = 100,
) -> MagazineHolder:
    """创建4孔子弹夹 - 正方形四角排布"""
    # 计算4个洞位的坐标（正方形四角排布）
    center_x = size_x / 2
    center_y = size_y / 2
    offset = hole_spacing / 2
    
    locations = [
        Coordinate(center_x - offset, center_y - offset, size_z - hole_depth),  # 左下
        Coordinate(center_x + offset, center_y - offset, size_z - hole_depth),  # 右下
        Coordinate(center_x - offset, center_y + offset, size_z - hole_depth),  # 左上
        Coordinate(center_x + offset, center_y + offset, size_z - hole_depth),  # 右上
    ]
    
    return magazine_factory(
        name=name,
        size_x=size_x,
        size_y=size_y,
        size_z=size_z,
        locations=locations,
        hole_diameter=hole_diameter,
        hole_depth=hole_depth,
        max_sheets_per_hole=max_sheets_per_hole,
        category="clip_magazine_four",
    )


def MagazineHolder_2(
    name: str,
    size_x: float = 80.0,
    size_y: float = 80.0,
    size_z: float = 10.0,
    hole_diameter: float = 14.0,
    hole_depth: float = 10.0,
    hole_spacing: float = 25.0,
    max_sheets_per_hole: int = 100,
) -> MagazineHolder:
    """创建2孔子弹夹 - 竖向排布"""
    # 计算2个洞位的坐标（竖向排布）
    center_x = size_x / 2
    center_y = size_y / 2
    offset = hole_spacing / 2
    
    locations = [
        Coordinate(center_x, center_y - offset, size_z - hole_depth),  # 下方
        Coordinate(center_x, center_y + offset, size_z - hole_depth),  # 上方
    ]
    
    return magazine_factory(
        name=name,
        size_x=size_x,
        size_y=size_y,
        size_z=size_z,
        locations=locations,
        hole_diameter=hole_diameter,
        hole_depth=hole_depth,
        max_sheets_per_hole=max_sheets_per_hole,
        category="clip_magazine_two",
    )


def MagazineHolder_1(
    name: str,
    size_x: float = 80.0,
    size_y: float = 80.0,
    size_z: float = 10.0,
    hole_diameter: float = 14.0,
    hole_depth: float = 10.0,
    max_sheets_per_hole: int = 100,
) -> MagazineHolder:
    """创建1孔子弹夹 - 中心单孔"""
    # 计算1个洞位的坐标（中心位置）
    center_x = size_x / 2
    center_y = size_y / 2
    
    locations = [
        Coordinate(center_x, center_y, size_z - hole_depth),  # 中心
    ]
    
    return magazine_factory(
        name=name,
        size_x=size_x,
        size_y=size_y,
        size_z=size_z,
        locations=locations,
        hole_diameter=hole_diameter,
        hole_depth=hole_depth,
        max_sheets_per_hole=max_sheets_per_hole,
        category="clip_magazine_one",
    )


def MagazineHolder_6(
    name: str,
    size_x: float = 80.0,
    size_y: float = 80.0,
    size_z: float = 40.0,
    hole_diameter: float = 14.0,
    hole_depth: float = 10.0,
    hole_spacing: float = 20.0,
    max_sheets_per_hole: int = 100,
) -> MagazineHolder:
    """创建6孔子弹夹 - 六边形排布"""
    # 计算6个洞位的坐标（六边形排布：中心1个，周围5个）
    center_x = size_x / 2
    center_y = size_y / 2
    
    locations = []
    
    # 周围6个孔，按六边形排布
    for i in range(6):
        angle = i * 60 * math.pi / 180  # 每60度一个孔
        x = center_x + hole_spacing * math.cos(angle)
        y = center_y + hole_spacing * math.sin(angle)
        locations.append(Coordinate(x, y, size_z - hole_depth))
    
    return magazine_factory(
        name=name,
        size_x=size_x,
        size_y=size_y,
        size_z=size_z,
        locations=locations,
        hole_diameter=hole_diameter,
        hole_depth=hole_depth,
        max_sheets_per_hole=max_sheets_per_hole,
        category="clip_magazine_six",
    )