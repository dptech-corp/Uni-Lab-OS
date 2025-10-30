from pylabrobot.resources import create_homogeneous_resources, Coordinate, ResourceHolder, create_ordered_items_2d

from unilabos.resources.itemized_carrier import Bottle, BottleCarrier
from unilabos.resources.bioyond.bottles import (
    BIOYOND_PolymerStation_Solid_Stock,
    BIOYOND_PolymerStation_Solid_Vial,
    BIOYOND_PolymerStation_Liquid_Vial,
    BIOYOND_PolymerStation_Solution_Beaker,
    BIOYOND_PolymerStation_Reagent_Bottle,
    # 配液站专用
    BIOYOND_DispensingStation_Solid_Stock,
    BIOYOND_DispensingStation_Solid_Vial,
    BIOYOND_DispensingStation_Liquid_Vial,
    # 反应站专用
    BIOYOND_ReactionStation_Reactor,
    BIOYOND_ReactionStation_Solid_Vial,
    BIOYOND_ReactionStation_Liquid_Vial,
)
# 命名约定：试剂瓶-Bottle，烧杯-Beaker，烧瓶-Flask,小瓶-Vial


def BIOYOND_Electrolyte_6VialCarrier(name: str) -> BottleCarrier:
    """6瓶载架 - 2x3布局"""

    # 载架尺寸 (mm)
    carrier_size_x = 127.8
    carrier_size_y = 85.5
    carrier_size_z = 50.0

    # 瓶位尺寸
    bottle_diameter = 30.0
    bottle_spacing_x = 42.0  # X方向间距
    bottle_spacing_y = 35.0  # Y方向间距

    # 计算起始位置 (居中排列)
    start_x = (carrier_size_x - (3 - 1) * bottle_spacing_x - bottle_diameter) / 2
    start_y = (carrier_size_y - (2 - 1) * bottle_spacing_y - bottle_diameter) / 2

    sites = create_ordered_items_2d(
        klass=ResourceHolder,
        num_items_x=3,
        num_items_y=2,
        dx=start_x,
        dy=start_y,
        dz=5.0,
        item_dx=bottle_spacing_x,
        item_dy=bottle_spacing_y,

        size_x=bottle_diameter,
        size_y=bottle_diameter,
        size_z=carrier_size_z,
    )
    for k, v in sites.items():
        v.name = f"{name}_{v.name}"

    carrier = BottleCarrier(
        name=name,
        size_x=carrier_size_x,
        size_y=carrier_size_y,
        size_z=carrier_size_z,
        sites=sites,
        model="BIOYOND_Electrolyte_6VialCarrier",
    )
    carrier.num_items_x = 3
    carrier.num_items_y = 2
    carrier.num_items_z = 1
    for i in range(6):
        carrier[i] = BIOYOND_PolymerStation_Solid_Vial(f"{name}_vial_{i+1}")
    return carrier


def BIOYOND_Electrolyte_1BottleCarrier(name: str) -> BottleCarrier:
    """1瓶载架 - 单个中央位置"""

    # 载架尺寸 (mm)
    carrier_size_x = 127.8
    carrier_size_y = 85.5
    carrier_size_z = 100.0

    # 烧杯尺寸
    beaker_diameter = 80.0

    # 计算中央位置
    center_x = (carrier_size_x - beaker_diameter) / 2
    center_y = (carrier_size_y - beaker_diameter) / 2
    center_z = 5.0

    carrier = BottleCarrier(
        name=name,
        size_x=carrier_size_x,
        size_y=carrier_size_y,
        size_z=carrier_size_z,
        sites=create_homogeneous_resources(
            klass=ResourceHolder,
            locations=[Coordinate(center_x, center_y, center_z)],
            resource_size_x=beaker_diameter,
            resource_size_y=beaker_diameter,
            name_prefix=name,
        ),
        model="BIOYOND_Electrolyte_1BottleCarrier",
    )
    carrier.num_items_x = 1
    carrier.num_items_y = 1
    carrier.num_items_z = 1
    carrier[0] = BIOYOND_PolymerStation_Solution_Beaker(f"{name}_beaker_1")
    return carrier


def BIOYOND_PolymerStation_6StockCarrier(name: str) -> BottleCarrier:
    """[已弃用] 请使用 BIOYOND_ReactionStation_6StockCarrier"""
    return BIOYOND_ReactionStation_6StockCarrier(name)


def BIOYOND_ReactionStation_6StockCarrier(name: str) -> BottleCarrier:
    """反应站-6孔样品板 - 2x3布局"""

    # 载架尺寸 (mm)
    carrier_size_x = 127.8
    carrier_size_y = 85.5
    carrier_size_z = 50.0

    # 瓶位尺寸
    bottle_diameter = 20.0
    bottle_spacing_x = 42.0  # X方向间距
    bottle_spacing_y = 35.0  # Y方向间距

    # 计算起始位置 (居中排列)
    start_x = (carrier_size_x - (3 - 1) * bottle_spacing_x - bottle_diameter) / 2
    start_y = (carrier_size_y - (2 - 1) * bottle_spacing_y - bottle_diameter) / 2

    sites = create_ordered_items_2d(
        klass=ResourceHolder,
        num_items_x=3,
        num_items_y=2,
        dx=start_x,
        dy=start_y,
        dz=5.0,
        item_dx=bottle_spacing_x,
        item_dy=bottle_spacing_y,

        size_x=bottle_diameter,
        size_y=bottle_diameter,
        size_z=carrier_size_z,
    )
    for k, v in sites.items():
        v.name = f"{name}_{v.name}"

    carrier = BottleCarrier(
        name=name,
        size_x=carrier_size_x,
        size_y=carrier_size_y,
        size_z=carrier_size_z,
        sites=sites,
        model="BIOYOND_ReactionStation_6StockCarrier",
    )
    carrier.num_items_x = 3
    carrier.num_items_y = 2
    carrier.num_items_z = 1
    ordering = ["A1", "A2", "A3", "B1", "B2", "B3"]  # 自定义顺序
    # 反应站6孔板: 第一排使用Liquid_Vial (10%分装小瓶), 第二排使用Solid_Vial (90%分装小瓶)
    for i in range(3):
        carrier[i] = BIOYOND_ReactionStation_Liquid_Vial(f"{name}_vial_{ordering[i]}")
    for i in range(3, 6):
        carrier[i] = BIOYOND_ReactionStation_Solid_Vial(f"{name}_vial_{ordering[i]}")
    return carrier


def BIOYOND_PolymerStation_8StockCarrier(name: str) -> BottleCarrier:
    """[已弃用] 请使用 BIOYOND_DispensingStation_8StockCarrier"""
    return BIOYOND_DispensingStation_8StockCarrier(name)


def BIOYOND_DispensingStation_8StockCarrier(name: str) -> BottleCarrier:
    """配液站-8孔样品板 - 2x4布局"""

    # 载架尺寸 (mm)
    carrier_size_x = 170.0
    carrier_size_y = 85.5
    carrier_size_z = 50.0

    # 瓶位尺寸
    bottle_diameter = 20.0
    bottle_spacing_x = 42.0  # X方向间距
    bottle_spacing_y = 35.0  # Y方向间距

    # 计算起始位置 (居中排列)
    start_x = (carrier_size_x - (4 - 1) * bottle_spacing_x - bottle_diameter) / 2
    start_y = (carrier_size_y - (2 - 1) * bottle_spacing_y - bottle_diameter) / 2

    sites = create_ordered_items_2d(
        klass=ResourceHolder,
        num_items_x=4,
        num_items_y=2,
        dx=start_x,
        dy=start_y,
        dz=5.0,
        item_dx=bottle_spacing_x,
        item_dy=bottle_spacing_y,

        size_x=bottle_diameter,
        size_y=bottle_diameter,
        size_z=carrier_size_z,
    )
    for k, v in sites.items():
        v.name = f"{name}_{v.name}"

    carrier = BottleCarrier(
        name=name,
        size_x=carrier_size_x,
        size_y=carrier_size_y,
        size_z=carrier_size_z,
        sites=sites,
        model="BIOYOND_DispensingStation_8StockCarrier",
    )
    carrier.num_items_x = 4
    carrier.num_items_y = 2
    carrier.num_items_z = 1
    ordering = ["A1", "A2", "A3", "A4", "B1", "B2", "B3", "B4"]
    for i in range(8):
        carrier[i] = BIOYOND_DispensingStation_Solid_Stock(f"{name}_vial_{ordering[i]}")
    return carrier




def BIOYOND_PolymerStation_6VialCarrier(name: str) -> BottleCarrier:
    """[已弃用] 请使用 BIOYOND_DispensingStation_6VialCarrier"""
    return BIOYOND_DispensingStation_6VialCarrier(name)


def BIOYOND_DispensingStation_6VialCarrier(name: str) -> BottleCarrier:
    """配液站-6孔分装板 - 2x3布局"""

    # 载架尺寸 (mm)
    carrier_size_x = 128.0
    carrier_size_y = 85.5
    carrier_size_z = 50.0

    # 瓶位尺寸
    bottle_diameter = 20.0
    bottle_spacing_x = 42.0  # X方向间距
    bottle_spacing_y = 35.0  # Y方向间距

    # 计算起始位置 (居中排列)
    start_x = (carrier_size_x - (3 - 1) * bottle_spacing_x - bottle_diameter) / 2
    start_y = (carrier_size_y - (2 - 1) * bottle_spacing_y - bottle_diameter) / 2

    sites = create_ordered_items_2d(
        klass=ResourceHolder,
        num_items_x=3,
        num_items_y=2,
        dx=start_x,
        dy=start_y,
        dz=5.0,
        item_dx=bottle_spacing_x,
        item_dy=bottle_spacing_y,

        size_x=bottle_diameter,
        size_y=bottle_diameter,
        size_z=carrier_size_z,
    )
    for k, v in sites.items():
        v.name = f"{name}_{v.name}"

    carrier = BottleCarrier(
        name=name,
        size_x=carrier_size_x,
        size_y=carrier_size_y,
        size_z=carrier_size_z,
        sites=sites,
        model="BIOYOND_DispensingStation_6VialCarrier",
    )
    carrier.num_items_x = 3
    carrier.num_items_y = 2
    carrier.num_items_z = 1
    ordering = ["A1", "A2", "A3", "B1", "B2", "B3"]
    # 第一排使用Liquid_Vial (10%分装小瓶), 第二排使用Solid_Vial (90%分装小瓶)
    for i in range(3):
        carrier[i] = BIOYOND_DispensingStation_Liquid_Vial(f"{name}_vial_{ordering[i]}")
    for i in range(3, 6):
        carrier[i] = BIOYOND_DispensingStation_Solid_Vial(f"{name}_vial_{ordering[i]}")
    return carrier



def BIOYOND_PolymerStation_1BottleCarrier(name: str) -> BottleCarrier:
    """[已弃用] 请根据实际工作站选择 BIOYOND_DispensingStation_1BottleCarrier 或 BIOYOND_ReactionStation_1BottleCarrier"""
    # 默认返回配液站版本以保持向后兼容
    return BIOYOND_DispensingStation_1BottleCarrier(name)


def BIOYOND_DispensingStation_1BottleCarrier(name: str) -> BottleCarrier:
    """配液站-单试剂瓶载架"""

    # 载架尺寸 (mm)
    carrier_size_x = 127.8
    carrier_size_y = 85.5
    carrier_size_z = 20.0

    # 烧杯尺寸
    beaker_diameter = 60.0

    # 计算中央位置
    center_x = (carrier_size_x - beaker_diameter) / 2
    center_y = (carrier_size_y - beaker_diameter) / 2
    center_z = 5.0

    carrier = BottleCarrier(
        name=name,
        size_x=carrier_size_x,
        size_y=carrier_size_y,
        size_z=carrier_size_z,
        sites=create_homogeneous_resources(
            klass=ResourceHolder,
            locations=[Coordinate(center_x, center_y, center_z)],
            resource_size_x=beaker_diameter,
            resource_size_y=beaker_diameter,
            name_prefix=name,
        ),
        model="BIOYOND_DispensingStation_1BottleCarrier",
    )
    carrier.num_items_x = 1
    carrier.num_items_y = 1
    carrier.num_items_z = 1
    carrier[0] = BIOYOND_PolymerStation_Reagent_Bottle(f"{name}_flask_1")
    return carrier


def BIOYOND_ReactionStation_1BottleCarrier(name: str) -> BottleCarrier:
    """反应站-单试剂瓶载架"""

    # 载架尺寸 (mm)
    carrier_size_x = 127.8
    carrier_size_y = 85.5
    carrier_size_z = 20.0

    # 烧杯尺寸
    beaker_diameter = 60.0

    # 计算中央位置
    center_x = (carrier_size_x - beaker_diameter) / 2
    center_y = (carrier_size_y - beaker_diameter) / 2
    center_z = 5.0

    carrier = BottleCarrier(
        name=name,
        size_x=carrier_size_x,
        size_y=carrier_size_y,
        size_z=carrier_size_z,
        sites=create_homogeneous_resources(
            klass=ResourceHolder,
            locations=[Coordinate(center_x, center_y, center_z)],
            resource_size_x=beaker_diameter,
            resource_size_y=beaker_diameter,
            name_prefix=name,
        ),
        model="BIOYOND_ReactionStation_1BottleCarrier",
    )
    carrier.num_items_x = 1
    carrier.num_items_y = 1
    carrier.num_items_z = 1
    carrier[0] = BIOYOND_PolymerStation_Reagent_Bottle(f"{name}_flask_1")
    return carrier


def BIOYOND_PolymerStation_1FlaskCarrier(name: str) -> BottleCarrier:
    """[已弃用] 请使用 BIOYOND_DispensingStation_1FlaskCarrier"""
    return BIOYOND_DispensingStation_1FlaskCarrier(name)


def BIOYOND_DispensingStation_1FlaskCarrier(name: str) -> BottleCarrier:
    """配液站-单烧杯载架"""

    # 载架尺寸 (mm)
    carrier_size_x = 127.8
    carrier_size_y = 85.5
    carrier_size_z = 20.0

    # 烧杯尺寸
    beaker_diameter = 70.0

    # 计算中央位置
    center_x = (carrier_size_x - beaker_diameter) / 2
    center_y = (carrier_size_y - beaker_diameter) / 2
    center_z = 5.0

    carrier = BottleCarrier(
        name=name,
        size_x=carrier_size_x,
        size_y=carrier_size_y,
        size_z=carrier_size_z,
        sites=create_homogeneous_resources(
            klass=ResourceHolder,
            locations=[Coordinate(center_x, center_y, center_z)],
            resource_size_x=beaker_diameter,
            resource_size_y=beaker_diameter,
            name_prefix=name,
        ),
        model="BIOYOND_DispensingStation_1FlaskCarrier",
    )
    carrier.num_items_x = 1
    carrier.num_items_y = 1
    carrier.num_items_z = 1
    carrier[0] = BIOYOND_PolymerStation_Reagent_Bottle(f"{name}_bottle_1")
    return carrier
