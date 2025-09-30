from pylabrobot.resources import create_homogeneous_resources, Coordinate, ResourceHolder, create_ordered_items_2d

from unilabos.resources.itemized_carrier import Bottle, BottleCarrier
from unilabos.resources.bioyond.bottles import BIOYOND_PolymerStation_Solid_Vial, BIOYOND_PolymerStation_Solution_Beaker, BIOYOND_PolymerStation_Reagent_Bottle
# 命名约定：试剂瓶-Bottle，烧杯-Beaker，烧瓶-Flask，小瓶-Vial


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


def BIOYOND_PolymerStation_6VialCarrier(name: str) -> BottleCarrier:
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
        model="BIOYOND_PolymerStation_6VialCarrier",
    )
    carrier.num_items_x = 3
    carrier.num_items_y = 2
    carrier.num_items_z = 1
    ordering = ["A1", "A2", "A3", "B1", "B2", "B3"]  # 自定义顺序
    for i in range(6):
        carrier[i] = BIOYOND_PolymerStation_Solid_Vial(f"{name}_vial_{ordering[i]}")
    return carrier


def BIOYOND_PolymerStation_1BottleCarrier(name: str) -> BottleCarrier:
    """1瓶载架 - 单个中央位置"""

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
        model="BIOYOND_PolymerStation_1BottleCarrier",
    )
    carrier.num_items_x = 1
    carrier.num_items_y = 1
    carrier.num_items_z = 1
    carrier[0] = BIOYOND_PolymerStation_Reagent_Bottle(f"{name}_flask_1")
    return carrier


def BIOYOND_PolymerStation_1FlaskCarrier(name: str) -> BottleCarrier:
    """1瓶载架 - 单个中央位置"""

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
        model="BIOYOND_PolymerStation_1FlaskCarrier",
    )
    carrier.num_items_x = 1
    carrier.num_items_y = 1
    carrier.num_items_z = 1
    carrier[0] = BIOYOND_PolymerStation_Reagent_Bottle(f"{name}_bottle_1")
    return carrier
