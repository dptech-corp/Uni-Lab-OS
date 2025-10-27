from pylabrobot.resources import create_homogeneous_resources, Coordinate, ResourceHolder, create_ordered_items_2d

from unilabos.resources.itemized_carrier import Bottle, BottleCarrier
from unilabos.resources.bioyond.bottles import (
    YB_jia_yang_tou_da,
)
# 命名约定：试剂瓶-Bottle，烧杯-Beaker，烧瓶-Flask，小瓶-Vial





def YB_jia_yang_tou_da_1X1_carrier(name: str) -> BottleCarrier:
    """加样头(大)板 - 1x1布局，1个位置"""

    # 载架尺寸 (mm)
    carrier_size_x = 127.8
    carrier_size_y = 85.5
    carrier_size_z = 95.0

    # 瓶位尺寸
    bottle_diameter = 35.0
    bottle_spacing_x = 42.0  # X方向间距
    bottle_spacing_y = 35.0  # Y方向间距

    # 计算起始位置 (居中排列)
    start_x = (carrier_size_x - (1 - 1) * bottle_spacing_x - bottle_diameter) / 2
    start_y = (carrier_size_y - (1 - 1) * bottle_spacing_y - bottle_diameter) / 2

    sites = create_ordered_items_2d(
        klass=ResourceHolder,
        num_items_x=1,
        num_items_y=1,
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
        model="YB_1X1_jia_yang_tou_da_carrier",
    )
    carrier.num_items_x = 1
    carrier.num_items_y = 1
    carrier.num_items_z = 1
    carrier[0] = YB_jia_yang_tou_da(f"{name}_head_1")
    return carrier


def BIOYOND_PolymerStation_AdapterBlock(name: str) -> BottleCarrier:
    """适配器块 - 单个中央位置"""

    # 载架尺寸 (mm)
    carrier_size_x = 127.8
    carrier_size_y = 85.5
    carrier_size_z = 30.0

    # 适配器尺寸
    adapter_diameter = 80.0

    # 计算中央位置
    center_x = (carrier_size_x - adapter_diameter) / 2
    center_y = (carrier_size_y - adapter_diameter) / 2
    center_z = 0.0

    carrier = BottleCarrier(
        name=name,
        size_x=carrier_size_x,
        size_y=carrier_size_y,
        size_z=carrier_size_z,
        sites=create_homogeneous_resources(
            klass=ResourceHolder,
            locations=[Coordinate(center_x, center_y, center_z)],
            resource_size_x=adapter_diameter,
            resource_size_y=adapter_diameter,
            name_prefix=name,
        ),
        model="AdapterBlock",
    )
    carrier.num_items_x = 1
    carrier.num_items_y = 1
    carrier.num_items_z = 1
    # 适配器块本身不包含瓶子，只是一个支撑结构
    return carrier


def BIOYOND_PolymerStation_TipBox(name: str) -> BottleCarrier:
    """枪头盒 - 8x12布局，96个位置"""

    # 载架尺寸 (mm)
    carrier_size_x = 127.8
    carrier_size_y = 85.5
    carrier_size_z = 55.0

    # 枪头尺寸
    tip_diameter = 10.0
    tip_spacing_x = 9.0  # X方向间距
    tip_spacing_y = 9.0  # Y方向间距

    # 计算起始位置 (居中排列)
    start_x = (carrier_size_x - (12 - 1) * tip_spacing_x - tip_diameter) / 2
    start_y = (carrier_size_y - (8 - 1) * tip_spacing_y - tip_diameter) / 2

    sites = create_ordered_items_2d(
        klass=ResourceHolder,
        num_items_x=12,
        num_items_y=8,
        dx=start_x,
        dy=start_y,
        dz=5.0,
        item_dx=tip_spacing_x,
        item_dy=tip_spacing_y,
        size_x=tip_diameter,
        size_y=tip_diameter,
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
        model="TipBox",
    )
    carrier.num_items_x = 12
    carrier.num_items_y = 8
    carrier.num_items_z = 1
    # 创建96个枪头
    for i in range(96):
        row = chr(65 + i // 12)  # A-H
        col = (i % 12) + 1  # 1-12
        carrier[i] = BIOYOND_PolymerStation_Pipette_Tip(f"{name}_tip_{row}{col}")
    return carrier

