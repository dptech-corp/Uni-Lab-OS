from unilabos.resources.itemized_carrier import Bottle, BottleCarrier
# 工厂函数

"""加样头（大）"""
def YB_jia_yang_tou_da(
    name: str,
    diameter: float = 20.0,
    height: float = 100.0,
    max_volume: float = 30000.0,  # 30mL
    barcode: str = None,
) -> Bottle:
    """创建粉末瓶"""
    return Bottle(
        name=name,
        diameter=diameter,# 未知
        height=height,
        max_volume=max_volume,
        barcode=barcode,
        model="Solid_Stock",
    )

"""液1x1"""
def YB_ye_Bottle(
    name: str,
    diameter: float = 40.0,
    height: float = 70.0,
    max_volume: float = 50000.0,  # 50mL
    barcode: str = None,
) -> Bottle:
    """创建液体瓶"""
    return Bottle(
        name=name,
        diameter=diameter,
        height=height,
        max_volume=max_volume,
        barcode=barcode,
        model="Liquid_Bottle",
    )    
