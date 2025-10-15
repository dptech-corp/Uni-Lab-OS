from unilabos.resources.itemized_carrier import Bottle, BottleCarrier
# 工厂函数


def BIOYOND_PolymerStation_Solid_Stock(
    name: str,
    diameter: float = 20.0,
    height: float = 100.0,
    max_volume: float = 30000.0,  # 30mL
    barcode: str = None,
) -> Bottle:
    """创建粉末瓶"""
    return Bottle(
        name=name,
        diameter=diameter,
        height=height,
        max_volume=max_volume,
        barcode=barcode,
        model="BIOYOND_PolymerStation_Solid_Stock",
    )


def BIOYOND_PolymerStation_Solid_Vial(
    name: str,
    diameter: float = 25.0,
    height: float = 60.0,
    max_volume: float = 30000.0,  # 30mL
    barcode: str = None,
) -> Bottle:
    """创建粉末瓶"""
    return Bottle(
        name=name,
        diameter=diameter,
        height=height,
        max_volume=max_volume,
        barcode=barcode,
        model="BIOYOND_PolymerStation_Solid_Vial",
    )


def BIOYOND_PolymerStation_Liquid_Vial(
    name: str,
    diameter: float = 25.0,
    height: float = 60.0,
    max_volume: float = 30000.0,  # 30mL
    barcode: str = None,
) -> Bottle:
    """创建滴定液瓶"""
    return Bottle(
        name=name,
        diameter=diameter,
        height=height,
        max_volume=max_volume,
        barcode=barcode,
        model="BIOYOND_PolymerStation_Liquid_Vial",
    )


def BIOYOND_PolymerStation_Solution_Beaker(
    name: str,
    diameter: float = 60.0,
    height: float = 70.0,
    max_volume: float = 200000.0,  # 200mL
    barcode: str = None,
) -> Bottle:
    """创建溶液烧杯"""
    return Bottle(
        name=name,
        diameter=diameter,
        height=height,
        max_volume=max_volume,
        barcode=barcode,
        model="BIOYOND_PolymerStation_Solution_Beaker",
    )


def BIOYOND_PolymerStation_Reagent_Bottle(
    name: str,
    diameter: float = 70.0,
    height: float = 120.0,
    max_volume: float = 500000.0,  # 500mL
    barcode: str = None,
) -> Bottle:
    """创建试剂瓶"""
    return Bottle(
        name=name,
        diameter=diameter,
        height=height,
        max_volume=max_volume,
        barcode=barcode,
        model="BIOYOND_PolymerStation_Reagent_Bottle",
    )
