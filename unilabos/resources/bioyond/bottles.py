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
        diameter=diameter,# 未知
        height=height,
        max_volume=max_volume,
        barcode=barcode,
        model="Solid_Stock",
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
        model="Solid_Vial",
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
        model="Liquid_Vial",
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
        model="Solution_Beaker",
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
        model="Reagent_Bottle",
    )


def BIOYOND_PolymerStation_100ml_Liquid_Bottle(
    name: str,
    diameter: float = 50.0,
    height: float = 80.0,
    max_volume: float = 100000.0,  # 100mL
    barcode: str = None,
) -> Bottle:
    """创建100ml液体瓶"""
    return Bottle(
        name=name,
        diameter=diameter,
        height=height,
        max_volume=max_volume,
        barcode=barcode,
        model="100ml_Liquid_Bottle",
    )


def BIOYOND_PolymerStation_Liquid_Bottle(
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


def BIOYOND_PolymerStation_High_Viscosity_Liquid_Bottle(
    name: str,
    diameter: float = 45.0,
    height: float = 75.0,
    max_volume: float = 60000.0,  # 60mL
    barcode: str = None,
) -> Bottle:
    """创建高粘液瓶"""
    return Bottle(
        name=name,
        diameter=diameter,
        height=height,
        max_volume=max_volume,
        barcode=barcode,
        model="High_Viscosity_Liquid_Bottle",
    )


def BIOYOND_PolymerStation_Large_Dispense_Head(
    name: str,
    diameter: float = 35.0,
    height: float = 90.0,
    max_volume: float = 50000.0,  # 50mL
    barcode: str = None,
) -> Bottle:
    """创建加样头(大)"""
    return Bottle(
        name=name,
        diameter=diameter,
        height=height,
        max_volume=max_volume,
        barcode=barcode,
        model="Large_Dispense_Head",
    )


def BIOYOND_PolymerStation_5ml_Dispensing_Vial(
    name: str,
    diameter: float = 15.0,
    height: float = 45.0,
    max_volume: float = 5000.0,  # 5mL
    barcode: str = None,
) -> Bottle:
    """创建5ml分液瓶"""
    return Bottle(
        name=name,
        diameter=diameter,
        height=height,
        max_volume=max_volume,
        barcode=barcode,
        model="5ml_Dispensing_Vial",
    )


def BIOYOND_PolymerStation_20ml_Dispensing_Vial(
    name: str,
    diameter: float = 20.0,
    height: float = 65.0,
    max_volume: float = 20000.0,  # 20mL
    barcode: str = None,
) -> Bottle:
    """创建20ml分液瓶"""
    return Bottle(
        name=name,
        diameter=diameter,
        height=height,
        max_volume=max_volume,
        barcode=barcode,
        model="20ml_Dispensing_Vial",
    )


def BIOYOND_PolymerStation_Small_Solution_Bottle(
    name: str,
    diameter: float = 35.0,
    height: float = 60.0,
    max_volume: float = 40000.0,  # 40mL
    barcode: str = None,
) -> Bottle:
    """创建配液瓶(小)"""
    return Bottle(
        name=name,
        diameter=diameter,
        height=height,
        max_volume=max_volume,
        barcode=barcode,
        model="Small_Solution_Bottle",
    )


def BIOYOND_PolymerStation_Large_Solution_Bottle(
    name: str,
    diameter: float = 55.0,
    height: float = 90.0,
    max_volume: float = 150000.0,  # 150mL
    barcode: str = None,
) -> Bottle:
    """创建配液瓶(大)"""
    return Bottle(
        name=name,
        diameter=diameter,
        height=height,
        max_volume=max_volume,
        barcode=barcode,
        model="Large_Solution_Bottle",
    )


def BIOYOND_PolymerStation_Pipette_Tip(
    name: str,
    diameter: float = 10.0,
    height: float = 50.0,
    max_volume: float = 1000.0,  # 1mL
    barcode: str = None,
) -> Bottle:
    """创建枪头"""
    return Bottle(
        name=name,
        diameter=diameter,
        height=height,
        max_volume=max_volume,
        barcode=barcode,
        model="Pipette_Tip",
    )

