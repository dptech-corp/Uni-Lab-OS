from unilabos.resources.itemized_carrier import Bottle, BottleCarrier
# 工厂函数


def YB_jia_yang_tou_da(
    name: str,
    diameter: float = 35.0,
    height: float = 90.0,
    max_volume: float = 50000.0,  # 50mL
    code: str = None,
    barcode: str = None,
) -> Bottle:
    """创建加样头(大)"""
    return Bottle(
        name=name,
        diameter=diameter,
        height=height,
        max_volume=max_volume,
        barcode=barcode,
        code=code,
        model="YB_jia_yang_tou_da",
    )


