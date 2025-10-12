

def register():
    # noinspection PyUnresolvedReferences
    from unilabos.devices.liquid_handling.prcxi.prcxi import PRCXI9300Deck
    # noinspection PyUnresolvedReferences
    from unilabos.devices.liquid_handling.prcxi.prcxi import PRCXI9300Container
    # noinspection PyUnresolvedReferences
    from unilabos.devices.workstation.workstation_base import WorkStationContainer
    
    from unilabos.devices.liquid_handling.rviz_backend import UniLiquidHandlerRvizBackend
