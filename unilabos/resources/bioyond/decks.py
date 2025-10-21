from os import name
from pylabrobot.resources import Deck, Coordinate, Rotation

from unilabos.resources.bioyond.warehouses import bioyond_warehouse_1x4x4, bioyond_warehouse_1x4x2, bioyond_warehouse_liquid_and_lid_handling, bioyond_warehouse_1x2x2, bioyond_warehouse_1x3x3, bioyond_warehouse_10x1x1, bioyond_warehouse_3x3x1, bioyond_warehouse_3x3x1_2, bioyond_warehouse_5x1x1


class BIOYOND_PolymerReactionStation_Deck(Deck):
    def __init__(
        self, 
        name: str = "PolymerReactionStation_Deck",
        size_x: float = 2700.0,
        size_y: float = 1080.0,
        size_z: float = 1500.0,
        category: str = "deck",
        setup: bool = False
    ) -> None:
        super().__init__(name=name, size_x=2700.0, size_y=1080.0, size_z=1500.0)
        if setup:
            self.setup()

    def setup(self) -> None:
        # 添加仓库
        self.warehouses = {
            "堆栈1": bioyond_warehouse_1x4x4("堆栈1"),
            "堆栈2": bioyond_warehouse_1x4x4("堆栈2"),
            "站内试剂存放堆栈": bioyond_warehouse_liquid_and_lid_handling("站内试剂存放堆栈"),
        }
        self.warehouse_locations = {
            "堆栈1": Coordinate(0.0, 430.0, 0.0),
            "堆栈2": Coordinate(2550.0, 430.0, 0.0),
            "站内试剂存放堆栈": Coordinate(800.0, 475.0, 0.0),
        }
        self.warehouses["站内试剂存放堆栈"].rotation = Rotation(z=90)

        for warehouse_name, warehouse in self.warehouses.items():
            self.assign_child_resource(warehouse, location=self.warehouse_locations[warehouse_name])


class BIOYOND_PolymerPreparationStation_Deck(Deck):
    def __init__(
        self, 
        name: str = "PolymerPreparationStation_Deck",
        size_x: float = 2700.0,
        size_y: float = 1080.0,
        size_z: float = 1500.0,
        category: str = "deck",
        setup: bool = False
    ) -> None:
        super().__init__(name=name, size_x=2700.0, size_y=1080.0, size_z=1500.0)
        if setup:
            self.setup()

    def setup(self) -> None:
        # 添加仓库
        self.warehouses = {
            "io_warehouse_left": bioyond_warehouse_1x4x4("io_warehouse_left"),
            "io_warehouse_right": bioyond_warehouse_1x4x4("io_warehouse_right"),
            "solutions": bioyond_warehouse_1x4x2("warehouse_solutions"),
            "liquid_and_lid_handling": bioyond_warehouse_liquid_and_lid_handling("warehouse_liquid_and_lid_handling"),
        }
        self.warehouse_locations = {
            "io_warehouse_left": Coordinate(0.0, 650.0, 0.0),
            "io_warehouse_right": Coordinate(2550.0, 650.0, 0.0),
            "solutions": Coordinate(1915.0, 900.0, 0.0),
            "liquid_and_lid_handling": Coordinate(1330.0, 490.0, 0.0),
        }

        for warehouse_name, warehouse in self.warehouses.items():
            self.assign_child_resource(warehouse, location=self.warehouse_locations[warehouse_name])

class BIOYOND_YB_Deck(Deck):
    def __init__(
        self, 
        name: str = "YB_Deck",
        size_x: float = 4150,
        size_y: float = 1400.0,
        size_z: float = 2670.0,
        category: str = "deck",
        setup: bool = False
    ) -> None:
        super().__init__(name=name, size_x=4150.0, size_y=1400.0, size_z=2670.0)
        if setup:
            self.setup()

    def setup(self) -> None:
        # 添加仓库
        self.warehouses = {
            "321窗口": bioyond_warehouse_1x2x2("321窗口"),
            "43窗口": bioyond_warehouse_1x2x2("43窗口"),
            "手动传递窗左": bioyond_warehouse_1x3x3("手动传递窗左"),
            "手动传递窗右": bioyond_warehouse_1x3x3("手动传递窗右"),
            "加样头堆栈左": bioyond_warehouse_10x1x1("加样头堆栈左"),
            "加样头堆栈右": bioyond_warehouse_10x1x1("加样头堆栈右"),

            "15ml配液堆栈左": bioyond_warehouse_3x3x1("15ml配液堆栈左"),
            "母液加样右": bioyond_warehouse_3x3x1_2("母液加样右"),
            "大瓶母液堆栈左": bioyond_warehouse_5x1x1("大瓶母液堆栈左"),
            "大瓶母液堆栈右": bioyond_warehouse_5x1x1("大瓶母液堆栈右"),
        }
        # warehouse 的位置
        self.warehouse_locations = {
            "321窗口": Coordinate(-150.0, 158.0, 0.0),
            "43窗口": Coordinate(4160.0, 158.0, 0.0),
            "手动传递窗左": Coordinate(-150.0, 877.0, 0.0),
            "手动传递窗右": Coordinate(4160.0, 877.0, 0.0),
            "加样头堆栈左": Coordinate(385.0, 1300.0, 0.0),
            "加样头堆栈右": Coordinate(2187.0, 1300.0, 0.0),

            "15ml配液堆栈左": Coordinate(749.0, 355.0, 0.0),
            "母液加样右": Coordinate(2152.0, 333.0, 0.0),
            "大瓶母液堆栈左": Coordinate(1164.0, 676.0, 0.0),
            "大瓶母液堆栈右": Coordinate(2717.0, 676.0, 0.0),
        }

        for warehouse_name, warehouse in self.warehouses.items():
            self.assign_child_resource(warehouse, location=self.warehouse_locations[warehouse_name])
            
def YB_Deck(name: str) -> Deck:
    by=BIOYOND_YB_Deck(name=name)
    by.setup()
    return by





