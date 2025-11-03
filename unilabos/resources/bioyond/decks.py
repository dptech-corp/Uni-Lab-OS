from os import name
from pylabrobot.resources import Deck, Coordinate, Rotation

from unilabos.resources.bioyond.YB_warehouses import bioyond_warehouse_1x4x4, bioyond_warehouse_1x4x2, bioyond_warehouse_liquid_and_lid_handling, bioyond_warehouse_1x2x2, bioyond_warehouse_1x3x3, bioyond_warehouse_10x1x1, bioyond_warehouse_3x3x1, bioyond_warehouse_3x3x1_2, bioyond_warehouse_5x1x1, bioyond_warehouse_20x1x1, bioyond_warehouse_2x2x1, bioyond_warehouse_3x5x1


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
            "自动堆栈-左": bioyond_warehouse_2x2x1("自动堆栈-左"),
            "自动堆栈-右": bioyond_warehouse_2x2x1("自动堆栈-右"),
            "手动堆栈-左": bioyond_warehouse_3x5x1("手动堆栈-左"),
            "手动堆栈-右": bioyond_warehouse_3x5x1("手动堆栈-右"),
            "粉末加样头堆栈-左": bioyond_warehouse_10x1x1("粉末加样头堆栈-左"),
            "粉末加样头堆栈-右": bioyond_warehouse_10x1x1("粉末加样头堆栈-右"),
            "配液站内试剂仓库": bioyond_warehouse_3x3x1("配液站内试剂仓库"),
            "试剂替换仓库-左": bioyond_warehouse_5x1x1("试剂替换仓库-左"),
            "试剂替换仓库-右": bioyond_warehouse_5x1x1("试剂替换仓库-右"),
        }
        # warehouse 的位置
        self.warehouse_locations = {
            "自动堆栈-左": Coordinate(-300.0, 158.0, 0.0),
            "自动堆栈-右": Coordinate(4160.0, 158.0, 0.0),
            "手动堆栈-左": Coordinate(-400.0, 877.0, 0.0),
            "手动堆栈-右": Coordinate(4160.0, 877.0, 0.0),
            "粉末加样头堆栈-左": Coordinate(415.0, 1301.0, 0.0),
            "粉末加样头堆栈-右": Coordinate(2200.0, 1304.0, 0.0),
            "配液站内试剂仓库": Coordinate(2162.0, 337.0, 0.0),
            "试剂替换仓库-左": Coordinate(1173.0, 702.0, 0.0),
            "试剂替换仓库-右": Coordinate(2721.0, 739.0, 0.0),
        }

        for warehouse_name, warehouse in self.warehouses.items():
            self.assign_child_resource(warehouse, location=self.warehouse_locations[warehouse_name])
            
def YB_Deck(name: str) -> Deck:
    by=BIOYOND_YB_Deck(name=name)
    by.setup()
    return by





