from unilabos.resources.warehouse import WareHouse, warehouse_factory


def bioyond_warehouse_1x4x4(name: str) -> WareHouse:
    """创建BioYond 4x1x4仓库"""
    return warehouse_factory(
        name=name,
        num_items_x=1,
        num_items_y=4,
        num_items_z=4,
        dx=10.0,
        dy=10.0,
        dz=10.0,
        item_dx=137.0,
        item_dy=96.0,
        item_dz=120.0,
        category="warehouse",
    )


def bioyond_warehouse_1x4x2(name: str) -> WareHouse:
    """创建BioYond 4x1x2仓库"""
    return warehouse_factory(
        name=name,
        num_items_x=1,
        num_items_y=4,
        num_items_z=2,
        dx=10.0,
        dy=10.0,
        dz=10.0,
        item_dx=137.0,
        item_dy=96.0,
        item_dz=120.0,
        category="warehouse",
        removed_positions=None
    )
 # 定义benyond的堆栈
def bioyond_warehouse_1x2x2(name: str) -> WareHouse:
    """创建BioYond 4x1x4仓库"""
    return warehouse_factory(
        name=name,
        num_items_x=1,
        num_items_y=2,
        num_items_z=2,
        dx=10.0,
        dy=10.0,
        dz=10.0,
        item_dx=137.0,
        item_dy=96.0,
        item_dz=120.0,
        category="warehouse",
    )
def bioyond_warehouse_10x1x1(name: str) -> WareHouse:
    """创建BioYond 4x1x4仓库"""
    return warehouse_factory(
        name=name,
        num_items_x=10,
        num_items_y=1,
        num_items_z=1,
        dx=10.0,
        dy=10.0,
        dz=10.0,
        item_dx=137.0,
        item_dy=96.0,
        item_dz=120.0,
        category="warehouse",
    )
def bioyond_warehouse_1x3x3(name: str) -> WareHouse:
    """创建BioYond 4x1x4仓库"""
    return warehouse_factory(
        name=name,
        num_items_x=1,
        num_items_y=3,
        num_items_z=3,
        dx=10.0,
        dy=10.0,
        dz=10.0,
        item_dx=137.0,
        item_dy=96.0,
        item_dz=120.0,
        category="warehouse",
    )
def bioyond_warehouse_2x1x3(name: str) -> WareHouse:
    """创建BioYond 4x1x4仓库"""
    return warehouse_factory(
        name=name,
        num_items_x=2,
        num_items_y=1,
        num_items_z=3,
        dx=10.0,
        dy=10.0,
        dz=10.0,
        item_dx=137.0,
        item_dy=96.0,
        item_dz=120.0,
        category="warehouse",
    )

def bioyond_warehouse_3x3x1(name: str) -> WareHouse:
    """创建BioYond 4x1x4仓库"""
    return warehouse_factory(
        name=name,
        num_items_x=3,
        num_items_y=3,
        num_items_z=1,
        dx=10.0,
        dy=10.0,
        dz=10.0,
        item_dx=137.0,
        item_dy=96.0,
        item_dz=120.0,
        category="warehouse",
    )
def bioyond_warehouse_5x1x1(name: str) -> WareHouse:
    """创建BioYond 4x1x4仓库"""
    return warehouse_factory(
        name=name,
        num_items_x=5,
        num_items_y=1,
        num_items_z=1,
        dx=10.0,
        dy=10.0,
        dz=10.0,
        item_dx=137.0,
        item_dy=96.0,
        item_dz=120.0,
        category="warehouse",
    )
def bioyond_warehouse_3x3x1_2(name: str) -> WareHouse:
    """创建BioYond 4x1x4仓库"""
    return warehouse_factory(
        name=name,
        num_items_x=3,
        num_items_y=3,
        num_items_z=1,
        dx=12.0,
        dy=12.0,
        dz=12.0,
        item_dx=137.0,
        item_dy=96.0,
        item_dz=120.0,
        category="warehouse",
    )

def bioyond_warehouse_liquid_and_lid_handling(name: str) -> WareHouse:
    """创建BioYond开关盖加液模块台面"""
    return warehouse_factory(
        name=name,
        num_items_x=2,
        num_items_y=5,
        num_items_z=1,
        dx=10.0,
        dy=10.0,
        dz=10.0,
        item_dx=137.0,
        item_dy=96.0,
        item_dz=120.0,
        category="warehouse",
        removed_positions=None
    )