from unilabos.resources.warehouse import WareHouse, warehouse_factory


def bioyond_warehouse_1x4x4(name: str) -> WareHouse:
    """创建BioYond 4x1x4仓库"""
    return warehouse_factory(
        name=name,
        num_items_x=4,
        num_items_y=4,
        num_items_z=1,
        dx=10.0,
        dy=10.0,
        dz=10.0,
        item_dx=147.0,
        item_dy=106.0,
        item_dz=130.0,
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