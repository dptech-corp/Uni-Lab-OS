"""
纽扣电池组装工作站物料类定义
Button Battery Assembly Station Resource Classes
"""

from __future__ import annotations

from collections import OrderedDict
from typing import Any, Dict, List, Optional, TypedDict, Union, cast

from pylabrobot.resources.coordinate import Coordinate
from pylabrobot.resources.container import Container
from pylabrobot.resources.deck import Deck
from pylabrobot.resources.itemized_resource import ItemizedResource
from pylabrobot.resources.resource import Resource
from pylabrobot.resources.resource_stack import ResourceStack
from pylabrobot.resources.tip_rack import TipRack, TipSpot
from pylabrobot.resources.trash import Trash
from pylabrobot.resources.utils import create_ordered_items_2d


class ElectrodeSheetState(TypedDict):
    diameter: float  # 直径 (mm)
    thickness: float  # 厚度 (mm)
    mass: float  # 质量 (g)
    material_type: str  # 材料类型（正极、负极、隔膜、弹片、垫片、铝箔等）
    height: float
    electrolyte_name: str
    data_electrolyte_code: str
    open_circuit_voltage: float
    assembly_pressure: float
    electrolyte_volume: float

    info: Optional[str]  # 附加信息

class ElectrodeSheet(Resource):
    """极片类 - 包含正负极片、隔膜、弹片、垫片、铝箔等所有片状材料"""

    def __init__(
        self,
        name: str = "极片",
        size_x=10,
        size_y=10,
        size_z=10,
        category: str = "electrode_sheet",
        model: Optional[str] = None,
    ):
        """初始化极片

        Args:
            name: 极片名称
            category: 类别
            model: 型号
        """
        super().__init__(
            name=name,
            size_x=size_x,
            size_y=size_y,
            size_z=size_z,
            category=category,
            model=model,
        )
        self._unilabos_state: ElectrodeSheetState = ElectrodeSheetState(
            diameter=14,
            thickness=0.1,
            mass=0.5,
            material_type="copper",
            info=None
        )

    # TODO: 这个还要不要？给self._unilabos_state赋值的？
    def load_state(self, state: Dict[str, Any]) -> None:
        """格式不变"""
        super().load_state(state)
        self._unilabos_state = state
    #序列化
    def serialize_state(self) -> Dict[str, Dict[str, Any]]:
        """格式不变"""
        data = super().serialize_state()
        data.update(self._unilabos_state)  # Container自身的信息，云端物料将保存这一data，本地也通过这里的data进行读写，当前类用来表示这个物料的长宽高大小的属性，而data（state用来表示物料的内容，细节等）
        return data

# TODO: 这个应该只能放一个极片
class MaterialHoleState(TypedDict):
    diameter: int
    depth: int
    max_sheets: int
    info: Optional[str]  # 附加信息

class MaterialHole(Resource):
    """料板洞位类"""
    children: List[ElectrodeSheet] = []

    def __init__(
        self,
        name: str,
        size_x: float,
        size_y: float,
        size_z: float,
        category: str = "material_hole",
        **kwargs
    ):
        super().__init__(
            name=name,
            size_x=size_x,
            size_y=size_y,
            size_z=size_z,
            category=category,
        )
        self._unilabos_state: MaterialHoleState = MaterialHoleState(
            diameter=20,
            depth=10,
            max_sheets=1,
            info=None
        )

    def get_all_sheet_info(self):
        info_list = []
        for sheet in self.children:
            info_list.append(sheet._unilabos_state["info"])
        return info_list
    
    #这个函数函数好像没用，一般不会集中赋值质量
    def set_all_sheet_mass(self):
        for sheet in self.children:
            sheet._unilabos_state["mass"] = 0.5  # 示例：设置质量为0.5g

    def load_state(self, state: Dict[str, Any]) -> None:
        """格式不变"""
        super().load_state(state)
        self._unilabos_state = state

    def serialize_state(self) -> Dict[str, Dict[str, Any]]:
        """格式不变"""
        data = super().serialize_state()
        data.update(self._unilabos_state)  # Container自身的信息，云端物料将保存这一data，本地也通过这里的data进行读写，当前类用来表示这个物料的长宽高大小的属性，而data（state用来表示物料的内容，细节等）
        return data
    #移动极片前先取出对象
    def get_sheet_with_name(self, name: str) -> Optional[ElectrodeSheet]:
        for sheet in self.children:
            if sheet.name == name:
                return sheet
        return None

    def has_electrode_sheet(self) -> bool:
        """检查洞位是否有极片"""
        return len(self.children) > 0

    def assign_child_resource(
        self,
        resource: ElectrodeSheet,
        location: Optional[Coordinate],
        reassign: bool = True,
    ):
        """放置极片"""
        # TODO: 这里要改，diameter找不到，加入._unilabos_state后应该没问题
        #if resource._unilabos_state["diameter"] > self._unilabos_state["diameter"]:
        #    raise ValueError(f"极片直径 {resource._unilabos_state['diameter']} 超过洞位直径 {self._unilabos_state['diameter']}")
        #if len(self.children) >= self._unilabos_state["max_sheets"]:
        #    raise ValueError(f"洞位已满，无法放置更多极片")
        super().assign_child_resource(resource, location, reassign)

    # 根据children的编号取物料对象。
    def get_electrode_sheet_info(self, index: int) -> ElectrodeSheet:
        return self.children[index]



class MaterialPlateState(TypedDict):
    hole_spacing_x: float
    hole_spacing_y: float
    hole_diameter: float
    info: Optional[str]  # 附加信息

class MaterialPlate(ItemizedResource[MaterialHole]):
    """料板类 - 4x4个洞位，每个洞位放1个极片"""
    
    children: List[MaterialHole]

    def __init__(
        self,
        name: str,
        size_x: float,
        size_y: float,
        size_z: float,
        ordered_items: Optional[Dict[str, MaterialHole]] = None,
        ordering: Optional[OrderedDict[str, str]] = None,
        category: str = "material_plate",
        model: Optional[str] = None,
        fill: bool = False
    ):
        """初始化料板

        Args:
            name: 料板名称
            size_x: 长度 (mm)
            size_y: 宽度 (mm)
            size_z: 高度 (mm)
            hole_diameter: 洞直径 (mm)
            hole_depth: 洞深度 (mm)
            hole_spacing_x: X方向洞位间距 (mm)
            hole_spacing_y: Y方向洞位间距 (mm)
            number: 编号
            category: 类别
            model: 型号
        """
        self._unilabos_state: MaterialPlateState = MaterialPlateState(
            hole_spacing_x=24.0,
            hole_spacing_y=24.0,
            hole_diameter=20.0,
            info="",
        )
        # 创建4x4的洞位
        # TODO: 这里要改，对应不同形状
        holes = create_ordered_items_2d(
            klass=MaterialHole,
            num_items_x=4,
            num_items_y=4,
            dx=(size_x - 4 * self._unilabos_state["hole_spacing_x"]) / 2,  # 居中
            dy=(size_y - 4 * self._unilabos_state["hole_spacing_y"]) / 2,  # 居中
            dz=size_z,
            item_dx=self._unilabos_state["hole_spacing_x"],
            item_dy=self._unilabos_state["hole_spacing_y"],
            size_x = 16,
            size_y = 16,
            size_z = 16,
        )
        if fill:
            super().__init__(
                name=name,
                size_x=size_x,
                size_y=size_y,
                size_z=size_z,
                ordered_items=holes,
                category=category,
                model=model,
            )
        else:
            super().__init__(
                name=name,
                size_x=size_x,
                size_y=size_y,
                size_z=size_z,
                ordered_items=ordered_items,
                ordering=ordering,
                category=category,
                model=model,
            )

    def update_locations(self):
        # TODO:调多次相加
        holes = create_ordered_items_2d(
            klass=MaterialHole,
            num_items_x=4,
            num_items_y=4,
            dx=(self._size_x - 3 * self._unilabos_state["hole_spacing_x"]) / 2,  # 居中
            dy=(self._size_y - 3 * self._unilabos_state["hole_spacing_y"]) / 2,  # 居中
            dz=self._size_z,
            item_dx=self._unilabos_state["hole_spacing_x"],
            item_dy=self._unilabos_state["hole_spacing_y"],
            size_x = 1,
            size_y = 1,
            size_z = 1,
        )
        for item, original_item in zip(holes.items(), self.children):
            original_item.location = item[1].location


class PlateSlot(ResourceStack):
    """板槽位类 - 1个槽上能堆放8个板，移板只能操作最上方的板"""

    def __init__(
        self,
        name: str,
        size_x: float,
        size_y: float,
        size_z: float,
        max_plates: int = 8,
        category: str = "plate_slot",
        model: Optional[str] = None
    ):
        """初始化板槽位

        Args:
            name: 槽位名称
            max_plates: 最大板数量
            category: 类别
        """
        super().__init__(
            name=name,
            direction="z",  # Z方向堆叠
            resources=[],
        )
        self.max_plates = max_plates
        self.category = category

    def can_add_plate(self) -> bool:
        """检查是否可以添加板"""
        return len(self.children) < self.max_plates

    def add_plate(self, plate: MaterialPlate) -> None:
        """添加料板"""
        if not self.can_add_plate():
            raise ValueError(f"槽位 {self.name} 已满，无法添加更多板")
        self.assign_child_resource(plate)

    def get_top_plate(self) -> MaterialPlate:
        """获取最上方的板"""
        if len(self.children) == 0:
            raise ValueError(f"槽位 {self.name} 为空")
        return cast(MaterialPlate, self.get_top_item())

    def take_top_plate(self) -> MaterialPlate:
        """取出最上方的板"""
        top_plate = self.get_top_plate()
        self.unassign_child_resource(top_plate)
        return top_plate

    def can_access_for_picking(self) -> bool:
        """检查是否可以进行取料操作（只有最上方的板能进行取料操作）"""
        return len(self.children) > 0

    def serialize(self) -> dict:
        return {
            **super().serialize(),
            "max_plates": self.max_plates,
        }


class ClipMagazineHole(Container):
    """子弹夹洞位类"""

    def __init__(
        self,
        name: str,
        diameter: float,
        depth: float,
        max_sheets: int = 100,
        category: str = "clip_magazine_hole",
    ):
        """初始化子弹夹洞位

        Args:
            name: 洞位名称
            diameter: 洞直径 (mm)
            depth: 洞深度 (mm)
            max_sheets: 最大极片数量
            category: 类别
        """
        super().__init__(
            name=name,
            size_x=diameter,
            size_y=diameter,
            size_z=depth,
            category=category,
        )
        self.diameter = diameter
        self.depth = depth
        self.max_sheets = max_sheets
        self._sheets: List[ElectrodeSheet] = []

    def can_add_sheet(self, sheet: ElectrodeSheet) -> bool:
        """检查是否可以添加极片"""
        return (len(self._sheets) < self.max_sheets and
                sheet.diameter <= self.diameter)

    def add_sheet(self, sheet: ElectrodeSheet) -> None:
        """添加极片"""
        if not self.can_add_sheet(sheet):
            raise ValueError(f"无法向洞位 {self.name} 添加极片")
        self._sheets.append(sheet)

    def take_sheet(self) -> ElectrodeSheet:
        """取出极片"""
        if len(self._sheets) == 0:
            raise ValueError(f"洞位 {self.name} 没有极片")
        return self._sheets.pop()

    def get_sheet_count(self) -> int:
        """获取极片数量"""
        return len(self._sheets)

    def serialize_state(self) -> Dict[str, Any]:
        return {
            "sheet_count": len(self._sheets),
            "sheets": [sheet.serialize() for sheet in self._sheets],
        }

# TODO: 这个要改
class ClipMagazine(Resource):
    """子弹夹类 - 有6个洞位，每个洞位放多个极片"""

    def __init__(
        self,
        name: str,
        size_x: float,
        size_y: float,
        size_z: float,
        hole_diameter: float = 20.0,
        hole_depth: float = 50.0,
        hole_spacing: float = 25.0,
        max_sheets_per_hole: int = 100,
        category: str = "clip_magazine",
        model: Optional[str] = None,
    ):
        """初始化子弹夹

        Args:
            name: 子弹夹名称
            size_x: 长度 (mm)
            size_y: 宽度 (mm)
            size_z: 高度 (mm)
            hole_diameter: 洞直径 (mm)
            hole_depth: 洞深度 (mm)
            hole_spacing: 洞位间距 (mm)
            max_sheets_per_hole: 每个洞位最大极片数量
            category: 类别
            model: 型号
        """
        # 创建6个洞位，排成2x3布局
        holes = create_ordered_items_2d(
            klass=ClipMagazineHole,
            num_items_x=3,
            num_items_y=2,
            dx=(size_x - 2 * hole_spacing) / 2,  # 居中
            dy=(size_y - hole_spacing) / 2,     # 居中
            dz=size_z - 0,
            item_dx=hole_spacing,
            item_dy=hole_spacing,
            diameter=hole_diameter,
            depth=hole_depth,
        )

        super().__init__(
            name=name,
            size_x=size_x,
            size_y=size_y,
            size_z=size_z,
            ordered_items=holes,
            category=category,
            model=model,
        )

        self.hole_diameter = hole_diameter
        self.hole_depth = hole_depth
        self.max_sheets_per_hole = max_sheets_per_hole

    def serialize(self) -> dict:
        return {
            **super().serialize(),
            "hole_diameter": self.hole_diameter,
            "hole_depth": self.hole_depth,
            "max_sheets_per_hole": self.max_sheets_per_hole,
        }
#是一种类型注解，不用self
class BatteryState(TypedDict):
    """电池状态字典"""
    diameter: float
    height: float
    assembly_pressure: float
    electrolyte_volume: float
    electrolyte_name: str

class Battery(Resource):
    """电池类 - 可容纳极片"""
    children: List[ElectrodeSheet] = []

    def __init__(
        self,
        name: str,
        size_x=1,
        size_y=1,
        size_z=1,
        category: str = "battery",
    ):
        """初始化电池

        Args:
            name: 电池名称
            diameter: 直径 (mm)
            height: 高度 (mm)
            max_volume: 最大容量 (μL)
            barcode: 二维码编号
            category: 类别
            model: 型号
        """
        super().__init__(
            name=name,
            size_x=1,
            size_y=1,
            size_z=1,
            category=category,
        )
        self._unilabos_state: BatteryState = BatteryState(
                                 diameter = 1.0,
                                 height = 1.0,
                                 assembly_pressure = 1.0,
                                 electrolyte_volume = 1.0,
                                 electrolyte_name = "DP001"
                                 )

    def add_electrolyte_with_bottle(self, bottle: Bottle) -> bool:
        to_add_name = bottle._unilabos_state["electrolyte_name"]
        if bottle.aspirate_electrolyte(10):
            if self.add_electrolyte(to_add_name, 10):
                pass
            else:
                bottle._unilabos_state["electrolyte_volume"] += 10

    def set_electrolyte(self, name: str, volume: float) -> None:
        """设置电解液信息"""
        self._unilabos_state["electrolyte_name"] = name
        self._unilabos_state["electrolyte_volume"] = volume
    #这个应该没用，不会有加了后再加的事情
    def add_electrolyte(self, name: str, volume: float) -> bool:
        """添加电解液信息"""
        if name != self._unilabos_state["electrolyte_name"]:
            return False
        self._unilabos_state["electrolyte_volume"] += volume

    def load_state(self, state: Dict[str, Any]) -> None:
        """格式不变"""
        super().load_state(state)
        self._unilabos_state = state

    def serialize_state(self) -> Dict[str, Dict[str, Any]]:
        """格式不变"""
        data = super().serialize_state()
        data.update(self._unilabos_state)  # Container自身的信息，云端物料将保存这一data，本地也通过这里的data进行读写，当前类用来表示这个物料的长宽高大小的属性，而data（state用来表示物料的内容，细节等）
        return data

# 电解液作为属性放进去

class BatteryPressSlotState(TypedDict):
    """电池状态字典"""
    diameter: float =20.0
    depth: float = 4.0

class BatteryPressSlot(Resource):
    """电池压制槽类 - 设备，可容纳一个电池"""
    children: List[Battery] = []

    def __init__(
        self,
        name: str = "BatteryPressSlot",
        category: str = "battery_press_slot",
    ):
        """初始化电池压制槽

        Args:
            name: 压制槽名称
            diameter: 直径 (mm)
            depth: 深度 (mm)
            category: 类别
            model: 型号
        """
        super().__init__(
            name=name,
            size_x=10,
            size_y=12,
            size_z=13,
            category=category,
        )
        self._unilabos_state: BatteryPressSlotState = BatteryPressSlotState()

    def has_battery(self) -> bool:
        """检查是否有电池"""
        return len(self.children) > 0

    def load_state(self, state: Dict[str, Any]) -> None:
        """格式不变"""
        super().load_state(state)
        self._unilabos_state = state

    def serialize_state(self) -> Dict[str, Dict[str, Any]]:
        """格式不变"""
        data = super().serialize_state()
        data.update(self._unilabos_state)  # Container自身的信息，云端物料将保存这一data，本地也通过这里的data进行读写，当前类用来表示这个物料的长宽高大小的属性，而data（state用来表示物料的内容，细节等）
        return data
    
    def assign_child_resource(
        self,
        resource: Battery,
        location: Optional[Coordinate],
        reassign: bool = True,
    ):
        """放置极片"""
        # TODO: 让高京看下槽位只有一个电池时是否这么写。
        if self.has_battery():
            raise ValueError(f"槽位已含有一个电池，无法再放置其他电池")
        super().assign_child_resource(resource, location, reassign)

    # 根据children的编号取物料对象。
    def get_battery_info(self, index: int) -> Battery:
        return self.children[0]

# TODO:这个移液枪架子看一下从哪继承
class TipBox64State(TypedDict):
    """电池状态字典"""
    tip_diameter: float = 5.0
    tip_length: float = 50.0
    with_tips: bool = True

class TipBox64(TipRack):
    """64孔枪头盒类"""

    children: List[TipSpot] = []
    def __init__(
        self,
        name: str,
        size_x: float = 127.8,
        size_y: float = 85.5,
        size_z: float = 60.0,
        category: str = "tip_box_64",
        model: Optional[str] = None,
    ):
        """初始化64孔枪头盒

        Args:
            name: 枪头盒名称
            size_x: 长度 (mm)
            size_y: 宽度 (mm)
            size_z: 高度 (mm)
            tip_diameter: 枪头直径 (mm)
            tip_length: 枪头长度 (mm)
            category: 类别
            model: 型号
            with_tips: 是否带枪头
        """
        from pylabrobot.resources.tip import Tip

        # 创建8x8=64个枪头位
        def make_tip():
            return Tip(
                has_filter=False,
                total_tip_length=20.0,
                maximal_volume=1000,  # 1mL
                fitting_depth=8.0,
            )

        tip_spots = create_ordered_items_2d(
            klass=TipSpot,
            num_items_x=8,
            num_items_y=8,
            dx=8.0,
            dy=8.0,
            dz=0.0,
            item_dx=9.0,
            item_dy=9.0,
            size_x=10,
            size_y=10,
            size_z=0.0,
            make_tip=make_tip,
        )
        self._unilabos_state: WasteTipBoxstate = WasteTipBoxstate()
        super().__init__(
            name=name,
            size_x=size_x,
            size_y=size_y,
            size_z=size_z,
            ordered_items=tip_spots,
            category=category,
            model=model,
            with_tips=True,
        )



class WasteTipBoxstate(TypedDict):
    """"废枪头盒状态字典"""
    max_tips: int = 100
    tip_count: int = 0

#枪头不是一次性的（同一溶液则反复使用），根据寄存器判断
class WasteTipBox(Trash):
    """废枪头盒类 - 100个枪头容量"""

    def __init__(
        self,
        name: str,
        size_x: float = 127.8,
        size_y: float = 85.5,
        size_z: float = 60.0,
        category: str = "waste_tip_box",
        model: Optional[str] = None,
    ):
        """初始化废枪头盒

        Args:
            name: 废枪头盒名称
            size_x: 长度 (mm)
            size_y: 宽度 (mm)
            size_z: 高度 (mm)
            max_tips: 最大枪头容量
            category: 类别
            model: 型号
        """
        super().__init__(
            name=name,
            size_x=size_x,
            size_y=size_y,
            size_z=size_z,
            category=category,
            model=model,
        )
        self._unilabos_state: WasteTipBoxstate = WasteTipBoxstate()

    def add_tip(self) -> None:
        """添加废枪头"""
        if self._unilabos_state["tip_count"] >= self._unilabos_state["max_tips"]:
            raise ValueError(f"废枪头盒 {self.name} 已满")
        self._unilabos_state["tip_count"] += 1

    def get_tip_count(self) -> int:
        """获取枪头数量"""
        return self._unilabos_state["tip_count"]

    def empty(self) -> None:
        """清空废枪头盒"""
        self._unilabos_state["tip_count"] = 0


    def load_state(self, state: Dict[str, Any]) -> None:
        """格式不变"""
        super().load_state(state)
        self._unilabos_state = state

    def serialize_state(self) -> Dict[str, Dict[str, Any]]:
        """格式不变"""
        data = super().serialize_state()
        data.update(self._unilabos_state)  # Container自身的信息，云端物料将保存这一data，本地也通过这里的data进行读写，当前类用来表示这个物料的长宽高大小的属性，而data（state用来表示物料的内容，细节等）
        return data


class BottleRackState(TypedDict):
    """     bottle_diameter: 瓶子直径 (mm)
            bottle_height: 瓶子高度 (mm)
            position_spacing: 位置间距 (mm)"""
    bottle_diameter: float
    bottle_height: float
    name_to_index: dict



class BottleRack(Resource):
    """瓶架类 - 12个待配位置+12个已配位置"""
    children: List[Bottle] = []

    def __init__(
        self,
        name: str,
        size_x: float,
        size_y: float,
        size_z: float,
        category: str = "bottle_rack",
        model: Optional[str] = None,
    ):
        """初始化瓶架

        Args:
            name: 瓶架名称
            size_x: 长度 (mm)
            size_y: 宽度 (mm)
            size_z: 高度 (mm)
            category: 类别
            model: 型号
        """
        super().__init__(
            name=name,
            size_x=size_x,
            size_y=size_y,
            size_z=size_z,
            category=category,
            model=model,
        )
        # TODO: 添加瓶位坐标映射
        self.index_to_pos = {
            0: Coordinate.zero(),
            1: Coordinate(x=1, y=2, z=3)  # 添加
        }
        self.name_to_index = {}
        self.name_to_pos = {}

    def load_state(self, state: Dict[str, Any]) -> None:
        """格式不变"""
        super().load_state(state)
        self._unilabos_state = state

    def serialize_state(self) -> Dict[str, Dict[str, Any]]:
        """格式不变"""
        data = super().serialize_state()
        data.update(self._unilabos_state)  # Container自身的信息，云端物料将保存这一data，本地也通过这里的data进行读写，当前类用来表示这个物料的长宽高大小的属性，而data（state用来表示物料的内容，细节等）
        return data

    # TODO: 这里有些问题要重新写一下
    def assign_child_resource(self, resource: Bottle, location=Coordinate.zero(), reassign = True):
        assert len(self.children) <= 12, "瓶架已满，无法添加更多瓶子"
        index = len(self.children)
        location = Coordinate(x=20 + (index % 4) * 15, y=20 + (index // 4) * 15, z=0)
        self.name_to_pos[resource.name] = location
        self.name_to_index[resource.name] = index
        return super().assign_child_resource(resource, location, reassign)
    
    def assign_child_resource_by_index(self, resource: Bottle, index: int):
        assert 0 <= index < 12, "无效的瓶子索引"
        self.name_to_index[resource.name] = index
        location = self.index_to_pos[index]
        return super().assign_child_resource(resource, location)

    def unassign_child_resource(self, resource: Bottle):
        super().unassign_child_resource(resource)
        self.index_to_pos.pop(self.name_to_index.pop(resource.name, None), None)

    # def serialize(self):
    #     self.children.sort(key=lambda x: self.name_to_index.get(x.name, 0))
    #     return super().serialize()


class BottleState(TypedDict):
    diameter: float
    height: float
    electrolyte_name: str
    electrolyte_volume: float
    max_volume: float

class Bottle(Resource):
    """瓶子类 - 容纳电解液"""

    def __init__(
        self,
        name: str,
        category: str = "bottle",
    ):
        """初始化瓶子

        Args:
            name: 瓶子名称
            diameter: 直径 (mm)
            height: 高度 (mm)
            max_volume: 最大体积 (μL)
            barcode: 二维码
            category: 类别
            model: 型号
        """
        super().__init__(
            name=name,
            size_x=1,
            size_y=1,
            size_z=1,
            category=category,
        )
        self._unilabos_state: BottleState = BottleState()
    
    def aspirate_electrolyte(self, volume: float) -> bool:
        current_volume = self._unilabos_state["electrolyte_volume"]
        assert current_volume > volume, f"Cannot aspirate {volume}μL, only {current_volume}μL available."
        self._unilabos_state["electrolyte_volume"] -= volume
        return True

    def load_state(self, state: Dict[str, Any]) -> None:
        """格式不变"""
        super().load_state(state)
        self._unilabos_state = state

    def serialize_state(self) -> Dict[str, Dict[str, Any]]:
        """格式不变"""
        data = super().serialize_state()
        data.update(self._unilabos_state)  # Container自身的信息，云端物料将保存这一data，本地也通过这里的data进行读写，当前类用来表示这个物料的长宽高大小的属性，而data（state用来表示物料的内容，细节等）
        return data

class CoincellDeck(Deck):
    """纽扣电池组装工作站台面类"""

    def __init__(
        self,
        name: str = "coin_cell_deck",
        size_x: float = 1000.0,  # 1m
        size_y: float = 1000.0,  # 1m
        size_z: float = 900.0,   # 0.9m
        origin: Coordinate = Coordinate(0, 0, 0),
        category: str = "coin_cell_deck",
        setup: bool = False,  # 是否自动执行 setup
    ):
        """初始化纽扣电池组装工作站台面

        Args:
            name: 台面名称
            size_x: 长度 (mm) - 1m
            size_y: 宽度 (mm) - 1m
            size_z: 高度 (mm) - 0.9m
            origin: 原点坐标
            category: 类别
            setup: 是否自动执行 setup 配置标准布局
        """
        super().__init__(
            name=name,
            size_x=size_x,
            size_y=size_y,
            size_z=size_z,
            origin=origin,
            category=category,
        )
        if setup:
            self.setup()

    def setup(self) -> None:
        """设置工作站的标准布局 - 包含3个料盘"""
        # 步骤 1: 创建所有料盘
        self.plates = {
            "liaopan1": MaterialPlate(
                name="liaopan1", 
                size_x=120.8, 
                size_y=120.5, 
                size_z=10.0, 
                fill=True
            ),
            "liaopan2": MaterialPlate(
                name="liaopan2", 
                size_x=120.8, 
                size_y=120.5, 
                size_z=10.0, 
                fill=True
            ),
            "电池料盘": MaterialPlate(
                name="电池料盘", 
                size_x=120.8, 
                size_y=160.5, 
                size_z=10.0, 
                fill=True
            ),
        }
        
        # 步骤 2: 定义料盘在 deck 上的位置
        # Deck 尺寸: 1000×1000mm，料盘尺寸: 120.8×120.5mm 或 120.8×160.5mm
        self.plate_locations = {
            "liaopan1": Coordinate(x=50, y=50, z=0),      # 左上角，留 50mm 边距
            "liaopan2": Coordinate(x=250, y=50, z=0),     # 中间，liaopan1 右侧
            "电池料盘": Coordinate(x=450, y=50, z=0),     # 右侧
        }
        
        # 步骤 3: 将料盘分配到 deck 上
        for plate_name, plate in self.plates.items():
            self.assign_child_resource(
                plate, 
                location=self.plate_locations[plate_name]
            )
        
        # 步骤 4: 为 liaopan1 添加初始极片
        for i in range(16):
            jipian = ElectrodeSheet(
                name=f"jipian1_{i}", 
                size_x=12, 
                size_y=12, 
                size_z=0.1
            )
            self.plates["liaopan1"].children[i].assign_child_resource(
                jipian, 
                location=None
            )


def create_coin_cell_deck(name: str = "coin_cell_deck", size_x: float = 1000.0, size_y: float = 1000.0, size_z: float = 900.0) -> CoincellDeck:
    """创建并配置标准的纽扣电池组装工作站台面
    
    Args:
        name: 台面名称
        size_x: 长度 (mm)
        size_y: 宽度 (mm)
        size_z: 高度 (mm)
    
    Returns:
        已配置好的 CoincellDeck 对象
    """
    deck = CoincellDeck(name=name, size_x=size_x, size_y=size_y, size_z=size_z)
    deck.setup()
    return deck
