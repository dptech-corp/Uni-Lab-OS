"""
工作站物料管理基类
Workstation Material Management Base Class

基于PyLabRobot的物料管理系统
"""
from typing import Dict, Any, List, Optional, Union, Type
from abc import ABC, abstractmethod
import json

from pylabrobot.resources import (
    Resource as PLRResource,
    Container,
    Deck,
    Coordinate as PLRCoordinate,
)

from unilabos.ros.nodes.resource_tracker import DeviceNodeResourceTracker
from unilabos.utils.log import logger
from unilabos.resources.graphio import resource_plr_to_ulab, resource_ulab_to_plr


class MaterialManagementBase(ABC):
    """物料管理基类
    
    定义工作站物料管理的标准接口：
    1. 物料初始化 - 根据配置创建物料资源
    2. 物料追踪 - 实时跟踪物料位置和状态
    3. 物料查找 - 按类型、位置、状态查找物料
    4. 物料转换 - PyLabRobot与UniLab资源格式转换
    """

    def __init__(
        self,
        device_id: str,
        deck_config: Dict[str, Any],
        resource_tracker: DeviceNodeResourceTracker,
        children_config: Dict[str, Dict[str, Any]] = None
    ):
        self.device_id = device_id
        self.deck_config = deck_config
        self.resource_tracker = resource_tracker
        self.children_config = children_config or {}
        
        # 创建主台面
        self.plr_deck = self._create_deck()
        
        # 扩展ResourceTracker
        self._extend_resource_tracker()
        
        # 注册deck到resource tracker
        self.resource_tracker.add_resource(self.plr_deck)
        
        # 初始化子资源
        self.plr_resources = {}
        self._initialize_materials()

    def _create_deck(self) -> Deck:
        """创建主台面"""
        return Deck(
            name=f"{self.device_id}_deck",
            size_x=self.deck_config.get("size_x", 1000.0),
            size_y=self.deck_config.get("size_y", 1000.0),
            size_z=self.deck_config.get("size_z", 500.0),
            origin=PLRCoordinate(0, 0, 0)
        )

    def _extend_resource_tracker(self):
        """扩展ResourceTracker以支持PyLabRobot特定功能"""
        
        def find_by_type(resource_type):
            """按类型查找资源"""
            return self._find_resources_by_type_recursive(self.plr_deck, resource_type)
        
        def find_by_category(category: str):
            """按类别查找资源"""
            found = []
            for resource in self._get_all_resources():
                if hasattr(resource, 'category') and resource.category == category:
                    found.append(resource)
            return found
        
        def find_by_name_pattern(pattern: str):
            """按名称模式查找资源"""
            import re
            found = []
            for resource in self._get_all_resources():
                if re.search(pattern, resource.name):
                    found.append(resource)
            return found
        
        # 动态添加方法到resource_tracker
        self.resource_tracker.find_by_type = find_by_type
        self.resource_tracker.find_by_category = find_by_category
        self.resource_tracker.find_by_name_pattern = find_by_name_pattern

    def _find_resources_by_type_recursive(self, resource, target_type):
        """递归查找指定类型的资源"""
        found = []
        if isinstance(resource, target_type):
            found.append(resource)
        
        # 递归查找子资源
        children = getattr(resource, "children", [])
        for child in children:
            found.extend(self._find_resources_by_type_recursive(child, target_type))
        
        return found

    def _get_all_resources(self) -> List[PLRResource]:
        """获取所有资源"""
        all_resources = []
        
        def collect_resources(resource):
            all_resources.append(resource)
            children = getattr(resource, "children", [])
            for child in children:
                collect_resources(child)
        
        collect_resources(self.plr_deck)
        return all_resources

    def _initialize_materials(self):
        """初始化物料"""
        try:
            # 确定创建顺序，确保父资源先于子资源创建
            creation_order = self._determine_creation_order()
            
            # 按顺序创建资源
            for resource_id in creation_order:
                config = self.children_config[resource_id]
                self._create_plr_resource(resource_id, config)
            
            logger.info(f"物料管理系统初始化完成，共创建 {len(self.plr_resources)} 个资源")
            
        except Exception as e:
            logger.error(f"物料初始化失败: {e}")

    def _determine_creation_order(self) -> List[str]:
        """确定资源创建顺序"""
        order = []
        visited = set()
        
        def visit(resource_id: str):
            if resource_id in visited:
                return
            visited.add(resource_id)
            
            config = self.children_config.get(resource_id, {})
            parent_id = config.get("parent")
            
            # 如果有父资源，先访问父资源
            if parent_id and parent_id in self.children_config:
                visit(parent_id)
            
            order.append(resource_id)
        
        for resource_id in self.children_config:
            visit(resource_id)
        
        return order

    def _create_plr_resource(self, resource_id: str, config: Dict[str, Any]):
        """创建PyLabRobot资源"""
        try:
            resource_type = config.get("type", "unknown")
            data = config.get("data", {})
            location_config = config.get("location", {})
            
            # 创建位置坐标
            location = PLRCoordinate(
                x=location_config.get("x", 0.0),
                y=location_config.get("y", 0.0),
                z=location_config.get("z", 0.0)
            )
            
            # 根据类型创建资源
            resource = self._create_resource_by_type(resource_id, resource_type, config, data, location)
            
            if resource:
                # 设置父子关系
                parent_id = config.get("parent")
                if parent_id and parent_id in self.plr_resources:
                    parent_resource = self.plr_resources[parent_id]
                    parent_resource.assign_child_resource(resource, location)
                else:
                    # 直接放在deck上
                    self.plr_deck.assign_child_resource(resource, location)
                
                # 保存资源引用
                self.plr_resources[resource_id] = resource
                
                # 注册到resource tracker
                self.resource_tracker.add_resource(resource)
                
                logger.debug(f"创建资源成功: {resource_id} ({resource_type})")
            
        except Exception as e:
            logger.error(f"创建资源失败 {resource_id}: {e}")

    @abstractmethod
    def _create_resource_by_type(
        self, 
        resource_id: str, 
        resource_type: str, 
        config: Dict[str, Any], 
        data: Dict[str, Any], 
        location: PLRCoordinate
    ) -> Optional[PLRResource]:
        """根据类型创建资源 - 子类必须实现"""
        pass

    # ============ 物料查找接口 ============
    
    def find_materials_by_type(self, material_type: str) -> List[PLRResource]:
        """按材料类型查找物料"""
        return self.resource_tracker.find_by_category(material_type)

    def find_material_by_id(self, resource_id: str) -> Optional[PLRResource]:
        """按ID查找物料"""
        return self.plr_resources.get(resource_id)

    def find_available_positions(self, position_type: str) -> List[PLRResource]:
        """查找可用位置"""
        positions = self.resource_tracker.find_by_category(position_type)
        available = []
        
        for pos in positions:
            if hasattr(pos, 'is_available') and pos.is_available():
                available.append(pos)
            elif hasattr(pos, 'children') and len(pos.children) == 0:
                available.append(pos)
        
        return available

    def get_material_inventory(self) -> Dict[str, int]:
        """获取物料库存统计"""
        inventory = {}
        
        for resource in self._get_all_resources():
            if hasattr(resource, 'category'):
                category = resource.category
                inventory[category] = inventory.get(category, 0) + 1
        
        return inventory

    # ============ 物料状态更新接口 ============
    
    def update_material_location(self, material_id: str, new_location: PLRCoordinate) -> bool:
        """更新物料位置"""
        try:
            material = self.find_material_by_id(material_id)
            if material:
                material.location = new_location
                return True
            return False
        except Exception as e:
            logger.error(f"更新物料位置失败: {e}")
            return False

    def move_material(self, material_id: str, target_container_id: str) -> bool:
        """移动物料到目标容器"""
        try:
            material = self.find_material_by_id(material_id)
            target = self.find_material_by_id(target_container_id)
            
            if material and target:
                # 从原位置移除
                if material.parent:
                    material.parent.unassign_child_resource(material)
                
                # 添加到新位置
                target.assign_child_resource(material)
                return True
            
            return False
            
        except Exception as e:
            logger.error(f"移动物料失败: {e}")
            return False

    # ============ 资源转换接口 ============
    
    def convert_to_unilab_format(self, plr_resource: PLRResource) -> Dict[str, Any]:
        """将PyLabRobot资源转换为UniLab格式"""
        return resource_plr_to_ulab(plr_resource)

    def convert_from_unilab_format(self, unilab_resource: Dict[str, Any]) -> PLRResource:
        """将UniLab格式转换为PyLabRobot资源"""
        return resource_ulab_to_plr(unilab_resource)

    def get_deck_state(self) -> Dict[str, Any]:
        """获取Deck状态"""
        try:
            return {
                "deck_info": {
                    "name": self.plr_deck.name,
                    "size": {
                        "x": self.plr_deck.size_x,
                        "y": self.plr_deck.size_y,
                        "z": self.plr_deck.size_z
                    },
                    "children_count": len(self.plr_deck.children)
                },
                "resources": {
                    resource_id: self.convert_to_unilab_format(resource)
                    for resource_id, resource in self.plr_resources.items()
                },
                "inventory": self.get_material_inventory()
            }
        except Exception as e:
            logger.error(f"获取Deck状态失败: {e}")
            return {"error": str(e)}

    # ============ 数据持久化接口 ============
    
    def save_state_to_file(self, file_path: str) -> bool:
        """保存状态到文件"""
        try:
            state = self.get_deck_state()
            with open(file_path, 'w', encoding='utf-8') as f:
                json.dump(state, f, indent=2, ensure_ascii=False)
            logger.info(f"状态已保存到: {file_path}")
            return True
        except Exception as e:
            logger.error(f"保存状态失败: {e}")
            return False

    def load_state_from_file(self, file_path: str) -> bool:
        """从文件加载状态"""
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                state = json.load(f)
            
            # 重新创建资源
            self._recreate_resources_from_state(state)
            logger.info(f"状态已从文件加载: {file_path}")
            return True
            
        except Exception as e:
            logger.error(f"加载状态失败: {e}")
            return False

    def _recreate_resources_from_state(self, state: Dict[str, Any]):
        """从状态重新创建资源"""
        # 清除现有资源
        self.plr_resources.clear()
        self.plr_deck.children.clear()
        
        # 从状态重新创建
        resources_data = state.get("resources", {})
        for resource_id, resource_data in resources_data.items():
            try:
                plr_resource = self.convert_from_unilab_format(resource_data)
                self.plr_resources[resource_id] = plr_resource
                self.plr_deck.assign_child_resource(plr_resource)
            except Exception as e:
                logger.error(f"重新创建资源失败 {resource_id}: {e}")


class CoinCellMaterialManagement(MaterialManagementBase):
    """纽扣电池物料管理类
    
    从 button_battery_station 抽取的物料管理功能
    """

    def _create_resource_by_type(
        self, 
        resource_id: str, 
        resource_type: str, 
        config: Dict[str, Any], 
        data: Dict[str, Any], 
        location: PLRCoordinate
    ) -> Optional[PLRResource]:
        """根据类型创建纽扣电池相关资源"""
        
        # 导入纽扣电池资源类
        from unilabos.device_comms.button_battery_station import (
            MaterialPlate, PlateSlot, ClipMagazine, BatteryPressSlot,
            TipBox64, WasteTipBox, BottleRack, Battery, ElectrodeSheet
        )
        
        try:
            if resource_type == "material_plate":
                return self._create_material_plate(resource_id, config, data, location)
            
            elif resource_type == "plate_slot":
                return self._create_plate_slot(resource_id, config, data, location)
            
            elif resource_type == "clip_magazine":
                return self._create_clip_magazine(resource_id, config, data, location)
            
            elif resource_type == "battery_press_slot":
                return self._create_battery_press_slot(resource_id, config, data, location)
            
            elif resource_type == "tip_box":
                return self._create_tip_box(resource_id, config, data, location)
            
            elif resource_type == "waste_tip_box":
                return self._create_waste_tip_box(resource_id, config, data, location)
            
            elif resource_type == "bottle_rack":
                return self._create_bottle_rack(resource_id, config, data, location)
            
            elif resource_type == "battery":
                return self._create_battery(resource_id, config, data, location)
            
            else:
                logger.warning(f"未知的资源类型: {resource_type}")
                return None
                
        except Exception as e:
            logger.error(f"创建资源失败 {resource_id} ({resource_type}): {e}")
            return None

    def _create_material_plate(self, resource_id: str, config: Dict[str, Any], data: Dict[str, Any], location: PLRCoordinate):
        """创建料板"""
        from unilabos.device_comms.button_battery_station import MaterialPlate, ElectrodeSheet
        
        plate = MaterialPlate(
            name=resource_id,
            size_x=config.get("size_x", 80.0),
            size_y=config.get("size_y", 80.0),
            size_z=config.get("size_z", 10.0),
            hole_diameter=config.get("hole_diameter", 15.0),
            hole_depth=config.get("hole_depth", 8.0),
            hole_spacing_x=config.get("hole_spacing_x", 20.0),
            hole_spacing_y=config.get("hole_spacing_y", 20.0),
            number=data.get("number", "")
        )
        plate.location = location
        
        # 如果有预填充的极片数据，创建极片
        electrode_sheets = data.get("electrode_sheets", [])
        for i, sheet_data in enumerate(electrode_sheets):
            if i < len(plate.children):  # 确保不超过洞位数量
                hole = plate.children[i]
                sheet = ElectrodeSheet(
                    name=f"{resource_id}_sheet_{i}",
                    diameter=sheet_data.get("diameter", 14.0),
                    thickness=sheet_data.get("thickness", 0.1),
                    mass=sheet_data.get("mass", 0.01),
                    material_type=sheet_data.get("material_type", "cathode"),
                    info=sheet_data.get("info", "")
                )
                hole.place_electrode_sheet(sheet)
        
        return plate

    def _create_plate_slot(self, resource_id: str, config: Dict[str, Any], data: Dict[str, Any], location: PLRCoordinate):
        """创建板槽位"""
        from unilabos.device_comms.button_battery_station import PlateSlot
        
        slot = PlateSlot(
            name=resource_id,
            max_plates=config.get("max_plates", 8)
        )
        slot.location = location
        return slot

    def _create_clip_magazine(self, resource_id: str, config: Dict[str, Any], data: Dict[str, Any], location: PLRCoordinate):
        """创建子弹夹"""
        from unilabos.device_comms.button_battery_station import ClipMagazine
        
        magazine = ClipMagazine(
            name=resource_id,
            size_x=config.get("size_x", 150.0),
            size_y=config.get("size_y", 100.0),
            size_z=config.get("size_z", 50.0),
            hole_diameter=config.get("hole_diameter", 15.0),
            hole_depth=config.get("hole_depth", 40.0),
            hole_spacing=config.get("hole_spacing", 25.0),
            max_sheets_per_hole=config.get("max_sheets_per_hole", 100)
        )
        magazine.location = location
        return magazine

    def _create_battery_press_slot(self, resource_id: str, config: Dict[str, Any], data: Dict[str, Any], location: PLRCoordinate):
        """创建电池压制槽"""
        from unilabos.device_comms.button_battery_station import BatteryPressSlot
        
        slot = BatteryPressSlot(
            name=resource_id,
            diameter=config.get("diameter", 20.0),
            depth=config.get("depth", 15.0)
        )
        slot.location = location
        return slot

    def _create_tip_box(self, resource_id: str, config: Dict[str, Any], data: Dict[str, Any], location: PLRCoordinate):
        """创建枪头盒"""
        from unilabos.device_comms.button_battery_station import TipBox64
        
        tip_box = TipBox64(
            name=resource_id,
            size_x=config.get("size_x", 127.8),
            size_y=config.get("size_y", 85.5),
            size_z=config.get("size_z", 60.0),
            with_tips=data.get("with_tips", True)
        )
        tip_box.location = location
        return tip_box

    def _create_waste_tip_box(self, resource_id: str, config: Dict[str, Any], data: Dict[str, Any], location: PLRCoordinate):
        """创建废枪头盒"""
        from unilabos.device_comms.button_battery_station import WasteTipBox
        
        waste_box = WasteTipBox(
            name=resource_id,
            size_x=config.get("size_x", 127.8),
            size_y=config.get("size_y", 85.5),
            size_z=config.get("size_z", 60.0),
            max_tips=config.get("max_tips", 100)
        )
        waste_box.location = location
        return waste_box

    def _create_bottle_rack(self, resource_id: str, config: Dict[str, Any], data: Dict[str, Any], location: PLRCoordinate):
        """创建瓶架"""
        from unilabos.device_comms.button_battery_station import BottleRack
        
        rack = BottleRack(
            name=resource_id,
            size_x=config.get("size_x", 210.0),
            size_y=config.get("size_y", 140.0),
            size_z=config.get("size_z", 100.0),
            bottle_diameter=config.get("bottle_diameter", 30.0),
            bottle_height=config.get("bottle_height", 100.0),
            position_spacing=config.get("position_spacing", 35.0)
        )
        rack.location = location
        return rack

    def _create_battery(self, resource_id: str, config: Dict[str, Any], data: Dict[str, Any], location: PLRCoordinate):
        """创建电池"""
        from unilabos.device_comms.button_battery_station import Battery
        
        battery = Battery(
            name=resource_id,
            diameter=config.get("diameter", 20.0),
            height=config.get("height", 3.2),
            max_volume=config.get("max_volume", 100.0),
            barcode=data.get("barcode", "")
        )
        battery.location = location
        return battery

    # ============ 纽扣电池特定查找方法 ============
    
    def find_material_plates(self):
        """查找所有料板"""
        from unilabos.device_comms.button_battery_station import MaterialPlate
        return self.resource_tracker.find_by_type(MaterialPlate)
    
    def find_batteries(self):
        """查找所有电池"""
        from unilabos.device_comms.button_battery_station import Battery
        return self.resource_tracker.find_by_type(Battery)
    
    def find_electrode_sheets(self):
        """查找所有极片"""
        found = []
        plates = self.find_material_plates()
        for plate in plates:
            for hole in plate.children:
                if hasattr(hole, 'has_electrode_sheet') and hole.has_electrode_sheet():
                    found.append(hole._electrode_sheet)
        return found
    
    def find_plate_slots(self):
        """查找所有板槽位"""
        from unilabos.device_comms.button_battery_station import PlateSlot
        return self.resource_tracker.find_by_type(PlateSlot)
    
    def find_clip_magazines(self):
        """查找所有子弹夹"""
        from unilabos.device_comms.button_battery_station import ClipMagazine
        return self.resource_tracker.find_by_type(ClipMagazine)
    
    def find_press_slots(self):
        """查找所有压制槽"""
        from unilabos.device_comms.button_battery_station import BatteryPressSlot
        return self.resource_tracker.find_by_type(BatteryPressSlot)
