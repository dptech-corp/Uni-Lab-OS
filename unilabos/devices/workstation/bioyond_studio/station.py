"""
Bioyond工作站实现
Bioyond Workstation Implementation

集成Bioyond物料管理的工作站示例
"""
import time
import traceback
from datetime import datetime
from typing import Dict, Any, List, Optional, Union
import json

from unilabos.devices.workstation.workstation_base import WorkstationBase, ResourceSynchronizer
from unilabos.devices.workstation.bioyond_studio.bioyond_rpc import BioyondV1RPC
from unilabos.registry.placeholder_type import ResourceSlot, DeviceSlot
from unilabos.resources.warehouse import WareHouse
from unilabos.utils.log import logger
from unilabos.resources.graphio import resource_bioyond_to_plr, resource_plr_to_bioyond

from unilabos.ros.nodes.base_device_node import ROS2DeviceNode, BaseROS2DeviceNode
from unilabos.ros.nodes.presets.workstation import ROS2WorkstationNode
from pylabrobot.resources.resource import Resource as ResourcePLR

from unilabos.devices.workstation.bioyond_studio.config import (
    API_CONFIG, WORKFLOW_MAPPINGS, MATERIAL_TYPE_MAPPINGS, WAREHOUSE_MAPPING
)


class BioyondResourceSynchronizer(ResourceSynchronizer):
    """Bioyond资源同步器

    负责与Bioyond系统进行物料数据的同步
    """

    def __init__(self, workstation: 'BioyondWorkstation'):
        super().__init__(workstation)
        self.bioyond_api_client = None
        self.sync_interval = 60  # 默认60秒同步一次
        self.last_sync_time = 0
        self.initialize()

    def initialize(self) -> bool:
        """初始化Bioyond资源同步器"""
        try:
            self.bioyond_api_client = self.workstation.hardware_interface
            if self.bioyond_api_client is None:
                logger.error("Bioyond API客户端未初始化")
                return False

            # 设置同步间隔
            self.sync_interval = self.workstation.bioyond_config.get("sync_interval", 600)

            logger.info("Bioyond资源同步器初始化完成")
            return True
        except Exception as e:
            logger.error(f"Bioyond资源同步器初始化失败: {e}")
            return False

    def sync_from_external(self) -> bool:
        """从Bioyond系统同步物料数据"""
        try:
            if self.bioyond_api_client is None:
                logger.error("Bioyond API客户端未初始化")
                return False

            # 同时查询样品类型(typeMode=1)和试剂类型(typeMode=2)
            all_bioyond_data = []

            # 查询样品类型物料（烧杯、试剂瓶、分装板等）
            bioyond_data_type1 = self.bioyond_api_client.stock_material('{"typeMode": 1, "includeDetail": true}')
            if bioyond_data_type1:
                all_bioyond_data.extend(bioyond_data_type1)
                logger.debug(f"从Bioyond查询到 {len(bioyond_data_type1)} 个样品类型物料")

            # 查询试剂类型物料（样品板、样品瓶等）
            bioyond_data_type2 = self.bioyond_api_client.stock_material('{"typeMode": 2, "includeDetail": true}')
            if bioyond_data_type2:
                all_bioyond_data.extend(bioyond_data_type2)
                logger.debug(f"从Bioyond查询到 {len(bioyond_data_type2)} 个试剂类型物料")

            if not all_bioyond_data:
                logger.warning("从Bioyond获取的物料数据为空")
                return False

            # 转换为UniLab格式
            unilab_resources = resource_bioyond_to_plr(
                all_bioyond_data,
                type_mapping=self.workstation.bioyond_config["material_type_mappings"],
                deck=self.workstation.deck
            )

            logger.info(f"从Bioyond同步了 {len(unilab_resources)} 个资源")
            return True
        except Exception as e:
            logger.error(f"从Bioyond同步物料数据失败: {e}")
            return False

    def sync_to_external(self, resource: Any) -> bool:
        """将本地物料数据变更同步到Bioyond系统"""
        try:
            # ✅ 跳过仓库类型的资源 - 仓库是容器，不是物料
            resource_category = getattr(resource, "category", None)
            if resource_category == "warehouse":
                logger.debug(f"[同步→Bioyond] 跳过仓库类型资源: {resource.name} (仓库是容器，不需要同步为物料)")
                return True

            logger.info(f"[同步→Bioyond] 收到物料变更: {resource.name}")

            # 获取物料的 Bioyond ID
            extra_info = getattr(resource, "unilabos_extra", {})
            material_bioyond_id = extra_info.get("material_bioyond_id")

            # ⭐ 如果没有 Bioyond ID，尝试从 Bioyond 系统中按名称查询
            if not material_bioyond_id:
                logger.warning(f"[同步→Bioyond] 物料 {resource.name} 没有 Bioyond ID，尝试按名称查询...")
                try:
                    # 查询所有类型的物料：0=耗材, 1=样品, 2=试剂
                    import json
                    all_materials = []

                    for type_mode in [0, 1, 2]:
                        query_params = json.dumps({
                            "typeMode": type_mode,
                            "filter": "",   # 空字符串表示查询所有
                            "includeDetail": True
                        })
                        materials = self.bioyond_api_client.stock_material(query_params)
                        if materials:
                            all_materials.extend(materials)

                    logger.info(f"[同步→Bioyond] 查询到 {len(all_materials)} 个物料")

                    # 按名称匹配
                    for mat in all_materials:
                        if mat.get("name") == resource.name:
                            material_bioyond_id = mat.get("id")
                            mat_type = mat.get("typeName", "未知")
                            logger.info(f"✅ 找到物料 {resource.name} ({mat_type}) 的 Bioyond ID: {material_bioyond_id[:8]}...")
                            # 保存 ID 到资源对象
                            extra_info["material_bioyond_id"] = material_bioyond_id
                            setattr(resource, "unilabos_extra", extra_info)
                            break

                    if not material_bioyond_id:
                        logger.warning(f"⚠️ 在 Bioyond 系统中未找到名为 {resource.name} 的物料")
                        logger.info(f"[同步→Bioyond] 这是一个新物料，将创建并入库到 Bioyond 系统")
                        # 不返回，继续执行后续的创建+入库流程
                except Exception as e:
                    logger.error(f"查询 Bioyond 物料失败: {e}")
                    return False

            # 检查是否有位置更新请求
            update_site = extra_info.get("update_resource_site")

            if not update_site:
                logger.debug(f"[同步→Bioyond] 物料 {resource.name} 无位置更新请求，跳过同步")
                return True

            # ===== 物料移动/创建流程 =====
            logger.info(f"[同步→Bioyond] 📍 物料 {resource.name} 目标库位: {update_site}")

            if material_bioyond_id:
                logger.info(f"[同步→Bioyond] 🔄 物料已存在于 Bioyond (ID: {material_bioyond_id[:8]}...)，执行移动操作")
            else:
                logger.info(f"[同步→Bioyond] ➕ 物料不存在于 Bioyond，将创建新物料并入库")

            # 第1步：获取仓库配置
            from .config import WAREHOUSE_MAPPING
            warehouse_mapping = WAREHOUSE_MAPPING

            # 确定目标仓库名称（优先使用 resource.parent.name）
            parent_name = None
            target_location_uuid = None

            # 如果资源有父节点，优先使用父节点名称
            if resource.parent is not None:
                parent_name = resource.parent.name
                logger.info(f"[同步→Bioyond] 从资源父节点获取仓库名称: {parent_name}")

                # 检查该仓库是否在配置中
                if parent_name in warehouse_mapping:
                    site_uuids = warehouse_mapping[parent_name].get("site_uuids", {})
                    if update_site in site_uuids:
                        target_location_uuid = site_uuids[update_site]
                        logger.info(f"[同步→Bioyond] 目标仓库: {parent_name}/{update_site}")
                        logger.info(f"[同步→Bioyond] 目标库位UUID: {target_location_uuid[:8]}...")
                    else:
                        logger.warning(f"⚠️ [同步→Bioyond] 仓库 {parent_name} 中没有库位 {update_site}")
                else:
                    logger.warning(f"⚠️ [同步→Bioyond] 仓库 {parent_name} 未在 WAREHOUSE_MAPPING 中配置")
                    parent_name = None

            # 如果没有找到，则遍历所有仓库查找
            if not parent_name or not target_location_uuid:
                logger.info(f"[同步→Bioyond] 从所有仓库中查找库位 {update_site}...")
                for warehouse_name, warehouse_info in warehouse_mapping.items():
                    site_uuids = warehouse_info.get("site_uuids", {})
                    if update_site in site_uuids:
                        parent_name = warehouse_name
                        target_location_uuid = site_uuids[update_site]
                        logger.info(f"[同步→Bioyond] 目标仓库: {parent_name}/{update_site}")
                        logger.info(f"[同步→Bioyond] 目标库位UUID: {target_location_uuid[:8]}...")
                        break

            if not parent_name or not target_location_uuid:
                logger.error(f"❌ [同步→Bioyond] 库位 {update_site} 没有在 WAREHOUSE_MAPPING 中配置")
                logger.debug(f"[同步→Bioyond] 可用仓库: {list(warehouse_mapping.keys())}")
                return False

            # 第2步：转换为 Bioyond 格式
            logger.info(f"[同步→Bioyond] 🔄 转换物料为 Bioyond 格式...")
            bioyond_material = resource_plr_to_bioyond(
                [resource],
                type_mapping=self.workstation.bioyond_config["material_type_mappings"],
                warehouse_mapping=self.workstation.bioyond_config["warehouse_mapping"]
            )[0]

            logger.debug(f"[同步→Bioyond] Bioyond 物料数据: {bioyond_material}")

            location_info = bioyond_material.pop("locations", None)
            logger.debug(f"[同步→Bioyond] 库位信息: {location_info}, 类型: {type(location_info)}")

            # 第3步：根据是否已有 Bioyond ID 决定创建还是使用现有物料
            if material_bioyond_id:
                # 物料已存在,直接使用现有 ID
                material_id = material_bioyond_id
                logger.info(f"✅ [同步→Bioyond] 使用已有物料 ID: {material_id[:8]}...")
            else:
                # 物料不存在,调用 API 创建新物料
                logger.info(f"[同步→Bioyond] 📤 调用 Bioyond API 添加物料...")
                material_id = self.bioyond_api_client.add_material(bioyond_material)

                if not material_id:
                    logger.error(f"❌ [同步→Bioyond] 添加物料失败，API 返回空")
                    return False

                logger.info(f"✅ [同步→Bioyond] 物料添加成功，Bioyond ID: {material_id[:8]}...")

                # 保存新创建的物料 ID 到资源对象
                extra_info["material_bioyond_id"] = material_id
                setattr(resource, "unilabos_extra", extra_info)

            # 第4步：物料入库前先检查目标库位是否被占用
            if location_info:
                logger.info(f"[同步→Bioyond] 📥 准备入库到库位 {update_site}...")

                # 处理不同的 location_info 数据结构
                if isinstance(location_info, list) and len(location_info) > 0:
                    location_id = location_info[0]["id"]
                elif isinstance(location_info, dict):
                    location_id = location_info["id"]
                else:
                    logger.warning(f"⚠️ [同步→Bioyond] 无效的库位信息格式: {location_info}")
                    location_id = None

                if location_id:
                    # 查询目标库位是否已有物料
                    logger.info(f"[同步→Bioyond] 🔍 检查库位 {update_site} (UUID: {location_id[:8]}...) 是否被占用...")

                    # 查询所有物料，检查是否有物料在目标库位
                    try:
                        all_materials_type1 = self.bioyond_api_client.stock_material('{"typeMode": 1, "includeDetail": true}')
                        all_materials_type2 = self.bioyond_api_client.stock_material('{"typeMode": 2, "includeDetail": true}')
                        all_materials = (all_materials_type1 or []) + (all_materials_type2 or [])

                        # 检查是否有物料已经在目标库位
                        location_occupied = False
                        occupying_material = None

                        for material in all_materials:
                            locations = material.get("locations", [])
                            for loc in locations:
                                if loc.get("id") == location_id:
                                    location_occupied = True
                                    occupying_material = material
                                    logger.warning(f"⚠️ [同步→Bioyond] 库位 {update_site} 已被占用！")
                                    logger.warning(f"   占用物料: {material.get('name')} (ID: {material.get('id', '')[:8]}...)")
                                    logger.warning(f"   占用位置: code={loc.get('code')}, x={loc.get('x')}, y={loc.get('y')}")
                                    logger.warning(f"   🔍 详细信息: location_id={loc.get('id')[:8]}..., 目标UUID={location_id[:8]}...")
                                    logger.warning(f"   🔍 完整location数据: {loc}")
                                    break
                            if location_occupied:
                                break

                        if location_occupied:
                            # 如果是同一个物料（名称相同），说明已经入库过了，跳过
                            if occupying_material and occupying_material.get("name") == resource.name:
                                logger.info(f"✅ [同步→Bioyond] 物料 {resource.name} 已经在库位 {update_site}，跳过重复入库")
                                return True
                            else:
                                logger.error(f"❌ [同步→Bioyond] 库位 {update_site} 已被其他物料占用，拒绝入库")
                                return False

                        logger.info(f"✅ [同步→Bioyond] 库位 {update_site} 可用，准备入库...")

                    except Exception as e:
                        logger.warning(f"⚠️ [同步→Bioyond] 检查库位状态时发生异常: {e}，继续尝试入库...")

                    # 执行入库
                    logger.info(f"[同步→Bioyond] 📥 调用 Bioyond API 物料入库...")
                    response = self.bioyond_api_client.material_inbound(material_id, location_id)

                    # 注意：Bioyond API 成功时返回空字典 {}，所以不能用 if not response 判断
                    # 只要没有抛出异常，就认为成功（response 是 dict 类型，即使是 {} 也不是 None）
                    if response is not None:
                        logger.info(f"✅ [同步→Bioyond] 物料 {resource.name} 成功入库到 {update_site}")

                        # 入库成功后，重新查询验证物料实际入库位置
                        logger.info(f"[同步→Bioyond] 🔍 验证物料实际入库位置...")
                        try:
                            all_materials_type1 = self.bioyond_api_client.stock_material('{"typeMode": 1, "includeDetail": true}')
                            all_materials_type2 = self.bioyond_api_client.stock_material('{"typeMode": 2, "includeDetail": true}')
                            all_materials = (all_materials_type1 or []) + (all_materials_type2 or [])

                            for material in all_materials:
                                if material.get("id") == material_id:
                                    locations = material.get("locations", [])
                                    if locations:
                                        actual_loc = locations[0]
                                        logger.info(f"📍 [同步→Bioyond] 物料实际位置: code={actual_loc.get('code')}, "
                                                  f"warehouse={actual_loc.get('whName')}, "
                                                  f"x={actual_loc.get('x')}, y={actual_loc.get('y')}")

                                        # 验证 UUID 是否匹配
                                        if actual_loc.get("id") != location_id:
                                            logger.error(f"❌ [同步→Bioyond] UUID 不匹配！")
                                            logger.error(f"   预期 UUID: {location_id}")
                                            logger.error(f"   实际 UUID: {actual_loc.get('id')}")
                                            logger.error(f"   这说明配置文件中的 UUID 映射有误，请检查 config.py 中的 WAREHOUSE_MAPPING")
                                    break
                        except Exception as e:
                            logger.warning(f"⚠️ [同步→Bioyond] 验证入库位置时发生异常: {e}")
                    else:
                        logger.error(f"❌ [同步→Bioyond] 物料入库失败")
                        return False
                else:
                    logger.warning(f"⚠️ [同步→Bioyond] 无法获取库位 ID，跳过入库操作")
            else:
                logger.warning(f"⚠️ [同步→Bioyond] 物料没有库位信息，跳过入库操作")
            return True

        except Exception as e:
            logger.error(f"❌ [同步→Bioyond] 同步物料 {resource.name} 时发生异常: {e}")
            import traceback
            traceback.print_exc()
            return False

    def handle_external_change(self, change_info: Dict[str, Any]) -> bool:
        """处理Bioyond系统的变更通知"""
        try:
            # 这里可以实现对Bioyond变更的处理逻辑
            logger.info(f"处理Bioyond变更通知: {change_info}")

            return True
        except Exception as e:
            logger.error(f"处理Bioyond变更通知失败: {e}")
            return False


class BioyondWorkstation(WorkstationBase):
    """Bioyond工作站

    集成Bioyond物料管理的工作站实现
    """

    def __init__(
        self,
        bioyond_config: Optional[Dict[str, Any]] = None,
        deck: Optional[Any] = None,
        *args,
        **kwargs,
    ):
        # 初始化父类
        super().__init__(
            # 桌子
            deck=deck,
            *args,
            **kwargs,
        )

        # 检查 deck 是否为 None，防止 AttributeError
        if self.deck is None:
            logger.error("❌ Deck 配置为空，请检查配置文件中的 deck 参数")
            raise ValueError("Deck 配置不能为空，请在配置文件中添加正确的 deck 配置")

        # 初始化 warehouses 属性
        self.deck.warehouses = {}
        for resource in self.deck.children:
            if isinstance(resource, WareHouse):
                self.deck.warehouses[resource.name] = resource

        # 创建通信模块
        self._create_communication_module(bioyond_config)
        self.resource_synchronizer = BioyondResourceSynchronizer(self)
        self.resource_synchronizer.sync_from_external()

        # TODO: self._ros_node里面拿属性

        # 工作流加载
        self.is_running = False
        self.workflow_mappings = {}
        self.workflow_sequence = []
        self.pending_task_params = []

        if "workflow_mappings" in bioyond_config:
            self._set_workflow_mappings(bioyond_config["workflow_mappings"])
        logger.info(f"Bioyond工作站初始化完成")

    def post_init(self, ros_node: ROS2WorkstationNode):
        self._ros_node = ros_node

        # ⭐ 上传 deck（包括所有 warehouses 及其中的物料）
        # 注意：如果有从 Bioyond 同步的物料，它们已经被放置到 warehouse 中了
        # 所以只需要上传 deck，物料会作为 warehouse 的 children 一起上传
        logger.info("正在上传 deck（包括 warehouses 和物料）到云端...")
        ROS2DeviceNode.run_async_func(self._ros_node.update_resource, True, **{
            "resources": [self.deck]
        })

        # 清理临时变量（物料已经在 deck 的 warehouse children 中，不需要单独上传）
        if hasattr(self, "_synced_resources"):
            logger.info(f"✅ {len(self._synced_resources)} 个从Bioyond同步的物料已包含在 deck 中")
            self._synced_resources = []

    def transfer_resource_to_another(self, resource: List[ResourceSlot], mount_resource: List[ResourceSlot], sites: List[str], mount_device_id: DeviceSlot):
        time.sleep(3)
        ROS2DeviceNode.run_async_func(self._ros_node.transfer_resource_to_another, True, **{
            "plr_resources": resource,
            "target_device_id": mount_device_id,
            "target_resources": mount_resource,
            "sites": sites,
        })

    def _create_communication_module(self, config: Optional[Dict[str, Any]] = None) -> None:
        """创建Bioyond通信模块"""
        # 创建默认配置
        default_config = {
            **API_CONFIG,
            "workflow_mappings": WORKFLOW_MAPPINGS,
            "material_type_mappings": MATERIAL_TYPE_MAPPINGS,
            "warehouse_mapping": WAREHOUSE_MAPPING
        }

        # 如果传入了 config，合并配置（config 中的值会覆盖默认值）
        if config:
            self.bioyond_config = {**default_config, **config}
        else:
            self.bioyond_config = default_config

        self.hardware_interface = BioyondV1RPC(self.bioyond_config)

    def resource_tree_add(self, resources: List[ResourcePLR]) -> None:
        """添加资源到资源树并更新ROS节点

        Args:
            resources (List[ResourcePLR]): 要添加的资源列表
        """
        logger.info(f"[resource_tree_add] 开始同步 {len(resources)} 个资源到 Bioyond 系统")
        for resource in resources:
            try:
                # 🔍 检查资源是否已有 Bioyond ID (避免重复入库)
                extra_info = getattr(resource, "unilabos_extra", {})
                material_bioyond_id = extra_info.get("material_bioyond_id")

                if material_bioyond_id:
                    logger.info(f"⏭️ [resource_tree_add] 跳过资源 {resource.name}: 已有 Bioyond ID ({material_bioyond_id[:8]}...)，可能由 transfer 已处理")
                    continue

                logger.info(f"[resource_tree_add] 同步资源: {resource}")
                self.resource_synchronizer.sync_to_external(resource)
            except Exception as e:
                logger.error(f"[resource_tree_add] 同步资源失败 {resource}: {e}")
                import traceback
                traceback.print_exc()

    def resource_tree_transfer(self, old_parent: Optional[ResourcePLR], resource: ResourcePLR, new_parent: ResourcePLR) -> None:
        """处理资源在设备间迁移时的同步

        当资源从一个设备迁移到 BioyondWorkstation 时，需要同步到 Bioyond 系统

        Args:
            old_parent: 资源的原父节点（可能为 None）
            resource: 要迁移的资源
            new_parent: 资源的新父节点
        """
        logger.info(f"[resource_tree_transfer] 资源迁移: {resource.name}")
        logger.info(f"  旧父节点: {old_parent.name if old_parent else 'None'}")
        logger.info(f"  新父节点: {new_parent.name}")

        try:
            # 同步资源到 Bioyond 系统
            logger.info(f"[resource_tree_transfer] 开始同步资源 {resource.name} 到 Bioyond 系统")
            result = self.resource_synchronizer.sync_to_external(resource)

            if result:
                logger.info(f"✅ [resource_tree_transfer] 资源 {resource.name} 成功同步到 Bioyond 系统")
            else:
                logger.warning(f"⚠️ [resource_tree_transfer] 资源 {resource.name} 同步到 Bioyond 系统失败")

        except Exception as e:
            logger.error(f"❌ [resource_tree_transfer] 资源 {resource.name} 同步异常: {e}")
            import traceback
            traceback.print_exc()

    @property
    def bioyond_status(self) -> Dict[str, Any]:
        """获取 Bioyond 系统状态信息

        这个属性被 ROS 节点用来发布设备状态

        Returns:
            Dict[str, Any]: Bioyond 系统的状态信息
        """
        try:
            # 基础状态信息
            status = {
            }

            # 如果有反应站接口，获取调度器状态
            if self.hardware_interface:
                try:
                    scheduler_status = self.hardware_interface.scheduler_status()
                    status["scheduler"] = scheduler_status
                except Exception as e:
                    logger.warning(f"获取调度器状态失败: {e}")
                    status["scheduler"] = {"error": str(e)}

            # 添加物料缓存信息
            if self.hardware_interface:
                try:
                    available_materials = self.hardware_interface.get_available_materials()
                    status["material_cache_count"] = len(available_materials)
                except Exception as e:
                    logger.warning(f"获取物料缓存失败: {e}")
                    status["material_cache_count"] = 0

            return status

        except Exception as e:
            logger.error(f"获取Bioyond状态失败: {e}")
            return {
                "status": "error",
                "message": str(e),
                "station_type": getattr(self, 'station_type', 'unknown'),
                "station_name": getattr(self, 'station_name', 'unknown')
            }

    # ==================== 工作流合并与参数设置 API ====================

    def append_to_workflow_sequence(self, web_workflow_name: str) -> bool:
        # 检查是否为JSON格式的字符串
        actual_workflow_name = web_workflow_name
        if web_workflow_name.startswith('{') and web_workflow_name.endswith('}'):
            try:
                data = json.loads(web_workflow_name)
                actual_workflow_name = data.get("web_workflow_name", web_workflow_name)
                print(f"解析JSON格式工作流名称: {web_workflow_name} -> {actual_workflow_name}")
            except json.JSONDecodeError:
                print(f"JSON解析失败，使用原始字符串: {web_workflow_name}")

        workflow_id = self._get_workflow(actual_workflow_name)
        if workflow_id:
            self.workflow_sequence.append(workflow_id)
            print(f"添加工作流到执行顺序: {actual_workflow_name} -> {workflow_id}")
            return True
        return False

    def set_workflow_sequence(self, json_str: str) -> List[str]:
        try:
            data = json.loads(json_str)
            web_workflow_names = data.get("web_workflow_names", [])
        except:
            return []

        sequence = []
        for web_name in web_workflow_names:
            workflow_id = self._get_workflow(web_name)
            if workflow_id:
                sequence.append(workflow_id)

    def get_all_workflows(self) -> Dict[str, str]:
        return self.workflow_mappings.copy()

    def _get_workflow(self, web_workflow_name: str) -> str:
        if web_workflow_name not in self.workflow_mappings:
            print(f"未找到工作流映射配置: {web_workflow_name}")
            return ""
        workflow_id = self.workflow_mappings[web_workflow_name]
        print(f"获取工作流: {web_workflow_name} -> {workflow_id}")
        return workflow_id

    def _set_workflow_mappings(self, mappings: Dict[str, str]):
        self.workflow_mappings = mappings
        print(f"设置工作流映射配置: {mappings}")

    def process_web_workflows(self, json_str: str) -> Dict[str, str]:
        try:
            data = json.loads(json_str)
            web_workflow_list = data.get("web_workflow_list", [])
        except json.JSONDecodeError:
            print(f"无效的JSON字符串: {json_str}")
            return {}
        result = {}

        self.workflow_sequence = []
        for web_name in web_workflow_list:
            workflow_id = self._get_workflow(web_name)
            if workflow_id:
                result[web_name] = workflow_id
                self.workflow_sequence.append(workflow_id)
            else:
                print(f"无法获取工作流ID: {web_name}")
        print(f"工作流执行顺序: {self.workflow_sequence}")
        return result

    def clear_workflows(self):
        self.workflow_sequence = []
        print("清空工作流执行顺序")

    # ==================== 基础物料管理接口 ====================

    # ============ 工作站状态管理 ============
    def get_station_info(self) -> Dict[str, Any]:
        """获取工作站基础信息

        Returns:
            Dict[str, Any]: 工作站基础信息，包括设备ID、状态等
        """
        return {
            "device_id": getattr(self._ros_node, 'device_id', 'unknown'),
            "station_type": "BioyondWorkstation",
            "workflow_status": self.current_workflow_status.value if hasattr(self, 'current_workflow_status') else "unknown",
            "is_busy": getattr(self, 'is_busy', False),
            "deck_info": {
                "name": self.deck.name if self.deck and hasattr(self.deck, 'name') else "unknown",
                "children_count": len(self.deck.children) if self.deck and hasattr(self.deck, 'children') else 0
            } if self.deck else None,
            "hardware_interface": type(self.hardware_interface).__name__ if self.hardware_interface else None
        }

    def get_workstation_status(self) -> Dict[str, Any]:
        """获取工作站状态

        Returns:
            Dict[str, Any]: 工作站状态信息
        """
        try:
            # 获取基础工作站状态
            base_status = {
                "station_info": self.get_station_info(),
                "bioyond_status": self.bioyond_status
            }

            # 如果有接口，获取设备列表
            if self.hardware_interface:
                try:
                    devices = self.hardware_interface.device_list()
                    base_status["devices"] = devices
                except Exception as e:
                    logger.warning(f"获取设备列表失败: {e}")
                    base_status["devices"] = []

            return {
                "success": True,
                "data": base_status,
                "action": "get_workstation_status"
            }

        except Exception as e:
            error_msg = f"获取工作站状态失败: {str(e)}"
            logger.error(error_msg)
            return {
                "success": False,
                "message": error_msg,
                "action": "get_workstation_status"
            }

    def get_bioyond_status(self) -> Dict[str, Any]:
        """获取完整的 Bioyond 状态信息

        这个方法提供了比 bioyond_status 属性更详细的状态信息，
        包括错误处理和格式化的响应结构

        Returns:
            Dict[str, Any]: 格式化的 Bioyond 状态响应
        """
        try:
            status = self.bioyond_status
            return {
                "success": True,
                "data": status,
                "action": "get_bioyond_status"
            }

        except Exception as e:
            error_msg = f"获取 Bioyond 状态失败: {str(e)}"
            logger.error(error_msg)
            return {
                "success": False,
                "message": error_msg,
                "action": "get_bioyond_status"
            }

    def reset_workstation(self) -> Dict[str, Any]:
        """重置工作站

        重置工作站到初始状态

        Returns:
            Dict[str, Any]: 操作结果
        """
        try:
            logger.info("开始重置工作站")

            # 重置调度器
            if self.hardware_interface:
                self.hardware_interface.scheduler_reset()

            # 刷新物料缓存
            if self.hardware_interface:
                self.hardware_interface.refresh_material_cache()

            # 重新同步资源
            if self.resource_synchronizer:
                self.resource_synchronizer.sync_from_external()

            logger.info("工作站重置完成")
            return {
                "success": True,
                "message": "工作站重置成功",
                "action": "reset_workstation"
            }

        except Exception as e:
            error_msg = f"重置工作站失败: {str(e)}"
            logger.error(error_msg)
            return {
                "success": False,
                "message": error_msg,
                "action": "reset_workstation"
            }

    def load_bioyond_data_from_file(self, file_path: str) -> bool:
        """从文件加载Bioyond数据（用于测试）"""
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                bioyond_data = json.load(f)

            logger.info(f"从文件加载Bioyond数据: {file_path}")

            # 转换为UniLab格式
            unilab_resources = resource_bioyond_to_plr(
                bioyond_data,
                type_mapping=self.bioyond_config["material_type_mappings"],
                deck=self.deck
            )

            logger.info(f"成功加载 {len(unilab_resources)} 个资源")
            return True

        except Exception as e:
            logger.error(f"从文件加载Bioyond数据失败: {e}")
            return False


# 使用示例
def create_bioyond_workstation_example():
    """创建Bioyond工作站示例"""

    # 配置参数
    device_id = "bioyond_workstation_001"

    # 子资源配置
    children = {
        "plate_1": {
            "name": "plate_1",
            "type": "plate",
            "position": {"x": 100, "y": 100, "z": 0},
            "config": {
                "size_x": 127.76,
                "size_y": 85.48,
                "size_z": 14.35,
                "model": "Generic 96 Well Plate"
            }
        }
    }

    # Bioyond配置
    bioyond_config = {
        "base_url": "http://bioyond.example.com/api",
        "api_key": "your_api_key_here",
        "sync_interval": 60,  # 60秒同步一次
        "timeout": 30
    }

    # Deck配置
    deck_config = {
        "size_x": 1000.0,
        "size_y": 1000.0,
        "size_z": 100.0,
        "model": "BioyondDeck"
    }

    # 创建工作站
    workstation = BioyondWorkstation(
        station_resource=deck_config,
        bioyond_config=bioyond_config,
        deck_config=deck_config,
    )

    return workstation


if __name__ == "__main__":
    pass