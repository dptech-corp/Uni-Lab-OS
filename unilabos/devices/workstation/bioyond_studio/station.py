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

            # 同步所有类型的物料：耗材(0)、样品(1)和试剂(2)
            all_bioyond_data = []
            type_names = {0: "耗材", 1: "样品", 2: "试剂"}

            for type_mode in [0, 1, 2]:  # 0=耗材, 1=样品, 2=试剂
                logger.info(f"正在从Bioyond同步类型 {type_mode} ({type_names[type_mode]})...")
                bioyond_data = self.bioyond_api_client.stock_material(
                    f'{{"typeMode": {type_mode}, "includeDetail": true}}'
                )
                if bioyond_data:
                    logger.info(f"  类型 {type_mode} 同步了 {len(bioyond_data)} 个物料：")
                    for mat in bioyond_data:
                        mat_name = mat.get("name", "未知")
                        mat_type = mat.get("typeName", "未知")
                        locations = mat.get("locations", [])
                        if locations:
                            loc = locations[0]
                            wh_name = loc.get("whName", "未知")
                            coords = f"x={loc.get('x')},y={loc.get('y')},z={loc.get('z')}"
                            logger.info(f"    - {mat_name} ({mat_type}) @ {wh_name} [{coords}]")
                        else:
                            logger.info(f"    - {mat_name} ({mat_type}) @ 未入库")
                    all_bioyond_data.extend(bioyond_data)
                else:
                    logger.warning(f"  类型 {type_mode} 没有物料数据")

            if not all_bioyond_data:
                logger.warning("从Bioyond获取的物料数据为空")
                return False

            logger.info(f"总共获取 {len(all_bioyond_data)} 个物料，开始转换为UniLab格式...")

            # 转换为UniLab格式
            unilab_resources = resource_bioyond_to_plr(
                all_bioyond_data,
                type_mapping=self.workstation.bioyond_config["material_type_mappings"],
                deck=self.workstation.deck
            )

            # 保存 Bioyond 物料ID 到每个资源对象，用于后续更新
            for i, resource in enumerate(unilab_resources):
                if i < len(all_bioyond_data):
                    material_id = all_bioyond_data[i].get("id")
                    if material_id:
                        # ⭐ 修复：使用 unilabos_extra 字典保存 Bioyond ID
                        extra_info = getattr(resource, "unilabos_extra", {})
                        extra_info["material_bioyond_id"] = material_id
                        setattr(resource, "unilabos_extra", extra_info)
                        logger.debug(f"物料 {resource.name} 的 Bioyond ID: {material_id[:8]}...")

            # ⭐ 重要：保存同步的资源列表，稍后在 post_init 中上传到云端
            self.workstation._synced_resources = unilab_resources

            logger.info(f"✅ 从Bioyond同步完成，转换后得到 {len(unilab_resources)} 个UniLab资源")
            return True
        except Exception as e:
            logger.error(f"从Bioyond同步物料数据失败: {e}")
            traceback.print_exc()
            return False

    def sync_to_external(self, resource: Any) -> bool:
        """将本地物料数据变更同步到Bioyond系统

        ⚠️ Bioyond物料移动的正确流程：
        1. 出库 (outbound) - 物料被删除
        2. 新建物料 (add_material) - 使用相同名称和属性
        3. 入库 (inbound) - 新物料出现在新位置

        Args:
            resource: 要同步的资源（PLR格式）

        Returns:
            bool: True=成功, False=失败
        """
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
                    import traceback
                    traceback.print_exc()
                    return False

            # 检查是否有位置更新请求
            update_site = extra_info.get("update_resource_site")

            if not update_site:
                logger.debug(f"[同步→Bioyond] 无位置更新请求")
                return True

            # ===== 物料移动/创建流程 =====
            if material_bioyond_id:
                logger.info(f"[同步→Bioyond] 🔄 开始移动物料 {resource.name} 到 {update_site}")
            else:
                logger.info(f"[同步→Bioyond] ➕ 开始创建新物料 {resource.name} 并入库到 {update_site}")            # 第1步：获取仓库配置
            from .config import WAREHOUSE_MAPPING
            warehouse_mapping = WAREHOUSE_MAPPING

            # 确定目标仓库名称（通过遍历所有仓库的库位配置）
            parent_name = None
            target_location_uuid = None

            for warehouse_name, warehouse_info in warehouse_mapping.items():
                site_uuids = warehouse_info.get("site_uuids", {})
                if update_site in site_uuids:
                    parent_name = warehouse_name
                    target_location_uuid = site_uuids[update_site]
                    logger.info(f"[同步] 目标仓库: {parent_name}/{update_site}")
                    logger.info(f"[同步] 目标库位UUID: {target_location_uuid[:8]}...")
                    break

            if not parent_name or not target_location_uuid:
                logger.error(f"❌ 库位 {update_site} 没有在 WAREHOUSE_MAPPING 中配置")
                logger.debug(f"可用仓库: {list(warehouse_mapping.keys())}")
                return False

            # 第2步：查询物料当前状态（仅对已有物料）
            current_material_info = None
            current_location_id = None

            if material_bioyond_id:
                # 已有物料：查询当前状态
                try:
                    for type_mode in [0, 1, 2]:  # 0=耗材, 1=样品, 2=试剂
                        stock_data = self.bioyond_api_client.stock_material(
                            f'{{"typeMode": {type_mode}, "includeDetail": true}}'
                        )

                        for material in stock_data:
                            if material.get("id") == material_bioyond_id:
                                current_material_info = material  # 保存完整物料信息
                                locations = material.get("locations", [])
                                if locations:
                                    loc = locations[0]
                                    current_location_id = loc.get("id")
                                    wh_name = loc.get("whName", "")
                                    x, y, z = loc.get("x"), loc.get("y"), loc.get("z")
                                    row_letter = chr(64 + x) if x else "?"
                                    col_number = f"{y:02d}" if y else "?"
                                    current_pos = f"{row_letter}{col_number}"
                                    logger.info(f"[同步] 物料当前位置: {wh_name}/{current_pos} (location_id: {current_location_id[:8]}...)")
                                break

                        if current_material_info:
                            break
                except Exception as e:
                    logger.error(f"❌ 查询物料信息失败: {e}")
                    import traceback
                    traceback.print_exc()
                    return False

                if not current_material_info:
                    logger.error(f"❌ 在Bioyond系统中未找到物料: {resource.name} (ID: {material_bioyond_id})")
                    return False

                # 第3步：出库（删除旧物料）
                if current_location_id:
                    logger.info(f"[同步] 步骤1/4: 🔻 出库物料（删除）")
                    outbound_response = self.bioyond_api_client.material_outbound_by_id(
                        material_bioyond_id,
                        current_location_id,
                        quantity=1
                    )
                    if outbound_response is None:
                        logger.error(f"❌ 物料出库失败")
                        return False
                    logger.info(f"[同步] ✅ 物料已出库（已删除）")
                else:
                    logger.info(f"[同步] 物料不在库中，跳过出库步骤")
            else:
                # 新物料：从 resource 对象构建物料信息
                logger.info(f"[同步] 这是新物料，将从资源对象获取属性")
                current_material_info = {
                    "name": resource.name,
                    "typeName": "烧杯",  # 默认类型，稍后会根据实际情况确定
                    "unit": "微升",
                    "quantity": 1000.0,  # 默认容量
                }
                logger.info(f"[同步] 新物料属性: {current_material_info}")

            # 第4步：查询物料类型ID
            logger.info(f"[同步] 步骤2/4: 🔍 查询物料类型ID")

            type_name = current_material_info.get("typeName", "")
            type_id = None

            try:
                # 直接调用API查询物料类型列表
                response = self.bioyond_api_client.post(
                    url=f'{self.bioyond_api_client.host}/api/lims/storage/material-types',
                    params={
                        'apiKey': self.bioyond_api_client.api_key,
                        'requestTime': self.bioyond_api_client.get_current_time_iso8601(),
                        'data': ''
                    })

                if response and response.get('code') == 1:
                    types = response.get('data', [])
                    for t in types:
                        if t.get("name") == type_name:
                            type_id = t.get("id")
                            logger.info(f"[同步] 找到物料类型: {type_name} (ID: {type_id[:8]}...)")
                            break

                    if not type_id:
                        logger.warning(f"[同步] 未找到物料类型 {type_name}")
            except Exception as e:
                logger.error(f"[同步] 查询物料类型失败: {e}")
                import traceback
                traceback.print_exc()

            if not type_id:
                logger.error(f"❌ 无法获取物料类型ID")
                return False

            # 第5步：新建物料（使用原物料的属性）
            logger.info(f"[同步] 步骤3/4: ➕ 新建物料")

            # 按照API文档构建参数
            new_material_data = {
                "typeId": type_id,
                "name": current_material_info.get("name"),
                "unit": current_material_info.get("unit", "微升"),
                "quantity": current_material_info.get("quantity", 0),
                "code": "",  # 物料编码（可选）
                "barCode": "",  # 物料条码（可选）
                "parameters": "",  # 参数（必填，可以为空字符串）
                "details": []  # 孔物料信息（如果有detail字段则填充）
            }

            new_material_response = self.bioyond_api_client.add_material(new_material_data)

            # add_material 可能返回字典（包含id字段）或直接返回ID字符串
            if isinstance(new_material_response, str):
                new_material_id = new_material_response
            elif isinstance(new_material_response, dict) and "id" in new_material_response:
                new_material_id = new_material_response["id"]
            else:
                new_material_id = None

            if not new_material_id:
                logger.error(f"❌ 新建物料失败")
                return False

            new_material_id = new_material_response["id"]
            logger.info(f"[同步] ✅ 新物料已创建 (ID: {new_material_id[:8]}...)")

            # 第5步：入库到新位置
            logger.info(f"[同步] 步骤3/3: 📥 入库到新位置 {update_site}")
            inbound_response = self.bioyond_api_client.material_inbound(
                new_material_id,
                target_location_uuid
            )

            if inbound_response is not None:
                logger.info(f"[同步] ✅ 物料已入库到 {parent_name}/{update_site}")
                logger.info(f"[同步] 🎉 物料移动完成！{resource.name} → {parent_name}/{update_site}")

                # ⭐ 更新 resource 的 Bioyond ID 为新 ID
                extra_info["material_bioyond_id"] = new_material_id
                setattr(resource, "unilabos_extra", extra_info)

                return True
            else:
                logger.error(f"❌ 物料入库到新位置失败")
                logger.error(f"   警告：物料已出库但入库失败，需要手动在Bioyond系统中处理")
                logger.error(f"   新物料ID: {new_material_id}")
                return False

        except Exception as e:
            logger.error(f"[同步→Bioyond] 处理物料变更时出错: {e}")
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
        ROS2DeviceNode.run_async_func(self._ros_node.update_resource, True, **{
            "resources": [self.deck]
        })

        # ⭐ 上传从 Bioyond 同步的物料到云端数据库
        if hasattr(self, "_synced_resources") and self._synced_resources:
            try:
                logger.info(f"开始将 {len(self._synced_resources)} 个从Bioyond同步的物料上传到云端...")
                # 调用 ROS 节点的 update_resource 方法，确保物料被上传到云端
                ROS2DeviceNode.run_async_func(self._ros_node.update_resource, True, **{
                    "resources": self._synced_resources
                })
                logger.info("✅ 从Bioyond同步的物料已上传到云端数据库")
                # 清理临时变量
                self._synced_resources = []
            except Exception as e:
                logger.error(f"上传Bioyond同步物料到云端失败: {e}")
                import traceback
                traceback.print_exc()

    def transfer_resource_to_another(self, resource: List[ResourceSlot], mount_resource: List[ResourceSlot], sites: List[str], mount_device_id: DeviceSlot):
        ROS2DeviceNode.run_async_func(self._ros_node.transfer_resource_to_another, True, **{
            "plr_resources": resource,
            "target_device_id": mount_device_id,
            "target_resources": mount_resource,
            "sites": sites,
        })

    def _create_communication_module(self, config: Optional[Dict[str, Any]] = None) -> None:
        """创建Bioyond通信模块"""
        # 如果没有提供配置，或者配置不完整，使用默认配置
        if config is None:
            config = {}

        # 合并配置，确保所有必要的键都存在
        self.bioyond_config = {
            **API_CONFIG,
            "workflow_mappings": WORKFLOW_MAPPINGS,
            "material_type_mappings": MATERIAL_TYPE_MAPPINGS,
            "warehouse_mapping": WAREHOUSE_MAPPING,
            **config  # 用户配置覆盖默认配置
        }

        # 调试：输出配置信息
        logger.debug(f"Bioyond 配置加载完成:")
        logger.debug(f"  - warehouse_mapping 仓库数: {len(self.bioyond_config.get('warehouse_mapping', {}))}")
        logger.debug(f"  - material_type_mappings 类型数: {len(self.bioyond_config.get('material_type_mappings', {}))}")
        logger.debug(f"  - material_type_mappings 详情: {list(self.bioyond_config.get('material_type_mappings', {}).keys())}")
        logger.debug(f"  - workflow_mappings 工作流数: {len(self.bioyond_config.get('workflow_mappings', {}))}")

        self.hardware_interface = BioyondV1RPC(self.bioyond_config)

    def resource_tree_add(self, resources: List[ResourcePLR]) -> None:
        """添加资源到资源树并更新ROS节点

        Args:
            resources (List[ResourcePLR]): 要添加的资源列表
        """
        self.resource_synchronizer.sync_to_external(resources)

    def resource_tree_update(self, resources: List[ResourcePLR]) -> None:
        """更新资源信息并同步到Bioyond系统

        Args:
            resources (List[ResourcePLR]): 要更新的资源列表
        """
        try:
            logger.info(f"开始同步 {len(resources)} 个资源的更新到Bioyond系统")

            for resource in resources:
                # 调用资源同步器将更新同步到外部系统
                success = self.resource_synchronizer.sync_to_external(resource)
                if success:
                    logger.info(f"资源 {resource.name} 更新同步成功")
                else:
                    logger.warning(f"资源 {resource.name} 更新同步失败")

        except Exception as e:
            logger.error(f"同步资源更新到Bioyond失败: {e}")
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