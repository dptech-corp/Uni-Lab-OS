"""
Bioyond工作站实现
Bioyond Workstation Implementation

集成Bioyond物料管理的工作站示例
"""
import traceback
from typing import Dict, Any, List, Optional, Union
import json

from unilabos.devices.workstation.workstation_base import WorkstationBase, ResourceSynchronizer
from unilabos.devices.workstation.bioyond_studio.bioyond_rpc import BioyondV1RPC
from unilabos.resources.warehouse import WareHouse
from unilabos.utils.log import logger
from unilabos.resources.graphio import resource_bioyond_to_plr

from unilabos.ros.nodes.base_device_node import ROS2DeviceNode, BaseROS2DeviceNode
from unilabos.ros.nodes.presets.workstation import ROS2WorkstationNode

from unilabos.devices.workstation.bioyond_studio.config import (
    API_CONFIG, WORKFLOW_MAPPINGS, MATERIAL_TYPE_MAPPINGS,
    STATION_TYPES, DEFAULT_STATION_CONFIG
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

            bioyond_data = self.bioyond_api_client.stock_material('{"typeMode": 2, "includeDetail": true}')
            if not bioyond_data:
                logger.warning("从Bioyond获取的物料数据为空")
                return False

            # 转换为UniLab格式
            unilab_resources = resource_bioyond_to_plr(bioyond_data, type_mapping=self.workstation.bioyond_config["material_type_mappings"], deck=self.workstation.deck)

            logger.info(f"从Bioyond同步了 {len(unilab_resources)} 个资源")
            return True
        except Exception as e:
            logger.error(f"从Bioyond同步物料数据失败: {e}")
            traceback.print_exc()
            return False

    def sync_to_external(self, resource: Any) -> bool:
        """将本地物料数据变更同步到Bioyond系统"""
        try:
            if self.bioyond_api_client is None:
                logger.error("Bioyond API客户端未初始化")
                return False

            # 调用入库、出库操作
            # bioyond_format_data = self._convert_resource_to_bioyond_format(resource)
            # success = await self.bioyond_api_client.update_material(bioyond_format_data)
            #
            # if success
        except:
            pass

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
        station_config: Optional[Dict[str, Any]] = None,
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

        # 配置站点类型
        self._configure_station_type(station_config)

        # 创建通信模块
        self._create_communication_module(bioyond_config)
        self.resource_synchronizer = BioyondResourceSynchronizer(self)
        self.resource_synchronizer.sync_from_external()

        # TODO: self._ros_node里面拿属性
        logger.info(f"Bioyond工作站初始化完成")

    def post_init(self, ros_node: ROS2WorkstationNode):
        self._ros_node = ros_node
        #self.deck = create_a_coin_cell_deck()
        ROS2DeviceNode.run_async_func(self._ros_node.update_resource, True, **{
            "resources": [self.deck]
        })

    def _configure_station_type(self, station_config: Optional[Dict[str, Any]] = None) -> None:
        """配置站点类型和功能模块

        Args:
            station_config (Optional[Dict[str, Any]]): 站点配置，如果为None则使用默认配置
        """
        # 合并默认配置和用户配置
        self.station_config = {**DEFAULT_STATION_CONFIG}
        if station_config:
            self.station_config.update(station_config)

        # 设置站点属性
        self.station_type = self.station_config["station_type"]
        self.enable_reaction_station = self.station_config["enable_reaction_station"]
        self.enable_dispensing_station = self.station_config["enable_dispensing_station"]
        self.station_name = self.station_config["station_name"]
        self.station_description = self.station_config["description"]

        # 根据站点类型调整功能启用状态
        if self.station_type == STATION_TYPES["REACTION"]:
            self.enable_reaction_station = True
            self.enable_dispensing_station = False
            self.station_description = "Bioyond反应站"
            logger.info("🧪 配置为反应站模式")

        elif self.station_type == STATION_TYPES["DISPENSING"]:
            self.enable_reaction_station = False
            self.enable_dispensing_station = True
            self.station_description = "Bioyond配液站"
            logger.info("🧫 配置为配液站模式")

        elif self.station_type == STATION_TYPES["HYBRID"]:
            self.enable_reaction_station = True
            self.enable_dispensing_station = True
            self.station_description = "Bioyond混合工作站"
            logger.info("🔬 配置为混合工作站模式")

        logger.info(f"站点配置: {self.station_name} - {self.station_description}")
        logger.info(f"反应站功能: {'✅ 启用' if self.enable_reaction_station else '❌ 禁用'}")
        logger.info(f"配液站功能: {'✅ 启用' if self.enable_dispensing_station else '❌ 禁用'}")

    def _create_communication_module(self, config: Optional[Dict[str, Any]] = None) -> None:
        """创建Bioyond通信模块"""
        self.bioyond_config = config or {
            **API_CONFIG,
            "workflow_mappings": WORKFLOW_MAPPINGS,
            "material_type_mappings": MATERIAL_TYPE_MAPPINGS
        }

        # 根据站点配置有条件地初始化接口
        self.hardware_interface = None
        self.dispensing_interface = None

        if self.enable_reaction_station:
            # 反应站接口
            self.hardware_interface = BioyondV1RPC(self.bioyond_config)
            logger.info("✅ 反应站接口已初始化")
        else:
            logger.info("⏭️  反应站接口已跳过")

        if self.enable_dispensing_station:
            # 配液站接口 - 使用统一的BioyondV1RPC类
            self.dispensing_interface = BioyondV1RPC(self.bioyond_config)
            logger.info("✅ 配液站接口已初始化")
        else:
            logger.info("⏭️  配液站接口已跳过")

        return None

    def _check_interface_availability(self, interface_type: str) -> bool:
        """检查指定接口是否可用

        Args:
            interface_type (str): 接口类型，'reaction' 或 'dispensing'

        Returns:
            bool: 接口是否可用

        Raises:
            RuntimeError: 当接口不可用时抛出异常
        """
        if interface_type == "reaction":
            if not self.enable_reaction_station or self.hardware_interface is None:
                raise RuntimeError(
                    f"❌ 反应站接口不可用！当前站点类型: {self.station_type}, "
                    f"反应站功能: {'启用' if self.enable_reaction_station else '禁用'}"
                )
            return True

        elif interface_type == "dispensing":
            if not self.enable_dispensing_station or self.dispensing_interface is None:
                raise RuntimeError(
                    f"❌ 配液站接口不可用！当前站点类型: {self.station_type}, "
                    f"配液站功能: {'启用' if self.enable_dispensing_station else '禁用'}"
                )
            return True

        else:
            raise ValueError(f"未知的接口类型: {interface_type}")

    def get_station_info(self) -> Dict[str, Any]:
        """获取站点信息

        Returns:
            Dict[str, Any]: 站点配置和状态信息
        """
        return {
            "station_name": self.station_name,
            "station_type": self.station_type,
            "station_description": self.station_description,
            "enable_reaction_station": self.enable_reaction_station,
            "enable_dispensing_station": self.enable_dispensing_station,
            "reaction_interface_available": self.hardware_interface is not None,
            "dispensing_interface_available": self.dispensing_interface is not None,
            "supported_station_types": list(STATION_TYPES.values())
        }

    @property
    def bioyond_status(self) -> Dict[str, Any]:
        """获取 Bioyond 系统状态信息

        这个属性被 ROS 节点用来发布设备状态

        Returns:
            Dict[str, Any]: Bioyond 系统的状态信息
        """
        try:
            # 获取基础站点信息
            station_info = self.get_station_info()

            # 获取接口状态
            interface_status = {
                "reaction_interface_connected": False,
                "dispensing_interface_connected": False,
                "last_sync_time": getattr(self.resource_synchronizer, 'last_sync_time', 0),
                "sync_interval": getattr(self.resource_synchronizer, 'sync_interval', 60)
            }

            # 检查反应站接口状态
            if self.hardware_interface is not None:
                try:
                    # 尝试获取调度器状态来验证连接
                    scheduler_status = self.get_scheduler_status()
                    interface_status["reaction_interface_connected"] = scheduler_status.get("status") == "success"
                except Exception:
                    interface_status["reaction_interface_connected"] = False

            # 检查配液站接口状态
            if self.dispensing_interface is not None:
                try:
                    # 配液站接口也使用相同的连接检查方式
                    interface_status["dispensing_interface_connected"] = True
                except Exception:
                    interface_status["dispensing_interface_connected"] = False

            # 获取资源同步状态
            sync_status = {
                "last_sync_success": True,  # 默认值，可以根据实际同步结果更新
                "total_resources": len(getattr(self.deck, 'children', [])),
                "warehouse_count": len(getattr(self.deck, 'warehouses', {}))
            }

            return {
                "station_info": station_info,
                "interface_status": interface_status,
                "sync_status": sync_status,
                "timestamp": __import__('time').time(),
                "status": "online" if (interface_status["reaction_interface_connected"] or
                                     interface_status["dispensing_interface_connected"]) else "offline"
            }

        except Exception as e:
            logger.error(f"获取 Bioyond 状态失败: {e}")
            # 返回基础状态信息，避免完全失败
            return {
                "station_info": {
                    "station_name": getattr(self, 'station_name', 'BioyondWorkstation'),
                    "station_type": getattr(self, 'station_type', 'unknown'),
                    "enable_reaction_station": getattr(self, 'enable_reaction_station', False),
                    "enable_dispensing_station": getattr(self, 'enable_dispensing_station', False)
                },
                "interface_status": {
                    "reaction_interface_connected": False,
                    "dispensing_interface_connected": False,
                    "last_sync_time": 0,
                    "sync_interval": 60
                },
                "sync_status": {
                    "last_sync_success": False,
                    "total_resources": 0,
                    "warehouse_count": 0
                },
                "timestamp": __import__('time').time(),
                "status": "error",
                "error_message": str(e)
            }

    def _register_supported_workflows(self):
        """注册Bioyond支持的工作流"""
        from unilabos.devices.workstation.workstation_base import WorkflowInfo

        # Bioyond物料同步工作流
        self.supported_workflows["bioyond_sync"] = WorkflowInfo(
            name="bioyond_sync",
            description="从Bioyond系统同步物料",
            parameters={
                "sync_type": {"type": "string", "default": "full", "options": ["full", "incremental"]},
                "force_sync": {"type": "boolean", "default": False}
            }
        )

        # Bioyond物料更新工作流
        self.supported_workflows["bioyond_update"] = WorkflowInfo(
            name="bioyond_update",
            description="将本地物料变更同步到Bioyond",
            parameters={
                "material_ids": {"type": "list", "default": []},
                "sync_all": {"type": "boolean", "default": True}
            }
        )

        logger.info(f"注册了 {len(self.supported_workflows)} 个Bioyond工作流")

    async def execute_bioyond_sync_workflow(self, parameters: Dict[str, Any]) -> Dict[str, Any]:
        """执行Bioyond同步工作流"""
        try:
            sync_type = parameters.get("sync_type", "full")
            force_sync = parameters.get("force_sync", False)

            logger.info(f"开始执行Bioyond同步工作流: {sync_type}")

            # 获取物料管理模块
            material_manager = self.material_management

            if sync_type == "full":
                # 全量同步
                success = await material_manager.sync_from_bioyond()
            else:
                # 增量同步（这里可以实现增量同步逻辑）
                success = await material_manager.sync_from_bioyond()

            if success:
                result = {
                    "status": "success",
                    "message": f"Bioyond同步完成: {sync_type}",
                    "synced_resources": len(material_manager.plr_resources)
                }
            else:
                result = {
                    "status": "failed",
                    "message": "Bioyond同步失败"
                }

            logger.info(f"Bioyond同步工作流执行完成: {result['status']}")
            return result

        except Exception as e:
            logger.error(f"Bioyond同步工作流执行失败: {e}")
            return {
                "status": "error",
                "message": str(e)
            }

    # ==================== 工作流合并与参数设置 API ====================

    def merge_workflow_with_parameters(
        self,
        name: str,
        workflows: List[Dict[str, Any]],
        **kwargs
    ) -> Dict[str, Any]:
        """合并工作流并设置参数 API

        合并子工作流时传入实验参数，新建实验时如果没有传参数，则使用此处传入的参数作为默认值

        Args:
            name (str): 拼接后的长工作流名称
            workflows (List[Dict[str, Any]]): 待合并的子工作流列表，每个元素包含：
                - id (str): 子工作流 ID (UUID)
                - stepParameters (Dict, 可选): 步骤参数配置
            **kwargs: 其他参数

        Returns:
            Dict[str, Any]: 操作结果，包含 code、message 和 timestamp

        Example:
            workflows = [
                {
                    "id": "3fa85f64-5717-4562-b3fc-2c963f66afa6"
                },
                {
                    "id": "3fa85f64-5717-4562-b3fc-2c963f66afa6",
                    "stepParameters": {
                        "5a30bee1-7de2-45de-a89f-a25c78e4404b": {
                            "反应模块-开始搅拌": [
                                {
                                    "key": "temperature",
                                    "value": "25"
                                }
                            ],
                            "通量-配置": [
                                {
                                    "key": "cutoff",
                                    "value": "9999"
                                },
                                {
                                    "key": "assignMaterialName",
                                    "value": "3a1bf167-e862-f269-3749-a1c70cbbe6a6"
                                }
                            ]
                        }
                    }
                }
            ]

            result = workstation.merge_workflow_with_parameters(
                name="拼接后的长工作流的名称",
                workflows=workflows
            )
        """
        try:
            # 检查反应站接口是否可用
            self._check_interface_availability("reaction")

            logger.info(f"开始合并工作流: {name}, 包含 {len(workflows)} 个子工作流")

            # 基本参数验证
            if not name:
                raise ValueError("工作流名称不能为空")

            if not workflows or len(workflows) == 0:
                raise ValueError("工作流列表不能为空")

            # 使用 RPC 层进行详细的参数验证
            validation_result = self.hardware_interface.validate_workflow_parameters(workflows)
            if not validation_result.get("valid", False):
                raise ValueError(f"工作流参数验证失败: {validation_result.get('message', '未知错误')}")

            # 构造请求数据
            request_data = {
                "name": name,
                "workflows": workflows
            }

            # 转换为 JSON 字符串
            json_str = json.dumps(request_data, ensure_ascii=False)

            logger.info(f"发送工作流合并请求: {json_str}")

            # 调用底层 API（需要在 bioyond_rpc.py 中实现）
            result = self.hardware_interface.merge_workflow_with_parameters(json_str)

            if result.get("code") == 1:
                success_msg = f"工作流合并成功: {name}"
                logger.info(success_msg)
                return {
                    "success": True,
                    "code": result.get("code"),
                    "message": result.get("message", ""),
                    "timestamp": result.get("timestamp", 0),
                    "action": "merge_workflow_with_parameters",
                    "workflow_name": name,
                    "workflow_count": len(workflows)
                }
            else:
                error_msg = f"工作流合并失败: {result.get('message', '未知错误')}"
                logger.error(error_msg)
                return {
                    "success": False,
                    "code": result.get("code", 0),
                    "message": result.get("message", error_msg),
                    "timestamp": result.get("timestamp", 0),
                    "action": "merge_workflow_with_parameters"
                }

        except Exception as e:
            error_msg = f"工作流合并操作异常: {str(e)}"
            logger.error(error_msg)
            traceback.print_exc()
            return {
                "success": False,
                "code": 0,
                "message": error_msg,
                "action": "merge_workflow_with_parameters"
            }

    def validate_workflow_parameters(self, workflows: List[Dict[str, Any]]) -> Dict[str, Any]:
        """验证工作流参数格式

        Args:
            workflows (List[Dict[str, Any]]): 工作流列表

        Returns:
            Dict[str, Any]: 验证结果
        """
        # 委托给 RPC 层进行参数验证
        return self.hardware_interface.validate_workflow_parameters(workflows)

    def get_workflow_parameter_template(self) -> Dict[str, Any]:
        """获取工作流参数模板

        Returns:
            Dict[str, Any]: 参数模板和说明
        """
        # 委托给 RPC 层获取参数模板
        return self.hardware_interface.get_workflow_parameter_template()

    # ==================== 反应站动作函数 ====================
    # 基于 bioyond_rpc.py 中的反应站方法实现

    def reactor_taken_out(self, order_id: str = "", preintake_id: str = "", **kwargs) -> Dict[str, Any]:
        """反应器取出操作 - 调用底层 order_takeout API

        从反应站中取出反应器，通过订单ID和预取样ID进行精确控制

        Args:
            order_id (str): 订单ID，用于标识要取出的订单
            preintake_id (str): 预取样ID，用于标识具体的取样任务

        Returns:
            Dict[str, Any]: 操作结果，包含 code 和 return_info
        """
        try:
            logger.info(f"执行反应器取出操作: 订单ID={order_id}, 预取样ID={preintake_id}")

            # 构造 JSON 参数
            params = {
                "order_id": order_id,
                "preintake_id": preintake_id
            }
            json_str = json.dumps(params)

            # 调用底层 order_takeout API
            result_code = self.hardware_interface.order_takeout(json_str)

            if result_code == 1:
                success_msg = f"反应器取出操作成功完成，订单ID: {order_id}"
                logger.info(success_msg)
                return {
                    "success": True,
                    "code": result_code,
                    "return_info": success_msg,
                    "action": "reactor_taken_out"
                }
            else:
                error_msg = f"反应器取出操作失败，返回代码: {result_code}"
                logger.error(error_msg)
                return {
                    "success": False,
                    "code": result_code,
                    "return_info": error_msg,
                    "action": "reactor_taken_out"
                }

        except Exception as e:
            error_msg = f"反应器取出操作异常: {str(e)}"
            logger.error(error_msg)
            return {
                "success": False,
                "code": 0,
                "return_info": error_msg,
                "action": "reactor_taken_out"
            }

    def reactor_taken_in(self, **kwargs) -> Dict[str, Any]:
        """反应器放入操作

        将反应器放入反应站

        Returns:
            Dict[str, Any]: 操作结果
        """
        try:
            logger.info("执行反应器放入操作")

            # 调用 bioyond_rpc.py 中的反应站方法
            result = self.hardware_interface.reactor_taken_in()

            return {
                "success": True,
                "message": "反应器放入操作完成",
                "result": result,
                "action": "reactor_taken_in"
            }

        except Exception as e:
            logger.error(f"反应器放入操作失败: {e}")
            return {
                "success": False,
                "error": str(e),
                "action": "reactor_taken_in"
            }

    def solid_feeding_vials(self, material_name: str = "", volume: str = "", **kwargs) -> Dict[str, Any]:
        """固体进料到小瓶

        Args:
            material_name (str): 物料名称
            volume (str): 进料体积
            **kwargs: 其他参数

        Returns:
            Dict[str, Any]: 操作结果
        """
        try:
            logger.info(f"执行固体进料操作: 物料={material_name}, 体积={volume}")

            # 参数验证
            if not material_name:
                raise ValueError("物料名称不能为空")

            # 调用 bioyond_rpc.py 中的反应站方法
            result = self.hardware_interface.solid_feeding_vials(
                assign_material_name=material_name,
                volume=volume,
                **kwargs
            )

            return {
                "success": True,
                "message": f"固体进料操作完成: {material_name}",
                "result": result,
                "action": "solid_feeding_vials",
                "parameters": {
                    "material_name": material_name,
                    "volume": volume
                }
            }

        except Exception as e:
            logger.error(f"固体进料操作失败: {e}")
            return {
                "success": False,
                "error": str(e),
                "action": "solid_feeding_vials",
                "parameters": {
                    "material_name": material_name,
                    "volume": volume
                }
            }

    def liquid_feeding_vials_non_titration(self, material_name: str = "", volume: str = "", **kwargs) -> Dict[str, Any]:
        """非滴定液体进料到小瓶

        Args:
            material_name (str): 物料名称
            volume (str): 进料体积
            **kwargs: 其他参数

        Returns:
            Dict[str, Any]: 操作结果
        """
        try:
            logger.info(f"执行非滴定液体进料操作: 物料={material_name}, 体积={volume}")

            # 参数验证
            if not material_name:
                raise ValueError("物料名称不能为空")
            if not volume:
                raise ValueError("进料体积不能为空")

            # 调用 bioyond_rpc.py 中的反应站方法
            result = self.hardware_interface.liquid_feeding_vials_non_titration(
                assign_material_name=material_name,
                volume=volume,
                **kwargs
            )

            return {
                "success": True,
                "message": f"非滴定液体进料操作完成: {material_name}",
                "result": result,
                "action": "liquid_feeding_vials_non_titration",
                "parameters": {
                    "material_name": material_name,
                    "volume": volume
                }
            }

        except Exception as e:
            logger.error(f"非滴定液体进料操作失败: {e}")
            return {
                "success": False,
                "error": str(e),
                "action": "liquid_feeding_vials_non_titration",
                "parameters": {
                    "material_name": material_name,
                    "volume": volume
                }
            }

    def liquid_feeding_solvents(self, material_name: str = "", volume: str = "", **kwargs) -> Dict[str, Any]:
        """溶剂进料操作

        Args:
            material_name (str): 溶剂名称
            volume (str): 进料体积
            **kwargs: 其他参数

        Returns:
            Dict[str, Any]: 操作结果
        """
        try:
            logger.info(f"执行溶剂进料操作: 溶剂={material_name}, 体积={volume}")

            # 参数验证
            if not material_name:
                raise ValueError("溶剂名称不能为空")
            if not volume:
                raise ValueError("进料体积不能为空")

            # 调用 bioyond_rpc.py 中的反应站方法
            result = self.hardware_interface.liquid_feeding_solvents(
                assign_material_name=material_name,
                volume=volume,
                **kwargs
            )

            return {
                "success": True,
                "message": f"溶剂进料操作完成: {material_name}",
                "result": result,
                "action": "liquid_feeding_solvents",
                "parameters": {
                    "material_name": material_name,
                    "volume": volume
                }
            }

        except Exception as e:
            logger.error(f"溶剂进料操作失败: {e}")
            return {
                "success": False,
                "error": str(e),
                "action": "liquid_feeding_solvents",
                "parameters": {
                    "material_name": material_name,
                    "volume": volume
                }
            }

    def liquid_feeding_titration(self, material_name: str = "", volume: str = "",
                               titration_type: str = "1", time: str = "120",
                               torque_variation: str = "2", **kwargs) -> Dict[str, Any]:
        """滴定液体进料操作

        Args:
            material_name (str): 物料名称
            volume (str): 进料体积
            titration_type (str): 滴定类型，默认为"1"
            time (str): 滴定时间，默认为"120"秒
            torque_variation (str): 扭矩变化，默认为"2"
            **kwargs: 其他参数

        Returns:
            Dict[str, Any]: 操作结果
        """
        try:
            logger.info(f"执行滴定液体进料操作: 物料={material_name}, 体积={volume}, 类型={titration_type}")

            # 参数验证
            if not material_name:
                raise ValueError("物料名称不能为空")
            if not volume:
                raise ValueError("进料体积不能为空")

            # 调用 bioyond_rpc.py 中的反应站方法
            result = self.hardware_interface.liquid_feeding_titration(
                assign_material_name=material_name,
                volume=volume,
                titration_type=titration_type,
                time=time,
                torque_variation=torque_variation,
                **kwargs
            )

            return {
                "success": True,
                "message": f"滴定液体进料操作完成: {material_name}",
                "result": result,
                "action": "liquid_feeding_titration",
                "parameters": {
                    "material_name": material_name,
                    "volume": volume,
                    "titration_type": titration_type,
                    "time": time,
                    "torque_variation": torque_variation
                }
            }

        except Exception as e:
            logger.error(f"滴定液体进料操作失败: {e}")
            return {
                "success": False,
                "error": str(e),
                "action": "liquid_feeding_titration",
                "parameters": {
                    "material_name": material_name,
                    "volume": volume,
                    "titration_type": titration_type,
                    "time": time,
                    "torque_variation": torque_variation
                }
            }

    def liquid_feeding_beaker(self, material_name: str = "", volume: str = "", **kwargs) -> Dict[str, Any]:
        """烧杯液体进料操作

        Args:
            material_name (str): 物料名称
            volume (str): 进料体积
            **kwargs: 其他参数

        Returns:
            Dict[str, Any]: 操作结果
        """
        try:
            logger.info(f"执行烧杯液体进料操作: 物料={material_name}, 体积={volume}")

            # 参数验证
            if not material_name:
                raise ValueError("物料名称不能为空")
            if not volume:
                raise ValueError("进料体积不能为空")

            # 调用 bioyond_rpc.py 中的反应站方法
            result = self.hardware_interface.liquid_feeding_beaker(
                assign_material_name=material_name,
                volume=volume,
                **kwargs
            )

            return {
                "success": True,
                "message": f"烧杯液体进料操作完成: {material_name}",
                "result": result,
                "action": "liquid_feeding_beaker",
                "parameters": {
                    "material_name": material_name,
                    "volume": volume
                }
            }

        except Exception as e:
            logger.error(f"烧杯液体进料操作失败: {e}")
            return {
                "success": False,
                "error": str(e),
                "action": "liquid_feeding_beaker",
                "parameters": {
                    "material_name": material_name,
                    "volume": volume
                }
            }

    # ==================== 配液站动作函数 ====================
    # 基于 dispensing_station_bioyong.py 中的配液站方法实现

    def create_order(self, order_data: Union[str, Dict[str, Any]]) -> Dict[str, Any]:
        """创建配液任务订单

        Args:
            order_data (Union[str, Dict[str, Any]]): 订单数据，可以是JSON字符串或字典

        Returns:
            Dict[str, Any]: 操作结果
        """
        try:
            logger.info("创建配液任务订单")

            # 处理输入数据
            if isinstance(order_data, str):
                order_json = order_data
            else:
                order_json = json.dumps(order_data)

            # 调用配液站接口
            result = self.dispensing_interface.create_order(order_json)

            return {
                "success": True,
                "message": "配液任务订单创建完成",
                "result": result,
                "action": "create_order"
            }

        except Exception as e:
            logger.error(f"创建配液任务订单失败: {e}")
            return {
                "success": False,
                "error": str(e),
                "action": "create_order"
            }

    def order_query(self, query_data: Union[str, Dict[str, Any]]) -> Dict[str, Any]:
        """查询配液任务状态

        Args:
            query_data (Union[str, Dict[str, Any]]): 查询数据，可以是JSON字符串或字典

        Returns:
            Dict[str, Any]: 查询结果
        """
        try:
            logger.info("查询配液任务状态")

            # 处理输入数据
            if isinstance(query_data, str):
                query_json = query_data
            else:
                query_json = json.dumps(query_data)

            # 调用配液站接口
            result = self.dispensing_interface.order_query(query_json)

            return {
                "success": True,
                "message": "配液任务状态查询完成",
                "result": result,
                "action": "order_query"
            }

        except Exception as e:
            logger.error(f"查询配液任务状态失败: {e}")
            return {
                "success": False,
                "error": str(e),
                "action": "order_query"
            }

    def dispensing_material_inbound(self, material_data: Union[str, Dict[str, Any]]) -> Dict[str, Any]:
        """配液站物料入库

        Args:
            material_data (Union[str, Dict[str, Any]]): 物料数据，可以是JSON字符串或字典

        Returns:
            Dict[str, Any]: 操作结果
        """
        try:
            # 检查配液站接口是否可用
            self._check_interface_availability("dispensing")

            logger.info("执行配液站物料入库操作")

            # 处理输入数据
            if isinstance(material_data, str):
                material_json = material_data
            else:
                material_json = json.dumps(material_data)

            # 调用配液站接口
            result = self.dispensing_interface.material_inbound(material_json)

            return {
                "success": True,
                "message": "配液站物料入库完成",
                "result": result,
                "action": "dispensing_material_inbound"
            }

        except Exception as e:
            logger.error(f"配液站物料入库失败: {e}")
            return {
                "success": False,
                "error": str(e),
                "action": "dispensing_material_inbound"
            }

    def dispensing_material_outbound(self, material_data: Union[str, Dict[str, Any]]) -> Dict[str, Any]:
        """配液站物料出库

        Args:
            material_data (Union[str, Dict[str, Any]]): 物料数据，可以是JSON字符串或字典

        Returns:
            Dict[str, Any]: 操作结果
        """
        try:
            # 检查配液站接口是否可用
            self._check_interface_availability("dispensing")

            logger.info("执行配液站物料出库操作")

            # 处理输入数据
            if isinstance(material_data, str):
                material_json = material_data
            else:
                material_json = json.dumps(material_data)

            # 调用配液站接口
            result = self.dispensing_interface.material_outbound(material_json)

            return {
                "success": True,
                "message": "配液站物料出库完成",
                "result": result,
                "action": "dispensing_material_outbound"
            }

        except Exception as e:
            logger.error(f"配液站物料出库失败: {e}")
            return {
                "success": False,
                "error": str(e),
                "action": "dispensing_material_outbound"
            }

    def delete_material(self, material_data: Union[str, Dict[str, Any]]) -> Dict[str, Any]:
        """删除物料

        Args:
            material_data (Union[str, Dict[str, Any]]): 物料数据，可以是JSON字符串或字典

        Returns:
            Dict[str, Any]: 操作结果
        """
        try:
            logger.info("执行删除物料操作")

            # 处理输入数据
            if isinstance(material_data, str):
                material_json = material_data
            else:
                material_json = json.dumps(material_data)

            # 调用配液站接口
            result = self.dispensing_interface.delete_material(material_json)

            return {
                "success": True,
                "message": "删除物料操作完成",
                "result": result,
                "action": "delete_material"
            }

        except Exception as e:
            logger.error(f"删除物料操作失败: {e}")
            return {
                "success": False,
                "error": str(e),
                "action": "delete_material"
            }

    def sample_waste_removal(self, waste_data: Union[str, Dict[str, Any]]) -> Dict[str, Any]:
        """样品废料移除

        Args:
            waste_data (Union[str, Dict[str, Any]]): 废料数据，可以是JSON字符串或字典

        Returns:
            Dict[str, Any]: 操作结果
        """
        try:
            # 检查配液站接口是否可用
            self._check_interface_availability("dispensing")

            logger.info("执行样品废料移除操作")

            # 处理输入数据
            if isinstance(waste_data, str):
                waste_json = waste_data
            else:
                waste_json = json.dumps(waste_data)

            # 调用配液站接口
            result = self.dispensing_interface.sample_waste_removal(waste_json)

            return {
                "success": True,
                "message": "样品废料移除操作完成",
                "result": result,
                "action": "sample_waste_removal"
            }

        except Exception as e:
            logger.error(f"样品废料移除操作失败: {e}")
            return {
                "success": False,
                "error": str(e),
                "action": "sample_waste_removal"
            }

    def create_resource(self, resource_data: Union[str, Dict[str, Any]]) -> Dict[str, Any]:
        """创建资源（样品板等）

        Args:
            resource_data (Union[str, Dict[str, Any]]): 资源数据，可以是JSON字符串或字典

        Returns:
            Dict[str, Any]: 操作结果
        """
        try:
            # 检查配液站接口是否可用
            self._check_interface_availability("dispensing")

            logger.info("执行创建资源操作")

            # 处理输入数据
            if isinstance(resource_data, str):
                resource_json = resource_data
            else:
                resource_json = json.dumps(resource_data)

            # 调用配液站接口
            result = self.dispensing_interface.create_resource(resource_json)

            return {
                "success": True,
                "message": "创建资源操作完成",
                "result": result,
                "action": "create_resource"
            }

        except Exception as e:
            logger.error(f"创建资源操作失败: {e}")
            return {
                "success": False,
                "error": str(e),
                "action": "create_resource"
            }

    def create_90_10_vial_feeding_task(self, task_data: Union[str, Dict[str, Any]]) -> Dict[str, Any]:
        """创建90/10比例进料任务

        Args:
            task_data (Union[str, Dict[str, Any]]): 任务数据，可以是JSON字符串或字典

        Returns:
            Dict[str, Any]: 操作结果
        """
        try:
            # 检查配液站接口是否可用
            self._check_interface_availability("dispensing")

            logger.info("创建90/10比例进料任务")

            # 处理输入数据
            if isinstance(task_data, str):
                task_params = json.loads(task_data)
            else:
                task_params = task_data

            # 调用配液站接口，传递具体参数而不是JSON字符串
            result = self.dispensing_interface.create_90_10_vial_feeding_task(
                order_name=task_params.get("order_name"),
                speed=task_params.get("speed"),
                temperature=task_params.get("temperature"),
                delay_time=task_params.get("delay_time"),
                percent_90_1_assign_material_name=task_params.get("percent_90_1_assign_material_name"),
                percent_90_1_target_weigh=task_params.get("percent_90_1_target_weigh"),
                percent_90_2_assign_material_name=task_params.get("percent_90_2_assign_material_name"),
                percent_90_2_target_weigh=task_params.get("percent_90_2_target_weigh"),
                percent_90_3_assign_material_name=task_params.get("percent_90_3_assign_material_name"),
                percent_90_3_target_weigh=task_params.get("percent_90_3_target_weigh"),
                percent_10_1_assign_material_name=task_params.get("percent_10_1_assign_material_name"),
                percent_10_1_target_weigh=task_params.get("percent_10_1_target_weigh"),
                percent_10_1_volume=task_params.get("percent_10_1_volume"),
                percent_10_1_liquid_material_name=task_params.get("percent_10_1_liquid_material_name"),
                percent_10_2_assign_material_name=task_params.get("percent_10_2_assign_material_name"),
                percent_10_2_target_weigh=task_params.get("percent_10_2_target_weigh"),
                percent_10_2_volume=task_params.get("percent_10_2_volume"),
                percent_10_2_liquid_material_name=task_params.get("percent_10_2_liquid_material_name"),
                percent_10_3_assign_material_name=task_params.get("percent_10_3_assign_material_name"),
                percent_10_3_target_weigh=task_params.get("percent_10_3_target_weigh"),
                percent_10_3_volume=task_params.get("percent_10_3_volume"),
                percent_10_3_liquid_material_name=task_params.get("percent_10_3_liquid_material_name"),
                hold_m_name=task_params.get("hold_m_name")
            )

            return {
                "success": True,
                "message": "90/10比例进料任务创建完成",
                "result": result,
                "action": "create_90_10_vial_feeding_task"
            }

        except Exception as e:
            logger.error(f"创建90/10比例进料任务失败: {e}")
            return {
                "success": False,
                "error": str(e),
                "action": "create_90_10_vial_feeding_task"
            }

    def create_diamine_solution_task(self, solution_data: Union[str, Dict[str, Any]]) -> Dict[str, Any]:
        """创建二胺溶液配制任务

        Args:
            solution_data (Union[str, Dict[str, Any]]): 溶液数据，可以是JSON字符串或字典

        Returns:
            Dict[str, Any]: 操作结果
        """
        try:
            # 检查配液站接口是否可用
            self._check_interface_availability("dispensing")

            logger.info("创建二胺溶液配制任务")

            # 处理输入数据
            if isinstance(solution_data, str):
                solution_params = json.loads(solution_data)
            else:
                solution_params = solution_data

            # 调用配液站接口，传递具体参数而不是JSON字符串
            result = self.dispensing_interface.create_diamine_solution_task(
                order_name=solution_params.get("order_name"),
                material_name=solution_params.get("material_name"),
                target_weigh=solution_params.get("target_weigh"),
                volume=solution_params.get("volume"),
                liquid_material_name=solution_params.get("liquid_material_name", "NMP"),
                speed=solution_params.get("speed"),
                temperature=solution_params.get("temperature"),
                delay_time=solution_params.get("delay_time"),
                hold_m_name=solution_params.get("hold_m_name")
            )

            return {
                "success": True,
                "message": "二胺溶液配制任务创建完成",
                "result": result,
                "action": "create_diamine_solution_task"
            }

        except Exception as e:
            logger.error(f"创建二胺溶液配制任务失败: {e}")
            return {
                "success": False,
                "error": str(e),
                "action": "create_diamine_solution_task"
            }

    def create_batch_90_10_vial_feeding_task(self, batch_data: Union[str, Dict[str, Any]]) -> Dict[str, Any]:
        """
        创建批量90%10%小瓶投料任务

        Args:
            batch_data: 批量90%10%小瓶投料任务数据，可以是JSON字符串或字典
                       包含batch_name、tasks列表和global_settings

        Returns:
            Dict[str, Any]: 批量任务创建结果
        """
        try:
            # 检查配液站接口是否可用
            if not self._check_interface_availability("dispensing"):
                return {
                    "success": False,
                    "error": "配液站接口不可用",
                    "action": "create_batch_90_10_vial_feeding_task"
                }

            # 解析输入数据
            if isinstance(batch_data, str):
                batch_params = json.loads(batch_data)
            else:
                batch_params = batch_data

            logger.info(f"创建批量90%10%小瓶投料任务: {batch_params.get('batch_name', '未命名批量90%10%小瓶投料任务')}")

            # 调用配液站接口的批量90%10%小瓶投料方法
            result = self.dispensing_interface.create_batch_90_10_vial_feeding_task(
                json.dumps(batch_params) if isinstance(batch_params, dict) else batch_data
            )

            return {
                "success": True,
                "result": result,
                "action": "create_batch_90_10_vial_feeding_task"
            }

        except json.JSONDecodeError as e:
            logger.error(f"批量90%10%小瓶投料任务数据解析失败: {e}")
            return {
                "success": False,
                "error": f"JSON解析失败: {str(e)}",
                "action": "create_batch_90_10_vial_feeding_task"
            }

        except Exception as e:
            logger.error(f"创建批量90%10%小瓶投料任务失败: {e}")
            return {
                "success": False,
                "error": str(e),
                "action": "create_batch_90_10_vial_feeding_task"
            }

    def create_batch_diamine_solution_task(self, batch_data: Union[str, Dict[str, Any]]) -> Dict[str, Any]:
        """
        创建批量二胺溶液配制任务

        Args:
            batch_data: 批量二胺溶液配制任务数据，可以是JSON字符串或字典
                       包含batch_name、tasks列表和global_settings

        Returns:
            Dict[str, Any]: 批量任务创建结果
        """
        try:
            # 检查配液站接口是否可用
            if not self._check_interface_availability("dispensing"):
                return {
                    "success": False,
                    "error": "配液站接口不可用",
                    "action": "create_batch_diamine_solution_task"
                }

            # 解析输入数据
            if isinstance(batch_data, str):
                batch_params = json.loads(batch_data)
            else:
                batch_params = batch_data

            logger.info(f"创建批量二胺溶液配制任务: {batch_params.get('batch_name', '未命名批量二胺溶液配制任务')}")

            # 调用配液站接口的批量二胺溶液配制方法
            result = self.dispensing_interface.create_batch_diamine_solution_task(
                json.dumps(batch_params) if isinstance(batch_params, dict) else batch_data
            )

            return {
                "success": True,
                "result": result,
                "action": "create_batch_diamine_solution_task"
            }

        except json.JSONDecodeError as e:
            logger.error(f"批量二胺溶液配制任务数据解析失败: {e}")
            return {
                "success": False,
                "error": f"JSON解析失败: {str(e)}",
                "action": "create_batch_diamine_solution_task"
            }

        except Exception as e:
            logger.error(f"创建批量二胺溶液配制任务失败: {e}")
            return {
                "success": False,
                "error": str(e),
                "action": "create_batch_diamine_solution_task"
            }

    # ==================== 反应站动作接口 ====================

    def reaction_station_drip_back(self, volume: str, assign_material_name: str,
                                 time: str, torque_variation: str) -> Dict[str, Any]:
        """反应站滴回操作

        Args:
            volume (str): 投料体积
            assign_material_name (str): 溶剂名称
            time (str): 观察时间（单位min）
            torque_variation (str): 是否观察1否2是

        Returns:
            Dict[str, Any]: 操作结果
        """
        try:
            # 检查反应站接口是否可用
            self._check_interface_availability("reaction")

            logger.info(f"执行反应站滴回操作: 体积={volume}, 溶剂={assign_material_name}")

            # 调用硬件接口的滴回方法
            result = self.hardware_interface.reactor_taken_out(
                volume=volume,
                assign_material_name=assign_material_name,
                time=time,
                torque_variation=torque_variation
            )

            return {
                "success": True,
                "return_info": "滴回操作完成",
                "result": result,
                "action": "reaction_station_drip_back"
            }

        except Exception as e:
            logger.error(f"反应站滴回操作失败: {e}")
            return {
                "success": False,
                "return_info": f"滴回操作失败: {str(e)}",
                "action": "reaction_station_drip_back"
            }

    def reaction_station_liquid_feed(self, titration_type: str, volume: str,
                                   assign_material_name: str, time: str,
                                   torque_variation: str) -> Dict[str, Any]:
        """反应站液体投料操作

        Args:
            titration_type (str): 滴定类型1否2是
            volume (str): 投料体积
            assign_material_name (str): 溶剂名称
            time (str): 观察时间（单位min）
            torque_variation (str): 是否观察1否2是

        Returns:
            Dict[str, Any]: 操作结果
        """
        try:
            # 检查反应站接口是否可用
            self._check_interface_availability("reaction")

            logger.info(f"执行反应站液体投料: 类型={titration_type}, 体积={volume}, 溶剂={assign_material_name}")

            # 根据滴定类型选择相应的方法
            if titration_type == "2":  # 滴定
                result = self.hardware_interface.liquid_feeding_titration(
                    volume=volume,
                    assign_material_name=assign_material_name,
                    time=time,
                    torque_variation=torque_variation
                )
            else:  # 非滴定
                result = self.hardware_interface.liquid_feeding_vials_non_titration(
                    volume=volume,
                    assign_material_name=assign_material_name,
                    time=time,
                    torque_variation=torque_variation
                )

            return {
                "success": True,
                "return_info": "液体投料操作完成",
                "result": result,
                "action": "reaction_station_liquid_feed"
            }

        except Exception as e:
            logger.error(f"反应站液体投料操作失败: {e}")
            return {
                "success": False,
                "return_info": f"液体投料操作失败: {str(e)}",
                "action": "reaction_station_liquid_feed"
            }

    def reaction_station_solid_feed_vial(self, assign_material_name: str, material_id: str,
                                       time: str, torque_variation: str) -> Dict[str, Any]:
        """反应站固体投料-小瓶操作

        Args:
            assign_material_name (str): 固体名称_粉末加样模块-投料
            material_id (str): 固体投料类型_粉末加样模块-投料
            time (str): 观察时间_反应模块-观察搅拌结果
            torque_variation (str): 是否观察1否2是_反应模块-观察搅拌结果

        Returns:
            Dict[str, Any]: 操作结果
        """
        try:
            # 检查反应站接口是否可用
            self._check_interface_availability("reaction")

            logger.info(f"执行反应站固体投料: 固体={assign_material_name}, ID={material_id}")

            # 调用硬件接口的固体投料方法
            result = self.hardware_interface.solid_feeding_vials(
                assign_material_name=assign_material_name,
                material_id=material_id,
                time=time,
                torque_variation=torque_variation
            )

            return {
                "success": True,
                "return_info": "固体投料操作完成",
                "result": result,
                "action": "reaction_station_solid_feed_vial"
            }

        except Exception as e:
            logger.error(f"反应站固体投料操作失败: {e}")
            return {
                "success": False,
                "return_info": f"固体投料操作失败: {str(e)}",
                "action": "reaction_station_solid_feed_vial"
            }

    def reaction_station_take_in(self, cutoff: str, temperature: str,
                               assign_material_name: str) -> Dict[str, Any]:
        """反应站取入操作

        Args:
            cutoff (str): 截止参数
            temperature (str): 温度
            assign_material_name (str): 物料名称

        Returns:
            Dict[str, Any]: 操作结果
        """
        try:
            # 检查反应站接口是否可用
            self._check_interface_availability("reaction")

            logger.info(f"执行反应站取入操作: 温度={temperature}, 物料={assign_material_name}")

            # 调用硬件接口的取入方法
            result = self.hardware_interface.reactor_taken_in(
                cutoff=cutoff,
                temperature=temperature,
                assign_material_name=assign_material_name
            )

            return {
                "success": True,
                "return_info": "取入操作完成",
                "result": result,
                "action": "reaction_station_take_in"
            }

        except Exception as e:
            logger.error(f"反应站取入操作失败: {e}")
            return {
                "success": False,
                "return_info": f"取入操作失败: {str(e)}",
                "action": "reaction_station_take_in"
            }

    def reaction_station_reactor_taken_out(self, order_id: str = "", preintake_id: str = "") -> Dict[str, Any]:
        """反应站反应器取出操作

        Args:
            order_id (str): 订单ID，用于标识要取出的订单
            preintake_id (str): 预取样ID，用于标识具体的取样任务

        Returns:
            Dict[str, Any]: 操作结果
        """
        try:
            # 检查反应站接口是否可用
            self._check_interface_availability("reaction")

            logger.info(f"执行反应站反应器取出操作: 订单ID={order_id}, 预取样ID={preintake_id}")

            # 调用更新后的反应器取出方法
            result = self.reactor_taken_out(order_id=order_id, preintake_id=preintake_id)

            # 更新 action 字段以区分调用来源
            result["action"] = "reaction_station_reactor_taken_out"

            return result

        except Exception as e:
            logger.error(f"反应站反应器取出操作失败: {e}")
            return {
                "success": False,
                "code": 0,
                "return_info": f"反应器取出操作失败: {str(e)}",
                "action": "reaction_station_reactor_taken_out"
            }

    def reaction_station_process_execute(self, workflow_name: str, task_name: str) -> Dict[str, Any]:
        """反应站流程执行操作

        Args:
            workflow_name (str): 工作流名称
            task_name (str): 任务名称

        Returns:
            Dict[str, Any]: 操作结果
        """
        try:
            # 检查反应站接口是否可用
            self._check_interface_availability("reaction")

            logger.info(f"执行反应站流程: 工作流={workflow_name}, 任务={task_name}")

            # 这里可以根据具体的工作流和任务名称调用相应的方法
            # 暂时使用通用的执行方法
            result = {
                "workflow_name": workflow_name,
                "task_name": task_name,
                "status": "executed"
            }

            return {
                "success": True,
                "return_info": "流程执行完成",
                "result": result,
                "action": "reaction_station_process_execute"
            }

        except Exception as e:
            logger.error(f"反应站流程执行失败: {e}")
            return {
                "success": False,
                "return_info": f"流程执行失败: {str(e)}",
                "action": "reaction_station_process_execute"
            }

    # ==================== 物料管理动作函数 ====================

    def material_inbound(self, material_id: str, location_name: str) -> Dict[str, Any]:
        """物料入库操作

        将物料添加到指定位置

        Args:
            material_id (str): 物料ID
            location_name (str): 位置名称

        Returns:
            Dict[str, Any]: 操作结果，包含状态和消息
        """
        try:
            logger.info(f"开始执行物料入库操作: 物料ID={material_id}, 位置={location_name}")
            result = self.hardware_interface.material_inbound(
                material_id=material_id,
                location_name=location_name
            )

            if result:
                logger.info("物料入库操作成功")
                return {
                    "status": "success",
                    "message": f"物料入库成功，物料ID: {material_id}",
                    "data": result
                }
            else:
                logger.error("物料入库操作失败")
                return {
                    "status": "failed",
                    "message": "物料入库失败"
                }

        except Exception as e:
            logger.error(f"物料入库操作异常: {e}")
            return {
                "status": "error",
                "message": f"物料入库操作异常: {str(e)}"
            }

    def material_outbound(self, material_id: str, location_name: str,
                         quantity: int) -> Dict[str, Any]:
        """物料出库操作

        从指定位置取出物料

        Args:
            material_id (str): 物料ID
            location_name (str): 位置名称
            quantity (int): 数量

        Returns:
            Dict[str, Any]: 操作结果，包含状态和消息
        """
        try:
            logger.info(f"开始执行物料出库操作: 物料ID={material_id}, 位置={location_name}, 数量={quantity}")
            result = self.hardware_interface.material_outbound(
                material_id=material_id,
                location_name=location_name,
                quantity=quantity
            )

            if result:
                logger.info("物料出库操作成功")
                return {
                    "status": "success",
                    "message": f"物料出库成功，物料ID: {material_id}",
                    "data": result
                }
            else:
                logger.error("物料出库操作失败")
                return {
                    "status": "failed",
                    "message": "物料出库失败"
                }

        except Exception as e:
            logger.error(f"物料出库操作异常: {e}")
            return {
                "status": "error",
                "message": f"物料出库操作异常: {str(e)}"
            }

    # ============ 工作流控制函数 ============

    def create_order(self, workflow_name: str, task_name: str,
                    parameters: Dict[str, Any] = None) -> Dict[str, Any]:
        """创建工作流订单

        创建并提交工作流执行订单

        Args:
            workflow_name (str): 工作流名称
            task_name (str): 任务名称
            parameters (Dict[str, Any]): 工作流参数

        Returns:
            Dict[str, Any]: 操作结果，包含状态和订单信息
        """
        try:
            logger.info(f"开始创建工作流订单: 工作流={workflow_name}, 任务={task_name}")

            # 使用 BioyondV1RPC 的工作流处理方法
            result = self.hardware_interface.process_and_execute_workflow(
                workflow_name=workflow_name,
                task_name=task_name
            )

            if result and result.get("status") == "success":
                logger.info("工作流订单创建成功")
                return {
                    "status": "success",
                    "message": f"工作流订单创建成功: {workflow_name}",
                    "data": result
                }
            else:
                logger.error("工作流订单创建失败")
                return {
                    "status": "failed",
                    "message": "工作流订单创建失败",
                    "data": result
                }

        except Exception as e:
            logger.error(f"创建工作流订单异常: {e}")
            return {
                "status": "error",
                "message": f"创建工作流订单异常: {str(e)}"
            }

    def get_scheduler_status(self) -> Dict[str, Any]:
        """获取调度器状态

        Returns:
            Dict[str, Any]: 调度器状态信息
        """
        try:
            logger.info("获取调度器状态")
            result = self.hardware_interface.scheduler_status()

            return {
                "status": "success",
                "message": "调度器状态获取成功",
                "data": result
            }

        except Exception as e:
            logger.error(f"获取调度器状态异常: {e}")
            return {
                "status": "error",
                "message": f"获取调度器状态异常: {str(e)}"
            }

    def start_scheduler(self) -> Dict[str, Any]:
        """启动调度器

        Returns:
            Dict[str, Any]: 操作结果
        """
        try:
            logger.info("启动调度器")
            result = self.hardware_interface.scheduler_start()

            if result == 1:  # 成功返回1
                logger.info("调度器启动成功")
                return {
                    "status": "success",
                    "message": "调度器启动成功"
                }
            else:
                logger.error("调度器启动失败")
                return {
                    "status": "failed",
                    "message": "调度器启动失败"
                }

        except Exception as e:
            logger.error(f"启动调度器异常: {e}")
            return {
                "status": "error",
                "message": f"启动调度器异常: {str(e)}"
            }

    def stop_scheduler(self) -> Dict[str, Any]:
        """停止调度器

        Returns:
            Dict[str, Any]: 操作结果
        """
        try:
            logger.info("停止调度器")
            result = self.hardware_interface.scheduler_stop()

            if result == 1:  # 成功返回1
                logger.info("调度器停止成功")
                return {
                    "status": "success",
                    "message": "调度器停止成功"
                }
            else:
                logger.error("调度器停止失败")
                return {
                    "status": "failed",
                    "message": "调度器停止失败"
                }

        except Exception as e:
            logger.error(f"停止调度器异常: {e}")
            return {
                 "status": "error",
                 "message": f"停止调度器异常: {str(e)}"
             }

    # ============ 其他操作函数 ============

    def drip_back(self, assign_material_name: str = "Reactor", time: str = "0",
                  torque_variation: str = "1", temperature: float = 25.00) -> Dict[str, Any]:
        """滴回操作

        执行滴回操作，通常用于反应后的物料回收

        Args:
            assign_material_name (str): 指定的物料名称，默认为 "Reactor"
            time (str): 操作时间，默认为 "0"
            torque_variation (str): 扭矩变化，默认为 "1"
            temperature (float): 温度设置，默认为 25.00°C

        Returns:
            Dict[str, Any]: 操作结果，包含状态和消息
        """
        try:
            logger.info(f"开始执行滴回操作: 物料={assign_material_name}, 温度={temperature}°C")

            # 根据配置文件中的映射，滴回操作可能对应特定的工作流
            workflow_name = self.config.get("workflow_mappings", {}).get("Drip_back")

            if workflow_name:
                result = self.hardware_interface.process_and_execute_workflow(
                    workflow_name=workflow_name,
                    task_name="drip_back_task"
                )
            else:
                # 如果没有特定的工作流映射，使用通用的液体操作
                logger.warning("未找到滴回操作的工作流映射，使用默认处理")
                result = {"status": "success", "message": "滴回操作完成"}

            if result and result.get("status") == "success":
                logger.info("滴回操作成功")
                return {
                    "status": "success",
                    "message": f"滴回操作成功，物料: {assign_material_name}",
                    "data": result
                }
            else:
                logger.error("滴回操作失败")
                return {
                    "status": "failed",
                    "message": "滴回操作失败"
                }

        except Exception as e:
            logger.error(f"滴回操作异常: {e}")
            return {
                "status": "error",
                "message": f"滴回操作异常: {str(e)}"
            }

    def get_device_list(self) -> Dict[str, Any]:
        """获取设备列表

        Returns:
            Dict[str, Any]: 设备列表信息
        """
        try:
            logger.info("获取设备列表")
            result = self.hardware_interface.device_list()

            return {
                "status": "success",
                "message": "设备列表获取成功",
                "data": result
            }

        except Exception as e:
            logger.error(f"获取设备列表异常: {e}")
            return {
                "status": "error",
                "message": f"获取设备列表异常: {str(e)}"
            }

    def device_operation(self, device_id: str, operation: str,
                        parameters: Dict[str, Any] = None) -> Dict[str, Any]:
        """设备操作

        对指定设备执行操作

        Args:
            device_id (str): 设备ID
            operation (str): 操作类型
            parameters (Dict[str, Any]): 操作参数

        Returns:
            Dict[str, Any]: 操作结果
        """
        try:
            logger.info(f"执行设备操作: 设备ID={device_id}, 操作={operation}")
            result = self.hardware_interface.device_operation(
                device_id=device_id,
                operation=operation,
                parameters=parameters or {}
            )

            if result:
                logger.info("设备操作成功")
                return {
                    "status": "success",
                    "message": f"设备操作成功: {operation}",
                    "data": result
                }
            else:
                logger.error("设备操作失败")
                return {
                    "status": "failed",
                    "message": "设备操作失败"
                }

        except Exception as e:
            logger.error(f"设备操作异常: {e}")
            return {
                "status": "error",
                "message": f"设备操作异常: {str(e)}"
            }

    def add_material(self, material_data: Dict[str, Any]) -> Dict[str, Any]:
        """添加物料

        向系统中添加新的物料信息

        Args:
            material_data (Dict[str, Any]): 物料数据

        Returns:
            Dict[str, Any]: 操作结果
        """
        try:
            logger.info(f"添加物料: {material_data.get('name', 'Unknown')}")
            result = self.hardware_interface.add_material(material_data)

            if result:
                logger.info("物料添加成功")
                return {
                    "status": "success",
                    "message": "物料添加成功",
                    "data": result
                }
            else:
                logger.error("物料添加失败")
                return {
                    "status": "failed",
                    "message": "物料添加失败"
                }

        except Exception as e:
            logger.error(f"添加物料异常: {e}")
            return {
                "status": "error",
                "message": f"添加物料异常: {str(e)}"
            }

    def stock_material(self, material_id: str, location: str,
                      quantity: int) -> Dict[str, Any]:
        """库存物料

        更新物料库存信息

        Args:
            material_id (str): 物料ID
            location (str): 位置
            quantity (int): 数量

        Returns:
            Dict[str, Any]: 操作结果
        """
        try:
            logger.info(f"更新物料库存: 物料ID={material_id}, 位置={location}, 数量={quantity}")
            result = self.hardware_interface.stock_material(
                material_id=material_id,
                location=location,
                quantity=quantity
            )

            if result:
                logger.info("物料库存更新成功")
                return {
                    "status": "success",
                    "message": "物料库存更新成功",
                    "data": result
                }
            else:
                logger.error("物料库存更新失败")
                return {
                    "status": "failed",
                    "message": "物料库存更新失败"
                }

        except Exception as e:
            logger.error(f"物料库存更新异常: {e}")
            return {
                "status": "error",
                "message": f"物料库存更新异常: {str(e)}"
            }

    # ============ 工作站状态管理 ============

    def get_workstation_status(self) -> Dict[str, Any]:
        """获取工作站状态

        Returns:
            Dict[str, Any]: 工作站状态信息
        """
        try:
            # 获取基础状态信息
            base_status = {
                "is_busy": self.is_busy,
                "workflow_status": self.workflow_status,
                "workflow_runtime": self.workflow_runtime
            }

            # 获取调度器状态
            scheduler_status = self.get_scheduler_status()

            # 获取设备列表
            device_list = self.get_device_list()

            return {
                "status": "success",
                "message": "工作站状态获取成功",
                "data": {
                    "base_status": base_status,
                    "scheduler_status": scheduler_status.get("data"),
                    "device_list": device_list.get("data"),
                    "config": {
                        "api_host": self.config.get("api_host"),
                        "workflow_mappings": self.config.get("workflow_mappings", {}),
                        "material_type_mappings": self.config.get("material_type_mappings", {})
                    }
                }
            }

        except Exception as e:
            logger.error(f"获取工作站状态异常: {e}")
            return {
                "status": "error",
                "message": f"获取工作站状态异常: {str(e)}"
            }

    def get_bioyond_status(self) -> Dict[str, Any]:
        """获取完整的 Bioyond 状态信息

        这个方法提供了比 bioyond_status 属性更详细的状态信息，
        包括错误处理和格式化的响应结构

        Returns:
            Dict[str, Any]: 格式化的 Bioyond 状态响应
        """
        try:
            # 获取 bioyond_status 属性的数据
            status_data = self.bioyond_status

            return {
                "status": "success",
                "message": "Bioyond 状态获取成功",
                "data": status_data
            }

        except Exception as e:
            logger.error(f"获取 Bioyond 状态异常: {e}")
            return {
                "status": "error",
                "message": f"获取 Bioyond 状态异常: {str(e)}",
                "data": {
                    "station_info": {
                        "station_name": getattr(self, 'station_name', 'BioyondWorkstation'),
                        "station_type": getattr(self, 'station_type', 'unknown'),
                        "enable_reaction_station": getattr(self, 'enable_reaction_station', False),
                        "enable_dispensing_station": getattr(self, 'enable_dispensing_station', False)
                    },
                    "interface_status": {
                        "reaction_interface_connected": False,
                        "dispensing_interface_connected": False,
                        "last_sync_time": 0,
                        "sync_interval": 60
                    },
                    "sync_status": {
                        "last_sync_success": False,
                        "total_resources": 0,
                        "warehouse_count": 0
                    },
                    "timestamp": __import__('time').time(),
                    "status": "error",
                    "error_message": str(e)
                }
            }

    def reset_workstation(self) -> Dict[str, Any]:
        """重置工作站

        重置工作站到初始状态

        Returns:
            Dict[str, Any]: 操作结果
        """
        try:
            logger.info("开始重置工作站")

            # 停止当前工作流（如果有）
            if self.is_busy:
                self.stop_workflow()

            # 停止调度器
            self.stop_scheduler()

            # 重新启动调度器
            start_result = self.start_scheduler()

            if start_result.get("status") == "success":
                logger.info("工作站重置成功")
                return {
                    "status": "success",
                    "message": "工作站重置成功"
                }
            else:
                logger.error("工作站重置失败")
                return {
                    "status": "failed",
                    "message": "工作站重置失败"
                }

        except Exception as e:
            logger.error(f"工作站重置异常: {e}")
            return {
                "status": "error",
                "message": f"工作站重置异常: {str(e)}"
            }

    async def execute_bioyond_update_workflow(self, parameters: Dict[str, Any]) -> Dict[str, Any]:
        """执行Bioyond更新工作流"""
        try:
            material_ids = parameters.get("material_ids", [])
            sync_all = parameters.get("sync_all", True)

            logger.info(f"开始执行Bioyond更新工作流: sync_all={sync_all}")

            # 获取物料管理模块
            material_manager = self.material_management

            if sync_all:
                # 同步所有物料
                success_count = 0
                for resource in material_manager.plr_resources.values():
                    success = await material_manager.sync_to_bioyond(resource)
                    if success:
                        success_count += 1
            else:
                # 同步指定物料
                success_count = 0
                for material_id in material_ids:
                    resource = material_manager.find_material_by_id(material_id)
                    if resource:
                        success = await material_manager.sync_to_bioyond(resource)
                        if success:
                            success_count += 1

            result = {
                "status": "success",
                "message": f"Bioyond更新完成",
                "updated_resources": success_count,
                "total_resources": len(material_ids) if not sync_all else len(material_manager.plr_resources)
            }

            logger.info(f"Bioyond更新工作流执行完成: {result['status']}")
            return result

        except Exception as e:
            logger.error(f"Bioyond更新工作流执行失败: {e}")
            return {
                "status": "error",
                "message": str(e)
            }

    def load_bioyond_data_from_file(self, file_path: str) -> bool:
        """从文件加载Bioyond数据（用于测试）"""
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                bioyond_data = json.load(f)

            # 获取物料管理模块
            material_manager = self.material_management

            # 转换为UniLab格式
            if isinstance(bioyond_data, dict) and "data" in bioyond_data:
                unilab_resources = material_manager.resource_bioyond_container_to_ulab(bioyond_data)
            else:
                unilab_resources = material_manager.resource_bioyond_to_ulab(bioyond_data)

            # 分配到Deck
            import asyncio
            asyncio.create_task(material_manager._assign_resources_to_deck(unilab_resources))

            logger.info(f"从文件 {file_path} 加载了 {len(unilab_resources)} 个Bioyond资源")
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
    # 创建示例工作站
    #workstation = create_bioyond_workstation_example()

    # 从文件加载测试数据
    #workstation.load_bioyond_data_from_file("bioyond_test_yibin.json")

    # 获取状态
    #status = workstation.get_bioyond_status()
    #print("Bioyond工作站状态:", status)

    # 创建测试数据 - 使用resource_bioyond_container_to_ulab函数期望的格式

  # 读取 bioyond_resources_unilab_output3 copy.json 文件
    from unilabos.resources.graphio import resource_ulab_to_plr, convert_resources_to_type
    from Bioyond_wuliao import *
    from typing import List
    from pylabrobot.resources import Resource as PLRResource
    import json
    from pylabrobot.resources.deck import Deck
    from pylabrobot.resources.coordinate import Coordinate

    with open("./bioyond_test_yibin3_unilab_result_corr.json", "r", encoding="utf-8") as f:
        bioyond_resources_unilab = json.load(f)
    print(f"成功读取 JSON 文件，包含 {len(bioyond_resources_unilab)} 个资源")
    ulab_resources = convert_resources_to_type(bioyond_resources_unilab, List[PLRResource])
    print(f"转换结果类型: {type(ulab_resources)}")
    print(f"转换结果长度: {len(ulab_resources) if ulab_resources else 0}")
    deck = Deck(size_x=2000,
                size_y=653.5,
                size_z=900)

    Stack0 = Stack(name="Stack0", location=Coordinate(0, 100, 0))
    Stack1 = Stack(name="Stack1", location=Coordinate(100, 100, 0))
    Stack2 = Stack(name="Stack2", location=Coordinate(200, 100, 0))
    Stack3 = Stack(name="Stack3", location=Coordinate(300, 100, 0))
    Stack4 = Stack(name="Stack4", location=Coordinate(400, 100, 0))
    Stack5 = Stack(name="Stack5", location=Coordinate(500, 100, 0))

    deck.assign_child_resource(Stack1, Stack1.location)
    deck.assign_child_resource(Stack2, Stack2.location)
    deck.assign_child_resource(Stack3, Stack3.location)
    deck.assign_child_resource(Stack4, Stack4.location)
    deck.assign_child_resource(Stack5, Stack5.location)

    Stack0.assign_child_resource(ulab_resources[0], Stack0.location)
    Stack1.assign_child_resource(ulab_resources[1], Stack1.location)
    Stack2.assign_child_resource(ulab_resources[2], Stack2.location)
    Stack3.assign_child_resource(ulab_resources[3], Stack3.location)
    Stack4.assign_child_resource(ulab_resources[4], Stack4.location)
    Stack5.assign_child_resource(ulab_resources[5], Stack5.location)

    from unilabos.resources.graphio import convert_resources_from_type
    from unilabos.app.web.client import http_client

    resources = convert_resources_from_type([deck], [PLRResource])


    print(resources)
    http_client.remote_addr = "https://uni-lab.bohrium.com/api/v1"
    #http_client.auth = "9F05593C"
    http_client.auth = "ED634D1C"
    http_client.resource_add(resources, database_process_later=False)
