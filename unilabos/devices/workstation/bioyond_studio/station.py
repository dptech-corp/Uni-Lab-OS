"""
Bioyond工作站实现
Bioyond Workstation Implementation

集成Bioyond物料管理的工作站示例
"""
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

            bioyond_data = self.bioyond_api_client.stock_material('{"typeMode": 2, "includeDetail": true}')
            if not bioyond_data:
                logger.warning("从Bioyond获取的物料数据为空")
                return False

            # 转换为UniLab格式
            unilab_resources = resource_bioyond_to_plr(
                bioyond_data,
                type_mapping=self.workstation.bioyond_config["material_type_mappings"],
                deck=self.workstation.deck
            )

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

            bioyond_material = resource_plr_to_bioyond(
                [resource],
                type_mapping=self.workstation.bioyond_config["material_type_mappings"],
                warehouse_mapping=self.workstation.bioyond_config["warehouse_mapping"]
            )[0]

            location_info = bioyond_material.pop("locations")

            material_id = self.bioyond_api_client.add_material(bioyond_material)

            response = self.bioyond_api_client.material_inbound(material_id, location_info[0]["id"])
            if not response:
                return {
                    "status": "error",
                    "message": "Failed to inbound material"
                }
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

    def transfer_resource_to_another(self, resource: List[ResourceSlot], mount_resource: List[ResourceSlot], sites: List[str], mount_device_id: DeviceSlot):
        ROS2DeviceNode.run_async_func(self._ros_node.transfer_resource_to_another, True, **{
            "plr_resources": resource,
            "target_device_id": mount_device_id,
            "target_resources": mount_resource,
            "sites": sites,
        })

    def _create_communication_module(self, config: Optional[Dict[str, Any]] = None) -> None:
        """创建Bioyond通信模块"""
        self.bioyond_config = config or {
            **API_CONFIG,
            "workflow_mappings": WORKFLOW_MAPPINGS,
            "material_type_mappings": MATERIAL_TYPE_MAPPINGS,
            "warehouse_mapping": WAREHOUSE_MAPPING
        }

        self.hardware_interface = BioyondV1RPC(self.bioyond_config)

    def resource_tree_add(self, resources: List[ResourcePLR]) -> None:
        """添加资源到资源树并更新ROS节点

        Args:
            resources (List[ResourcePLR]): 要添加的资源列表
        """
        self.resource_synchronizer.sync_to_external(resources)

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

    def merge_workflow_with_parameters(self, json_str: str) -> dict:
        """合并工作流并设置参数"""
        try:
            # 解析输入的 JSON 数据
            data = json.loads(json_str)

            # 构造 API 请求参数
            params = {
                "name": data.get("name", ""),
                "workflows": data.get("workflows", [])
            }

            # 验证必要参数
            if not params["name"]:
                return {
                    "code": 0,
                    "message": "工作流名称不能为空",
                    "timestamp": int(datetime.now().timestamp() * 1000)
                }

            if not params["workflows"]:
                return {
                    "code": 0,
                    "message": "工作流列表不能为空",
                    "timestamp": int(datetime.now().timestamp() * 1000)
                }

        except json.JSONDecodeError as e:
            return {
                "code": 0,
                "message": f"JSON 解析错误: {str(e)}",
                "timestamp": int(datetime.now().timestamp() * 1000)
            }
        except Exception as e:
            return {
                "code": 0,
                "message": f"参数处理错误: {str(e)}",
                "timestamp": int(datetime.now().timestamp() * 1000)
            }

        # 发送 POST 请求到 Bioyond API
        try:
            response = self.hardware_interface.post(
                url=f'{self.hardware_interface.host}/api/lims/workflow/merge-workflow-with-parameters',
                params={
                    "apiKey": self.hardware_interface.api_key,
                    "requestTime": self.hardware_interface.get_current_time_iso8601(),
                    "data": params,
                })

            # 处理响应
            if not response:
                return {
                    "code": 0,
                    "message": "API 请求失败，未收到响应",
                    "timestamp": int(datetime.now().timestamp() * 1000)
                }

            # 返回完整的响应结果
            return {
                "code": response.get("code", 0),
                "message": response.get("message", ""),
                "timestamp": response.get("timestamp", int(datetime.now().timestamp() * 1000))
            }

        except Exception as e:
            return {
                "code": 0,
                "message": f"API 请求异常: {str(e)}",
                "timestamp": int(datetime.now().timestamp() * 1000)
            }

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