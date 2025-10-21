"""
Bioyond物料管理实现
Bioyond Material Management Implementation

基于Bioyond系统的物料管理，支持从Bioyond系统同步物料到UniLab工作站
"""
from typing import Dict, Any, List, Optional, Union
import json
import asyncio
from abc import ABC, abstractmethod

from pylabrobot.resources import (
    Resource as PLRResource,
    Container,
    Deck,
    Coordinate as PLRCoordinate,
)

from unilabos.ros.nodes.resource_tracker import DeviceNodeResourceTracker
from unilabos.utils.log import logger
from unilabos.resources.graphio import (
    resource_plr_to_ulab, 
    resource_ulab_to_plr,
    resource_bioyond_to_ulab,
    resource_bioyond_container_to_ulab,
    resource_ulab_to_bioyond
)
from .workstation_material_management import MaterialManagementBase


class BioyondMaterialManagement(MaterialManagementBase):
    """Bioyond物料管理类
    
    实现从Bioyond系统同步物料到UniLab工作站的功能：
    1. 从Bioyond系统获取物料数据
    2. 转换为UniLab格式
    3. 同步到PyLabRobot Deck
    4. 支持双向同步
    """
    
    def __init__(
        self,
        device_id: str,
        deck_config: Dict[str, Any],
        resource_tracker: DeviceNodeResourceTracker,
        children_config: Dict[str, Dict[str, Any]] = None,
        bioyond_config: Dict[str, Any] = None
    ):
        self.bioyond_config = bioyond_config or {}
        self.bioyond_api_client = None
        self.sync_interval = self.bioyond_config.get("sync_interval", 30)  # 同步间隔（秒）
        
        # 初始化父类
        super().__init__(device_id, deck_config, resource_tracker, children_config)
        
        # 初始化Bioyond API客户端
        self._initialize_bioyond_client()
        
        # 启动同步任务
        self._start_sync_task()
    
    def _initialize_bioyond_client(self):
        """初始化Bioyond API客户端"""
        try:
            # 这里应该根据实际的Bioyond API实现
            # 暂时使用模拟客户端
            self.bioyond_api_client = BioyondAPIClient(self.bioyond_config)
            logger.info(f"Bioyond API客户端初始化成功")
        except Exception as e:
            logger.error(f"Bioyond API客户端初始化失败: {e}")
            self.bioyond_api_client = None
    
    def _start_sync_task(self):
        """启动同步任务"""
        if self.bioyond_api_client:
            # 创建异步同步任务
            asyncio.create_task(self._periodic_sync())
            logger.info(f"Bioyond同步任务已启动，间隔: {self.sync_interval}秒")
    
    async def _periodic_sync(self):
        """定期同步任务"""
        while True:
            try:
                await self.sync_from_bioyond()
                await asyncio.sleep(self.sync_interval)
            except Exception as e:
                logger.error(f"Bioyond同步任务出错: {e}")
                await asyncio.sleep(self.sync_interval)
    
    async def sync_from_bioyond(self) -> bool:
        """从Bioyond系统同步物料"""
        try:
            if not self.bioyond_api_client:
                logger.warning("Bioyond API客户端未初始化")
                return False
            
            # 1. 从Bioyond获取物料数据
            bioyond_data = await self.bioyond_api_client.get_materials()
            if not bioyond_data:
                logger.warning("从Bioyond获取物料数据为空")
                return False
            
            # 2. 转换为UniLab格式
            if isinstance(bioyond_data, dict) and "data" in bioyond_data:
                # 容器格式数据
                unilab_resources = resource_bioyond_container_to_ulab(bioyond_data)
            else:
                # 物料列表格式数据
                unilab_resources = resource_bioyond_to_ulab(bioyond_data)
            
            # 3. 转换为PLR格式并分配到Deck
            await self._assign_resources_to_deck(unilab_resources)
            
            logger.info(f"从Bioyond同步了 {len(unilab_resources)} 个资源")
            return True
            
        except Exception as e:
            logger.error(f"从Bioyond同步物料失败: {e}")
            return False
    
    async def sync_to_bioyond(self, plr_resource: PLRResource) -> bool:
        """将本地物料变更同步到Bioyond系统"""
        try:
            if not self.bioyond_api_client:
                logger.warning("Bioyond API客户端未初始化")
                return False
            
            # 1. 转换为UniLab格式
            unilab_resource = resource_plr_to_ulab(plr_resource)
            
            # 2. 转换为Bioyond格式
            bioyond_materials = resource_ulab_to_bioyond([unilab_resource])
            
            # 3. 发送到Bioyond系统
            success = await self.bioyond_api_client.update_materials(bioyond_materials)
            
            if success:
                logger.info(f"成功同步物料 {plr_resource.name} 到Bioyond")
            else:
                logger.warning(f"同步物料 {plr_resource.name} 到Bioyond失败")
            
            return success
            
        except Exception as e:
            logger.error(f"同步物料到Bioyond失败: {e}")
            return False
    
    async def _assign_resources_to_deck(self, unilab_resources: List[Dict[str, Any]]):
        """将UniLab资源分配到Deck"""
        try:
            # 转换为PLR格式
            from unilabos.resources.graphio import list_to_nested_dict
            nested_resources = list_to_nested_dict(unilab_resources)
            plr_resources = resource_ulab_to_plr(nested_resources)
            
            # 分配资源到Deck
            if hasattr(plr_resources, 'children'):
                resources_to_assign = plr_resources.children
            elif isinstance(plr_resources, list):
                resources_to_assign = plr_resources
            else:
                resources_to_assign = [plr_resources]
            
            for resource in resources_to_assign:
                try:
                    # 获取资源位置
                    if hasattr(resource, 'location') and resource.location:
                        location = PLRCoordinate(resource.location.x, resource.location.y, resource.location.z)
                    else:
                        location = PLRCoordinate(0, 0, 0)
                    
                    # 分配资源到Deck
                    self.plr_deck.assign_child_resource(resource, location)
                    
                    # 注册到resource tracker
                    self.resource_tracker.add_resource(resource)
                    
                    # 保存资源引用
                    self.plr_resources[resource.name] = resource
                    
                except Exception as e:
                    logger.error(f"分配资源 {resource.name} 到Deck失败: {e}")
            
            logger.info(f"成功分配了 {len(resources_to_assign)} 个资源到Deck")
            
        except Exception as e:
            logger.error(f"分配资源到Deck失败: {e}")
    
    def _create_resource_by_type(
        self, 
        resource_id: str, 
        resource_type: str, 
        config: Dict[str, Any], 
        data: Dict[str, Any], 
        location: PLRCoordinate
    ) -> Optional[PLRResource]:
        """根据类型创建Bioyond相关资源"""
        try:
            # 这里可以根据需要实现特定的Bioyond资源类型
            # 目前使用通用的容器类型
            if resource_type in ["container", "plate", "well"]:
                return self._create_generic_container(resource_id, resource_type, config, data, location)
            else:
                logger.warning(f"未知的Bioyond资源类型: {resource_type}")
                return None
                
        except Exception as e:
            logger.error(f"创建Bioyond资源失败 {resource_id} ({resource_type}): {e}")
            return None
    
    def _create_generic_container(
        self, 
        resource_id: str, 
        resource_type: str, 
        config: Dict[str, Any], 
        data: Dict[str, Any], 
        location: PLRCoordinate
    ) -> Optional[PLRResource]:
        """创建通用容器资源"""
        try:
            from pylabrobot.resources import Plate, Well
            
            if resource_type == "plate":
                return Plate(
                    name=resource_id,
                    size_x=config.get("size_x", 127.76),
                    size_y=config.get("size_y", 85.48),
                    size_z=config.get("size_z", 14.35),
                    location=location,
                    category="plate"
                )
            elif resource_type == "well":
                return Well(
                    name=resource_id,
                    size_x=config.get("size_x", 9.0),
                    size_y=config.get("size_y", 9.0),
                    size_z=config.get("size_z", 10.0),
                    location=location,
                    category="well"
                )
            else:
                return Container(
                    name=resource_id,
                    size_x=config.get("size_x", 50.0),
                    size_y=config.get("size_y", 50.0),
                    size_z=config.get("size_z", 10.0),
                    location=location,
                    category="container"
                )
                
        except Exception as e:
            logger.error(f"创建通用容器失败 {resource_id}: {e}")
            return None
    
    def get_bioyond_materials(self) -> List[Dict[str, Any]]:
        """获取当前Bioyond物料列表"""
        try:
            # 将当前PLR资源转换为Bioyond格式
            bioyond_materials = []
            for resource in self.plr_resources.values():
                unilab_resource = resource_plr_to_ulab(resource)
                bioyond_materials.extend(resource_ulab_to_bioyond([unilab_resource]))
            return bioyond_materials
        except Exception as e:
            logger.error(f"获取Bioyond物料列表失败: {e}")
            return []
    
    def update_material_from_bioyond(self, material_id: str, bioyond_data: Dict[str, Any]) -> bool:
        """从Bioyond数据更新指定物料"""
        try:
            # 查找现有物料
            material = self.find_material_by_id(material_id)
            if not material:
                logger.warning(f"未找到物料: {material_id}")
                return False
            
            # 转换Bioyond数据为UniLab格式
            unilab_resources = resource_bioyond_to_ulab([bioyond_data])
            if not unilab_resources:
                logger.warning(f"转换Bioyond数据失败: {material_id}")
                return False
            
            # 更新物料属性
            unilab_resource = unilab_resources[0]
            material.name = unilab_resource.get("name", material.name)
            
            # 更新位置
            position = unilab_resource.get("position", {})
            if position:
                material.location = PLRCoordinate(
                    position.get("x", 0),
                    position.get("y", 0),
                    position.get("z", 0)
                )
            
            logger.info(f"成功更新物料: {material_id}")
            return True
            
        except Exception as e:
            logger.error(f"更新物料失败 {material_id}: {e}")
            return False


class BioyondAPIClient:
    """Bioyond API客户端（模拟实现）
    
    实际使用时需要根据Bioyond系统的API接口实现
    """
    
    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self.base_url = config.get("base_url", "http://localhost:8080")
        self.api_key = config.get("api_key", "")
        self.timeout = config.get("timeout", 30)
    
    async def get_materials(self) -> Optional[Union[Dict[str, Any], List[Dict[str, Any]]]]:
        """从Bioyond系统获取物料数据"""
        try:
            # 这里应该实现实际的API调用
            # 暂时返回模拟数据
            logger.info("从Bioyond API获取物料数据")
            
            # 模拟API调用延迟
            await asyncio.sleep(0.1)
            
            # 返回模拟数据（实际应该从API获取）
            return {
                "data": [],
                "code": 1,
                "message": "success",
                "timestamp": 1234567890
            }
            
        except Exception as e:
            logger.error(f"Bioyond API调用失败: {e}")
            return None
    
    async def update_materials(self, materials: List[Dict[str, Any]]) -> bool:
        """更新Bioyond系统中的物料数据"""
        try:
            # 这里应该实现实际的API调用
            logger.info(f"更新Bioyond系统中的 {len(materials)} 个物料")
            
            # 模拟API调用延迟
            await asyncio.sleep(0.1)
            
            # 模拟成功响应
            return True
            
        except Exception as e:
            logger.error(f"更新Bioyond物料失败: {e}")
            return False
    
    async def get_material_by_id(self, material_id: str) -> Optional[Dict[str, Any]]:
        """根据ID获取单个物料"""
        try:
            # 这里应该实现实际的API调用
            logger.info(f"从Bioyond API获取物料: {material_id}")
            
            # 模拟API调用延迟
            await asyncio.sleep(0.1)
            
            # 返回模拟数据
            return {
                "id": material_id,
                "name": f"material_{material_id}",
                "type": "container",
                "quantity": 1.0,
                "unit": "个"
            }
            
        except Exception as e:
            logger.error(f"获取Bioyond物料失败 {material_id}: {e}")
            return None
