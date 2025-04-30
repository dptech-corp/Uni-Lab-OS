"""
设备类实例创建工厂

这个模块包含用于创建设备类实例的工厂类。
基础工厂类提供通用的实例创建方法，而特定工厂类提供针对特定设备类的创建方法。
"""
import asyncio
import inspect
import traceback
from abc import abstractmethod
from typing import Type, Any, Dict, Optional, TypeVar, Generic

from unilabos.resources.graphio import nested_dict_to_list, resource_ulab_to_plr
from unilabos.ros.nodes.resource_tracker import DeviceNodeResourceTracker
from unilabos.utils import logger, import_manager
from unilabos.utils.cls_creator import create_instance_from_config

# 定义泛型类型变量
T = TypeVar("T")


class ClassCreator(Generic[T]):
    @abstractmethod
    def create_instance(self, *args, **kwargs) -> T:
        pass


class DeviceClassCreator(Generic[T]):
    """
    设备类实例创建器基类

    这个类提供了从任意类创建实例的通用方法。
    """

    def __init__(self, cls: Type[T]):
        """
        初始化设备类创建器

        Args:
            cls: 要创建实例的类
        """
        self.device_cls = cls
        self.device_instance: Optional[T] = None

    def create_instance(self, data: Dict[str, Any]) -> T:
        """
        创建设备类实例

        Args:


        Returns:
            设备类的实例
        """
        self.device_instance = create_instance_from_config(
            {
                "_cls": self.device_cls.__module__ + ":" + self.device_cls.__name__,
                "_params": data,
            }
        )
        self.post_create()
        return self.device_instance

    def get_instance(self) -> Optional[T]:
        """
        获取当前实例

        Returns:
            当前设备类实例，如果尚未创建则返回None
        """
        return self.device_instance

    def post_create(self):
        pass


class PyLabRobotCreator(DeviceClassCreator[T]):
    """
    PyLabRobot设备类创建器

    这个类提供了针对PyLabRobot设备类的实例创建方法，特别处理deserialize方法。
    """

    def __init__(self, cls: Type[T], children: Dict[str, Any], resource_tracker: DeviceNodeResourceTracker):
        """
        初始化PyLabRobot设备类创建器

        Args:
            cls: PyLabRobot设备类
            children: 子资源字典，用于资源替换
        """
        super().__init__(cls)
        self.children = children
        self.resource_tracker = resource_tracker
        # 检查类是否具有deserialize方法
        self.has_deserialize = hasattr(cls, "deserialize") and callable(getattr(cls, "deserialize"))
        if not self.has_deserialize:
            logger.warning(f"类 {cls.__name__} 没有deserialize方法，将使用标准构造函数")

    def _process_resource_mapping(self, resource, source_type):
        if source_type == dict:
            from pylabrobot.resources.resource import Resource

            return nested_dict_to_list(resource), Resource
        return resource, source_type

    def _process_resource_references(self, data: Any, to_dict=False) -> Any:
        """
        递归处理资源引用，替换_resource_child_name对应的资源

        Args:
            data: 需要处理的数据，可能是字典、列表或其他类型
            to_dict: 转换成对应的实例，还是转换成对应的字典

        Returns:
            处理后的数据
        """
        from pylabrobot.resources import Deck, Resource

        if isinstance(data, dict):
            # 检查是否包含资源引用
            if "_resource_child_name" in data:
                child_name = data["_resource_child_name"]
                if child_name in self.children:
                    # 找到了对应的资源
                    resource = self.children[child_name]

                    # 检查是否需要转换资源类型
                    if "_resource_type" in data:
                        type_path = data["_resource_type"]
                        try:
                            # 尝试导入指定的类型
                            target_type = import_manager.get_class(type_path)
                            contain_model = not issubclass(target_type, Deck)
                            resource, target_type = self._process_resource_mapping(resource, target_type)
                            # 在截图中格式，是deserialize，所以这里要转成plr resource可deserialize的字典
                            # 这样后面执行deserialize的时候能够正确反序列化对应的物料
                            resource_instance: Resource = resource_ulab_to_plr(resource, contain_model)
                            if to_dict:
                                return resource_instance.serialize()
                            else:
                                self.resource_tracker.add_resource(resource_instance)
                            return resource_instance
                        except Exception as e:
                            logger.warning(f"无法导入资源类型 {type_path}: {e}")
                            logger.warning(traceback.format_exc())
                    else:
                        logger.debug(f"找不到资源类型，请补全_resource_type {self.device_cls.__name__} {data.keys()}")
                    return resource
                else:
                    logger.warning(f"找不到资源引用 '{child_name}'，保持原值不变")

            # 递归处理字典的每个值
            result = {}
            for key, value in data.items():
                result[key] = self._process_resource_references(value, to_dict)
            return result

        # 处理列表类型
        elif isinstance(data, list):
            return [self._process_resource_references(item, to_dict) for item in data]

        # 其他类型直接返回
        return data

    def create_instance(self, data: Dict[str, Any]) -> Optional[T]:
        """
        从数据创建PyLabRobot设备实例

        Args:
            data: 用于反序列化的数据

        Returns:
            PyLabRobot设备类实例
        """
        deserialize_error = None
        stack = None
        if self.has_deserialize:
            deserialize_method = getattr(self.device_cls, "deserialize")
            spect = inspect.signature(deserialize_method)
            spec_args = spect.parameters
            for param_name, param_value in data.copy().items():
                if "_resource_child_name" in param_value and "_resource_type" not in param_value:
                    arg_value = spec_args[param_name].annotation
                    data[param_name]["_resource_type"] = self.device_cls.__module__ + ":" + arg_value
                    logger.debug(f"自动补充 _resource_type: {data[param_name]['_resource_type']}")

            # 首先处理资源引用
            processed_data = self._process_resource_references(data, to_dict=True)

            try:
                self.device_instance = deserialize_method(**processed_data)
                self.resource_tracker.add_resource(self.device_instance)
                self.post_create()
                return self.device_instance  # type: ignore
            except Exception as e:
                # 先静默继续，尝试另外一种创建方法
                deserialize_error = e
                stack = traceback.format_exc()

        if self.device_instance is None:
            try:
                spect = inspect.signature(self.device_cls.__init__)
                spec_args = spect.parameters
                for param_name, param_value in data.copy().items():
                    if "_resource_child_name" in param_value and "_resource_type" not in param_value:
                        arg_value = spec_args[param_name].annotation
                        data[param_name]["_resource_type"] = self.device_cls.__module__ + ":" + arg_value
                        logger.debug(f"自动补充 _resource_type: {data[param_name]['_resource_type']}")
                processed_data = self._process_resource_references(data, to_dict=False)
                self.device_instance = super(PyLabRobotCreator, self).create_instance(processed_data)
            except Exception as e:
                logger.error(f"PyLabRobot创建实例失败: {e}")
                logger.error(f"PyLabRobot创建实例堆栈: {traceback.format_exc()}")
            finally:
                if self.device_instance is None:
                    if deserialize_error:
                        logger.error(f"PyLabRobot反序列化失败: {deserialize_error}")
                        logger.error(f"PyLabRobot反序列化堆栈: {stack}")

            return self.device_instance

    def post_create(self):
        if hasattr(self.device_instance, "setup") and asyncio.iscoroutinefunction(getattr(self.device_instance, "setup")):
            from unilabos.ros.nodes.base_device_node import ROS2DeviceNode
            ROS2DeviceNode.run_async_func(getattr(self.device_instance, "setup")).add_done_callback(lambda x: logger.debug(f"PyLabRobot设备实例 {self.device_instance} 设置完成"))


class ProtocolNodeCreator(DeviceClassCreator[T]):
    """
    ProtocolNode设备类创建器

    这个类提供了针对ProtocolNode设备类的实例创建方法，处理children参数。
    """

    def __init__(self, cls: Type[T], children: Dict[str, Any]):
        """
        初始化ProtocolNode设备类创建器

        Args:
            cls: ProtocolNode设备类
            children: 子资源字典，用于资源替换
        """
        super().__init__(cls)
        self.children = children

    def create_instance(self, data: Dict[str, Any]) -> T:
        """
        从数据创建ProtocolNode设备实例

        Args:
            data: 用于创建实例的数据

        Returns:
            ProtocolNode设备类实例
        """
        try:

            # 创建实例
            data["children"] = self.children
            self.device_instance = super(ProtocolNodeCreator, self).create_instance(data)
            self.post_create()
            return self.device_instance
        except Exception as e:
            logger.error(f"ProtocolNode创建实例失败: {e}")
            logger.error(f"ProtocolNode创建实例堆栈: {traceback.format_exc()}")
            raise
