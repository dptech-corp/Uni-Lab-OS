"""
LaiYu_Liquid 液体处理工作站集成模块

该模块提供了 LaiYu_Liquid 工作站与 UniLabOS 的完整集成，包括：
- 硬件后端和抽象接口
- 资源定义和管理
- 协议执行和液体传输
- 工作台配置和布局

主要组件：
- LaiYuLiquidBackend: 硬件后端实现
- LaiYuLiquid: 液体处理器抽象接口
- 各种资源类：枪头架、板、容器等
- 便捷创建函数和配置管理

使用示例：
    from unilabos.devices.laiyu_liquid import (
        LaiYuLiquid, 
        LaiYuLiquidBackend,
        create_standard_deck,
        create_tip_rack_1000ul
    )
    
    # 创建后端和液体处理器
    backend = LaiYuLiquidBackend()
    lh = LaiYuLiquid(backend=backend)
    
    # 创建工作台
    deck = create_standard_deck()
    lh.deck = deck
    
    # 设置和运行
    await lh.setup()
"""

# 版本信息
__version__ = "1.0.0"
__author__ = "LaiYu_Liquid Integration Team"
__description__ = "LaiYu_Liquid 液体处理工作站 UniLabOS 集成模块"

# 驱动程序导入
from .drivers import (
    XYZStepperController,
    SOPAPipette,
    MotorAxis,
    MotorStatus,
    SOPAConfig,
    SOPAStatusCode,
    StepperMotorDriver
)

# 控制器导入
from .controllers import (
    XYZController,
    PipetteController,
)

# 后端导入
from .backend.rviz_backend import (
    LiquidHandlerRvizBackend,
)

# 资源类和创建函数导入
from .core.laiyu_liquid_res import (
    LaiYuLiquidDeck,
    LaiYuLiquidContainer,
    LaiYuLiquidTipRack
)

# 主设备类和配置
from .core.laiyu_liquid_main import (
    LaiYuLiquid,
    LaiYuLiquidConfig,
    LaiYuLiquidDeck,
    LaiYuLiquidContainer,
    LaiYuLiquidTipRack,
    create_quick_setup
)

# 后端创建函数导入
from .backend import (
    LaiYuLiquidBackend,
    create_laiyu_backend,
)

# 导出所有公共接口
__all__ = [
    # 版本信息
    "__version__",
    "__author__",
    "__description__",
    
    # 驱动程序
    "SOPAPipette",
    "SOPAConfig", 
    "StepperMotorDriver",
    "XYZStepperController",
    
    # 控制器
    "PipetteController",
    "XYZController",
    
    # 后端
    "LiquidHandlerRvizBackend",
    
    # 资源创建函数
    "create_tip_rack_1000ul",
    "create_tip_rack_200ul", 
    "create_96_well_plate",
    "create_deep_well_plate",
    "create_8_tube_rack",
    "create_standard_deck",
    "create_waste_container",
    "create_wash_container",
    "create_reagent_container",
    "load_deck_config",
    
    # 后端创建函数
    "create_laiyu_backend",
    
    # 主要类
    "LaiYuLiquid",
    "LaiYuLiquidConfig", 
    "LaiYuLiquidBackend",
    "LaiYuLiquidDeck",
    
    # 工具函数
    "get_version",
    "get_supported_resources",
    "create_quick_setup",
    "validate_installation",
    "print_module_info",
    "setup_logging",
]

# 别名定义，为了向后兼容
LaiYuLiquidDevice = LaiYuLiquid  # 主设备类别名
LaiYuLiquidController = XYZController  # 控制器别名
LaiYuLiquidDriver = XYZStepperController  # 驱动器别名

# 模块级别的便捷函数

def get_version() -> str:
    """
    获取模块版本
    
    Returns:
        str: 版本号
    """
    return __version__


def get_supported_resources() -> dict:
    """
    获取支持的资源类型
    
    Returns:
        dict: 支持的资源类型字典
    """
    return {
        "tip_racks": {
            "LaiYuLiquidTipRack": LaiYuLiquidTipRack,
        },
        "containers": {
            "LaiYuLiquidContainer": LaiYuLiquidContainer,
        },
        "decks": {
            "LaiYuLiquidDeck": LaiYuLiquidDeck,
        },
        "devices": {
            "LaiYuLiquid": LaiYuLiquid,
        }
    }


def create_quick_setup() -> tuple:
    """
    快速创建基本设置
    
    Returns:
        tuple: (backend, controllers, resources) 的元组
    """
    # 创建后端
    backend = LiquidHandlerRvizBackend()
    
    # 创建控制器（使用默认端口进行演示）
    pipette_controller = PipetteController(port="/dev/ttyUSB0", address=4)
    xyz_controller = XYZController(port="/dev/ttyUSB1", auto_connect=False)
    
    # 创建测试资源
    tip_rack_1000 = create_tip_rack_1000ul("tip_rack_1000")
    tip_rack_200 = create_tip_rack_200ul("tip_rack_200")
    well_plate = create_96_well_plate("96_well_plate")
    
    controllers = {
        'pipette': pipette_controller,
        'xyz': xyz_controller
    }
    
    resources = {
        'tip_rack_1000': tip_rack_1000,
        'tip_rack_200': tip_rack_200,
        'well_plate': well_plate
    }
    
    return backend, controllers, resources


def validate_installation() -> bool:
    """
    验证模块安装是否正确
    
    Returns:
        bool: 安装是否正确
    """
    try:
        # 检查核心类是否可以导入
        from .core.laiyu_liquid_main import LaiYuLiquid, LaiYuLiquidConfig
        from .backend import LaiYuLiquidBackend
        from .controllers import XYZController, PipetteController
        from .drivers import XYZStepperController, SOPAPipette
        
        # 尝试创建基本对象
        config = LaiYuLiquidConfig()
        backend = create_laiyu_backend("validation_test")
        
        print("模块安装验证成功")
        return True
        
    except Exception as e:
        print(f"模块安装验证失败: {e}")
        return False


def print_module_info():
    """打印模块信息"""
    print(f"LaiYu_Liquid 集成模块")
    print(f"版本: {__version__}")
    print(f"作者: {__author__}")
    print(f"描述: {__description__}")
    print(f"")
    print(f"支持的资源类型:")
    
    resources = get_supported_resources()
    for category, types in resources.items():
        print(f"  {category}:")
        for type_name, type_class in types.items():
            print(f"    - {type_name}: {type_class.__name__}")
    
    print(f"")
    print(f"主要功能:")
    print(f"  - 硬件集成: LaiYuLiquidBackend")
    print(f"  - 抽象接口: LaiYuLiquid")
    print(f"  - 资源管理: 各种资源类和创建函数")
    print(f"  - 协议执行: transfer_liquid 和相关函数")
    print(f"  - 配置管理: deck.json 和加载函数")


# 模块初始化时的检查
def _check_dependencies():
    """检查依赖项"""
    try:
        import pylabrobot
        import asyncio
        import json
        import logging
        return True
    except ImportError as e:
        import logging
        logging.warning(f"缺少依赖项 {e}")
        return False


# 执行依赖检查
_dependencies_ok = _check_dependencies()

if not _dependencies_ok:
    import logging
    logging.warning("某些依赖项缺失，模块功能可能受限")


# 模块级别的日志配置
import logging

def setup_logging(level: str = "INFO"):
    """
    设置模块日志
    
    Args:
        level: 日志级别 (DEBUG, INFO, WARNING, ERROR)
    """
    logger = logging.getLogger("LaiYu_Liquid")
    logger.setLevel(getattr(logging, level.upper()))
    
    if not logger.handlers:
        handler = logging.StreamHandler()
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        handler.setFormatter(formatter)
        logger.addHandler(handler)
    
    return logger


# 默认日志设置
_logger = setup_logging()