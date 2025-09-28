"""
LaiYu_Liquid 驱动程序模块

该模块包含了LaiYu_Liquid液体处理工作站的硬件驱动程序：
- SOPA移液器驱动程序
- XYZ步进电机驱动程序
"""

# SOPA移液器驱动程序导入
from .sopa_pipette_driver import SOPAPipette, SOPAConfig

# XYZ步进电机驱动程序导入  
from .xyz_stepper_driver import StepperMotorDriver, XYZStepperController

__all__ = [
    # SOPA移液器
    "SOPAPipette",
    "SOPAConfig",
    
    # XYZ步进电机
    "StepperMotorDriver", 
    "XYZStepperController",
]

__version__ = "1.0.0"
__author__ = "LaiYu_Liquid Driver Team"
__description__ = "LaiYu_Liquid 硬件驱动程序集合"