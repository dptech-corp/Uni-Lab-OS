#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
移液控制器模块
封装SOPA移液器的高级控制功能
"""

# 添加项目根目录到Python路径以解决模块导入问题
import sys
import os

# 无论如何都添加项目根目录到路径
current_file = os.path.abspath(__file__)
# 从 .../Uni-Lab-OS/unilabos/devices/LaiYu_Liquid/controllers/pipette_controller.py
# 向上5级到 .../Uni-Lab-OS
project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(current_file)))))
# 强制添加项目根目录到sys.path的开头
sys.path.insert(0, project_root)

import time
import logging
from typing import Optional, List, Dict, Tuple
from dataclasses import dataclass
from enum import Enum

from unilabos.devices.LaiYu_Liquid.drivers.sopa_pipette_driver import (
    SOPAPipette,
    SOPAConfig,
    SOPAStatusCode,
    DetectionMode,
    create_sopa_pipette,
)

logger = logging.getLogger(__name__)


class TipStatus(Enum):
    """枪头状态"""
    NO_TIP = "no_tip"
    TIP_ATTACHED = "tip_attached"
    TIP_USED = "tip_used"


class LiquidClass(Enum):
    """液体类型"""
    WATER = "water"
    SERUM = "serum"
    VISCOUS = "viscous"
    VOLATILE = "volatile"
    CUSTOM = "custom"


@dataclass
class LiquidParameters:
    """液体处理参数"""
    aspirate_speed: int = 500      # 吸液速度
    dispense_speed: int = 800      # 排液速度
    air_gap: float = 10.0          # 空气间隙
    blow_out: float = 5.0          # 吹出量
    pre_wet: bool = False          # 预润湿
    mix_cycles: int = 0            # 混合次数
    mix_volume: float = 50.0       # 混合体积
    touch_tip: bool = False        # 接触壁
    delay_after_aspirate: float = 0.5  # 吸液后延时
    delay_after_dispense: float = 0.5  # 排液后延时


class PipetteController:
    """移液控制器"""

    # 预定义液体参数
    LIQUID_PARAMS = {
        LiquidClass.WATER: LiquidParameters(
            aspirate_speed=500,
            dispense_speed=800,
            air_gap=10.0
        ),
        LiquidClass.SERUM: LiquidParameters(
            aspirate_speed=200,
            dispense_speed=400,
            air_gap=15.0,
            pre_wet=True,
            delay_after_aspirate=1.0
        ),
        LiquidClass.VISCOUS: LiquidParameters(
            aspirate_speed=100,
            dispense_speed=200,
            air_gap=20.0,
            delay_after_aspirate=2.0,
            delay_after_dispense=2.0
        ),
        LiquidClass.VOLATILE: LiquidParameters(
            aspirate_speed=800,
            dispense_speed=1000,
            air_gap=5.0,
            delay_after_aspirate=0.2,
            delay_after_dispense=0.2
        )
    }

    def __init__(self, port: str, address: int = 4):
        """
        初始化移液控制器

        Args:
            port: 串口端口
            address: RS485地址
        """
        self.config = SOPAConfig(
            port=port,
            address=address,
            baudrate=115200
        )
        self.pipette = SOPAPipette(self.config)
        self.tip_status = TipStatus.NO_TIP
        self.current_volume = 0.0
        self.max_volume = 1000.0  # 默认1000ul
        self.liquid_class = LiquidClass.WATER
        self.liquid_params = self.LIQUID_PARAMS[LiquidClass.WATER]

        # 统计信息
        self.tip_count = 0
        self.aspirate_count = 0
        self.dispense_count = 0

    def connect(self) -> bool:
        """连接移液器"""
        try:
            if self.pipette.connect():
                logger.info("移液器连接成功")
                return True
            return False
        except Exception as e:
            logger.error(f"移液器连接失败: {e}")
            return False

    def initialize(self) -> bool:
        """初始化移液器"""
        try:
            if self.pipette.initialize():
                logger.info("移液器初始化成功")
                # 检查枪头状态
                self._update_tip_status()
                return True
            return False
        except Exception as e:
            logger.error(f"移液器初始化失败: {e}")
            return False

    def disconnect(self):
        """断开连接"""
        self.pipette.disconnect()
        logger.info("移液器已断开")

    def pickup_tip(self) -> bool:
        """
        装载枪头

        Returns:
            是否成功
        """
        if self.tip_status == TipStatus.TIP_ATTACHED:
            logger.warning("已有枪头，无需重复装载")
            return True

        # 这里需要与运动控制配合，移动到枪头盒位置
        # 暂时模拟装载过程
        logger.info("装载枪头")
        time.sleep(1.0)

        self.tip_status = TipStatus.TIP_ATTACHED
        self.tip_count += 1
        self.current_volume = 0.0
        return True

    def eject_tip(self) -> bool:
        """
        弹出枪头

        Returns:
            是否成功
        """
        if self.tip_status == TipStatus.NO_TIP:
            logger.warning("无枪头可弹出")
            return True

        try:
            if self.pipette.eject_tip():
                self.tip_status = TipStatus.NO_TIP
                self.current_volume = 0.0
                logger.info("枪头已弹出")
                return True
            return False
        except Exception as e:
            logger.error(f"弹出枪头失败: {e}")
            return False

    def aspirate(self, volume: float, liquid_class: Optional[LiquidClass] = None,
                detection: bool = True) -> bool:
        """
        吸液

        Args:
            volume: 吸液体积(ul)
            liquid_class: 液体类型
            detection: 是否开启液位检测

        Returns:
            是否成功
        """
        if self.tip_status != TipStatus.TIP_ATTACHED:
            logger.error("无枪头，无法吸液")
            return False

        if self.current_volume + volume > self.max_volume:
            logger.error(f"吸液量超过枪头容量: {self.current_volume + volume} > {self.max_volume}")
            return False

        # 设置液体参数
        if liquid_class:
            self.set_liquid_class(liquid_class)

        try:
            # 设置吸液速度
            self.pipette.set_max_speed(self.liquid_params.aspirate_speed)

            # 执行液位检测
            if detection:
                if not self.pipette.liquid_level_detection():
                    logger.warning("液位检测失败，继续吸液")

            # 预润湿
            if self.liquid_params.pre_wet and self.current_volume == 0:
                logger.info("执行预润湿")
                self._pre_wet(volume * 0.2)

            # 吸液
            if self.pipette.aspirate(volume, detection=False):
                self.current_volume += volume
                self.aspirate_count += 1

                # 吸液后延时
                time.sleep(self.liquid_params.delay_after_aspirate)

                # 吸取空气间隙
                if self.liquid_params.air_gap > 0:
                    self.pipette.aspirate(self.liquid_params.air_gap, detection=False)
                    self.current_volume += self.liquid_params.air_gap

                logger.info(f"吸液完成: {volume}ul, 当前体积: {self.current_volume}ul")
                return True
            else:
                logger.error("吸液失败")
                return False

        except Exception as e:
            logger.error(f"吸液异常: {e}")
            return False

    def dispense(self, volume: float, blow_out: bool = False) -> bool:
        """
        排液

        Args:
            volume: 排液体积(ul)
            blow_out: 是否吹出

        Returns:
            是否成功
        """
        if self.tip_status != TipStatus.TIP_ATTACHED:
            logger.error("无枪头，无法排液")
            return False

        if volume > self.current_volume:
            logger.error(f"排液量超过当前体积: {volume} > {self.current_volume}")
            return False

        try:
            # 设置排液速度
            self.pipette.set_max_speed(self.liquid_params.dispense_speed)

            # 排液
            if self.pipette.dispense(volume):
                self.current_volume -= volume
                self.dispense_count += 1

                # 排液后延时
                time.sleep(self.liquid_params.delay_after_dispense)

                # 吹出
                if blow_out and self.liquid_params.blow_out > 0:
                    self.pipette.dispense(self.liquid_params.blow_out)
                    logger.debug(f"执行吹出: {self.liquid_params.blow_out}ul")

                # 接触壁
                if self.liquid_params.touch_tip:
                    self._touch_tip()

                logger.info(f"排液完成: {volume}ul, 剩余体积: {self.current_volume}ul")
                return True
            else:
                logger.error("排液失败")
                return False

        except Exception as e:
            logger.error(f"排液异常: {e}")
            return False

    def transfer(self, volume: float,
                 source_well: Optional[str] = None,
                 dest_well: Optional[str] = None,
                 liquid_class: Optional[LiquidClass] = None,
                 new_tip: bool = True,
                 mix_before: Optional[Tuple[int, float]] = None,
                 mix_after: Optional[Tuple[int, float]] = None) -> bool:
        """
        液体转移

        Args:
            volume: 转移体积
            source_well: 源孔位
            dest_well: 目标孔位
            liquid_class: 液体类型
            new_tip: 是否使用新枪头
            mix_before: 吸液前混合(次数, 体积)
            mix_after: 排液后混合(次数, 体积)

        Returns:
            是否成功
        """
        try:
            # 装载新枪头
            if new_tip:
                self.eject_tip()
                if not self.pickup_tip():
                    return False

            # 设置液体类型
            if liquid_class:
                self.set_liquid_class(liquid_class)

            # 吸液前混合
            if mix_before:
                cycles, mix_vol = mix_before
                self.mix(cycles, mix_vol)

            # 吸液
            if not self.aspirate(volume):
                return False

            # 排液
            if not self.dispense(volume, blow_out=True):
                return False

            # 排液后混合
            if mix_after:
                cycles, mix_vol = mix_after
                self.mix(cycles, mix_vol)

            logger.info(f"液体转移完成: {volume}ul")
            return True

        except Exception as e:
            logger.error(f"液体转移失败: {e}")
            return False

    def mix(self, cycles: int = 3, volume: Optional[float] = None) -> bool:
        """
        混合

        Args:
            cycles: 混合次数
            volume: 混合体积

        Returns:
            是否成功
        """
        volume = volume or self.liquid_params.mix_volume

        logger.info(f"开始混合: {cycles}次, {volume}ul")

        for i in range(cycles):
            if not self.aspirate(volume, detection=False):
                return False
            if not self.dispense(volume):
                return False

        logger.info("混合完成")
        return True

    def _pre_wet(self, volume: float):
        """预润湿"""
        self.pipette.aspirate(volume, detection=False)
        time.sleep(0.2)
        self.pipette.dispense(volume)
        time.sleep(0.2)

    def _touch_tip(self):
        """接触壁(需要与运动控制配合)"""
        # TODO: 实现接触壁动作
        logger.debug("执行接触壁")
        time.sleep(0.5)

    def _update_tip_status(self):
        """更新枪头状态"""
        if self.pipette.get_tip_status():
            self.tip_status = TipStatus.TIP_ATTACHED
        else:
            self.tip_status = TipStatus.NO_TIP

    def set_liquid_class(self, liquid_class: LiquidClass):
        """设置液体类型"""
        self.liquid_class = liquid_class
        if liquid_class in self.LIQUID_PARAMS:
            self.liquid_params = self.LIQUID_PARAMS[liquid_class]
        logger.info(f"液体类型设置为: {liquid_class.value}")

    def set_custom_parameters(self, params: LiquidParameters):
        """设置自定义液体参数"""
        self.liquid_params = params
        self.liquid_class = LiquidClass.CUSTOM

    def calibrate_volume(self, expected: float, actual: float):
        """
        体积校准

        Args:
            expected: 期望体积
            actual: 实际体积
        """
        factor = actual / expected
        self.pipette.set_calibration_factor(factor)
        logger.info(f"体积校准系数: {factor}")

    def get_status(self) -> Dict:
        """获取状态信息"""
        return {
            'tip_status': self.tip_status.value,
            'current_volume': self.current_volume,
            'max_volume': self.max_volume,
            'liquid_class': self.liquid_class.value,
            'statistics': {
                'tip_count': self.tip_count,
                'aspirate_count': self.aspirate_count,
                'dispense_count': self.dispense_count
            }
        }

    def reset_statistics(self):
        """重置统计信息"""
        self.tip_count = 0
        self.aspirate_count = 0
        self.dispense_count = 0

# ============================================================================
# 实例化代码块 - 移液控制器使用示例
# ============================================================================
