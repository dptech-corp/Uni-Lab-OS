#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
XYZ三轴步进电机控制器
支持坐标系管理、限位开关回零、工作原点设定等功能

主要功能：
- 坐标系转换层（步数↔毫米）
- 限位开关回零功能
- 工作原点示教和保存
- 安全限位检查
- 运动控制接口

"""

import json
import os
import time
from typing import Optional, Dict, Tuple, Union
from dataclasses import dataclass, asdict
from pathlib import Path
import logging

# 添加项目根目录到Python路径以解决模块导入问题
import sys
import os

# 无论如何都添加项目根目录到路径
current_file = os.path.abspath(__file__)
# 从 .../Uni-Lab-OS/unilabos/devices/LaiYu_Liquid/controllers/xyz_controller.py
# 向上5级到 .../Uni-Lab-OS
project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(current_file)))))
# 强制添加项目根目录到sys.path的开头
sys.path.insert(0, project_root)

# 导入原有的驱动
from unilabos.devices.LaiYu_Liquid.drivers.xyz_stepper_driver import XYZStepperController, MotorAxis, MotorStatus

logger = logging.getLogger(__name__)


@dataclass
class MachineConfig:
    """机械配置参数"""
    # 步距配置 (基于16384步/圈的步进电机)
    steps_per_mm_x: float = 195.05   # X轴步距 (16384步/圈 ÷ 84mm导程)
    steps_per_mm_y: float = 195.05   # Y轴步距 (16384步/圈 ÷ 84mm导程)  
    steps_per_mm_z: float = 3276.8   # Z轴步距 (16384步/圈 ÷ 5mm导程)
    
    # 行程限制
    max_travel_x: float = 340.0     # X轴最大行程
    max_travel_y: float = 250.0     # Y轴最大行程
    max_travel_z: float = 160.0     # Z轴最大行程
    
    # 回零参数
    homing_speed: int = 50          # 回零速度 (rpm)
    homing_timeout: float = 30.0    # 回零超时时间
    safe_clearance: float = 1.0     # 安全间隙 (mm)
    position_stable_time: float = 3.0  # 位置稳定检测时间（秒）
    position_check_interval: float = 0.2  # 位置检查间隔（秒）
    
    # 运动参数
    default_speed: int = 50         # 默认运动速度 (rpm)
    default_acceleration: int = 1000 # 默认加速度


@dataclass 
class CoordinateOrigin:
    """坐标原点信息"""
    machine_origin_steps: Dict[str, int] = None    # 机械原点步数位置
    work_origin_steps: Dict[str, int] = None       # 工作原点步数位置
    is_homed: bool = False                         # 是否已回零
    timestamp: str = ""                            # 设定时间戳
    
    def __post_init__(self):
        if self.machine_origin_steps is None:
            self.machine_origin_steps = {"x": 0, "y": 0, "z": 0}
        if self.work_origin_steps is None:
            self.work_origin_steps = {"x": 0, "y": 0, "z": 0}


class CoordinateSystemError(Exception):
    """坐标系统异常"""
    pass


class XYZController(XYZStepperController):
    """XYZ三轴控制器"""
    
    def __init__(self, port: str, baudrate: int = 115200, 
                 machine_config: Optional[MachineConfig] = None,
                 config_file: str = "machine_config.json",
                 auto_connect: bool = True):
        """
        初始化XYZ控制器
        
        Args:
            port: 串口端口
            baudrate: 波特率
            machine_config: 机械配置参数
            config_file: 配置文件路径
            auto_connect: 是否自动连接设备
        """
        super().__init__(port, baudrate)
        
        # 机械配置
        self.machine_config = machine_config or MachineConfig()
        self.config_file = config_file
        
        # 坐标系统
        self.coordinate_origin = CoordinateOrigin()
        self.origin_file = "coordinate_origin.json"
        
        # 连接状态
        self.is_connected = False
        
        # 加载配置
        self._load_config()
        self._load_coordinate_origin()
        
        # 自动连接设备
        if auto_connect:
            self.connect_device()
    
    def connect_device(self) -> bool:
        """
        连接设备并初始化
        
        Returns:
            bool: 连接是否成功
        """
        try:
            logger.info(f"正在连接设备: {self.port}")
            
            # 连接硬件
            if not self.connect():
                logger.error("硬件连接失败")
                return False
            
            self.is_connected = True
            logger.info("设备连接成功")
            
            # 使能所有轴
            enable_results = self.enable_all_axes(True)
            success_count = sum(1 for result in enable_results.values() if result)
            logger.info(f"轴使能结果: {success_count}/{len(enable_results)} 成功")
            
            # 获取系统状态
            try:
                status = self.get_system_status()
                logger.info(f"系统状态获取成功: {len(status)} 项信息")
            except Exception as e:
                logger.warning(f"获取系统状态失败: {e}")
            
            return True
            
        except Exception as e:
            logger.error(f"设备连接失败: {e}")
            self.is_connected = False
            return False
    
    def disconnect_device(self):
        """断开设备连接"""
        try:
            if self.is_connected:
                self.disconnect()  # 使用父类的disconnect方法
                self.is_connected = False
                logger.info("设备连接已断开")
        except Exception as e:
            logger.error(f"断开连接失败: {e}")
    
    def _load_config(self):
        """加载机械配置"""
        try:
            if os.path.exists(self.config_file):
                with open(self.config_file, 'r', encoding='utf-8') as f:
                    config_data = json.load(f)
                    # 更新配置参数
                    for key, value in config_data.items():
                        if hasattr(self.machine_config, key):
                            setattr(self.machine_config, key, value)
                logger.info("机械配置加载完成")
        except Exception as e:
            logger.warning(f"加载机械配置失败: {e}，使用默认配置")
    
    def _save_config(self):
        """保存机械配置"""
        try:
            with open(self.config_file, 'w', encoding='utf-8') as f:
                json.dump(asdict(self.machine_config), f, indent=2, ensure_ascii=False)
            logger.info("机械配置保存完成")
        except Exception as e:
            logger.error(f"保存机械配置失败: {e}")
    
    def _load_coordinate_origin(self):
        """加载坐标原点信息"""
        try:
            if os.path.exists(self.origin_file):
                with open(self.origin_file, 'r', encoding='utf-8') as f:
                    origin_data = json.load(f)
                    self.coordinate_origin = CoordinateOrigin(**origin_data)
                logger.info("坐标原点信息加载完成")
        except Exception as e:
            logger.warning(f"加载坐标原点失败: {e}，使用默认设置")
    
    def _save_coordinate_origin(self):
        """保存坐标原点信息"""
        try:
            # 更新时间戳
            from datetime import datetime
            self.coordinate_origin.timestamp = datetime.now().isoformat()
            
            with open(self.origin_file, 'w', encoding='utf-8') as f:
                json.dump(asdict(self.coordinate_origin), f, indent=2, ensure_ascii=False)
            logger.info("坐标原点信息保存完成")
        except Exception as e:
            logger.error(f"保存坐标原点失败: {e}")
    
    # ==================== 坐标转换方法 ====================
    
    def mm_to_steps(self, axis: MotorAxis, mm: float) -> int:
        """毫米转步数"""
        if axis == MotorAxis.X:
            return int(mm * self.machine_config.steps_per_mm_x)
        elif axis == MotorAxis.Y:
            return int(mm * self.machine_config.steps_per_mm_y)
        elif axis == MotorAxis.Z:
            return int(mm * self.machine_config.steps_per_mm_z)
        else:
            raise ValueError(f"未知轴: {axis}")
    
    def steps_to_mm(self, axis: MotorAxis, steps: int) -> float:
        """步数转毫米"""
        if axis == MotorAxis.X:
            return steps / self.machine_config.steps_per_mm_x
        elif axis == MotorAxis.Y:
            return steps / self.machine_config.steps_per_mm_y
        elif axis == MotorAxis.Z:
            return steps / self.machine_config.steps_per_mm_z
        else:
            raise ValueError(f"未知轴: {axis}")
    
    def work_to_machine_steps(self, x: float = None, y: float = None, z: float = None) -> Dict[str, int]:
        """工作坐标转机械坐标步数"""
        machine_steps = {}
        
        if x is not None:
            work_steps = self.mm_to_steps(MotorAxis.X, x)
            machine_steps['x'] = self.coordinate_origin.work_origin_steps['x'] + work_steps
        
        if y is not None:
            work_steps = self.mm_to_steps(MotorAxis.Y, y)
            machine_steps['y'] = self.coordinate_origin.work_origin_steps['y'] + work_steps
            
        if z is not None:
            work_steps = self.mm_to_steps(MotorAxis.Z, z)
            machine_steps['z'] = self.coordinate_origin.work_origin_steps['z'] + work_steps
            
        return machine_steps
    
    def machine_to_work_coords(self, machine_steps: Dict[str, int]) -> Dict[str, float]:
        """机械坐标步数转工作坐标"""
        work_coords = {}
        
        for axis_name, steps in machine_steps.items():
            axis = MotorAxis[axis_name.upper()]
            work_origin_steps = self.coordinate_origin.work_origin_steps[axis_name]
            relative_steps = steps - work_origin_steps
            work_coords[axis_name] = self.steps_to_mm(axis, relative_steps)
            
        return work_coords
    
    def check_travel_limits(self, x: float = None, y: float = None, z: float = None) -> bool:
        """检查行程限制"""
        if x is not None and (x < 0 or x > self.machine_config.max_travel_x):
            raise CoordinateSystemError(f"X轴超出行程范围: {x}mm (0 ~ {self.machine_config.max_travel_x}mm)")
        
        if y is not None and (y < 0 or y > self.machine_config.max_travel_y):
            raise CoordinateSystemError(f"Y轴超出行程范围: {y}mm (0 ~ {self.machine_config.max_travel_y}mm)")
            
        if z is not None and (z < 0 or z > self.machine_config.max_travel_z):
            raise CoordinateSystemError(f"Z轴超出行程范围: {z}mm (0 ~ {self.machine_config.max_travel_z}mm)")
            
        return True
    
    # ==================== 回零和原点设定方法 ====================
    
    def home_axis(self, axis: MotorAxis, direction: int = -1) -> bool:
        """
        单轴回零到限位开关 - 使用步数变化检测
        
        Args:
            axis: 要回零的轴
            direction: 回零方向 (-1负方向, 1正方向)
            
        Returns:
            bool: 回零是否成功
        """
        if not self.is_connected:
            logger.error("设备未连接，无法执行回零操作")
            return False
            
        try:
            logger.info(f"开始{axis.name}轴回零")
            
            # 使能电机
            if not self.enable_motor(axis, True):
                raise CoordinateSystemError(f"{axis.name}轴使能失败")
            
            # 设置回零速度模式，根据方向设置正负
            speed = self.machine_config.homing_speed * direction
            if not self.set_speed_mode(axis, speed):
                raise CoordinateSystemError(f"{axis.name}轴设置回零速度失败")
            

            
            # 智能回零检测 - 基于步数变化
            start_time = time.time()
            limit_detected = False
            final_position = None
            
            # 步数变化检测参数（从配置获取）
            position_stable_time = self.machine_config.position_stable_time
            check_interval = self.machine_config.position_check_interval
            last_position = None
            stable_start_time = None
            
            logger.info(f"{axis.name}轴开始移动，监测步数变化...")
            
            while time.time() - start_time < self.machine_config.homing_timeout:
                status = self.get_motor_status(axis)
                current_position = status.steps
                
                # 检查是否明确触碰限位开关
                if (direction < 0 and status.status == MotorStatus.REVERSE_LIMIT_STOP) or \
                   (direction > 0 and status.status == MotorStatus.FORWARD_LIMIT_STOP):
                    # 停止运动
                    self.emergency_stop(axis)
                    time.sleep(0.5)
                    
                    # 记录机械原点位置
                    final_position = current_position
                    limit_detected = True
                    logger.info(f"{axis.name}轴检测到限位开关信号，位置: {final_position}步")
                    break
                
                # 检查是否发生碰撞
                if status.status == MotorStatus.COLLISION_STOP:
                    raise CoordinateSystemError(f"{axis.name}轴回零时发生碰撞")
                
                # 步数变化检测逻辑
                if last_position is not None:
                    # 检查位置是否发生变化
                    if abs(current_position - last_position) <= 1:  # 允许1步的误差
                        # 位置基本没有变化
                        if stable_start_time is None:
                            stable_start_time = time.time()
                            logger.debug(f"{axis.name}轴位置开始稳定在 {current_position}步")
                        elif time.time() - stable_start_time >= position_stable_time:
                            # 位置稳定超过指定时间，认为已到达限位
                            self.emergency_stop(axis)
                            time.sleep(0.5)
                            
                            final_position = current_position
                            limit_detected = True
                            logger.info(f"{axis.name}轴位置稳定{position_stable_time}秒，假设已到达限位开关，位置: {final_position}步")
                            break
                    else:
                        # 位置发生变化，重置稳定计时
                        stable_start_time = None
                        logger.debug(f"{axis.name}轴位置变化: {last_position} -> {current_position}")
                
                last_position = current_position
                time.sleep(check_interval)
            
            # 超时处理
            if not limit_detected:
                logger.warning(f"{axis.name}轴回零超时({self.machine_config.homing_timeout}秒)，强制停止")
                self.emergency_stop(axis)
                time.sleep(0.5)
                
                # 获取当前位置作为机械原点
                try:
                    status = self.get_motor_status(axis)
                    final_position = status.steps
                    logger.info(f"{axis.name}轴超时后位置: {final_position}步")
                except Exception as e:
                    logger.error(f"获取{axis.name}轴位置失败: {e}")
                    return False
            
            # 记录机械原点位置
            self.coordinate_origin.machine_origin_steps[axis.name.lower()] = final_position
            
            # 从限位开关退出安全距离
            try:
                clearance_steps = self.mm_to_steps(axis, self.machine_config.safe_clearance)
                safe_position = final_position + (clearance_steps * -direction)  # 反方向退出
                
                if not self.move_to_position(axis, safe_position, 
                                            self.machine_config.default_speed):
                    logger.warning(f"{axis.name}轴无法退出到安全位置")
                else:
                    self.wait_for_completion(axis, 10.0)
                    logger.info(f"{axis.name}轴已退出到安全位置: {safe_position}步")
            except Exception as e:
                logger.warning(f"{axis.name}轴退出安全位置时出错: {e}")
            
            status_msg = "限位检测成功" if limit_detected else "超时假设成功"
            logger.info(f"{axis.name}轴回零完成 ({status_msg})，机械原点: {final_position}步")
            return True
            
        except Exception as e:
            logger.error(f"{axis.name}轴回零失败: {e}")
            self.emergency_stop(axis)
            return False
    
    def home_all_axes(self, sequence: list = None) -> bool:
        """
        全轴回零
        
        Args:
            sequence: 回零顺序，默认为Z->X->Y
            
        Returns:
            bool: 全轴回零是否成功
        """
        if not self.is_connected:
            logger.error("设备未连接，无法执行回零操作")
            return False
            
        sequence = sequence or [MotorAxis.Z, MotorAxis.X, MotorAxis.Y]
        
        logger.info("开始全轴回零")
        
        try:
            for axis in sequence:
                if not self.home_axis(axis):
                    logger.error(f"全轴回零失败，停止在{axis.name}轴")
                    return False
                
                # 轴间等待时间
                time.sleep(0.5)
            
            # 标记为已回零
            self.coordinate_origin.is_homed = True
            self._save_coordinate_origin()
            
            logger.info("全轴回零完成")
            return True
            
        except Exception as e:
            logger.error(f"全轴回零异常: {e}")
            return False
    
    def set_work_origin_here(self) -> bool:
        """将当前位置设置为工作原点"""
        if not self.is_connected:
            logger.error("设备未连接，无法设置工作原点")
            return False
            
        try:
            if not self.coordinate_origin.is_homed:
                logger.warning("建议先执行回零操作再设置工作原点")
            
            # 获取当前各轴位置
            positions = self.get_all_positions()
            
            for axis in MotorAxis:
                axis_name = axis.name.lower()
                current_steps = positions[axis].steps
                self.coordinate_origin.work_origin_steps[axis_name] = current_steps
                
                logger.info(f"{axis.name}轴工作原点设置为: {current_steps}步 "
                          f"({self.steps_to_mm(axis, current_steps):.2f}mm)")
            
            self._save_coordinate_origin()
            logger.info("工作原点设置完成")
            return True
            
        except Exception as e:
            logger.error(f"设置工作原点失败: {e}")
            return False
    
    # ==================== 高级运动控制方法 ====================
    
    def move_to_work_coord(self, x: float = None, y: float = None, z: float = None,
                          speed: int = None, acceleration: int = None) -> bool:
        """
        移动到工作坐标系指定位置
        
        Args:
            x, y, z: 工作坐标系下的目标位置 (mm)
            speed: 运动速度 (rpm)
            acceleration: 加速度 (rpm/s)
            
        Returns:
            bool: 移动是否成功
        """
        if not self.is_connected:
            logger.error("设备未连接，无法执行移动操作")
            return False
            
        try:
            # 检查坐标系是否已设置
            if not self.coordinate_origin.work_origin_steps:
                raise CoordinateSystemError("工作原点未设置，请先调用set_work_origin_here()")
            
            # 检查行程限制
            self.check_travel_limits(x, y, z)
            
            # 转换为机械坐标步数
            machine_steps = self.work_to_machine_steps(x, y, z)
            
            # 设置运动参数
            speed = speed or self.machine_config.default_speed
            acceleration = acceleration or self.machine_config.default_acceleration
            
            # 执行运动
            success_count = 0
            total_axes = 0
            
            if x is not None:
                if self.move_to_position(MotorAxis.X, machine_steps['x'], speed, acceleration):
                    success_count += 1
                total_axes += 1
                
            if y is not None:
                if self.move_to_position(MotorAxis.Y, machine_steps['y'], speed, acceleration):
                    success_count += 1
                total_axes += 1
                
            if z is not None:
                if self.move_to_position(MotorAxis.Z, machine_steps['z'], speed, acceleration):
                    success_count += 1
                total_axes += 1
            
            success = (success_count == total_axes)
            
            if success:
                logger.info(f"移动到工作坐标 X:{x} Y:{y} Z:{z} (mm)")
            else:
                logger.error("部分轴移动失败")
                
            return success
            
        except Exception as e:
            logger.error(f"工作坐标移动失败: {e}")
            return False
    
    def move_relative_work_coord(self, dx: float = 0, dy: float = 0, dz: float = 0,
                               speed: int = None, acceleration: int = None) -> bool:
        """
        相对当前位置移动
        
        Args:
            dx, dy, dz: 相对移动距离 (mm)
            speed: 运动速度 (rpm)  
            acceleration: 加速度 (rpm/s)
            
        Returns:
            bool: 移动是否成功
        """
        if not self.is_connected:
            logger.error("设备未连接，无法执行移动操作")
            return False
            
        try:
            # 获取当前工作坐标
            current_work = self.get_current_work_coords()
            
            # 计算目标坐标
            target_x = current_work['x'] + dx if dx != 0 else None
            target_y = current_work['y'] + dy if dy != 0 else None  
            target_z = current_work['z'] + dz if dz != 0 else None
            
            return self.move_to_work_coord(target_x, target_y, target_z, speed, acceleration)
            
        except Exception as e:
            logger.error(f"相对移动失败: {e}")
            return False
    
    def get_current_work_coords(self) -> Dict[str, float]:
        """获取当前工作坐标"""
        if not self.is_connected:
            logger.error("设备未连接，无法获取当前坐标")
            return {'x': 0.0, 'y': 0.0, 'z': 0.0}
            
        try:
            # 获取当前机械坐标
            positions = self.get_all_positions()
            machine_steps = {axis.name.lower(): pos.steps for axis, pos in positions.items()}
            
            # 转换为工作坐标
            return self.machine_to_work_coords(machine_steps)
            
        except Exception as e:
            logger.error(f"获取工作坐标失败: {e}")
            return {'x': 0.0, 'y': 0.0, 'z': 0.0}
    
    def get_current_position_mm(self) -> Dict[str, float]:
        """获取当前位置坐标（毫米单位）"""
        return self.get_current_work_coords()
    
    def wait_for_move_completion(self, timeout: float = 30.0) -> bool:
        """等待所有轴运动完成"""
        if not self.is_connected:
            return False
            
        for axis in MotorAxis:
            if not self.wait_for_completion(axis, timeout):
                return False
        return True
    
    # ==================== 系统状态和配置方法 ====================
    
    def get_system_status(self) -> Dict:
        """获取系统状态信息"""
        status = {
            "connection": {
                "is_connected": self.is_connected,
                "port": self.port,
                "baudrate": self.baudrate
            },
            "coordinate_system": {
                "is_homed": self.coordinate_origin.is_homed,
                "machine_origin": self.coordinate_origin.machine_origin_steps,
                "work_origin": self.coordinate_origin.work_origin_steps,
                "timestamp": self.coordinate_origin.timestamp
            },
            "machine_config": asdict(self.machine_config),
            "current_position": {}
        }
        
        if self.is_connected:
            try:
                # 获取当前位置
                positions = self.get_all_positions()
                for axis, pos in positions.items():
                    axis_name = axis.name.lower()
                    status["current_position"][axis_name] = {
                        "steps": pos.steps,
                        "mm": self.steps_to_mm(axis, pos.steps),
                        "status": pos.status.name if hasattr(pos.status, 'name') else str(pos.status)
                    }
                    
                # 获取工作坐标
                work_coords = self.get_current_work_coords()
                status["current_work_coords"] = work_coords
                
            except Exception as e:
                status["position_error"] = str(e)
        
        return status
    
    def update_machine_config(self, **kwargs):
        """更新机械配置参数"""
        for key, value in kwargs.items():
            if hasattr(self.machine_config, key):
                setattr(self.machine_config, key, value)
                logger.info(f"更新配置参数 {key}: {value}")
            else:
                logger.warning(f"未知配置参数: {key}")
        
        # 保存配置
        self._save_config()
    
    def reset_coordinate_system(self):
        """重置坐标系统"""
        self.coordinate_origin = CoordinateOrigin()
        self._save_coordinate_origin()
        logger.info("坐标系统已重置")
    
    def __enter__(self):
        """上下文管理器入口"""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """上下文管理器出口"""
        self.disconnect_device()
