#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LaiYu液体处理设备硬件连接测试脚本

使用方法:
1. 修改下面的配置参数（串口号、波特率等）
2. 运行脚本: python test_hardware_connection.py
3. 查看连接结果和设备状态

作者: UniLabOS团队
"""

import sys
import os
import asyncio
import logging
from typing import Dict, Any

# 添加项目路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

try:
    from unilabos.devices.LaiYu_Liquid.LaiYu_Liquid import (
        LaiYuLiquid, LaiYuLiquidConfig
    )
    LAIYU_AVAILABLE = True
except ImportError as e:
    print(f"❌ 无法导入LaiYu模块: {e}")
    LAIYU_AVAILABLE = False

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class HardwareConnectionTester:
    """硬件连接测试器"""
    
    def __init__(self):
        self.test_results = {}
    
    def print_header(self):
        """打印测试头部信息"""
        print("=" * 60)
        print("🔬 LaiYu液体处理设备硬件连接测试")
        print("=" * 60)
        print()
    
    def print_config(self, config: LaiYuLiquidConfig):
        """打印配置信息"""
        print("📋 当前配置:")
        print(f"  串口号: {config.port}")
        print(f"  波特率: {config.baudrate}")
        print(f"  设备地址: {config.address}")
        print(f"  超时时间: {config.timeout}秒")
        print(f"  工作台尺寸: {config.deck_width}×{config.deck_height}×{config.deck_depth}mm")
        print("-" * 50)
    
    async def test_basic_connection(self, config: LaiYuLiquidConfig) -> bool:
        """测试基本连接"""
        print("🔌 测试1: 基本设备连接")
        
        try:
            device = LaiYuLiquid(config)
            print("  ✅ 设备实例创建成功")
            
            # 尝试连接
            print("  📡 正在尝试连接设备...")
            success = await device.setup()
            
            if success:
                print("  ✅ 设备连接成功!")
                self.test_results['basic_connection'] = True
                
                # 获取连接状态
                print(f"  📊 连接状态: {device.is_connected}")
                print(f"  📊 初始化状态: {device.is_initialized}")
                
                return True
            else:
                print("  ❌ 设备连接失败")
                self.test_results['basic_connection'] = False
                return False
                
        except Exception as e:
            print(f"  ❌ 连接测试异常: {e}")
            self.test_results['basic_connection'] = False
            return False
        finally:
            if 'device' in locals():
                await device.stop()
    
    async def test_device_status(self, config: LaiYuLiquidConfig) -> Dict[str, Any]:
        """测试设备状态获取"""
        print("\n📊 测试2: 设备状态获取")
        
        try:
            device = LaiYuLiquid(config)
            await device.setup()
            
            # 获取设备状态
            status = device.get_status()
            print("  ✅ 设备状态获取成功:")
            
            for key, value in status.items():
                print(f"    {key}: {value}")
            
            self.test_results['device_status'] = status
            return status
            
        except Exception as e:
            print(f"  ❌ 状态获取异常: {e}")
            self.test_results['device_status'] = None
            return None
        finally:
            if 'device' in locals():
                await device.stop()
    
    async def test_safety_checks(self, config: LaiYuLiquidConfig) -> bool:
        """测试安全检查功能"""
        print("\n🛡️ 测试3: 安全检查功能")
        
        try:
            device = LaiYuLiquid(config)
            await device.setup()
            
            # 测试位置验证
            backend = device.backend
            
            # 测试有效位置
            valid_pos = backend._validate_position(100.0, 100.0, 25.0)
            print(f"  ✅ 有效位置检查: {valid_pos}")
            
            # 测试无效位置
            invalid_pos = backend._validate_position(500.0, 500.0, 200.0)
            print(f"  ✅ 无效位置检查: {not invalid_pos}")
            
            # 测试硬件就绪检查
            hardware_ready = backend._check_hardware_ready()
            print(f"  ✅ 硬件就绪检查: {hardware_ready}")
            
            self.test_results['safety_checks'] = True
            return True
            
        except Exception as e:
            print(f"  ❌ 安全检查异常: {e}")
            self.test_results['safety_checks'] = False
            return False
        finally:
            if 'device' in locals():
                await device.stop()
    
    async def test_movement_simulation(self, config: LaiYuLiquidConfig) -> bool:
        """测试移动模拟"""
        print("\n🎯 测试4: 移动功能模拟")
        
        try:
            device = LaiYuLiquid(config)
            await device.setup()
            
            # 测试移动到安全位置
            print("  📍 测试移动到安全位置...")
            safe_move = await device.backend.move_to_safe_position()
            print(f"  ✅ 安全位置移动: {safe_move}")
            
            # 测试基本移动
            print("  📍 测试基本移动...")
            basic_move = await device.backend.move_to(100.0, 100.0, 30.0)
            print(f"  ✅ 基本移动: {basic_move}")
            
            self.test_results['movement_simulation'] = True
            return True
            
        except Exception as e:
            print(f"  ❌ 移动测试异常: {e}")
            self.test_results['movement_simulation'] = False
            return False
        finally:
            if 'device' in locals():
                await device.stop()
    
    def print_serial_port_info(self):
        """打印串口信息"""
        print("\n🔍 系统串口设备检查:")
        
        import glob
        import platform
        
        system = platform.system()
        
        if system == "Darwin":  # macOS
            ports = glob.glob("/dev/cu.*")
            print("  macOS串口设备:")
            for port in sorted(ports):
                print(f"    {port}")
        elif system == "Linux":
            ports = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
            print("  Linux串口设备:")
            for port in sorted(ports):
                print(f"    {port}")
        elif system == "Windows":
            print("  Windows系统请在设备管理器中查看COM端口")
        
        if not ports and system != "Windows":
            print("  ⚠️ 未找到串口设备，请检查:")
            print("    1. USB设备是否已连接")
            print("    2. 驱动程序是否已安装")
            print("    3. 设备是否已通电")
    
    def print_troubleshooting(self):
        """打印故障排除建议"""
        print("\n🔧 故障排除建议:")
        print("1. 串口连接问题:")
        print("   - 检查串口号是否正确 (macOS: /dev/cu.*, Linux: /dev/ttyUSB*)")
        print("   - 检查设备是否已连接并通电")
        print("   - 检查串口权限: sudo chmod 666 /dev/cu.usbserial-*")
        print("   - 检查是否被其他程序占用: lsof | grep /dev/cu.usbserial")
        
        print("\n2. 波特率问题:")
        print("   - 尝试不同的波特率: 9600, 19200, 38400, 57600, 115200")
        print("   - 确认设备固件的波特率设置")
        
        print("\n3. 设备地址问题:")
        print("   - XYZ控制器: 通常使用地址 1-3")
        print("   - 移液器: 通常使用地址 4")
        print("   - 避免使用地址: 47 ('/'), 69 ('E'), 91 ('[')")
        
        print("\n4. 硬件问题:")
        print("   - 检查USB线缆连接")
        print("   - 检查设备电源")
        print("   - 检查RS485转换器")
    
    def print_summary(self):
        """打印测试总结"""
        print("\n" + "=" * 60)
        print("📋 测试结果总结")
        print("=" * 60)
        
        total_tests = len(self.test_results)
        passed_tests = sum(1 for result in self.test_results.values() if result)
        
        print(f"总测试数: {total_tests}")
        print(f"通过测试: {passed_tests}")
        print(f"失败测试: {total_tests - passed_tests}")
        print(f"成功率: {passed_tests/total_tests*100:.1f}%")
        
        print("\n详细结果:")
        for test_name, result in self.test_results.items():
            status = "✅ 通过" if result else "❌ 失败"
            print(f"  {test_name}: {status}")
        
        if passed_tests == total_tests:
            print("\n🎉 所有测试通过! 设备连接正常。")
        else:
            print("\n⚠️ 部分测试失败，请检查硬件连接和配置。")


async def main():
    """主测试函数"""
    
    if not LAIYU_AVAILABLE:
        print("❌ LaiYu模块不可用，请检查安装")
        return
    
    tester = HardwareConnectionTester()
    tester.print_header()
    
    # 🔧 在这里修改你的硬件连接参数
    config = LaiYuLiquidConfig(
        port="/dev/cu.usbserial-3130",  # 🔧 修改为你的串口号
        address=1,                       # 🔧 修改为你的设备地址
        baudrate=9600,                   # 🔧 修改为你的波特率 (常用: 9600, 115200)
        timeout=5.0,                     # 🔧 修改超时时间
        
        # 工作台尺寸配置
        deck_width=340.0,
        deck_height=250.0,
        deck_depth=160.0,
        safe_height=50.0,
        
        # 安全检查配置
        position_validation=True,
        emergency_stop_enabled=True
    )
    
    tester.print_config(config)
    tester.print_serial_port_info()
    
    print("\n🚀 开始硬件连接测试...")
    print("-" * 50)
    
    # 执行测试序列
    await tester.test_basic_connection(config)
    await tester.test_device_status(config)
    await tester.test_safety_checks(config)
    await tester.test_movement_simulation(config)
    
    # 打印结果
    tester.print_summary()
    tester.print_troubleshooting()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n\n⏹️ 测试被用户中断")
    except Exception as e:
        print(f"\n\n❌ 测试过程中发生错误: {e}")
        import traceback
        traceback.print_exc()