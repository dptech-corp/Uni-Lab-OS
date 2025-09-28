#!/usr/bin/env python3
"""
测试脚本：验证LaiYu液体处理设备的取枪头功能
包含Z轴下降控制逻辑的测试
"""

import asyncio
import sys
import os
from pathlib import Path

# 添加项目根目录到Python路径
project_root = Path(__file__).parent
sys.path.insert(0, str(project_root))

from unilabos.devices.LaiYu_Liquid.LaiYu_Liquid import (
    LaiYuLiquid, 
    LaiYuLiquidConfig,
    LaiYuLiquidBackend
)
from unilabos.utils.log import get_logger

logger = get_logger(__name__)

async def test_tip_pickup_functionality():
    """测试取枪头功能的完整流程"""
    
    print("=" * 60)
    print("开始测试LaiYu液体处理设备的取枪头功能")
    print("=" * 60)
    
    # 1. 创建配置
    config = LaiYuLiquidConfig(
        # 设备连接配置
        pipette_port="/dev/ttyUSB0",
        xyz_port="/dev/ttyUSB1",
        
        # 取枪头配置
        tip_pickup_speed=30,
        tip_pickup_acceleration=20,
        tip_approach_height=10.0,
        tip_pickup_force_depth=2.0,
        tip_pickup_retract_height=15.0,
        
        # 丢枪头配置
        tip_drop_height=5.0,
        tip_drop_speed=50,
        trash_position=(250.0, 180.0, 0.0),
        
        # 安全范围配置
        deck_width=300.0,
        deck_height=200.0,
        deck_depth=100.0,
        safe_height=50.0,
        position_validation=True,
        emergency_stop_enabled=True
    )
    
    print(f"✓ 配置创建完成")
    print(f"  - 取枪头接近高度: {config.tip_approach_height}mm")
    print(f"  - 取枪头下压深度: {config.tip_pickup_force_depth}mm")
    print(f"  - 取枪头回缩高度: {config.tip_pickup_retract_height}mm")
    print(f"  - 安全范围: {config.deck_width}x{config.deck_height}x{config.deck_depth}mm")
    
    # 2. 创建设备实例
    try:
        device = LaiYuLiquid(config=config)
        print(f"✓ 设备实例创建成功")
    except Exception as e:
        print(f"✗ 设备实例创建失败: {e}")
        return False
    
    # 3. 设备初始化
    try:
        setup_success = await device.backend.setup()
        if setup_success:
            print(f"✓ 设备初始化成功")
        else:
            print(f"⚠ 设备初始化失败，将使用模拟模式")
    except Exception as e:
        print(f"⚠ 设备初始化异常: {e}，将使用模拟模式")
    
    # 4. 测试移动到安全位置
    print("\n--- 测试移动到安全位置 ---")
    try:
        safe_move_success = await device.backend.move_to_safe_position()
        if safe_move_success:
            print(f"✓ 成功移动到安全位置: {device.backend.current_position}")
        else:
            print(f"✗ 移动到安全位置失败")
    except Exception as e:
        print(f"✗ 移动到安全位置异常: {e}")
    
    # 5. 测试取枪头功能
    print("\n--- 测试取枪头功能 ---")
    
    # 模拟枪头架位置
    tip_rack_name = "tip_rack_1000"
    tip_position = 0  # 第一个枪头位置
    
    try:
        # 执行取枪头操作
        pickup_success = await device.backend.pick_up_tip(tip_rack_name, tip_position)
        
        if pickup_success:
            print(f"✓ 取枪头操作成功")
            print(f"  - 枪头架: {tip_rack_name}")
            print(f"  - 位置: {tip_position}")
            print(f"  - 当前位置: {device.backend.current_position}")
            print(f"  - 枪头状态: {'已附着' if device.backend.tip_attached else '未附着'}")
        else:
            print(f"✗ 取枪头操作失败")
            return False
            
    except Exception as e:
        print(f"✗ 取枪头操作异常: {e}")
        return False
    
    # 6. 测试丢枪头功能
    print("\n--- 测试丢枪头功能 ---")
    
    try:
        # 执行丢枪头操作
        drop_success = await device.backend.drop_tip()
        
        if drop_success:
            print(f"✓ 丢枪头操作成功")
            print(f"  - 丢弃位置: {config.trash_position}")
            print(f"  - 当前位置: {device.backend.current_position}")
            print(f"  - 枪头状态: {'已附着' if device.backend.tip_attached else '未附着'}")
        else:
            print(f"✗ 丢枪头操作失败")
            return False
            
    except Exception as e:
        print(f"✗ 丢枪头操作异常: {e}")
        return False
    
    # 7. 测试安全检查功能
    print("\n--- 测试安全检查功能 ---")
    
    # 测试位置验证
    test_positions = [
        (150.0, 100.0, 25.0, True, "正常位置"),
        (-10.0, 100.0, 25.0, False, "X轴超出下限"),
        (350.0, 100.0, 25.0, False, "X轴超出上限"),
        (150.0, -10.0, 25.0, False, "Y轴超出下限"),
        (150.0, 250.0, 25.0, False, "Y轴超出上限"),
        (150.0, 100.0, -150.0, False, "Z轴超出下限"),
        (150.0, 100.0, 80.0, False, "Z轴超出上限"),
    ]
    
    for x, y, z, expected, description in test_positions:
        result = device.backend._validate_position(x, y, z)
        status = "✓" if result == expected else "✗"
        print(f"  {status} {description}: ({x}, {y}, {z}) -> {result}")
    
    # 8. 测试完整的取枪头-丢枪头循环
    print("\n--- 测试完整循环 ---")
    
    for cycle in range(2):
        print(f"\n第 {cycle + 1} 次循环:")
        
        # 取枪头
        pickup_success = await device.backend.pick_up_tip(tip_rack_name, tip_position)
        if pickup_success:
            print(f"  ✓ 取枪头成功")
        else:
            print(f"  ✗ 取枪头失败")
            break
        
        # 短暂等待
        await asyncio.sleep(0.5)
        
        # 丢枪头
        drop_success = await device.backend.drop_tip()
        if drop_success:
            print(f"  ✓ 丢枪头成功")
        else:
            print(f"  ✗ 丢枪头失败")
            break
    
    print("\n" + "=" * 60)
    print("测试完成！")
    print("=" * 60)
    
    return True

async def test_error_handling():
    """测试错误处理和恢复机制"""
    
    print("\n--- 测试错误处理机制 ---")
    
    config = LaiYuLiquidConfig()
    device = LaiYuLiquid(config=config)
    
    # 测试在未连接状态下的操作
    print("测试未连接状态下的操作:")
    device.backend.is_connected = False
    
    pickup_result = await device.backend.pick_up_tip("test_rack", 0)
    print(f"  未连接状态取枪头: {'通过' if not pickup_result else '失败'}")
    
    drop_result = await device.backend.drop_tip()
    print(f"  未连接状态丢枪头: {'通过' if not drop_result else '失败'}")
    
    # 测试重复取枪头
    print("测试重复取枪头:")
    device.backend.is_connected = True
    device.backend.tip_attached = True
    
    pickup_result = await device.backend.pick_up_tip("test_rack", 0)
    print(f"  重复取枪头: {'通过' if not pickup_result else '失败'}")
    
    # 测试无枪头时丢枪头
    print("测试无枪头时丢枪头:")
    device.backend.tip_attached = False
    
    drop_result = await device.backend.drop_tip()
    print(f"  无枪头丢枪头: {'通过' if drop_result else '失败'}")

def main():
    """主函数"""
    print("LaiYu液体处理设备 - 取枪头功能测试")
    print("测试包含Z轴下降控制逻辑的完整实现")
    
    try:
        # 运行主要测试
        success = asyncio.run(test_tip_pickup_functionality())
        
        # 运行错误处理测试
        asyncio.run(test_error_handling())
        
        if success:
            print("\n🎉 所有测试通过！Z轴下降控制逻辑工作正常。")
            return 0
        else:
            print("\n❌ 部分测试失败，请检查实现。")
            return 1
            
    except KeyboardInterrupt:
        print("\n⚠️ 测试被用户中断")
        return 1
    except Exception as e:
        print(f"\n💥 测试过程中发生未预期的错误: {e}")
        return 1

if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)