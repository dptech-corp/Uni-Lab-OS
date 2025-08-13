#!/usr/bin/env python3
"""
Dummy2 机械臂接入 UniLab 系统测试脚本
"""

import os
import sys
import time
import yaml
import json

def test_device_model_files():
    """测试设备模型文件是否完整"""
    print("=== 测试设备模型文件 ===")
    
    device_path = "/Users/dp/Documents/Uni-Lab-OS/unilabos/device_mesh/devices/dummy2_robot"
    
    required_files = [
        "macro_device.xacro",
        "dummy2_robot.json",
        "config/joint_limits.yaml",
        "config/default_kinematics.yaml", 
        "config/physical_parameters.yaml",
        "config/visual_parameters.yaml"
    ]
    
    required_meshes = [
        "meshes/base_link.stl",
        "meshes/J1_1.stl",
        "meshes/J2_1.stl", 
        "meshes/J3_1.stl",
        "meshes/J4_1.stl",
        "meshes/J5_1.stl",
        "meshes/J6_1.stl",
        "meshes/camera_1.stl"
    ]
    
    all_files = required_files + required_meshes
    missing_files = []
    
    for file_path in all_files:
        full_path = os.path.join(device_path, file_path)
        if not os.path.exists(full_path):
            missing_files.append(file_path)
        else:
            print(f"✅ {file_path}")
    
    if missing_files:
        print(f"❌ 缺少文件: {missing_files}")
        return False
    else:
        print("✅ 所有模型文件都存在")
        return True

def test_driver_file():
    """测试驱动文件"""
    print("\n=== 测试驱动文件 ===")
    
    driver_path = "/Users/dp/Documents/Uni-Lab-OS/unilabos/devices/ros_dev/moveit_interface.py"
    
    if not os.path.exists(driver_path):
        print(f"❌ 驱动文件不存在: {driver_path}")
        return False
    
    try:
        # 尝试导入驱动类
        sys.path.insert(0, os.path.dirname(driver_path))
        from unilabos.devices.ros_dev.moveit_interface import MoveitInterface
        print("✅ 驱动文件存在且可导入")
        
        # 检查必要的方法
        required_methods = [
            '__init__',
            'post_init',
            'check_tf_update_actions',
            'resource_manager',
            'wait_for_resource_action',
            'moveit_joint_task',
            'moveit_task'
        ]
        
        missing_methods = []
        for method in required_methods:
            if not hasattr(MoveitInterface, method):
                missing_methods.append(method)
        
        if missing_methods:
            print(f"❌ 驱动类缺少方法: {missing_methods}")
            return False
        else:
            print("✅ 驱动类包含所有必要方法")
            return True
            
    except ImportError as e:
        print(f"❌ 驱动文件导入失败: {e}")
        return False

def test_registry_config():
    """测试注册表配置"""
    print("\n=== 测试注册表配置 ===")
    
    registry_path = "/Users/dp/Documents/Uni-Lab-OS/unilabos/registry/devices/robot_arm.yaml"
    
    if not os.path.exists(registry_path):
        print(f"❌ 注册表文件不存在: {registry_path}")
        return False
    
    try:
        with open(registry_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
        
        if 'robotic_arm.Dummy2' not in config:
            print("❌ 注册表中没有找到 robotic_arm.Dummy2 配置")
            return False
        
        dummy2_config = config['robotic_arm.Dummy2']
        
        # 检查必要的配置项
        required_keys = [
            'category',
            'class', 
            'description',
            'init_param_schema',
            'model',
            'version'
        ]
        
        missing_keys = []
        for key in required_keys:
            if key not in dummy2_config:
                missing_keys.append(key)
        
        if missing_keys:
            print(f"❌ Dummy2配置缺少字段: {missing_keys}")
            return False
        
        # 检查模块路径
        module_path = dummy2_config.get('class', {}).get('module')
        if module_path != 'unilabos.devices.ros_dev.moveit_interface:MoveitInterface':
            print(f"❌ 模块路径不正确: {module_path}")
            return False
        
        # 检查动作定义
        actions = dummy2_config.get('class', {}).get('action_value_mappings', {})
        required_actions = [
            'auto-check_tf_update_actions',
            'auto-post_init',
            'auto-resource_manager', 
            'auto-wait_for_resource_action',
            'auto-moveit_joint_task',
            'auto-moveit_task',
            'pick_and_place'
        ]
        
        missing_actions = []
        for action in required_actions:
            if action not in actions:
                missing_actions.append(action)
        
        if missing_actions:
            print(f"❌ 缺少动作定义: {missing_actions}")
            return False
        
        print("✅ 注册表配置完整且正确")
        return True
        
    except Exception as e:
        print(f"❌ 注册表配置检查失败: {e}")
        return False

def test_can2eth_connectivity():
    """测试CAN2ETH连接（可选）"""
    print("\n=== 测试CAN2ETH连接 ===")
    
    try:
        import socket
        import struct
        
        # 尝试连接CAN2ETH网关
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(2.0)
        
        can2eth_host = "192.168.8.88"
        can2eth_port = 8080
        
        # 发送ping命令
        ping_cmd = struct.pack('>B', 0xFF)
        sock.sendto(ping_cmd, (can2eth_host, can2eth_port))
        
        try:
            data, addr = sock.recvfrom(1024)
            if len(data) > 0:
                print(f"✅ CAN2ETH网关 {can2eth_host}:{can2eth_port} 连接成功")
                return True
        except socket.timeout:
            print(f"⚠️  CAN2ETH网关 {can2eth_host}:{can2eth_port} 无响应（可能未启动）")
            return False
        
    except Exception as e:
        print(f"⚠️  CAN2ETH连接测试失败: {e}")
        return False
    finally:
        if 'sock' in locals():
            sock.close()

def main():
    """主测试函数"""
    print("🤖 Dummy2 机械臂接入 UniLab 系统测试")
    print("=" * 50)
    
    tests = [
        ("设备模型文件", test_device_model_files),
        ("驱动文件", test_driver_file),
        ("注册表配置", test_registry_config),
        ("CAN2ETH连接", test_can2eth_connectivity)
    ]
    
    results = []
    
    for test_name, test_func in tests:
        try:
            result = test_func()
            results.append((test_name, result))
        except Exception as e:
            print(f"❌ {test_name}测试异常: {e}")
            results.append((test_name, False))
    
    print("\n" + "=" * 50)
    print("📊 测试结果汇总:")
    
    passed = 0
    total = len(results)
    
    for test_name, result in results:
        status = "✅ 通过" if result else "❌ 失败"
        print(f"  {test_name}: {status}")
        if result:
            passed += 1
    
    print(f"\n总体结果: {passed}/{total} 项测试通过")
    
    if passed == total:
        print("🎉 Dummy2 机械臂已成功接入 UniLab 系统！")
        print("\n📋 后续步骤:")
        print("1. 启动 CAN2ETH 服务: ros2 launch dummy2_can2eth dummy2_can2eth_server.launch.py")
        print("2. 在 UniLab 界面中添加 Dummy2 设备实例")
        print("3. 测试设备初始化和基本功能")
    else:
        print("⚠️  还有一些问题需要解决才能完全接入")
    
    return passed == total

if __name__ == "__main__":
    main()
