#!/usr/bin/env python3
"""
Dummy2 Unilab集成调试脚本
用于测试Dummy2机械臂在Unilab系统中的控制功能
"""

import json
import time
import sys
import os

# 添加Unilab路径
sys.path.insert(0, '/home/hh/Uni-Lab-OS')

def test_device_registration():
    """测试设备注册配置"""
    print("=" * 50)
    print("测试1: 设备注册配置")
    print("=" * 50)
    
    try:
        import yaml
        with open('/home/hh/Uni-Lab-OS/unilabos/registry/devices/robot_arm.yaml', 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
        
        if 'robotic_arm.Dummy2' in config:
            print("✓ Dummy2设备已注册")
            
            # 检查关键配置
            dummy2_config = config['robotic_arm.Dummy2']
            
            # 检查模块配置
            if 'class' in dummy2_config and 'module' in dummy2_config['class']:
                module_path = dummy2_config['class']['module']
                print(f"✓ 模块路径: {module_path}")
                
                # 检查action配置
                if 'action_value_mappings' in dummy2_config['class']:
                    actions = dummy2_config['class']['action_value_mappings']
                    print(f"✓ 可用actions: {list(actions.keys())}")
                else:
                    print("✗ 未找到action配置")
            else:
                print("✗ 未找到模块配置")
        else:
            print("✗ Dummy2设备未注册")
            
    except Exception as e:
        print(f"✗ 配置文件读取错误: {e}")

def test_device_mesh_config():
    """测试设备网格配置"""
    print("\n" + "=" * 50)
    print("测试2: 设备网格配置")
    print("=" * 50)
    
    try:
        # 检查move_group.json
        config_path = '/home/hh/Uni-Lab-OS/unilabos/device_mesh/devices/dummy2_robot/config/move_group.json'
        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                move_group_config = json.load(f)
            print("✓ move_group.json配置存在")
            print(f"  关节组: {list(move_group_config.keys())}")
            
            for group, config in move_group_config.items():
                print(f"  {group}组配置:")
                print(f"    关节名称: {config.get('joint_names', [])}")
                print(f"    基础连接: {config.get('base_link_name', 'N/A')}")
                print(f"    末端执行器: {config.get('end_effector_name', 'N/A')}")
        else:
            print("✗ move_group.json配置文件不存在")
            
        # 检查XACRO文件
        xacro_path = '/home/hh/Uni-Lab-OS/unilabos/device_mesh/devices/dummy2_robot/meshes/dummy2.xacro'
        if os.path.exists(xacro_path):
            print("✓ dummy2.xacro模型文件存在")
        else:
            print("✗ dummy2.xacro模型文件不存在")
            
    except Exception as e:
        print(f"✗ 设备网格配置检查错误: {e}")

def test_moveit_interface_import():
    """测试MoveitInterface模块导入"""
    print("\n" + "=" * 50)
    print("测试3: MoveitInterface模块导入")
    print("=" * 50)
    
    try:
        from unilabos.devices.ros_dev.moveit_interface import MoveitInterface
        print("✓ MoveitInterface模块导入成功")
        
        # 检查必要的方法
        methods = ['post_init', 'moveit_task', 'moveit_joint_task']
        for method in methods:
            if hasattr(MoveitInterface, method):
                print(f"✓ 方法 {method} 存在")
            else:
                print(f"✗ 方法 {method} 不存在")
                
    except ImportError as e:
        print(f"✗ MoveitInterface模块导入失败: {e}")
    except Exception as e:
        print(f"✗ 模块检查错误: {e}")

def test_ros2_dependencies():
    """测试ROS2依赖"""
    print("\n" + "=" * 50)
    print("测试4: ROS2依赖检查")
    print("=" * 50)
    
    try:
        import rclpy
        print("✓ rclpy导入成功")
        
        from moveit_msgs.msg import JointConstraint, Constraints
        print("✓ moveit_msgs导入成功")
        
        from unilabos_msgs.action import SendCmd
        print("✓ unilabos_msgs导入成功")
        
        from tf2_ros import Buffer, TransformListener
        print("✓ tf2_ros导入成功")
        
    except ImportError as e:
        print(f"✗ ROS2依赖导入失败: {e}")

def test_dummy2_configuration():
    """测试Dummy2配置参数"""
    print("\n" + "=" * 50)
    print("测试5: Dummy2配置参数验证")
    print("=" * 50)
    
    try:
        # 模拟MoveitInterface初始化参数
        test_params = {
            'moveit_type': 'dummy2_robot',
            'joint_poses': '[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]',
            'device_config': None
        }
        
        print("测试参数:")
        for key, value in test_params.items():
            print(f"  {key}: {value}")
            
        # 检查config文件是否可以被正确加载
        config_path = f"/home/hh/Uni-Lab-OS/unilabos/device_mesh/devices/{test_params['moveit_type']}/config/move_group.json"
        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                config_data = json.load(f)
            print(f"✓ 配置文件可正常加载: {list(config_data.keys())}")
        else:
            print(f"✗ 配置文件不存在: {config_path}")
            
    except Exception as e:
        print(f"✗ 配置参数验证错误: {e}")

def test_create_dummy2_instance():
    """测试创建Dummy2实例"""
    print("\n" + "=" * 50)
    print("测试6: 创建Dummy2实例")
    print("=" * 50)
    
    try:
        from unilabos.devices.ros_dev.moveit_interface import MoveitInterface
        
        # 创建MoveitInterface实例
        dummy2_interface = MoveitInterface(
            moveit_type='dummy2_robot',
            joint_poses='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]',
            device_config=None
        )
        
        print("✓ Dummy2 MoveitInterface实例创建成功")
        print(f"  数据配置: {dummy2_interface.data_config}")
        print(f"  关节位置: {dummy2_interface.joint_poses}")
        
    except Exception as e:
        print(f"✗ Dummy2实例创建失败: {e}")

def check_ros2_environment():
    """检查ROS2环境"""
    print("\n" + "=" * 50)
    print("测试7: ROS2环境检查")
    print("=" * 50)
    
    ros_distro = os.environ.get('ROS_DISTRO')
    if ros_distro:
        print(f"✓ ROS2版本: {ros_distro}")
    else:
        print("✗ ROS_DISTRO环境变量未设置")
        
    ament_prefix_path = os.environ.get('AMENT_PREFIX_PATH')
    if ament_prefix_path:
        print("✓ AMENT_PREFIX_PATH已设置")
    else:
        print("✗ AMENT_PREFIX_PATH环境变量未设置")

def main():
    """主测试函数"""
    print("Dummy2 Unilab集成调试测试")
    print("=" * 60)
    
    # 运行所有测试
    test_device_registration()
    test_device_mesh_config()
    test_moveit_interface_import()
    test_ros2_dependencies()
    test_dummy2_configuration()
    test_create_dummy2_instance()
    check_ros2_environment()
    
    print("\n" + "=" * 60)
    print("调试测试完成")
    print("=" * 60)

if __name__ == "__main__":
    main()
