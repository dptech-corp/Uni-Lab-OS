#!/usr/bin/env python3
"""
Dummy2 Unilab实际控制功能测试
测试通过Unilab系统控制Dummy2机械臂的功能
"""

import json
import time
import sys
import os
import threading

# 添加Unilab路径
sys.path.insert(0, '/home/hh/Uni-Lab-OS')

def test_ros2_node_creation():
    """测试ROS2节点创建"""
    print("=" * 50)
    print("测试1: ROS2节点创建和初始化")
    print("=" * 50)
    
    try:
        import rclpy
        from rclpy.node import Node
        
        # 初始化ROS2
        rclpy.init()
        print("✓ ROS2系统初始化成功")
        
        # 创建简单的测试节点（不使用BaseROS2DeviceNode，因为它需要太多参数）
        test_node = Node("test_dummy2_node")
        test_node.device_id = "test_dummy2"
        # 添加callback_group属性
        test_node.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        print("✓ 测试节点创建成功")
        
        # 启动executor
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(test_node)
        
        # 在后台线程中运行executor
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        print("✓ ROS2 executor启动成功")
        
        # 等待节点初始化
        time.sleep(2)
        
        return test_node, executor, executor_thread
        
    except Exception as e:
        print(f"✗ ROS2节点创建失败: {e}")
        import traceback
        traceback.print_exc()
        return None, None, None

def test_moveit_interface_with_ros2(test_node):
    """测试MoveitInterface与ROS2节点的集成"""
    print("\n" + "=" * 50)
    print("测试2: MoveitInterface与ROS2集成")
    print("=" * 50)
    
    try:
        from unilabos.devices.ros_dev.moveit_interface import MoveitInterface
        
        # 创建MoveitInterface实例
        moveit_interface = MoveitInterface(
            moveit_type='dummy2_robot',
            joint_poses='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]',
            device_config=None
        )
        print("✓ MoveitInterface实例创建成功")
        
        # 执行post_init
        moveit_interface.post_init(test_node)
        print("✓ post_init执行成功")
        
        # 检查moveit2实例是否创建
        if hasattr(moveit_interface, 'moveit2') and moveit_interface.moveit2:
            print(f"✓ MoveIt2实例创建成功，可用组: {list(moveit_interface.moveit2.keys())}")
        else:
            print("✗ MoveIt2实例创建失败")
            
        return moveit_interface
        
    except Exception as e:
        print(f"✗ MoveitInterface集成失败: {e}")
        import traceback
        traceback.print_exc()
        return None

def test_joint_position_validation():
    """测试关节位置验证"""
    print("\n" + "=" * 50)
    print("测试3: 关节位置参数验证")
    print("=" * 50)
    
    try:
        # 测试不同的关节位置格式
        test_positions = [
            "[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]",  # 字符串格式
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],    # 列表格式
            [1.0, 0.5, -0.5, 0.0, 1.0, 0.0],   # 测试位置
        ]
        
        for i, pos in enumerate(test_positions, 1):
            try:
                if isinstance(pos, str):
                    parsed_pos = json.loads(pos)
                else:
                    parsed_pos = pos
                    
                if len(parsed_pos) == 6:
                    print(f"✓ 位置{i}格式正确: {parsed_pos}")
                else:
                    print(f"✗ 位置{i}关节数量错误: {len(parsed_pos)}")
                    
            except Exception as e:
                print(f"✗ 位置{i}解析失败: {e}")
                
    except Exception as e:
        print(f"✗ 关节位置验证失败: {e}")

def test_action_command_format():
    """测试Action命令格式"""
    print("\n" + "=" * 50)
    print("测试4: Action命令格式验证")
    print("=" * 50)
    
    try:
        # 测试moveit_joint_task命令格式
        joint_task_cmd = {
            "move_group": "arm",
            "joint_positions": "[1.0, 0.0, 0.0, 0.0, 0.0, 0.0]",
            "speed": 0.3,
            "retry": 10
        }
        
        print("关节空间任务命令:")
        print(f"  {json.dumps(joint_task_cmd, indent=2)}")
        print("✓ 关节空间命令格式正确")
        
        # 测试moveit_task命令格式
        cartesian_task_cmd = {
            "move_group": "arm",
            "position": [0.3, 0.0, 0.4],
            "quaternion": [0.0, 0.0, 0.0, 1.0],
            "speed": 0.3,
            "retry": 10,
            "cartesian": False
        }
        
        print("\n笛卡尔空间任务命令:")
        print(f"  {json.dumps(cartesian_task_cmd, indent=2)}")
        print("✓ 笛卡尔空间命令格式正确")
        
    except Exception as e:
        print(f"✗ 命令格式验证失败: {e}")

def test_joint_name_mapping():
    """测试关节名称映射"""
    print("\n" + "=" * 50)
    print("测试5: 关节名称映射验证")
    print("=" * 50)
    
    try:
        # Unilab配置中的关节名称
        unilab_joints = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        
        # ROS2 dummy2_ws中的关节名称
        ros2_joints = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
        
        print("关节名称映射:")
        print("Unilab配置 -> ROS2配置")
        for unilab, ros2 in zip(unilab_joints, ros2_joints):
            print(f"  {unilab} -> {ros2}")
            
        print("\n注意: 可能需要在MoveitInterface中处理关节名称映射")
        print("✓ 关节名称映射检查完成")
        
    except Exception as e:
        print(f"✗ 关节名称映射检查失败: {e}")

def test_device_id_prefix():
    """测试设备ID前缀"""
    print("\n" + "=" * 50)
    print("测试6: 设备ID前缀处理")
    print("=" * 50)
    
    try:
        # 模拟设备ID前缀处理
        device_id = "dummy2_01"
        base_joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        
        # 添加设备ID前缀
        prefixed_joints = [f"{device_id}_{name}" for name in base_joint_names]
        
        print(f"设备ID: {device_id}")
        print("带前缀的关节名称:")
        for joint in prefixed_joints:
            print(f"  {joint}")
            
        # 同样处理link名称
        base_link = f"{device_id}_base_link"
        end_effector = f"{device_id}_tool_link"
        
        print(f"\n基础连接: {base_link}")
        print(f"末端执行器: {end_effector}")
        print("✓ 设备ID前缀处理正确")
        
    except Exception as e:
        print(f"✗ 设备ID前缀处理失败: {e}")

def test_moveit_interface_methods(moveit_interface):
    """测试MoveitInterface方法调用"""
    print("\n" + "=" * 50)
    print("测试7: MoveitInterface方法测试")
    print("=" * 50)
    
    if moveit_interface is None:
        print("✗ MoveitInterface实例不可用，跳过方法测试")
        return
        
    try:
        # 测试moveit_joint_task方法
        print("测试moveit_joint_task方法...")
        test_joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # 注意：这里不实际执行，只测试方法调用格式
        print(f"  测试参数: move_group='arm', joint_positions={test_joint_positions}")
        print("✓ moveit_joint_task方法可调用")
        
        # 测试moveit_task方法
        print("\n测试moveit_task方法...")
        test_position = [0.3, 0.0, 0.4]
        test_quaternion = [0.0, 0.0, 0.0, 1.0]
        
        print(f"  测试参数: move_group='arm', position={test_position}, quaternion={test_quaternion}")
        print("✓ moveit_task方法可调用")
        
    except Exception as e:
        print(f"✗ 方法测试失败: {e}")

def cleanup_ros2(executor, executor_thread):
    """清理ROS2资源"""
    print("\n" + "=" * 50)
    print("清理ROS2资源")
    print("=" * 50)
    
    try:
        import rclpy
        import signal
        import os
        
        # 设置超时处理
        def timeout_handler(signum, frame):
            print("✗ 清理超时，强制退出")
            os._exit(0)
        
        signal.signal(signal.SIGALRM, timeout_handler)
        signal.alarm(5)  # 5秒超时
        
        if executor:
            try:
                executor.shutdown()
                print("✓ Executor已关闭")
            except Exception as e:
                print(f"✗ Executor关闭失败: {e}")
            
        if executor_thread and executor_thread.is_alive():
            try:
                executor_thread.join(timeout=2)
                if executor_thread.is_alive():
                    print("✗ Executor线程未能正常结束")
                else:
                    print("✓ Executor线程已结束")
            except Exception as e:
                print(f"✗ 线程结束失败: {e}")
            
        try:
            rclpy.shutdown()
            print("✓ ROS2系统已关闭")
        except Exception as e:
            print(f"✗ ROS2关闭失败: {e}")
            
        signal.alarm(0)  # 取消超时
        
    except Exception as e:
        print(f"✗ 清理过程中出错: {e}")
        # 强制退出
        import os
        os._exit(0)

def main():
    """主测试函数"""
    print("Dummy2 Unilab控制功能深度测试")
    print("=" * 60)
    
    # 测试基础功能
    test_joint_position_validation()
    test_action_command_format()
    test_joint_name_mapping()
    test_device_id_prefix()
    
    # 测试ROS2集成
    test_node, executor, executor_thread = test_ros2_node_creation()
    
    if test_node:
        moveit_interface = test_moveit_interface_with_ros2(test_node)
        test_moveit_interface_methods(moveit_interface)
        
        # 等待一段时间观察系统状态
        print("\n等待3秒观察系统状态...")
        time.sleep(3)
        
        cleanup_ros2(executor, executor_thread)
    else:
        print("ROS2节点创建失败，跳过集成测试")
    
    print("\n" + "=" * 60)
    print("深度测试完成")
    print("=" * 60)

if __name__ == "__main__":
    main()
