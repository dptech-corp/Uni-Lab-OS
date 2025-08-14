#!/usr/bin/env python3
"""
Dummy2 Unilab实际控制测试
需要先启动ROS2服务，然后测试通过Unilab控制Dummy2
"""

import json
import time
import sys
import os
import threading

# 添加Unilab路径
sys.path.insert(0, '/home/hh/Uni-Lab-OS')

def check_ros2_services():
    """检查ROS2服务状态"""
    print("=" * 50)
    print("检查ROS2服务状态")
    print("=" * 50)
    
    try:
        import subprocess
        import rclpy
        
        # 初始化ROS2
        rclpy.init()
        
        # 检查话题
        result = subprocess.run(['ros2', 'topic', 'list'], 
                               capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            topics = result.stdout.strip().split('\n')
            print(f"✓ 发现 {len(topics)} 个ROS2话题")
            
            # 查找dummy2相关话题
            dummy2_topics = [t for t in topics if 'dummy2' in t.lower()]
            if dummy2_topics:
                print("Dummy2相关话题:")
                for topic in dummy2_topics[:5]:  # 只显示前5个
                    print(f"  {topic}")
            else:
                print("✗ 未发现Dummy2相关话题")
        else:
            print("✗ 无法获取ROS2话题列表")
            
        # 检查服务
        result = subprocess.run(['ros2', 'service', 'list'], 
                               capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            services = result.stdout.strip().split('\n')
            print(f"✓ 发现 {len(services)} 个ROS2服务")
            
            # 查找MoveIt相关服务
            moveit_services = [s for s in services if 'moveit' in s.lower()]
            if moveit_services:
                print("MoveIt相关服务:")
                for service in moveit_services[:5]:  # 只显示前5个
                    print(f"  {service}")
            else:
                print("✗ 未发现MoveIt相关服务")
        else:
            print("✗ 无法获取ROS2服务列表")
            
        rclpy.shutdown()
        return True
        
    except subprocess.TimeoutExpired:
        print("✗ ROS2命令超时")
        return False
    except Exception as e:
        print(f"✗ 检查ROS2服务失败: {e}")
        return False

def test_actual_moveit_control():
    """测试实际的MoveIt控制"""
    print("\n" + "=" * 50)
    print("测试实际MoveIt控制")
    print("=" * 50)
    
    try:
        import rclpy
        from rclpy.node import Node
        from unilabos.devices.ros_dev.moveit_interface import MoveitInterface
        
        # 初始化ROS2
        rclpy.init()
        
        # 创建节点
        test_node = Node("dummy2_test_node")
        test_node.device_id = "dummy2_test"
        test_node.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        
        # 启动executor
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(test_node)
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        
        print("✓ 测试节点创建成功")
        
        # 创建MoveitInterface
        moveit_interface = MoveitInterface(
            moveit_type='dummy2_robot',
            joint_poses='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]',
            device_config=None
        )
        
        # 执行post_init
        moveit_interface.post_init(test_node)
        print("✓ MoveitInterface初始化完成")
        
        # 等待服务可用
        print("等待MoveIt服务...")
        time.sleep(3)
        
        # 测试关节运动（安全位置）
        print("测试关节运动到安全位置...")
        safe_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        try:
            result = moveit_interface.moveit_joint_task(
                move_group='arm',
                joint_positions=safe_positions,
                speed=0.1,  # 慢速运动
                retry=3
            )
            
            if result:
                print("✓ 关节运动成功执行")
            else:
                print("✗ 关节运动执行失败")
                
        except Exception as e:
            print(f"✗ 关节运动测试失败: {e}")
        
        # 清理
        executor.shutdown()
        executor_thread.join(timeout=2)
        rclpy.shutdown()
        
        return True
        
    except Exception as e:
        print(f"✗ 实际控制测试失败: {e}")
        import traceback
        traceback.print_exc()
        return False

def print_startup_instructions():
    """打印启动说明"""
    print("\n" + "=" * 60)
    print("Dummy2 ROS2服务启动说明")
    print("=" * 60)
    
    print("在测试Unilab控制之前，需要先启动ROS2服务：")
    print("\n1. 打开新终端，导航到dummy2_ws:")
    print("   cd /home/hh/dummy2/ros2/dummy2_ws")
    
    print("\n2. 设置ROS2环境:")
    print("   source /opt/ros/humble/setup.bash")
    print("   source install/setup.bash")
    
    print("\n3. 启动dummy2硬件接口:")
    print("   ros2 launch dummy2_hw dummy2_hw.launch.py")
    
    print("\n4. 在另一个终端启动MoveIt2:")
    print("   cd /home/hh/dummy2/ros2/dummy2_ws")
    print("   source /opt/ros/humble/setup.bash")
    print("   source install/setup.bash")
    print("   ros2 launch dummy2_moveit_config demo.launch.py")
    
    print("\n5. 然后回到这里运行实际控制测试:")
    print("   python test_dummy2_real_control.py --test-control")
    
    print("\n注意事项:")
    print("- 确保dummy2硬件连接正常")
    print("- 检查CAN2ETH网络连接")
    print("- 确保机械臂处于安全位置")

def main():
    """主函数"""
    if '--test-control' in sys.argv:
        # 实际控制测试模式
        print("Dummy2实际控制测试")
        print("=" * 60)
        
        if check_ros2_services():
            test_actual_moveit_control()
        else:
            print("\n请先启动ROS2服务后再测试")
            print_startup_instructions()
    else:
        # 检查模式
        print("Dummy2 ROS2服务检查")
        print("=" * 60)
        
        if check_ros2_services():
            print("\n✓ ROS2服务运行正常，可以进行实际控制测试")
            print("运行以下命令进行实际控制测试:")
            print("python test_dummy2_real_control.py --test-control")
        else:
            print("\n需要先启动ROS2服务")
            print_startup_instructions()

if __name__ == "__main__":
    main()
