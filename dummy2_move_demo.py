#!/usr/bin/env python3
"""
Dummy2实际运动控制测试
让Dummy2机械臂实际动起来！
"""

import json
import time
import sys
import os
import threading
import signal

# 添加Unilab路径
sys.path.insert(0, '/home/hh/Uni-Lab-OS')

class Dummy2Controller:
    def __init__(self):
        self.moveit_interface = None
        self.test_node = None
        self.executor = None
        self.executor_thread = None
        self.running = False

    def initialize_ros2(self):
        """初始化ROS2环境"""
        print("初始化ROS2环境...")
        
        try:
            import rclpy
            from rclpy.node import Node
            from unilabos.devices.ros_dev.moveit_interface import MoveitInterface
            
            # 初始化ROS2
            rclpy.init()
            
            # 创建节点
            self.test_node = Node("dummy2_controller")
            self.test_node.device_id = "dummy2_ctrl"
            self.test_node.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
            
            # 启动executor
            self.executor = rclpy.executors.MultiThreadedExecutor()
            self.executor.add_node(self.test_node)
            self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
            self.executor_thread.start()
            
            print("✓ ROS2节点创建成功")
            
            # 创建MoveitInterface
            self.moveit_interface = MoveitInterface(
                moveit_type='dummy2_robot',
                joint_poses='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]',
                device_config=None
            )
            
            # 执行post_init
            self.moveit_interface.post_init(self.test_node)
            print("✓ MoveitInterface初始化完成")
            
            # 等待服务可用
            print("等待MoveIt服务可用...")
            time.sleep(3)
            
            self.running = True
            return True
            
        except Exception as e:
            print(f"✗ ROS2初始化失败: {e}")
            return False

    def move_to_home_position(self):
        """移动到Home位置"""
        print("\n🏠 移动到Home位置...")
        
        # Home位置：所有关节归零
        home_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        try:
            result = self.moveit_interface.moveit_joint_task(
                move_group='arm',
                joint_positions=home_positions,
                speed=0.2,  # 慢速运动
                retry=5
            )
            
            if result:
                print("✓ 成功移动到Home位置")
                return True
            else:
                print("✗ 移动到Home位置失败")
                return False
                
        except Exception as e:
            print(f"✗ Home位置移动异常: {e}")
            return False

    def move_to_test_positions(self):
        """移动到几个测试位置"""
        print("\n🔄 执行测试运动序列...")
        
        # 定义几个安全的测试位置（单位：弧度）
        test_positions = [
            {
                "name": "位置1 - 轻微弯曲",
                "joints": [0.0, 0.5, -0.5, 0.0, 0.0, 0.0],
                "speed": 0.15
            },
            {
                "name": "位置2 - 侧向运动", 
                "joints": [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "speed": 0.15
            },
            {
                "name": "位置3 - 复合运动",
                "joints": [0.5, 0.3, -0.3, 0.5, 0.0, 0.3],
                "speed": 0.1
            },
            {
                "name": "位置4 - 回到Home",
                "joints": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "speed": 0.2
            }
        ]
        
        success_count = 0
        
        for i, position in enumerate(test_positions, 1):
            print(f"\n📍 执行 {position['name']}...")
            print(f"   关节角度: {position['joints']}")
            
            try:
                result = self.moveit_interface.moveit_joint_task(
                    move_group='arm',
                    joint_positions=position['joints'],
                    speed=position['speed'],
                    retry=3
                )
                
                if result:
                    print(f"✓ {position['name']} 执行成功")
                    success_count += 1
                    time.sleep(2)  # 等待运动完成
                else:
                    print(f"✗ {position['name']} 执行失败")
                    
            except Exception as e:
                print(f"✗ {position['name']} 执行异常: {e}")
                
            # 检查是否需要停止
            if not self.running:
                break
                
        print(f"\n📊 运动序列完成: {success_count}/{len(test_positions)} 个位置成功")
        return success_count > 0

    def test_cartesian_movement(self):
        """测试笛卡尔空间运动"""
        print("\n📐 测试笛卡尔空间运动...")
        
        # 定义一些安全的笛卡尔位置
        cartesian_positions = [
            {
                "name": "前方位置",
                "position": [0.4, 0.0, 0.3],
                "quaternion": [0.0, 0.0, 0.0, 1.0]
            },
            {
                "name": "右侧位置", 
                "position": [0.3, -0.2, 0.3],
                "quaternion": [0.0, 0.0, 0.0, 1.0]
            },
            {
                "name": "左侧位置",
                "position": [0.3, 0.2, 0.3], 
                "quaternion": [0.0, 0.0, 0.0, 1.0]
            }
        ]
        
        success_count = 0
        
        for position in cartesian_positions:
            print(f"\n📍 移动到 {position['name']}...")
            print(f"   位置: {position['position']}")
            print(f"   姿态: {position['quaternion']}")
            
            try:
                result = self.moveit_interface.moveit_task(
                    move_group='arm',
                    position=position['position'],
                    quaternion=position['quaternion'],
                    speed=0.1,
                    retry=3,
                    cartesian=False
                )
                
                if result:
                    print(f"✓ {position['name']} 到达成功")
                    success_count += 1
                    time.sleep(3)  # 等待运动完成
                else:
                    print(f"✗ {position['name']} 到达失败")
                    
            except Exception as e:
                print(f"✗ {position['name']} 执行异常: {e}")
                
            if not self.running:
                break
                
        print(f"\n📊 笛卡尔运动完成: {success_count}/{len(cartesian_positions)} 个位置成功")
        return success_count > 0

    def cleanup(self):
        """清理资源"""
        print("\n🧹 清理资源...")
        self.running = False
        
        try:
            if self.executor:
                self.executor.shutdown()
            if self.executor_thread and self.executor_thread.is_alive():
                self.executor_thread.join(timeout=2)
            
            import rclpy
            rclpy.shutdown()
            print("✓ 资源清理完成")
            
        except Exception as e:
            print(f"✗ 清理过程异常: {e}")

def signal_handler(signum, frame):
    """信号处理器"""
    print("\n\n⚠️  收到停止信号，正在安全停止...")
    global controller
    if controller:
        controller.cleanup()
    sys.exit(0)

def main():
    """主函数"""
    global controller
    
    # 设置信号处理
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    print("🤖 Dummy2机械臂运动控制测试")
    print("=" * 50)
    
    controller = Dummy2Controller()
    
    try:
        # 初始化ROS2
        if not controller.initialize_ros2():
            print("❌ 初始化失败，退出程序")
            return
            
        print("\n🚀 开始运动控制测试...")
        print("⚠️  请确保机械臂周围安全，按Ctrl+C可随时停止")
        
        # 等待用户确认
        input("\n按Enter键开始运动测试...")
        
        # 1. 移动到Home位置
        if not controller.move_to_home_position():
            print("❌ Home位置移动失败，停止测试")
            return
            
        # 2. 执行关节空间运动
        print("\n" + "="*30)
        print("开始关节空间运动测试")
        print("="*30)
        controller.move_to_test_positions()
        
        # 3. 执行笛卡尔空间运动
        if controller.running:
            print("\n" + "="*30)  
            print("开始笛卡尔空间运动测试")
            print("="*30)
            controller.test_cartesian_movement()
        
        print("\n🎉 运动控制测试完成！")
        print("Dummy2已成功通过Unilab系统进行控制！")
        
    except KeyboardInterrupt:
        print("\n⚠️  用户中断程序")
    except Exception as e:
        print(f"\n❌ 程序异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        controller.cleanup()

if __name__ == "__main__":
    controller = None
    main()
