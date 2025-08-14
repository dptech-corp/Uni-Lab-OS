#!/usr/bin/env python3
"""
Dummy2直接运动控制
使用正确的action名称直接控制Dummy2
"""

import time
import sys
from threading import Thread

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class Dummy2DirectController:
    def __init__(self):
        self.node = None
        self.action_client = None
        self.executor = None
        self.executor_thread = None

    def initialize(self):
        """初始化ROS2环境"""
        print("🔧 初始化Dummy2直接控制器...")
        
        try:
            rclpy.init()
            
            # 创建节点
            self.node = Node("dummy2_direct_controller")
            callback_group = ReentrantCallbackGroup()
            
            # 创建action客户端
            self.action_client = ActionClient(
                self.node,
                FollowJointTrajectory,
                '/dummy2_arm_controller/follow_joint_trajectory',
                callback_group=callback_group
            )
            
            # 启动executor
            self.executor = rclpy.executors.MultiThreadedExecutor()
            self.executor.add_node(self.node)
            self.executor_thread = Thread(target=self.executor.spin, daemon=True)
            self.executor_thread.start()
            
            print("✓ 节点创建成功")
            
            # 等待action服务可用
            print("⏳ 等待action服务可用...")
            if self.action_client.wait_for_server(timeout_sec=10.0):
                print("✓ Action服务连接成功")
                return True
            else:
                print("✗ Action服务连接超时")
                return False
                
        except Exception as e:
            print(f"✗ 初始化失败: {e}")
            return False

    def move_joints(self, joint_positions, duration_sec=3.0):
        """移动关节到指定位置"""
        print(f"🎯 移动关节到位置: {joint_positions}")
        
        try:
            # 创建轨迹消息
            goal_msg = FollowJointTrajectory.Goal()
            
            # 设置关节轨迹
            trajectory = JointTrajectory()
            trajectory.joint_names = [
                'Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6'
            ]
            
            # 创建轨迹点
            point = JointTrajectoryPoint()
            point.positions = joint_positions
            point.time_from_start.sec = int(duration_sec)
            point.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
            
            trajectory.points = [point]
            goal_msg.trajectory = trajectory
            
            # 发送目标
            print("📤 发送运动目标...")
            future = self.action_client.send_goal_async(goal_msg)
            
            # 等待结果
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            
            if future.result() is not None:
                goal_handle = future.result()
                if goal_handle.accepted:
                    print("✓ 运动目标被接受")
                    
                    # 等待执行完成
                    result_future = goal_handle.get_result_async()
                    rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=duration_sec + 2.0)
                    
                    if result_future.result() is not None:
                        result = result_future.result().result
                        if result.error_code == 0:
                            print("✓ 运动执行成功")
                            return True
                        else:
                            print(f"✗ 运动执行失败，错误代码: {result.error_code}")
                            return False
                    else:
                        print("✗ 等待执行结果超时")
                        return False
                else:
                    print("✗ 运动目标被拒绝")
                    return False
            else:
                print("✗ 发送目标超时")
                return False
                
        except Exception as e:
            print(f"✗ 运动控制异常: {e}")
            return False

    def run_demo(self):
        """运行演示序列"""
        print("\n🤖 开始Dummy2运动演示...")
        print("⚠️  请确保机械臂周围安全！")
        
        # 等待用户确认
        input("\n按Enter键开始演示...")
        
        # 定义运动序列
        movements = [
            {
                "name": "Home位置",
                "positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "duration": 3.0
            },
            {
                "name": "抬起第2关节",
                "positions": [0.0, 0.5, 0.0, 0.0, 0.0, 0.0],
                "duration": 2.0
            },
            {
                "name": "弯曲第3关节",
                "positions": [0.0, 0.5, -0.5, 0.0, 0.0, 0.0],
                "duration": 2.0
            },
            {
                "name": "旋转基座",
                "positions": [1.0, 0.5, -0.5, 0.0, 0.0, 0.0],
                "duration": 3.0
            },
            {
                "name": "复合运动",
                "positions": [0.5, 0.3, -0.3, 0.5, 0.2, 0.3],
                "duration": 4.0
            },
            {
                "name": "回到Home",
                "positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "duration": 4.0
            }
        ]
        
        success_count = 0
        
        for i, movement in enumerate(movements, 1):
            print(f"\n📍 步骤 {i}: {movement['name']}")
            print(f"   目标位置: {movement['positions']}")
            print(f"   执行时间: {movement['duration']}秒")
            
            if self.move_joints(movement['positions'], movement['duration']):
                success_count += 1
                print(f"✅ 步骤 {i} 完成")
                time.sleep(1)  # 短暂停顿
            else:
                print(f"❌ 步骤 {i} 失败")
                break
                
        print(f"\n🎉 演示完成！成功执行 {success_count}/{len(movements)} 个动作")

    def cleanup(self):
        """清理资源"""
        print("\n🧹 清理资源...")
        try:
            if self.executor:
                self.executor.shutdown()
            if self.executor_thread and self.executor_thread.is_alive():
                self.executor_thread.join(timeout=2)
            rclpy.shutdown()
            print("✓ 清理完成")
        except Exception as e:
            print(f"✗ 清理异常: {e}")

def main():
    """主函数"""
    controller = Dummy2DirectController()
    
    try:
        # 初始化
        if not controller.initialize():
            print("❌ 初始化失败，退出程序")
            return
            
        # 运行演示
        controller.run_demo()
        
    except KeyboardInterrupt:
        print("\n⚠️  用户中断")
    except Exception as e:
        print(f"\n❌ 程序异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        controller.cleanup()

if __name__ == "__main__":
    main()
