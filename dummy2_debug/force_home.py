#!/usr/bin/env python3
"""
强制回到Home位置
确保机械臂真正回到零位
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time

class ForceHome(Node):
    def __init__(self):
        super().__init__('force_home')
        self.client = ActionClient(self, FollowJointTrajectory, '/dummy2_arm_controller/follow_joint_trajectory')
        
        print("🏠 强制回到Home位置...")
        if not self.client.wait_for_server(timeout_sec=5.0):
            print("❌ 控制器服务不可用")
            return
        print("✅ 控制器已连接")
        
    def go_home(self, duration=8):
        """强制回到home位置"""
        print(f"🎯 执行回home运动 (时长: {duration}秒)...")
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = JointTrajectory()
        goal_msg.trajectory.header.frame_id = ""
        goal_msg.trajectory.joint_names = ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"]
        
        # 明确的零位点
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 明确的home位置
        point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 停止速度
        point.time_from_start.sec = duration
        point.time_from_start.nanosec = 0
        
        goal_msg.trajectory.points = [point]
        
        print("📤 发送回home指令...")
        future = self.client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("❌ 回home指令被拒绝")
            return False
            
        print("✅ 回home指令已接受，机械臂运动中...")
        print(f"⏱️  等待 {duration} 秒执行完成...")
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration+2)
        
        result = result_future.result().result
        print(f"📊 执行结果错误码: {result.error_code}")
        
        if result.error_code == 0:
            print("🎉 成功回到Home位置!")
            return True
        else:
            print(f"⚠️  执行警告，错误码: {result.error_code}")
            return False

def main():
    print("🏠 Dummy2强制回Home程序")
    print("=" * 40)
    
    rclpy.init()
    
    try:
        home_node = ForceHome()
        
        print("⚠️  即将执行回home运动，请确保机械臂周围安全")
        print("⚠️  可随时按 Ctrl+C 紧急停止")
        print()
        
        input("按Enter键开始回home...")
        
        success = home_node.go_home(duration=10)  # 给更长时间确保到位
        
        if success:
            print("\n🎉 机械臂应该已回到Home位置")
            print("💡 请检查机械臂是否在零位")
        else:
            print("\n⚠️  回home过程有警告，请检查机械臂状态")
            
        # 等待一下再检查状态
        print("\n等待2秒后检查关节状态...")
        time.sleep(2)
        
    except KeyboardInterrupt:
        print("\n⚠️  用户中断操作")
    except Exception as e:
        print(f"\n❌ 异常: {e}")
    finally:
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()
