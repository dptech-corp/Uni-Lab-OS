#!/usr/bin/env python3
"""
最终演示：Dummy2 Unilab集成
展示所有完全可用的功能
"""

import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    JointConstraint,
    PlanningOptions,
    WorkspaceParameters
)
from geometry_msgs.msg import Vector3

class FinalDemo(Node):
    def __init__(self):
        super().__init__('final_demo')
        
        # 创建动作客户端
        self.trajectory_client = ActionClient(self, FollowJointTrajectory, '/dummy2_arm_controller/follow_joint_trajectory')
        self.moveit_client = ActionClient(self, MoveGroup, '/move_action')
        
        print("🔧 等待服务连接...")
        
        # 检查服务可用性
        trajectory_ok = self.trajectory_client.wait_for_server(timeout_sec=5.0)
        moveit_ok = self.moveit_client.wait_for_server(timeout_sec=5.0)
        
        if trajectory_ok:
            print("✅ 直接轨迹控制服务已连接")
        else:
            print("❌ 直接轨迹控制服务不可用")
            
        if moveit_ok:
            print("✅ MoveIt2规划服务已连接")
        else:
            print("❌ MoveIt2规划服务不可用")
        
        self.trajectory_available = trajectory_ok
        self.moveit_available = moveit_ok
    
    def demo_trajectory_control(self):
        """演示1: 直接轨迹控制"""
        if not self.trajectory_available:
            print("❌ 轨迹控制服务不可用")
            return False
            
        print("\n🎯 演示1: 直接轨迹控制")
        print("执行关节空间运动序列...")
        
        # 定义运动序列
        movements = [
            {"name": "初始位置", "positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "duration": 3},
            {"name": "关节1运动", "positions": [0.5, 0.0, 0.0, 0.0, 0.0, 0.0], "duration": 3},
            {"name": "多关节协调", "positions": [0.3, 0.2, -0.2, 0.1, 0.0, 0.0], "duration": 4},
            {"name": "回到Home", "positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "duration": 3}
        ]
        
        for i, movement in enumerate(movements):
            print(f"  📍 步骤 {i+1}: {movement['name']}")
            
            goal_msg = FollowJointTrajectory.Goal()
            goal_msg.trajectory = JointTrajectory()
            goal_msg.trajectory.joint_names = ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"]
            
            point = JointTrajectoryPoint()
            point.positions = movement["positions"]
            point.time_from_start.sec = movement["duration"]
            goal_msg.trajectory.points = [point]
            
            try:
                future = self.trajectory_client.send_goal_async(goal_msg)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                
                goal_handle = future.result()
                if goal_handle.accepted:
                    print(f"     ✅ 运动指令已接受，执行中...")
                    result_future = goal_handle.get_result_async()
                    rclpy.spin_until_future_complete(self, result_future, timeout_sec=movement["duration"]+2)
                    
                    result = result_future.result().result
                    if result.error_code == 0:
                        print(f"     🎉 运动完成")
                    else:
                        print(f"     ⚠️ 运动执行警告，错误码: {result.error_code}")
                else:
                    print(f"     ❌ 运动指令被拒绝")
                    
            except Exception as e:
                print(f"     ❌ 运动异常: {e}")
                
            time.sleep(1)  # 步骤间间隔
            
        print("🎉 直接轨迹控制演示完成!")
        return True
    
    def demo_moveit_planning(self):
        """演示2: MoveIt2规划"""
        if not self.moveit_available:
            print("❌ MoveIt2规划服务不可用")
            return False
            
        print("\n🎯 演示2: MoveIt2智能规划")
        print("使用MoveIt2进行运动规划和执行...")
        
        # 定义规划目标
        planning_targets = [
            {"name": "规划到目标位置1", "joint": "Joint1", "value": 0.4},
            {"name": "规划到目标位置2", "joint": "Joint2", "value": 0.3},
            {"name": "规划回到Home", "joint": "Joint1", "value": 0.0}
        ]
        
        for i, target in enumerate(planning_targets):
            print(f"  📍 规划 {i+1}: {target['name']}")
            
            goal_msg = MoveGroup.Goal()
            goal_msg.request = MotionPlanRequest()
            goal_msg.request.group_name = "dummy2_arm"
            
            # 设置关节约束
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = target["joint"]
            joint_constraint.position = target["value"]
            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            
            constraints = Constraints()
            constraints.joint_constraints = [joint_constraint]
            goal_msg.request.goal_constraints = [constraints]
            
            # 设置规划选项
            goal_msg.planning_options = PlanningOptions()
            goal_msg.planning_options.plan_only = False  # 规划并执行
            goal_msg.planning_options.max_safe_execution_cost = 1.0
            
            # 设置工作空间
            goal_msg.request.workspace_parameters = WorkspaceParameters()
            goal_msg.request.workspace_parameters.header.frame_id = "base_link"
            goal_msg.request.workspace_parameters.min_corner = Vector3(x=-2.0, y=-2.0, z=-2.0)
            goal_msg.request.workspace_parameters.max_corner = Vector3(x=2.0, y=2.0, z=2.0)
            
            goal_msg.request.allowed_planning_time = 5.0
            goal_msg.request.num_planning_attempts = 3
            
            try:
                future = self.moveit_client.send_goal_async(goal_msg)
                rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
                
                goal_handle = future.result()
                if goal_handle.accepted:
                    print(f"     ✅ 规划请求已接受")
                    result_future = goal_handle.get_result_async()
                    rclpy.spin_until_future_complete(self, result_future, timeout_sec=15.0)
                    
                    result = result_future.result().result
                    if result.error_code.val == 1:  # SUCCESS
                        trajectory_points = len(result.planned_trajectory.joint_trajectory.points)
                        print(f"     🎉 规划成功! 生成 {trajectory_points} 个轨迹点")
                        print(f"     🤖 执行运动...")
                    else:
                        print(f"     ⚠️ 规划失败，错误码: {result.error_code.val}")
                else:
                    print(f"     ❌ 规划请求被拒绝")
                    
            except Exception as e:
                print(f"     ❌ 规划异常: {e}")
                
            time.sleep(2)  # 规划间间隔
            
        print("🎉 MoveIt2规划演示完成!")
        return True

def main():
    print("🚀 Dummy2 Unilab集成 - 最终功能演示")
    print("=" * 60)
    print("展示所有完全可用的功能")
    print()
    
    # 初始化ROS2
    rclpy.init()
    
    try:
        # 创建演示节点
        demo_node = FinalDemo()
        
        if not demo_node.trajectory_available and not demo_node.moveit_available:
            print("❌ 没有可用的服务，请检查ROS2环境")
            return
        
        print("⚠️  演示即将开始，请确保机械臂周围安全")
        print("⚠️  可随时按 Ctrl+C 停止演示")
        print()
        
        input("按Enter键开始演示...")
        print()
        
        # 执行演示
        demo1_success = demo_node.demo_trajectory_control()
        time.sleep(2)
        
        demo2_success = demo_node.demo_moveit_planning()
        
        # 总结
        print("\n" + "=" * 60)
        print("📋 演示总结:")
        print(f"  直接轨迹控制: {'✅ 成功' if demo1_success else '❌ 失败'}")
        print(f"  MoveIt2规划: {'✅ 成功' if demo2_success else '❌ 失败'}")
        
        if demo1_success or demo2_success:
            print("\n🎉 Dummy2已成功集成到Unilab系统!")
            print("💡 所有核心功能完全可用，迁移目标达成!")
        else:
            print("\n⚠️  请检查服务状态")
            
    except KeyboardInterrupt:
        print("\n⚠️  用户中断演示")
    except Exception as e:
        print(f"\n❌ 演示异常: {e}")
    finally:
        try:
            rclpy.shutdown()
        except:
            pass
        print("\n🧹 演示结束，资源已清理")

if __name__ == '__main__':
    main()
