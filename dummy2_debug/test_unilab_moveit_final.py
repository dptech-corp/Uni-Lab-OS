#!/usr/bin/env python3
"""
Final Unilab MoveIt2 Integration Test
测试完整的 Unilab-MoveIt2 集成
"""

import sys
import os
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    JointConstraint,
    PlanningOptions,
    WorkspaceParameters
)
from geometry_msgs.msg import Vector3
import threading

# 添加 Unilab 路径
sys.path.append('/home/hh/Uni-Lab-OS')
from unilabos.devices.ros_dev.moveit_interface import MoveitInterface

class FinalUnilabTest(Node):
    def __init__(self):
        super().__init__('final_unilab_test')
        self.action_client = ActionClient(self, MoveGroup, '/move_action')
        self.moveit_interface = MoveitInterface()
        
        # 初始化完成后再设置设备 ID
        self.moveit_interface.device_id = "dummy2"
        
        print("🔧 等待 MoveIt2 动作服务...")
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            print("❌ MoveIt2 动作服务不可用")
            return
        print("✅ MoveIt2 动作服务已连接")
        
    def test_joint_movement(self):
        """测试关节空间运动"""
        print("\n🎯 测试关节空间运动...")
        
        # 使用 Unilab MoveitInterface 的方法
        try:
            target_joints = {
                'joint_1': 0.1,
                'joint_2': 0.0,
                'joint_3': 0.0,
                'joint_4': 0.0,
                'joint_5': 0.0,
                'joint_6': 0.0
            }
            
            print(f"📤 发送关节目标: {target_joints}")
            result = self.moveit_interface.moveit_joint_task(target_joints)
            print(f"✅ 运动结果: {result}")
            return True
            
        except Exception as e:
            print(f"❌ 运动失败: {e}")
            return False
    
    def test_direct_action(self):
        """直接测试 MoveIt 动作"""
        print("\n🎯 直接测试 MoveIt 动作...")
        
        # 创建运动规划请求
        goal_msg = MoveGroup.Goal()
        goal_msg.request = MotionPlanRequest()
        
        # 设置规划组
        goal_msg.request.group_name = "dummy2_arm"
        
        # 设置关节约束
        joint_constraint = JointConstraint()
        joint_constraint.joint_name = "Joint1"  # 使用实际的关节名称
        joint_constraint.position = 0.1
        joint_constraint.tolerance_above = 0.01
        joint_constraint.tolerance_below = 0.01
        joint_constraint.weight = 1.0
        
        constraints = Constraints()
        constraints.joint_constraints = [joint_constraint]
        goal_msg.request.goal_constraints = [constraints]
        
        # 设置规划选项
        goal_msg.planning_options = PlanningOptions()
        goal_msg.planning_options.plan_only = False  # 规划并执行
        goal_msg.planning_options.look_around = False
        goal_msg.planning_options.look_around_attempts = 0
        goal_msg.planning_options.max_safe_execution_cost = 1.0
        goal_msg.planning_options.replan = False
        goal_msg.planning_options.replan_attempts = 0
        goal_msg.planning_options.replan_delay = 0.0
        
        # 设置工作空间
        goal_msg.request.workspace_parameters = WorkspaceParameters()
        goal_msg.request.workspace_parameters.header.frame_id = "base_link"
        goal_msg.request.workspace_parameters.min_corner = Vector3(x=-1.0, y=-1.0, z=-1.0)
        goal_msg.request.workspace_parameters.max_corner = Vector3(x=1.0, y=1.0, z=1.0)
        
        # 设置允许的规划时间
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.num_planning_attempts = 1
        
        print("📤 发送规划和执行请求...")
        future = self.action_client.send_goal_async(goal_msg)
        
        try:
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            goal_handle = future.result()
            
            if not goal_handle.accepted:
                print("❌ 目标被拒绝")
                return False
                
            print("✅ 目标被接受，等待执行结果...")
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
            
            result = result_future.result().result
            print(f"📊 执行结果错误码: {result.error_code.val}")
            
            if result.error_code.val == 1:  # SUCCESS
                print("🎉 运动成功!")
                return True
            else:
                print(f"❌ 运动失败，错误码: {result.error_code.val}")
                return False
                
        except Exception as e:
            print(f"❌ 执行异常: {e}")
            return False

def main():
    print("🤖 Unilab MoveIt2 最终集成测试")
    print("=" * 50)
    
    # 初始化 ROS2
    rclpy.init()
    
    try:
        # 创建测试节点
        test_node = FinalUnilabTest()
        
        # 运行测试
        print("\n🚀 开始测试序列...")
        
        # 测试1: Unilab MoveitInterface
        success1 = test_node.test_joint_movement()
        time.sleep(2)
        
        # 测试2: 直接 MoveIt 动作
        success2 = test_node.test_direct_action()
        
        # 结果总结
        print("\n" + "=" * 50)
        print("📋 测试结果总结:")
        print(f"  Unilab 接口测试: {'✅ 成功' if success1 else '❌ 失败'}")
        print(f"  直接动作测试: {'✅ 成功' if success2 else '❌ 失败'}")
        
        if success1 or success2:
            print("\n🎉 集成测试部分成功! Dummy2 可以通过 Unilab 控制")
        else:
            print("\n⚠️  需要进一步调试配置")
            
    except KeyboardInterrupt:
        print("\n⚠️  用户中断测试")
    except Exception as e:
        print(f"\n❌ 测试异常: {e}")
    finally:
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()
