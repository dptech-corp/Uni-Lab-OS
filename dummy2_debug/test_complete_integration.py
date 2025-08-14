#!/usr/bin/env python3
"""
Simplified Unilab MoveIt2 Integration Test
简化的 Unilab-MoveIt2 集成测试
"""

import sys
import os
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

class SimplifiedUnilabTest(Node):
    def __init__(self):
        super().__init__('simplified_unilab_test')
        
        # 创建动作客户端
        self.trajectory_client = ActionClient(self, FollowJointTrajectory, '/dummy2_arm_controller/follow_joint_trajectory')
        self.moveit_client = ActionClient(self, MoveGroup, '/move_action')
        
        print("🔧 等待动作服务...")
        
        # 等待轨迹控制器
        if self.trajectory_client.wait_for_server(timeout_sec=5.0):
            print("✅ FollowJointTrajectory 服务已连接")
        else:
            print("❌ FollowJointTrajectory 服务不可用")
            
        # 等待 MoveIt 服务
        if self.moveit_client.wait_for_server(timeout_sec=5.0):
            print("✅ MoveIt 动作服务已连接")
        else:
            print("❌ MoveIt 动作服务不可用")
        
    def test_direct_trajectory_control(self):
        """测试直接轨迹控制（已验证工作）"""
        print("\n🎯 测试直接轨迹控制...")
        
        try:
            # 创建轨迹目标
            goal_msg = FollowJointTrajectory.Goal()
            goal_msg.trajectory = JointTrajectory()
            goal_msg.trajectory.header.frame_id = ""
            goal_msg.trajectory.joint_names = ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"]
            
            # 添加轨迹点
            point = JointTrajectoryPoint()
            point.positions = [0.2, 0.0, 0.0, 0.0, 0.0, 0.0]  # 只移动第一个关节
            point.time_from_start.sec = 2
            goal_msg.trajectory.points = [point]
            
            print("📤 发送轨迹目标...")
            future = self.trajectory_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            goal_handle = future.result()
            if not goal_handle.accepted:
                print("❌ 轨迹目标被拒绝")
                return False
                
            print("✅ 轨迹目标被接受，等待执行...")
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)
            
            result = result_future.result().result
            print(f"📊 轨迹执行结果: {result.error_code}")
            
            if result.error_code == 0:  # SUCCESSFUL
                print("🎉 直接轨迹控制成功!")
                return True
            else:
                print(f"❌ 轨迹执行失败，错误码: {result.error_code}")
                return False
                
        except Exception as e:
            print(f"❌ 直接控制异常: {e}")
            return False
    
    def test_moveit_planning(self):
        """测试 MoveIt 规划（仅规划不执行）"""
        print("\n🎯 测试 MoveIt 规划...")
        
        try:
            # 创建规划请求
            goal_msg = MoveGroup.Goal()
            goal_msg.request = MotionPlanRequest()
            goal_msg.request.group_name = "dummy2_arm"
            
            # 设置关节约束
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = "Joint1"
            joint_constraint.position = 0.3
            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            
            constraints = Constraints()
            constraints.joint_constraints = [joint_constraint]
            goal_msg.request.goal_constraints = [constraints]
            
            # 设置规划选项（仅规划）
            goal_msg.planning_options = PlanningOptions()
            goal_msg.planning_options.plan_only = True  # 仅规划，不执行
            goal_msg.planning_options.look_around = False
            goal_msg.planning_options.max_safe_execution_cost = 1.0
            goal_msg.planning_options.replan = False
            
            # 设置工作空间
            goal_msg.request.workspace_parameters = WorkspaceParameters()
            goal_msg.request.workspace_parameters.header.frame_id = "base_link"
            goal_msg.request.workspace_parameters.min_corner = Vector3(x=-2.0, y=-2.0, z=-2.0)
            goal_msg.request.workspace_parameters.max_corner = Vector3(x=2.0, y=2.0, z=2.0)
            
            goal_msg.request.allowed_planning_time = 5.0
            goal_msg.request.num_planning_attempts = 3
            
            print("📤 发送规划请求...")
            future = self.moveit_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            goal_handle = future.result()
            if not goal_handle.accepted:
                print("❌ 规划目标被拒绝")
                return False
                
            print("✅ 规划目标被接受，等待规划结果...")
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=15.0)
            
            result = result_future.result().result
            print(f"📊 规划结果错误码: {result.error_code.val}")
            
            if result.error_code.val == 1:  # SUCCESS
                print("🎉 MoveIt 规划成功!")
                if result.planned_trajectory:
                    print(f"✅ 生成轨迹包含 {len(result.planned_trajectory.joint_trajectory.points)} 个点")
                return True
            else:
                print(f"❌ 规划失败，错误码: {result.error_code.val}")
                return False
                
        except Exception as e:
            print(f"❌ 规划异常: {e}")
            return False

def test_unilab_integration():
    """测试 Unilab 设备注册和配置"""
    print("\n🎯 测试 Unilab 设备集成...")
    
    try:
        # 检查设备注册文件
        registry_file = "/home/hh/Uni-Lab-OS/unilabos/registry/devices/robot_arm.yaml"
        if os.path.exists(registry_file):
            print("✅ 找到设备注册文件")
            with open(registry_file, 'r') as f:
                content = f.read()
                if 'robotic_arm.Dummy2' in content:
                    print("✅ Dummy2 设备已注册")
                else:
                    print("❌ Dummy2 设备未注册")
                    return False
        else:
            print("❌ 设备注册文件不存在")
            return False
            
        # 检查设备配置
        config_dir = "/home/hh/Uni-Lab-OS/unilabos/device_mesh/devices/dummy2_robot"
        if os.path.exists(config_dir):
            print("✅ 找到设备配置目录")
            
            move_group_file = f"{config_dir}/config/move_group.json"
            if os.path.exists(move_group_file):
                print("✅ 找到 MoveGroup 配置文件")
            else:
                print("❌ MoveGroup 配置文件不存在")
                return False
        else:
            print("❌ 设备配置目录不存在")
            return False
            
        print("🎉 Unilab 设备集成配置完整!")
        return True
        
    except Exception as e:
        print(f"❌ Unilab 集成检查异常: {e}")
        return False

def main():
    print("🤖 简化 Unilab MoveIt2 集成测试")
    print("=" * 50)
    
    # 测试 Unilab 配置
    unilab_ok = test_unilab_integration()
    
    if not unilab_ok:
        print("\n❌ Unilab 配置有问题，请检查设备注册和配置")
        return
    
    # 初始化 ROS2
    rclpy.init()
    
    try:
        # 创建测试节点
        test_node = SimplifiedUnilabTest()
        
        print("\n🚀 开始 ROS2 控制测试...")
        
        # 测试1: 直接轨迹控制
        direct_success = test_node.test_direct_trajectory_control()
        time.sleep(2)
        
        # 测试2: MoveIt 规划
        moveit_success = test_node.test_moveit_planning()
        
        # 结果总结
        print("\n" + "=" * 50)
        print("📋 完整集成测试结果:")
        print(f"  Unilab 设备配置: {'✅ 完整' if unilab_ok else '❌ 缺失'}")
        print(f"  直接轨迹控制: {'✅ 成功' if direct_success else '❌ 失败'}")
        print(f"  MoveIt 规划功能: {'✅ 成功' if moveit_success else '❌ 失败'}")
        
        if unilab_ok and direct_success:
            print("\n🎉 核心功能完整! Dummy2 已成功移植到 Unilab 系统")
            print("💡 建议:")
            print("   - 直接轨迹控制已完全可用")
            if moveit_success:
                print("   - MoveIt2 规划功能也已可用")
            else:
                print("   - MoveIt2 规划可能需要进一步配置调优")
        else:
            print("\n⚠️  需要解决基础连接问题")
            
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
