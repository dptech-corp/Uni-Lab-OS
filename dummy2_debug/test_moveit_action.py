#!/usr/bin/env python3
"""
Dummy2 MoveIt2控制测试（修复版本）
解决设备名称映射和action问题
"""

import json
import time
import sys
import os
import threading
import signal

# 添加Unilab路径
sys.path.insert(0, '/home/hh/Uni-Lab-OS')

def test_direct_moveit_action():
    """直接测试MoveIt action服务"""
    print("🔧 直接测试MoveIt action服务...")
    
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.action import ActionClient
        from moveit_msgs.action import MoveGroup
        from moveit_msgs.msg import (
            MotionPlanRequest, 
            PlanningOptions,
            Constraints,
            JointConstraint
        )
        from geometry_msgs.msg import PoseStamped
        
        # 初始化ROS2
        rclpy.init()
        
        # 创建节点
        node = Node('moveit_test_client')
        
        # 创建action客户端
        action_client = ActionClient(node, MoveGroup, '/move_action')
        
        # 启动executor
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        
        print("✓ 节点和action客户端创建成功")
        
        # 等待action服务
        if not action_client.wait_for_server(timeout_sec=10.0):
            print("❌ MoveIt action服务连接超时")
            return False
        
        print("✅ MoveIt action服务连接成功")
        
        # 创建运动规划请求
        goal_msg = MoveGroup.Goal()
        
        # 设置请求参数
        goal_msg.request.group_name = "dummy2_arm"  # 注意这里的组名
        goal_msg.request.num_planning_attempts = 3
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.2
        goal_msg.request.max_acceleration_scaling_factor = 0.2
        
        # 设置关节约束（简单的home位置）
        joint_constraint = JointConstraint()
        joint_constraint.joint_name = "Joint1"
        joint_constraint.position = 0.0
        joint_constraint.tolerance_above = 0.01
        joint_constraint.tolerance_below = 0.01
        joint_constraint.weight = 1.0
        
        constraints = Constraints()
        constraints.joint_constraints = [joint_constraint]
        goal_msg.request.goal_constraints = [constraints]
        
        # 设置规划选项
        goal_msg.planning_options.planning_scene_diff.is_diff = True
        goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = True
        goal_msg.planning_options.plan_only = False  # 执行规划结果
        
        print("📤 发送MoveIt规划请求...")
        print(f"   目标组: {goal_msg.request.group_name}")
        print(f"   关节约束: {joint_constraint.joint_name} = {joint_constraint.position}")
        
        # 发送目标
        future = action_client.send_goal_async(goal_msg)
        
        # 等待结果
        rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
        
        if future.result() is not None:
            goal_handle = future.result()
            if goal_handle.accepted:
                print("✅ 规划请求被接受")
                
                # 等待执行结果
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(node, result_future, timeout_sec=10.0)
                
                if result_future.result() is not None:
                    result = result_future.result().result
                    print(f"📊 规划结果: {result.error_code.val}")
                    
                    if result.error_code.val == 1:  # SUCCESS
                        print("🎉 MoveIt规划和执行成功！")
                        return True
                    else:
                        print(f"❌ MoveIt执行失败，错误代码: {result.error_code.val}")
                        return False
                else:
                    print("❌ 等待执行结果超时")
                    return False
            else:
                print("❌ 规划请求被拒绝")
                return False
        else:
            print("❌ 发送规划请求超时")
            return False
            
    except Exception as e:
        print(f"❌ MoveIt测试失败: {e}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        try:
            executor.shutdown()
            rclpy.shutdown()
        except:
            pass

def check_moveit_groups():
    """检查MoveIt规划组"""
    print("\n🔍 检查MoveIt规划组...")
    
    try:
        import subprocess
        
        # 获取规划组信息
        result = subprocess.run([
            'ros2', 'service', 'call', '/query_planner_params',
            'moveit_msgs/srv/QueryPlannerParams', '{}'
        ], capture_output=True, text=True, timeout=10)
        
        if result.returncode == 0:
            print("✅ 成功查询规划器参数")
            print("响应:")
            print(result.stdout)
        else:
            print("❌ 查询规划器参数失败")
            print(result.stderr)
            
    except Exception as e:
        print(f"❌ 检查规划组失败: {e}")

def check_robot_description():
    """检查机器人描述"""
    print("\n🔍 检查机器人描述...")
    
    try:
        import subprocess
        
        # 获取机器人描述参数
        result = subprocess.run([
            'ros2', 'param', 'get', '/move_group', 'robot_description'
        ], capture_output=True, text=True, timeout=10)
        
        if result.returncode == 0:
            urdf_content = result.stdout
            # 检查关节名称
            joint_names = []
            for line in urdf_content.split('\n'):
                if 'joint name=' in line and 'type=' in line:
                    # 简单解析关节名称
                    start = line.find('name="') + 6
                    end = line.find('"', start)
                    if start > 5 and end > start:
                        joint_name = line[start:end]
                        if 'Joint' in joint_name:
                            joint_names.append(joint_name)
            
            print(f"✅ 找到关节: {joint_names}")
            return joint_names
        else:
            print("❌ 获取机器人描述失败")
            return []
            
    except Exception as e:
        print(f"❌ 检查机器人描述失败: {e}")
        return []

def main():
    """主函数"""
    print("🔧 MoveIt2控制测试（修复版本）")
    print("=" * 50)
    
    # 1. 检查机器人描述和关节
    joint_names = check_robot_description()
    
    # 2. 检查规划组
    check_moveit_groups()
    
    # 3. 直接测试MoveIt action
    print("\n" + "="*30)
    print("开始MoveIt Action测试")
    print("="*30)
    
    if test_direct_moveit_action():
        print("\n🎉 MoveIt2控制测试成功！")
        print("Dummy2可以通过MoveIt2进行规划和控制")
    else:
        print("\n❌ MoveIt2控制测试失败")
        print("需要进一步调试配置问题")
    
    print("\n📋 下一步建议:")
    print("1. 检查SRDF文件中的规划组配置")
    print("2. 验证关节名称映射")
    print("3. 调试运动学求解器配置")

if __name__ == "__main__":
    main()
