#!/usr/bin/env python3
"""
MoveIt2配置问题诊断和修复脚本
"""

import subprocess
import time
import sys
import os

def check_current_services():
    """检查当前ROS2服务状态"""
    print("🔍 检查当前ROS2服务状态...")
    
    try:
        # 检查节点
        result = subprocess.run(['ros2', 'node', 'list'], 
                               capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            nodes = result.stdout.strip().split('\n')
            print(f"当前运行的节点 ({len(nodes)}):")
            for node in nodes:
                print(f"  - {node}")
                
            # 检查是否有move_group
            if '/move_group' in nodes:
                print("✅ move_group节点正在运行")
                return True
            else:
                print("❌ move_group节点未运行")
                return False
        else:
            print("❌ 无法获取节点列表")
            return False
            
    except Exception as e:
        print(f"❌ 检查服务状态失败: {e}")
        return False

def check_moveit_launch_files():
    """检查MoveIt启动文件"""
    print("\n🔍 检查MoveIt启动文件...")
    
    dummy2_ws = "/home/hh/dummy2/ros2/dummy2_ws"
    
    # 检查demo.launch.py
    demo_files = [
        f"{dummy2_ws}/install/dummy2_moveit_config/share/dummy2_moveit_config/launch/demo.launch.py",
        f"{dummy2_ws}/src/dummy2_moveit_config/launch/demo.launch.py"
    ]
    
    for demo_file in demo_files:
        if os.path.exists(demo_file):
            print(f"✅ 找到demo.launch.py: {demo_file}")
            return demo_file
    
    print("❌ 未找到demo.launch.py")
    return None

def start_moveit_service():
    """启动MoveIt服务"""
    print("\n🚀 启动MoveIt2服务...")
    
    dummy2_ws = "/home/hh/dummy2/ros2/dummy2_ws"
    
    try:
        # 设置环境
        env = os.environ.copy()
        env['ROS_DISTRO'] = 'humble'
        
        # 切换到工作空间
        os.chdir(dummy2_ws)
        
        # 构建启动命令
        cmd = [
            'bash', '-c',
            'source /opt/ros/humble/setup.bash && '
            'source install/setup.bash && '
            'ros2 launch dummy2_moveit_config demo.launch.py'
        ]
        
        print("执行命令:", ' '.join(cmd))
        print("⚠️  这将启动MoveIt2服务，按Ctrl+C停止")
        
        # 启动服务
        process = subprocess.Popen(cmd, env=env)
        process.wait()
        
    except KeyboardInterrupt:
        print("\n⚠️  用户中断服务")
    except Exception as e:
        print(f"❌ 启动MoveIt服务失败: {e}")

def test_moveit_actions():
    """测试MoveIt action服务"""
    print("\n🧪 测试MoveIt action服务...")
    
    try:
        # 等待服务启动
        time.sleep(3)
        
        # 检查action列表
        result = subprocess.run(['ros2', 'action', 'list'], 
                               capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            actions = result.stdout.strip().split('\n')
            print(f"可用的action服务 ({len(actions)}):")
            for action in actions:
                print(f"  - {action}")
                
            # 查找MoveIt相关actions
            moveit_actions = [a for a in actions if 'move' in a.lower()]
            if moveit_actions:
                print(f"\nMoveIt相关actions:")
                for action in moveit_actions:
                    print(f"  ✅ {action}")
                return True
            else:
                print("❌ 未找到MoveIt相关actions")
                return False
        else:
            print("❌ 无法获取action列表")
            return False
            
    except Exception as e:
        print(f"❌ 测试action服务失败: {e}")
        return False

def create_moveit_fix_script():
    """创建MoveIt修复脚本"""
    print("\n📝 创建MoveIt修复脚本...")
    
    script_content = """#!/bin/bash
# MoveIt2服务启动脚本

DUMMY2_WS="/home/hh/dummy2/ros2/dummy2_ws"

echo "🚀 启动MoveIt2服务..."
echo "工作空间: $DUMMY2_WS"

cd "$DUMMY2_WS"

# 设置环境
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "📋 可用的启动文件:"
find install/ -name "*.launch.py" | grep moveit | head -5

echo ""
echo "🎯 启动move_group服务..."
echo "命令: ros2 launch dummy2_moveit_config move_group.launch.py"

# 启动move_group
ros2 launch dummy2_moveit_config move_group.launch.py
"""
    
    script_path = "/home/hh/Uni-Lab-OS/dummy2_debug/start_moveit.sh"
    with open(script_path, 'w') as f:
        f.write(script_content)
    
    # 设置可执行权限
    os.chmod(script_path, 0o755)
    print(f"✅ 创建脚本: {script_path}")
    
    return script_path

def diagnose_moveit_config():
    """诊断MoveIt配置"""
    print("\n🔧 诊断MoveIt配置问题...")
    
    # 检查配置文件
    dummy2_ws = "/home/hh/dummy2/ros2/dummy2_ws"
    config_dirs = [
        f"{dummy2_ws}/install/dummy2_moveit_config/share/dummy2_moveit_config/config",
        f"{dummy2_ws}/src/dummy2_moveit_config/config"
    ]
    
    for config_dir in config_dirs:
        if os.path.exists(config_dir):
            print(f"✅ 找到配置目录: {config_dir}")
            
            # 列出配置文件
            config_files = os.listdir(config_dir)
            print("配置文件:")
            for file in config_files[:10]:  # 只显示前10个
                print(f"  - {file}")
            break
    else:
        print("❌ 未找到MoveIt配置目录")
    
    # 检查URDF文件
    urdf_dirs = [
        f"{dummy2_ws}/install/dummy2_description/share/dummy2_description",
        f"{dummy2_ws}/src/dummy2_description"
    ]
    
    for urdf_dir in urdf_dirs:
        if os.path.exists(urdf_dir):
            print(f"✅ 找到URDF目录: {urdf_dir}")
            break
    else:
        print("❌ 未找到URDF目录")

def main():
    """主函数"""
    print("🔧 MoveIt2配置诊断工具")
    print("=" * 50)
    
    # 1. 检查当前状态
    move_group_running = check_current_services()
    
    # 2. 诊断配置
    diagnose_moveit_config()
    
    # 3. 检查启动文件
    demo_file = check_moveit_launch_files()
    
    # 4. 创建修复脚本
    fix_script = create_moveit_fix_script()
    
    print("\n" + "=" * 50)
    print("📋 诊断结果总结")
    print("=" * 50)
    
    if move_group_running:
        print("✅ MoveIt2服务正在运行")
        test_moveit_actions()
    else:
        print("❌ MoveIt2服务未运行")
        print("\n🔧 解决方案:")
        print("1. 使用修复脚本启动MoveIt:")
        print(f"   {fix_script}")
        print("\n2. 或手动启动:")
        print("   cd /home/hh/dummy2/ros2/dummy2_ws")
        print("   source /opt/ros/humble/setup.bash")
        print("   source install/setup.bash") 
        print("   ros2 launch dummy2_moveit_config move_group.launch.py")
        
        print("\n3. 在新终端测试Unilab控制:")
        print("   cd /home/hh/Uni-Lab-OS/dummy2_debug")
        print("   python dummy2_move_demo.py")

if __name__ == "__main__":
    main()
