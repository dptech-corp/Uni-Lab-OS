#!/bin/bash
# MoveIt2服务启动脚本

DUMMY2_WS="/home/hh/dummy2/ros2/dummy2_ws"

echo "🚀 启动MoveIt2服务..."
echo "工作空间: $DUMMY2_WS"

cd "$DUMMY2_WS"

# 设置环境
# source /opt/ros/humble/setup.bash
source install/setup.bash

echo "📋 可用的启动文件:"
find install/ -name "*.launch.py" | grep moveit | head -5

echo ""
echo "🎯 启动move_group服务..."
echo "命令: ros2 launch dummy2_moveit_config move_group.launch.py"

# 启动move_group
ros2 launch dummy2_moveit_config move_group.launch.py
