#!/bin/bash
# Dummy2 ROS2服务启动脚本
# 用于启动Dummy2机械臂的ROS2服务

echo "==================================="
echo "Dummy2 ROS2服务启动脚本"
echo "==================================="

# 设置变量
DUMMY2_WS="/home/hh/dummy2/ros2/dummy2_ws"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 检查workspace是否存在
if [ ! -d "$DUMMY2_WS" ]; then
    echo "错误: Dummy2工作空间不存在: $DUMMY2_WS"
    exit 1
fi

echo "Dummy2工作空间: $DUMMY2_WS"

# 函数：检查ROS2环境
check_ros2_environment() {
    echo "检查ROS2环境..."
    
    if [ -z "$ROS_DISTRO" ]; then
        echo "警告: ROS_DISTRO环境变量未设置"
        echo "尝试设置ROS2 Humble环境..."
        source /opt/ros/humble/setup.bash
    fi
    
    echo "ROS2版本: $ROS_DISTRO"
    
    # 检查ROS2命令是否可用
    if command -v ros2 &> /dev/null; then
        echo "✓ ROS2命令可用"
    else
        echo "✗ ROS2命令不可用"
        echo "请确保ROS2已正确安装"
        exit 1
    fi
}

# 函数：构建workspace
build_workspace() {
    echo ""
    echo "构建Dummy2工作空间..."
    
    cd "$DUMMY2_WS"
    
    # 设置ROS2环境
    source /opt/ros/humble/setup.bash
    
    # 构建workspace
    echo "运行colcon build..."
    if colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release; then
        echo "✓ 构建成功"
    else
        echo "✗ 构建失败"
        return 1
    fi
    
    # 设置环境
    source install/setup.bash
    echo "✓ 环境设置完成"
}

# 函数：启动硬件接口
start_hardware_interface() {
    echo ""
    echo "启动Dummy2硬件接口..."
    
    cd "$DUMMY2_WS"
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    
    echo "启动命令: ros2 launch dummy2_hw dummy2_hw.launch.py"
    echo "注意: 这将在前台运行，按Ctrl+C停止"
    echo "启动后请在新终端中运行MoveIt服务"
    echo ""
    
    # 启动硬件接口
    ros2 launch dummy2_hw dummy2_hw.launch.py
}

# 函数：启动MoveIt服务
start_moveit_service() {
    echo ""
    echo "启动MoveIt2服务..."
    
    cd "$DUMMY2_WS"
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    
    echo "启动命令: ros2 launch dummy2_moveit_config demo.launch.py"
    echo "注意: 这将在前台运行，按Ctrl+C停止"
    echo ""
    
    # 启动MoveIt服务
    ros2 launch dummy2_moveit_config demo.launch.py
}

# 函数：检查服务状态
check_services() {
    echo ""
    echo "检查ROS2服务状态..."
    
    source /opt/ros/humble/setup.bash
    
    echo "ROS2话题:"
    ros2 topic list | head -10
    
    echo ""
    echo "ROS2服务:"
    ros2 service list | head -10
    
    echo ""
    echo "ROS2节点:"
    ros2 node list
}

# 主菜单
show_menu() {
    echo ""
    echo "请选择操作:"
    echo "1. 构建Dummy2工作空间"
    echo "2. 启动硬件接口"
    echo "3. 启动MoveIt服务"
    echo "4. 检查服务状态"
    echo "5. 显示启动说明"
    echo "0. 退出"
    echo ""
    read -p "请输入选项 (0-5): " choice
}

# 显示启动说明
show_instructions() {
    echo ""
    echo "==================================="
    echo "Dummy2启动说明"
    echo "==================================="
    echo ""
    echo "完整启动流程:"
    echo ""
    echo "1. 首先构建工作空间 (选项1)"
    echo ""
    echo "2. 在终端1启动硬件接口 (选项2):"
    echo "   ./start_dummy2_ros2.sh"
    echo "   然后选择选项2"
    echo ""
    echo "3. 在终端2启动MoveIt服务 (选项3):"
    echo "   打开新终端，运行:"
    echo "   cd $DUMMY2_WS"
    echo "   source /opt/ros/humble/setup.bash"
    echo "   source install/setup.bash"
    echo "   ros2 launch dummy2_moveit_config demo.launch.py"
    echo ""
    echo "4. 在终端3测试Unilab控制:"
    echo "   cd /home/hh/Uni-Lab-OS"
    echo "   python test_dummy2_real_control.py --test-control"
    echo ""
    echo "注意事项:"
    echo "- 确保Dummy2硬件已连接"
    echo "- 检查CAN2ETH网络设置"
    echo "- 确保机械臂在安全位置"
}

# 主程序
main() {
    check_ros2_environment
    
    if [ "$1" = "hw" ]; then
        start_hardware_interface
    elif [ "$1" = "moveit" ]; then
        start_moveit_service
    elif [ "$1" = "check" ]; then
        check_services
    elif [ "$1" = "build" ]; then
        build_workspace
    else
        while true; do
            show_menu
            case $choice in
                1)
                    build_workspace
                    ;;
                2)
                    start_hardware_interface
                    ;;
                3)
                    start_moveit_service
                    ;;
                4)
                    check_services
                    ;;
                5)
                    show_instructions
                    ;;
                0)
                    echo "退出"
                    exit 0
                    ;;
                *)
                    echo "无效选项，请重新选择"
                    ;;
            esac
        done
    fi
}

# 运行主程序
main "$@"
