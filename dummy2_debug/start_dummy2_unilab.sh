#!/bin/bash
# Dummy2 Unilab环境启动脚本
# 专为unilab conda环境设计，不依赖系统ROS2

set -e

# 配置
DUMMY2_WS="/home/hh/dummy2/ros2/dummy2_ws"

# 初始化mamba
eval "$(mamba shell hook --shell bash)"

# 函数：激活环境并检查
setup_environment() {
    echo "激活unilab环境..."
    mamba activate unilab
    
    if ! command -v ros2 &> /dev/null; then
        echo "错误: ROS2在unilab环境中不可用"
        exit 1
    fi
    
    echo "✓ ROS2环境准备就绪"
}

# 函数：构建工作空间
build_workspace() {
    echo "==================================="
    echo "构建Dummy2工作空间"
    echo "==================================="
    
    setup_environment
    cd "$DUMMY2_WS"
    
    echo "清理旧构建文件..."
    rm -rf build/ install/ log/
    
    echo "开始构建..."
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
    
    echo "✓ 构建完成"
}

# 函数：启动硬件接口
start_hardware() {
    echo "==================================="
    echo "启动Dummy2硬件接口"
    echo "==================================="
    
    setup_environment
    cd "$DUMMY2_WS"
    source install/setup.bash
    
    echo "启动硬件接口节点..."
    echo "按Ctrl+C停止"
    ros2 launch dummy2_hw dummy2_hw.launch.py
}

# 函数：启动MoveIt服务
start_moveit() {
    echo "==================================="
    echo "启动Dummy2 MoveIt服务"
    echo "==================================="
    
    setup_environment
    cd "$DUMMY2_WS"
    source install/setup.bash
    
    echo "启动MoveIt2规划服务..."
    echo "按Ctrl+C停止"
    ros2 launch dummy2_moveit_config demo.launch.py
}

# 函数：检查状态
check_status() {
    echo "==================================="
    echo "检查ROS2服务状态"
    echo "==================================="
    
    setup_environment
    
    echo "当前节点:"
    ros2 node list || echo "无节点运行"
    
    echo ""
    echo "可用话题:"
    ros2 topic list | head -10 || echo "无话题"
    
    echo ""
    echo "可用动作:"
    ros2 action list || echo "无动作服务"
}

# 函数：运行测试
run_test() {
    echo "==================================="
    echo "运行Dummy2控制测试"
    echo "==================================="
    
    setup_environment
    source "$DUMMY2_WS/install/setup.bash"
    cd /home/hh/Uni-Lab-OS
    
    echo "运行直接控制测试..."
    python dummy2_debug/dummy2_direct_move.py
}

# 主程序
case "${1:-help}" in
    "build")
        build_workspace
        ;;
    "hw"|"hardware")
        start_hardware
        ;;
    "moveit")
        start_moveit
        ;;
    "check"|"status")
        check_status
        ;;
    "test")
        run_test
        ;;
    "help"|*)
        echo "Dummy2 Unilab环境启动脚本"
        echo ""
        echo "用法: $0 [命令]"
        echo ""
        echo "命令:"
        echo "  build     - 构建工作空间"
        echo "  hw        - 启动硬件接口"
        echo "  moveit    - 启动MoveIt服务"
        echo "  check     - 检查服务状态"
        echo "  test      - 运行控制测试"
        echo "  help      - 显示此帮助信息"
        echo ""
        echo "启动顺序:"
        echo "1. $0 build     (首次或更新后)"
        echo "2. $0 hw        (终端1)"
        echo "3. $0 moveit    (终端2)"
        echo "4. $0 test      (终端3)"
        ;;
esac
