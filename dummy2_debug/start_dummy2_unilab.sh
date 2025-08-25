#!/bin/bash
# Dummy2 Unilab环境启动脚本
# 专为unilab conda环境设计，不依赖系统ROS2

set -e

# 动态配置路径
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# 自动检测 dummy2_ws 路径
DUMMY2_WS_CANDIDATES=(
    "$HOME/dummy2/ros2/dummy2_ws"
    "$HOME/ros2/dummy2_ws"
    "$PROJECT_ROOT/../dummy2_ws"
    "$(find "$HOME" -maxdepth 3 -name "dummy2_ws" -type d 2>/dev/null | head -1)"
)

DUMMY2_WS=""
for ws in "${DUMMY2_WS_CANDIDATES[@]}"; do
    if [ -d "$ws" ] && [ -f "$ws/src/dummy2_hw/package.xml" ]; then
        DUMMY2_WS="$ws"
        break
    fi
done

if [ -z "$DUMMY2_WS" ]; then
    echo "错误: 无法找到 dummy2_ws 工作空间"
    echo "请确保以下位置之一存在 dummy2_ws:"
    for ws in "${DUMMY2_WS_CANDIDATES[@]}" ; do
        echo "  $ws"
    done
    exit 1
fi

echo "使用 dummy2_ws: $DUMMY2_WS"

# 初始化mamba
eval "$(mamba shell hook --shell bash)"

# 函数：激活环境并检查
setup_environment() {
    echo "激活unilab环境..."
    
    # 检查 mamba 是否可用
    if ! command -v mamba &> /dev/null; then
        echo "错误: mamba 未安装或不在PATH中"
        echo "请先安装 mamba 或 conda"
        exit 1
    fi
    
    # 激活 unilab 环境
    if ! mamba activate unilab 2>/dev/null; then
        echo "错误: 无法激活 unilab 环境"
        echo "请确保 unilab 环境已创建"
        exit 1
    fi
    
    # 检查 ROS2 是否可用
    if ! command -v ros2 &> /dev/null; then
        echo "错误: ROS2在unilab环境中不可用"
        echo "请在 unilab 环境中安装 ROS2"
        exit 1
    fi
    
    echo "✓ ROS2环境准备就绪 ($(ros2 --version 2>/dev/null || echo '未知版本'))"
    echo "✓ 工作空间: $DUMMY2_WS"
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

# 函数：启动CAN2ETH通信服务
start_can2eth() {
    echo "==================================="
    echo "启动CAN2ETH通信服务"
    echo "==================================="
    
    setup_environment
    cd "$DUMMY2_WS"
    source install/setup.bash
    
    echo "启动CAN2ETH通信服务..."
    echo "作用: 建立与机械臂硬件的CAN总线通信连接"
    echo "按Ctrl+C停止"
    ros2 launch dummy2_can2eth dummy2_can2eth_server.launch.py
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
    echo "注意: 请确保CAN2ETH服务已在另一个终端启动"
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
    
    # 检查是否指定无图形界面模式
    if [ "$2" = "no-gui" ] || [ "$2" = "headless" ]; then
        echo "启动MoveIt2规划服务 (无图形界面)..."
        echo "作用: 启动MoveIt后端服务进行路径规划和运动控制"
        echo "按Ctrl+C停止"
        ros2 launch dummy2_moveit_config demo.launch.py use_rviz:=false
    else
        echo "启动MoveIt2规划服务 (带RViz图形界面)..."
        echo "作用: 启动MoveIt服务和RViz可视化界面"
        echo "按Ctrl+C停止"
        ros2 launch dummy2_moveit_config demo.launch.py
    fi
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
    cd "$SCRIPT_DIR"
    
    # 检查可用的测试脚本
    AVAILABLE_TESTS=()
    if [ -f "dummy2_direct_move.py" ]; then
        AVAILABLE_TESTS+=("direct:dummy2_direct_move.py")
    fi
    if [ -f "force_home.py" ]; then
        AVAILABLE_TESTS+=("home:force_home.py")
    fi
    if [ -f "test_complete_integration.py" ]; then
        AVAILABLE_TESTS+=("integration:test_complete_integration.py")
    fi
    if [ -f "dummy2_move_demo.py" ]; then
        AVAILABLE_TESTS+=("demo:dummy2_move_demo.py")
    fi
    if [ -f "final_demo.py" ]; then
        AVAILABLE_TESTS+=("final:final_demo.py")
    fi
    
    if [ ${#AVAILABLE_TESTS[@]} -eq 0 ]; then
        echo "错误: 未找到测试脚本"
        exit 1
    fi
    
    # 如果指定了测试类型
    if [ -n "$2" ]; then
        case "$2" in
            "direct"|"d")
                echo "运行直接控制测试..."
                python dummy2_direct_move.py
                ;;
            "home"|"h")
                echo "运行归位测试..."
                python force_home.py
                ;;
            "integration"|"i")
                echo "运行集成测试..."
                python test_complete_integration.py
                ;;
            "demo")
                echo "运行移动演示..."
                python dummy2_move_demo.py
                ;;
            "final"|"f")
                echo "运行最终演示..."
                python final_demo.py
                ;;
            *)
                echo "未知测试类型: $2"
                echo "可用类型: direct, home, integration, demo, final"
                exit 1
                ;;
        esac
    else
        # 默认运行直接控制测试
        echo "运行直接控制测试..."
        python dummy2_direct_move.py
    fi
}

# 函数：显示配置信息
show_info() {
    echo "==================================="
    echo "Dummy2 配置信息"
    echo "==================================="
    
    echo "脚本位置: $SCRIPT_DIR"
    echo "项目根目录: $PROJECT_ROOT"
    echo "工作空间: $DUMMY2_WS"
    echo ""
    
    echo "环境检查:"
    if command -v mamba &> /dev/null; then
        echo "✓ mamba 可用: $(which mamba)"
    else
        echo "✗ mamba 不可用"
    fi
    
    echo ""
    echo "可用测试脚本:"
    cd "$SCRIPT_DIR"
    for script in dummy2_direct_move.py force_home.py test_complete_integration.py dummy2_move_demo.py final_demo.py; do
        if [ -f "$script" ]; then
            echo "✓ $script"
        else
            echo "✗ $script (未找到)"
        fi
    done
    
    echo ""
    echo "工作空间状态:"
    if [ -f "$DUMMY2_WS/install/setup.bash" ]; then
        echo "✓ 工作空间已构建"
    else
        echo "✗ 工作空间未构建 (运行: $0 build)"
    fi
}

# 主程序
case "${1:-help}" in
    "build")
        build_workspace
        ;;
    "can2eth"|"can")
        start_can2eth
        ;;
    "hw"|"hardware")
        start_hardware
        ;;
    "moveit")
        start_moveit "$@"
        ;;
    "check"|"status")
        check_status
        ;;
    "info")
        show_info
        ;;
    "test")
        run_test "$@"
        ;;
    "help"|*)
        echo "Dummy2 Unilab环境启动脚本"
        echo ""
        echo "用法: $0 [命令] [选项]"
        echo ""
        echo "命令:"
        echo "  build         - 构建工作空间"
        echo "  can2eth (can) - 启动CAN2ETH通信服务"
        echo "  hw            - 启动硬件接口"
        echo "  moveit [模式] - 启动MoveIt服务"
        echo "  check         - 检查服务状态"
        echo "  info          - 显示配置信息"
        echo "  test [类型]   - 运行控制测试"
        echo "  help          - 显示此帮助信息"
        echo ""
        echo "MoveIt模式:"
        echo "  (默认)       - 启动带RViz图形界面"
        echo "  no-gui       - 启动无图形界面模式"
        echo "  headless     - 同no-gui"
        echo ""
        echo "测试类型:"
        echo "  direct (d)     - 直接关节控制测试 (默认)"
        echo "  home (h)       - 归位控制测试"
        echo "  integration (i) - Unilab集成测试"
        echo "  demo           - 移动演示"
        echo "  final (f)      - 最终演示"
        echo ""
        echo "启动顺序 (标准流程):"
        echo "1. $0 build              (首次或更新后)"
        echo "2. $0 can2eth            (终端1 - CAN2ETH通信)"
        echo "3. $0 moveit no-gui      (终端2 - MoveIt服务)"
        echo "4. $0 test [类型]        (终端3 - 控制测试)"
        echo ""
        echo "快速启动 (如果已有CAN2ETH):"
        echo "1. $0 hw                 (终端1 - 硬件接口)"
        echo "2. $0 test [类型]        (终端2 - 控制测试)"
        echo ""
        echo "示例:"
        echo "  $0 can2eth              # 启动CAN2ETH通信"
        echo "  $0 moveit no-gui        # 启动MoveIt(无界面)"
        echo "  $0 test direct          # 运行直接控制测试"
        echo "  $0 test home            # 运行归位测试"
        echo ""
        echo "自动检测路径:"
        echo "  脚本目录: $SCRIPT_DIR"
        if [ -n "$DUMMY2_WS" ]; then
            echo "  工作空间: $DUMMY2_WS"
        else
            echo "  工作空间: 未找到"
        fi
        ;;
esac
