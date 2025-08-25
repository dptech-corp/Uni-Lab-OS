# Dummy2 Unilab 启动指南

## 🚀 快速启动 (推荐)

### 标准启动流程 (三步启动)
```bash
cd /home/hh/Uni-Lab-OS/dummy2_debug

# 1. 启动CAN2ETH通信服务 (终端1)
./start_dummy2_unilab.sh can2eth

# 2. 启动MoveIt服务 (终端2)
./start_dummy2_unilab.sh moveit no-gui

# 3. 运行控制测试 (终端3)
./start_dummy2_unilab.sh test direct
```

### 快速启动 (如果CAN2ETH已运行)
```bash
# 1. 检查状态
./start_dummy2_unilab.sh check

# 2. 启动硬件接口 (终端1)
./start_dummy2_unilab.sh hw

# 3. 运行控制测试 (终端2)
./start_dummy2_unilab.sh test direct
```

## 📋 详细步骤

### 首次使用或更新后构建
```bash
./start_dummy2_unilab.sh build
```

### 手动启动流程
```bash
# CAN2ETH通信服务 (终端1)
cd /home/hh/dummy2/ros2/dummy2_ws
mamba activate unilab
source install/setup.bash
ros2 launch dummy2_can2eth dummy2_can2eth_server.launch.py

# MoveIt服务 (终端2)
cd /home/hh/dummy2/ros2/dummy2_ws
mamba activate unilab
source install/setup.bash
ros2 launch dummy2_moveit_config demo.launch.py use_rviz:=false

# 控制脚本 (终端3)
cd /home/hh/Uni-Lab-OS/dummy2_debug  
mamba activate unilab
source /home/hh/dummy2/ros2/dummy2_ws/install/setup.bash
python dummy2_direct_move.py
```

## ⚙️ 可用命令

| 命令 | 功能 |
|------|------|
| `./start_dummy2_unilab.sh can2eth` | 启动CAN2ETH通信服务 |
| `./start_dummy2_unilab.sh hw` | 启动硬件接口 |
| `./start_dummy2_unilab.sh moveit` | 启动MoveIt服务 (带图形界面) |
| `./start_dummy2_unilab.sh moveit no-gui` | 启动MoveIt服务 (无图形界面) |
| `./start_dummy2_unilab.sh test [类型]` | 运行控制测试 |
| `./start_dummy2_unilab.sh check` | 检查系统状态 |
| `./start_dummy2_unilab.sh info` | 显示配置信息 |

## ⚠️ 重要提示
- ✅ 使用 `mamba activate unilab` (ROS2已包含在内)
- ❌ 不需要 `source /opt/ros/humble/setup.bash`
- ✅ **必须先启动CAN2ETH通信服务**
- ✅ 确保按顺序启动各个服务
- ✅ 机械臂需在安全位置
