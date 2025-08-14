# Dummy2 Unilab 启动指南

## 🚀 快速启动 (推荐)

### 使用统一脚本
```bash
cd /home/hh/Uni-Lab-OS/dummy2_debug

# 1. 检查状态
./start_dummy2_unilab.sh check

# 2. 启动硬件接口 (终端1)
./start_dummy2_unilab.sh hw

# 3. 运行控制测试 (终端2) 
./start_dummy2_unilab.sh test

# 4. 启动MoveIt服务 (可选，终端3)
./start_dummy2_unilab.sh moveit
```

## 📋 详细步骤

### 首次使用或更新后构建
```bash
./start_dummy2_unilab.sh build
```

### 手动启动流程
```bash
# 硬件接口 (终端1)
cd /home/hh/dummy2/ros2/dummy2_ws
mamba activate unilab
source install/setup.bash
ros2 launch dummy2_hw dummy2_hw.launch.py

# 控制脚本 (终端2)
cd /home/hh/Uni-Lab-OS/dummy2_debug  
mamba activate unilab
source /home/hh/dummy2/ros2/dummy2_ws/install/setup.bash
python dummy2_direct_move.py
```

## ⚙️ 可用命令

| 命令 | 功能 |
|------|------|
| `./start_dummy2_unilab.sh check` | 检查系统状态 |
| `./start_dummy2_unilab.sh build` | 构建工作空间 |
| `./start_dummy2_unilab.sh hw` | 启动硬件接口 |
| `./start_dummy2_unilab.sh test` | 运行控制测试 |
| `./start_dummy2_unilab.sh moveit` | 启动MoveIt服务 |

## ⚠️ 重要提示
- ✅ 使用 `mamba activate unilab` (ROS2已包含在内)
- ❌ 不需要 `source /opt/ros/humble/setup.bash`
- ✅ 确保硬件接口先启动
- ✅ 机械臂需在安全位置
