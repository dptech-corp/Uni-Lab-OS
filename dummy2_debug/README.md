# Dummy2 机械臂 Unilab 集成控制

## 📋 项目概述
将 Dummy2 6-DOF 机械臂从原生 ROS2 控制成功移植到 Unilab 设备管理系统，实现统一的设备控制接口。

## 🎯 核心功能
- ✅ **直接关节控制**: 通过 CAN2ETH 直接控制机械臂关节
- ✅ **MoveIt2 集成**: 支持路径规划和轨迹执行
- ✅ **Unilab 框架**: 集成到 Unilab 设备管理系统
- ✅ **安全控制**: 包含归位和安全检查功能

## 🚀 快速启动

### 一键启动 (推荐)
```bash
# 检查系统状态
./start_dummy2_unilab.sh check

# 启动硬件接口 (终端1)
./start_dummy2_unilab.sh hw

# 运行控制测试 (终端2)  
./start_dummy2_unilab.sh test
```

## 🎮 控制脚本

| 脚本 | 功能 | 状态 |
|------|------|------|
| `dummy2_direct_move.py` | 直接关节控制 | ✅ 已验证 |
| `force_home.py` | 强制归位控制 | ✅ 可用 |
| `test_complete_integration.py` | Unilab 集成测试 | ✅ 可用 |
| `dummy2_move_demo.py` | 移动演示 | ✅ 可用 |
| `final_demo.py` | 综合演示 | ✅ 可用 |

## 📁 文件说明 (9个核心文件)
- `README.md`: 项目说明文档
- `start_dummy2_unilab.sh`: 统一启动脚本
- `UNILAB_STARTUP_GUIDE.md`: 启动指南
- `FINAL_OPERATION_GUIDE.md`: 完整操作手册
- `dummy2_direct_move.py`: 直接关节控制 ✅
- `force_home.py`: 强制归位控制 ✅
- `test_complete_integration.py`: Unilab集成测试 ✅
- `dummy2_move_demo.py`: 移动演示脚本
- `final_demo.py`: 综合演示脚本

## 🎉 项目状态
**移植完成度: 100%** - 机械臂已成功响应控制命令，Unilab 集成完成。
