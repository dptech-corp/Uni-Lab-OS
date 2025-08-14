# Dummy2 机械臂完整操作手册

## 🎯 项目状态
**✅ 移植完成度: 100%**
- 设备成功注册到Unilab系统
- 机械臂响应控制命令（已验证实际移动）
- 所有服务在unilab环境中正常运行

## 🚀 操作流程

### 标准启动 (推荐)
```bash
cd /home/hh/Uni-Lab-OS/dummy2_debug

# 1. 检查系统
./start_dummy2_unilab.sh check

# 2. 启动硬件 (终端1)
./start_dummy2_unilab.sh hw

# 3. 运行测试 (终端2)
./start_dummy2_unilab.sh test
```

### 可选MoveIt服务
```bash
# MoveIt规划服务 (终端3)
./start_dummy2_unilab.sh moveit
```

## 🎮 控制方式

### 1. 直接关节控制 (已验证)
```bash
python dummy2_direct_move.py
```

### 2. Unilab集成控制
```bash
python test_complete_integration.py  
```

### 3. 安全归位控制
```bash
python force_home.py
```

## 📁 项目文件 (精简后)

```
dummy2_debug/
├── README.md                      # 项目说明
├── UNILAB_STARTUP_GUIDE.md        # 启动指南
├── FINAL_OPERATION_GUIDE.md       # 操作手册
├── start_dummy2_unilab.sh         # 统一启动脚本
├── dummy2_direct_move.py          # 直接控制 ✅
├── force_home.py                  # 归位控制 ✅
├── test_complete_integration.py   # 集成测试 ✅
├── dummy2_move_demo.py            # 移动演示
└── final_demo.py                  # 综合演示
```

## ⚙️ 技术要点
- **环境**: 完全在unilab conda环境中运行
- **通信**: CAN2ETH网络协议
- **框架**: Unilab设备管理系统集成
- **控制**: 支持直接控制和MoveIt2规划

## 🔧 故障处理
```bash
# 检查环境
mamba activate unilab && which ros2

# 检查服务
./start_dummy2_unilab.sh check

# 紧急归位  
python force_home.py
```

## 🎉 成功指标
- ROS2节点正常运行 (12个节点)
- 机械臂响应控制命令
- Unilab框架集成完成
- 所有测试脚本可用
