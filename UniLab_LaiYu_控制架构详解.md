# UniLab 控制 LaiYu_Liquid 设备架构详解

## 概述

UniLab 通过分层架构控制 LaiYu_Liquid 设备，实现了从高级实验协议到底层硬件驱动的完整控制链路。

## 🏗️ 架构层次

### 1. 应用层 (Application Layer)
- **实验协议**: 用户定义的实验流程
- **设备抽象**: 通过 `LaiYu_Liquid` 类提供统一接口

### 2. 控制层 (Control Layer)
- **LaiYuLiquidBackend**: 设备业务逻辑控制器
- **PipetteController**: 移液器控制器
- **XYZController**: 三轴运动控制器

### 3. 驱动层 (Driver Layer)
- **SOPAPipetteDriver**: SOPA 移液器驱动
- **XYZStepperDriver**: 三轴步进电机驱动

### 4. 硬件层 (Hardware Layer)
- **串口通信**: 通过 `/dev/cu.usbserial-3130` 等串口设备
- **物理设备**: 移液器和三轴运动平台

## 🔧 核心组件详解

### LaiYu_Liquid 主类
```python
class LaiYu_Liquid:
    """LaiYu液体处理设备的主要接口类"""
    
    def __init__(self, config: LaiYuLiquidConfig):
        self.config = config
        self.deck = LaiYuLiquidDeck(config)
        self.backend = LaiYuLiquidBackend(config, self.deck)
```

**核心功能**:
- 设备配置管理
- 工作台资源管理
- 硬件控制接口封装

### LaiYuLiquidBackend 控制器
```python
class LaiYuLiquidBackend:
    """设备后端控制逻辑"""
    
    async def setup(self):
        """初始化硬件控制器"""
        # 初始化移液器控制器
        self.pipette_controller = PipetteController(
            port=self.config.port,
            address=self.config.address
        )
        
        # 初始化XYZ控制器
        self.xyz_controller = XYZController(
            port=self.config.port,
            baudrate=self.config.baudrate,
            machine_config=MachineConfig()
        )
```

**核心功能**:
- 硬件初始化和连接
- 移液操作控制
- 运动控制
- 错误处理和状态管理

## 🎯 控制流程

### 1. 设备初始化流程
```
用户代码 → LaiYu_Liquid.setup() → LaiYuLiquidBackend.setup()
    ↓
PipetteController 初始化 ← → XYZController 初始化
    ↓                           ↓
SOPAPipetteDriver.connect()    XYZStepperDriver.connect()
    ↓                           ↓
串口连接 (/dev/cu.usbserial-3130)
```

### 2. 移液操作流程
```
用户调用 aspirate(volume) → LaiYuLiquidBackend.aspirate()
    ↓
检查设备状态 (连接、枪头、体积)
    ↓
PipetteController.aspirate(volume)
    ↓
SOPAPipetteDriver.aspirate_volume()
    ↓
串口命令发送到移液器硬件
```

### 3. 运动控制流程
```
用户调用 move_to(position) → LaiYuLiquidBackend.move_to()
    ↓
坐标转换和安全检查
    ↓
XYZController.move_to_work_coord()
    ↓
XYZStepperDriver.move_to()
    ↓
串口命令发送到步进电机
```

## 🔌 硬件通信

### 串口配置
- **端口**: `/dev/cu.usbserial-3130` (macOS)
- **波特率**: 115200 (移液器), 可配置 (XYZ控制器)
- **协议**: SOPA协议 (移液器), 自定义协议 (XYZ)

### 通信协议
1. **SOPA移液器协议**:
   - 地址寻址: `address` 参数
   - 命令格式: 二进制协议
   - 响应处理: 异步等待

2. **XYZ步进电机协议**:
   - G代码风格命令
   - 坐标系管理
   - 实时状态反馈

## 🛡️ 安全机制

### 1. 连接检查
```python
def _check_hardware_ready(self):
    """检查硬件是否就绪"""
    if not self.is_connected:
        raise DeviceError("设备未连接")
    if not self.is_initialized:
        raise DeviceError("设备未初始化")
```

### 2. 状态验证
- 移液前检查枪头状态
- 体积范围验证
- 位置边界检查

### 3. 错误处理
- 硬件连接失败自动切换到模拟模式
- 异常捕获和日志记录
- 优雅的错误恢复

## 📊 配置管理

### LaiYuLiquidConfig
```python
@dataclass
class LaiYuLiquidConfig:
    port: str = "/dev/cu.usbserial-3130"
    address: int = 1
    baudrate: int = 115200
    max_volume: float = 1000.0
    min_volume: float = 0.1
    # ... 其他配置参数
```

### 配置文件支持
- **YAML**: `laiyu_liquid.yaml`
- **JSON**: `laiyu_liquid.json`
- **环境变量**: 动态配置覆盖

## 🔄 异步操作

所有硬件操作都是异步的，支持:
- 并发操作
- 非阻塞等待
- 超时处理
- 取消操作

```python
async def aspirate(self, volume: float) -> bool:
    """异步吸液操作"""
    try:
        # 硬件操作
        result = await self.pipette_controller.aspirate(volume)
        # 状态更新
        self._update_volume_state(volume)
        return result
    except Exception as e:
        logger.error(f"吸液失败: {e}")
        return False
```

## 🎮 实际使用示例

```python
# 1. 创建设备配置
config = LaiYuLiquidConfig(
    port="/dev/cu.usbserial-3130",
    address=1,
    baudrate=115200
)

# 2. 初始化设备
device = LaiYu_Liquid(config)
await device.setup()

# 3. 执行移液操作
await device.pick_up_tip()
await device.aspirate(100.0)  # 吸取100μL
await device.move_to((50, 50, 10))  # 移动到目标位置
await device.dispense(100.0)  # 分配100μL
await device.drop_tip()

# 4. 清理
await device.stop()
```

## 🔍 调试和监控

### 日志系统
- 详细的操作日志
- 错误追踪
- 性能监控

### 状态查询
```python
# 实时状态查询
print(f"连接状态: {device.is_connected}")
print(f"当前位置: {device.current_position}")
print(f"当前体积: {device.current_volume}")
print(f"枪头状态: {device.tip_attached}")
```

## 📈 扩展性

### 1. 新设备支持
- 继承抽象基类
- 实现标准接口
- 插件式架构

### 2. 协议扩展
- 新通信协议支持
- 自定义命令集
- 协议适配器

### 3. 功能扩展
- 新的移液模式
- 高级运动控制
- 智能优化算法

## 🎯 总结

UniLab 通过精心设计的分层架构，实现了对 LaiYu_Liquid 设备的完整控制:

1. **高层抽象**: 提供简洁的API接口
2. **中层控制**: 实现复杂的业务逻辑
3. **底层驱动**: 处理硬件通信细节
4. **安全可靠**: 完善的错误处理机制
5. **易于扩展**: 模块化设计支持功能扩展

这种架构使得用户可以专注于实验逻辑，而无需关心底层硬件控制的复杂性。