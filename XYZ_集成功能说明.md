# XYZ步进电机与移液器集成功能说明

## 概述

本文档描述了XYZ步进电机控制器与SOPA移液器的集成功能，实现了移液器在Z轴方向的精确运动控制，特别是在吸头装载过程中的自动定位功能。

## 新增功能

### 1. XYZ控制器集成

#### 初始化参数
```python
controller = PipetteController(
    port="/dev/ttyUSB0",      # 移液器串口
    address=4,                # 移液器Modbus地址
    xyz_port="/dev/ttyUSB1"   # XYZ控制器串口（可选）
)
```

#### 连接管理
- 自动检测并连接XYZ步进电机控制器
- 支持独立的移液器操作（不依赖XYZ控制器）
- 提供连接状态查询功能

### 2. Z轴运动控制

#### 相对运动功能
```python
# 向下移动10mm（用于吸头装载）
success = controller.move_z_relative(-10.0, speed=2000, acceleration=500)

# 向上移动5mm
success = controller.move_z_relative(5.0, speed=1500)
```

#### 参数说明
- `distance_mm`: 移动距离（毫米），负值向下，正值向上
- `speed`: 运动速度（100-5000 rpm）
- `acceleration`: 加速度（默认500）

#### 步距转换
- 转换比例: 1mm = 1638.4步
- 支持精确的毫米到步数转换
- 自动处理位置计算和验证

### 3. 安全检查机制

#### 运动限制
- **移动距离限制**: 单次移动最大15mm
- **速度限制**: 100-5000 rpm
- **位置限制**: Z轴位置范围 -50000 到 50000步
- **单次移动步数限制**: 最大20000步（约12mm）

#### 安全检查功能
```python
def _check_xyz_safety(self, axis: MotorAxis, target_position: int) -> bool:
    """执行XYZ轴运动前的安全检查"""
    # 检查电机使能状态
    # 检查错误状态
    # 验证位置限制
    # 验证移动距离
```

### 4. 增强的吸头装载功能

#### 自动Z轴定位
```python
def pickup_tip(self) -> bool:
    """装载吸头（包含Z轴自动定位）"""
    # 检查当前吸头状态
    # 执行Z轴下降运动（10mm）
    # 执行移液器吸头装载
    # 更新吸头状态
```

#### 工作流程
1. 检查是否已有吸头装载
2. 如果配置了XYZ控制器，执行Z轴下降10mm
3. 执行移液器的吸头装载动作
4. 更新吸头状态和统计信息

### 5. 紧急停止功能

#### 全系统停止
```python
def emergency_stop(self) -> bool:
    """紧急停止所有运动"""
    # 停止移液器运动
    # 停止XYZ步进电机运动
    # 记录停止状态
```

#### 使用场景
- 检测到异常情况时立即停止
- 用户手动中断操作
- 系统故障保护

### 6. 状态监控

#### 设备状态查询
```python
status = controller.get_status()
# 返回包含移液器和XYZ控制器状态的字典
{
    'pipette': {
        'connected': True,
        'tip_status': 'tip_attached',
        'current_volume': 0.0,
        # ...
    },
    'xyz_controller': {
        'connected': True,
        'port': '/dev/ttyUSB1'
    }
}
```

## 使用示例

### 基本使用流程

```python
from unilabos.devices.LaiYu_Liquid.controllers.pipette_controller import PipetteController

# 1. 创建控制器实例
controller = PipetteController(
    port="/dev/ttyUSB0",
    address=4,
    xyz_port="/dev/ttyUSB1"  # 可选
)

# 2. 连接设备
if controller.connect():
    print("设备连接成功")
    
    # 3. 初始化
    if controller.initialize():
        print("设备初始化成功")
        
        # 4. 装载吸头（自动Z轴定位）
        if controller.pickup_tip():
            print("吸头装载成功")
            
            # 5. 执行液体操作
            controller.aspirate(100.0)  # 吸取100μL
            controller.dispense(100.0)  # 排出100μL
            
            # 6. 弹出吸头
            controller.eject_tip()
    
    # 7. 断开连接
    controller.disconnect()
```

### 手动Z轴控制

```python
# 精确的Z轴运动控制
controller.move_z_relative(-5.0, speed=1000)  # 下降5mm
time.sleep(1)  # 等待运动完成
controller.move_z_relative(5.0, speed=1000)   # 上升5mm
```

## 集成测试

### 测试脚本
使用 `test_xyz_pipette_integration.py` 脚本进行完整的集成测试：

```bash
python test_xyz_pipette_integration.py
```

### 测试项目
1. **连接状态测试** - 验证设备连接
2. **Z轴运动测试** - 验证运动控制
3. **吸头装载测试** - 验证集成功能
4. **安全检查测试** - 验证安全机制
5. **紧急停止测试** - 验证停止功能
6. **液体操作测试** - 验证基本功能

## 配置要求

### 硬件要求
- SOPA移液器（支持Modbus RTU通信）
- XYZ步进电机控制器（可选）
- 串口连接线

### 软件依赖
- Python 3.7+
- pymodbus库
- pyserial库

### 串口配置
- 波特率: 9600
- 数据位: 8
- 停止位: 1
- 校验位: None

## 注意事项

### 安全提醒
1. **首次使用前必须进行安全检查**
2. **确保Z轴运动范围内无障碍物**
3. **紧急情况下立即使用紧急停止功能**
4. **定期检查设备连接状态**

### 故障排除
1. **连接失败**: 检查串口配置和设备电源
2. **运动异常**: 检查电机使能状态和位置限制
3. **吸头装载失败**: 检查Z轴位置和吸头供应
4. **通信错误**: 检查Modbus地址和通信参数

## 更新记录

- **2024-01**: 初始版本，实现基本集成功能
- **2024-01**: 添加安全检查和紧急停止功能
- **2024-01**: 完善错误处理和状态监控

## 技术支持

如有技术问题，请联系开发团队或查阅相关技术文档。