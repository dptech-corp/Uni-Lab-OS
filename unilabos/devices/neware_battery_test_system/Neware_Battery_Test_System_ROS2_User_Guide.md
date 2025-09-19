# 新威电池测试系统 ROS2 使用指南 / Neware Battery Test System ROS2 User Guide

## 概述 / Overview

新威电池测试系统是一个支持720个通道的电池测试设备，通过TCP通信协议实现远程监控和控制。该系统可以实时获取电池测试状态、导出数据，并支持多设备管理。

The Neware Battery Test System is a battery testing device that supports 720 channels, enabling remote monitoring and control through TCP communication protocol. The system can obtain battery test status in real-time, export data, and supports multi-device management.

## 设备特性 / Device Features

- **通道数量 / Channel Count**: 支持720个测试通道 / Supports 720 test channels
- **通信协议 / Communication Protocol**: TCP/IP
- **数据格式 / Data Format**: JSON格式状态导出 / JSON format status export
- **设备管理 / Device Management**: 支持多设备ID管理 / Supports multi-device ID management
- **实时监控 / Real-time Monitoring**: 实时获取通道状态和设备摘要 / Real-time channel status and device summary

## 硬件连接 / Hardware Connection

### 网络连接 / Network Connection
1. 确保设备已连接到网络 / Ensure device is connected to network
2. 获取设备的IP地址（默认：127.0.0.1）/ Get device IP address (default: 127.0.0.1)
3. 确认通信端口（默认：502）/ Confirm communication port (default: 502)
4. 测试网络连通性 / Test network connectivity

### 设备配置 / Device Configuration
- **IP地址 / IP Address**: 设备的网络IP地址 / Device network IP address
- **端口 / Port**: TCP通信端口 / TCP communication port
- **机器ID / Machine ID**: 多设备环境下的设备标识 / Device identifier in multi-device environment
- **设备类型 / Device Type**: 设备类型标识符（默认："27"）/ Device type identifier (default: "27")
- **超时时间 / Timeout**: TCP通信超时时间（默认：20秒）/ TCP communication timeout (default: 20 seconds)

## ROS2 节点配置 / ROS2 Node Configuration

### 启动参数 / Launch Parameters

```yaml
ip: "127.0.0.1"          # 设备IP地址 / Device IP address
port: 502                # TCP端口 / TCP port
machine_id: 1            # 机器ID / Machine ID
devtype: "27"            # 设备类型 / Device type
timeout: 20              # 超时时间（秒）/ Timeout (seconds)
size_x: 500.0            # 物理尺寸X（毫米）/ Physical dimension X (mm)
size_y: 500.0            # 物理尺寸Y（毫米）/ Physical dimension Y (mm)
size_z: 2000.0           # 物理尺寸Z（毫米）/ Physical dimension Z (mm)
```

### 状态发布 / Status Publishing

系统会定期发布以下状态信息 / System periodically publishes the following status information:

- **channel_status** (dict): 所有通道的详细状态信息 / Detailed status information of all channels
- **connection_info** (dict): TCP连接信息 / TCP connection information
- **total_channels** (int): 总通道数量 / Total number of channels

## 可用动作 / Available Actions

新威电池测试系统支持以下ROS2动作：

The Neware Battery Test System supports the following ROS2 actions:

### 1. 测试连接 / Test Connection (`test_connection`)

**动作类型 / Action Type**: `unilabos_msgs/action/EmptyIn`

测试与电池测试系统的TCP连接状态。

Test TCP connection status with the battery test system.

**参数 / Parameters**: 无 / None

**返回 / Returns**:
- `return_info` (string): 连接测试结果信息 / Connection test result information
- `success` (boolean): 连接测试是否成功 / Whether connection test was successful

**ROS2命令行使用 / ROS2 Command Line Usage**:
```bash
ros2 action send_goal /devices/NEWARE_BATTERY_TEST_SYSTEM/test_connection unilabos_msgs/action/EmptyIn "{}"
```

**PowerShell使用 / PowerShell Usage**:
```powershell
ros2 action send_goal /devices/NEWARE_BATTERY_TEST_SYSTEM/test_connection unilabos_msgs/action/EmptyIn --% "{}"
```

### 2. 获取通道状态 / Get Channel Status (`get_channel_status`)

**动作类型 / Action Type**: `unilabos_msgs/action/EmptyIn`

获取所有电池测试通道的当前状态。

Get current status of all battery test channels.

**参数 / Parameters**: 无 / None

**返回 / Returns**:
- `return_info` (string): JSON格式的通道状态信息 / Channel status information in JSON format
- `success` (boolean): 状态查询是否成功 / Whether status query was successful

**ROS2命令行使用 / ROS2 Command Line Usage**:
```bash
ros2 action send_goal /devices/NEWARE_BATTERY_TEST_SYSTEM/get_channel_status unilabos_msgs/action/EmptyIn "{}"
```

**PowerShell使用 / PowerShell Usage**:
```powershell
ros2 action send_goal /devices/NEWARE_BATTERY_TEST_SYSTEM/get_channel_status unilabos_msgs/action/EmptyIn --% "{}"
```

### 3. 获取板状态 / Get Plate Status (`get_plate_status`)

**动作类型 / Action Type**: `unilabos_msgs/action/IntSingleInput`

获取指定电池板（1或2）的状态信息。

Get status information of specific battery plate (1 or 2).

**参数 / Parameters**:
- `int_input` (integer): 板号（1或2）/ Plate number (1 or 2)

**返回 / Returns**:
- `return_info` (string): JSON格式的板状态信息 / Plate status information in JSON format
- `success` (boolean): 板状态查询是否成功 / Whether plate status query was successful

**ROS2命令行使用 / ROS2 Command Line Usage**:
```bash
ros2 action send_goal /devices/NEWARE_BATTERY_TEST_SYSTEM/get_plate_status unilabos_msgs/action/IntSingleInput "{int_input: 1}"
```

**PowerShell使用 / PowerShell Usage**:
```powershell
ros2 action send_goal /devices/NEWARE_BATTERY_TEST_SYSTEM/get_plate_status unilabos_msgs/action/IntSingleInput --% "{int_input: 1}"
```

### 4. 获取设备摘要 / Get Device Summary (`get_device_summary`)

**动作类型 / Action Type**: `unilabos_msgs/action/EmptyIn`

获取设备摘要统计信息。

Get device summary statistics.

**参数 / Parameters**: 无 / None

**返回 / Returns**:
- `return_info` (string): JSON格式的设备摘要信息 / Device summary information in JSON format
- `success` (boolean): 摘要查询是否成功 / Whether summary query was successful

**ROS2命令行使用 / ROS2 Command Line Usage**:
```bash
ros2 action send_goal /devices/NEWARE_BATTERY_TEST_SYSTEM/get_device_summary unilabos_msgs/action/EmptyIn "{}"
```

**PowerShell使用 / PowerShell Usage**:
```powershell
ros2 action send_goal /devices/NEWARE_BATTERY_TEST_SYSTEM/get_device_summary unilabos_msgs/action/EmptyIn --% "{}"
```

### 5. 导出状态JSON / Export Status JSON (`export_status_json`)

**动作类型 / Action Type**: `unilabos_msgs/action/StrSingleInput`

将当前状态导出到JSON文件。

Export current status to JSON file.

**参数 / Parameters**:
- `string` (string): 输出文件路径 / Output file path for JSON export

**返回 / Returns**:
- `return_info` (string): 导出操作结果信息 / Export operation result information
- `success` (boolean): 导出是否成功 / Whether export was successful

**ROS2命令行使用 / ROS2 Command Line Usage**:
```bash
ros2 action send_goal /devices/NEWARE_BATTERY_TEST_SYSTEM/export_status_json unilabos_msgs/action/StrSingleInput "{string: 'bts_status.json'}"
```

**PowerShell使用 / PowerShell Usage**:
```powershell
ros2 action send_goal /devices/NEWARE_BATTERY_TEST_SYSTEM/export_status_json unilabos_msgs/action/StrSingleInput --% "{string: 'bts_status.json'}"
```

### 6. 发送命令 / Send Command (`send_cmd`)

**动作类型 / Action Type**: `unilabos_msgs/action/SendCmd`

发送自定义命令到设备。

Send custom command to device.

**参数 / Parameters**:
- `command` (string): 要发送的命令 / Command to send

**返回 / Returns**:
- `return_info` (string): 命令执行结果信息 / Command execution result information
- `success` (boolean): 命令执行是否成功 / Whether command execution was successful

**反馈 / Feedback**:
- `status` (string): 命令执行状态 / Command execution status

**ROS2命令行使用 / ROS2 Command Line Usage**:
```bash
ros2 action send_goal /devices/NEWARE_BATTERY_TEST_SYSTEM/send_cmd unilabos_msgs/action/SendCmd "{command: 'your_command_here'}"
```

**PowerShell使用 / PowerShell Usage**:
```powershell
ros2 action send_goal /devices/NEWARE_BATTERY_TEST_SYSTEM/send_cmd unilabos_msgs/action/SendCmd --% "{command: 'your_command_here'}"
```

## 推荐的监控流程 / Recommended Monitoring Workflow

以下是推荐的电池测试监控流程：

Here is the recommended battery test monitoring workflow:

### 1. 测试连接 / Test Connection
```bash
ros2 action send_goal /devices/NEWARE_BATTERY_TEST_SYSTEM/test_connection unilabos_msgs/action/EmptyIn "{}"
```

### 2. 获取设备摘要 / Get Device Summary
```bash
ros2 action send_goal /devices/NEWARE_BATTERY_TEST_SYSTEM/get_device_summary unilabos_msgs/action/EmptyIn "{}"
```

### 3. 获取第1板状态 / Get Plate 1 Status
```bash
ros2 action send_goal /devices/NEWARE_BATTERY_TEST_SYSTEM/get_plate_status unilabos_msgs/action/IntSingleInput "{int_input: 1}"
```

### 4. 获取第2板状态 / Get Plate 2 Status
```bash
ros2 action send_goal /devices/NEWARE_BATTERY_TEST_SYSTEM/get_plate_status unilabos_msgs/action/IntSingleInput "{int_input: 2}"
```

## 数据结构 / Data Structures

### 通道状态数据结构 / Channel Status Data Structure

每个通道包含以下信息 / Each channel contains the following information:
- 电压 / Voltage (V)
- 电流 / Current (A)
- 容量 / Capacity (Ah)
- 能量 / Energy (Wh)
- 总时间 / Total time (s)
- 相对时间 / Relative time (s)
- 通道状态 / Channel status
- 步骤类型 / Step type
- 循环ID / Cycle ID
- 步骤ID / Step ID

### 设备摘要数据结构 / Device Summary Data Structure

设备摘要包含 / Device summary contains：
- 总通道数 / Total channels
- 活跃通道数 / Active channels
- 空闲通道数 / Idle channels
- 错误通道数 / Error channels
- 系统状态 / System status

## 故障排除 / Troubleshooting

### 常见问题 / Common Issues

1. **动作类型无效 / Invalid Action Type**
   - 错误信息 / Error message："The passed action type is invalid"
   - 解决方案 / Solution：确保使用正确的动作类型：
     - `test_connection`, `get_channel_status`, `get_device_summary`: 使用 `unilabos_msgs/action/EmptyIn`
     - `get_plate_status`: 使用 `unilabos_msgs/action/IntSingleInput`
     - `export_status_json`: 使用 `unilabos_msgs/action/StrSingleInput`
     - `send_cmd`: 使用 `unilabos_msgs/action/SendCmd`

2. **连接失败 / Connection Failed**
   - 检查网络连接 / Check network connection
   - 验证IP地址和端口 / Verify IP address and port
   - 确认设备电源状态 / Confirm device power status
   - 检查防火墙设置 / Check firewall settings

3. **数据获取失败 / Data Retrieval Failed**
   - 检查TCP连接状态 / Check TCP connection status
   - 验证设备ID和类型 / Verify device ID and type
   - 检查超时设置 / Check timeout settings
   - 查看设备日志 / View device logs

4. **状态更新延迟 / Status Update Delay**
   - 检查网络延迟 / Check network latency
   - 调整超时参数 / Adjust timeout parameters
   - 验证设备负载 / Verify device load

5. **参数格式错误 / Parameter Format Error**
   - 确保JSON格式正确 / Ensure correct JSON format
   - 检查参数类型匹配 / Check parameter type matching
   - 验证必需参数 / Verify required parameters

### 调试技巧 / Debugging Tips

1. **查看动作服务器状态 / Check Action Server Status**:
```bash
ros2 action list
```

2. **查看动作类型信息 / Check Action Type Information**:
```bash
ros2 interface show unilabos_msgs/action/EmptyIn
```

3. **监控设备状态 / Monitor Device Status**:
```bash
ros2 topic echo /devices/NEWARE_BATTERY_TEST_SYSTEM/status
```

4. **查看ROS2日志 / View ROS2 Logs**:
```bash
ros2 log view
```

## 成功执行示例 / Successful Execution Example

以下是成功执行测试连接的示例输出：

Here is an example output of successful test connection execution:

```bash
$ ros2 action send_goal /devices/NEWARE_BATTERY_TEST_SYSTEM/test_connection unilabos_msgs/action/EmptyIn "{}"

Waiting for an action server to become available...
Sending goal:
     {}

Goal accepted :)

Result:
    success: True
    return_info: {"connection_status": "connected", "ip": "127.0.0.1", "port": 502, "response_time_ms": 15}

Goal finished with status: SUCCEEDED
```

## Python代码使用 / Python Code Usage

```python
from unilabos.devices.battery_test_system.neware_battery_test_system import NewareBatteryTestSystem

# 初始化设备 / Initialize device
bts = NewareBatteryTestSystem(
    ip='127.0.0.1',
    port=502,
    machine_id=1,
    devtype='27',
    timeout=20
)

# 测试连接 / Test connection
if bts.test_connection():
    print("连接成功 / Connection successful")
    
    # 获取设备摘要 / Get device summary
    summary = bts.get_device_summary()
    print(f"设备摘要 / Device summary: {summary}")
    
    # 获取通道状态 / Get channel status
    status = bts.get_channel_status()
    print(f"通道状态 / Channel status: {status}")
    
    # 获取板状态 / Get plate status
    plate1_status = bts.get_plate_status(1)
    print(f"第1板状态 / Plate 1 status: {plate1_status}")
else:
    print("连接失败 / Connection failed")
```

## 安全注意事项 / Security Considerations

1. **网络安全 / Network Security**
   - 使用安全的网络连接 / Use secure network connections
   - 限制设备访问权限 / Limit device access permissions
   - 定期更新设备固件 / Regularly update device firmware

2. **数据安全 / Data Security**
   - 定期备份测试数据 / Regularly backup test data
   - 验证数据完整性 / Verify data integrity
   - 保护敏感测试信息 / Protect sensitive test information

## 使用注意事项 / Usage Notes

1. **设备连接 / Device Connection**: 确保新威电池测试系统设备已连接并可通过TCP访问 / Ensure Neware Battery Test System device is connected and accessible via TCP
2. **网络配置 / Network Configuration**: 确保IP地址和端口配置正确 / Ensure IP address and port are configured correctly
3. **动作类型 / Action Type**: 注意使用正确的动作类型，不同动作使用不同的消息类型 / Pay attention to using the correct action type, different actions use different message types
4. **通道映射 / Channel Mapping**: 设备支持720个通道，使用devid->subdevid->chlid三级结构 / Device supports 720 channels using devid->subdevid->chlid three-level structure
5. **状态监控 / Status Monitoring**: 支持实时监控电池测试状态，包括电压、电流、时间等参数 / Supports real-time monitoring of battery test status including voltage, current, time and other parameters

## 总结 / Summary

新威电池测试系统设备现在支持 / Neware Battery Test System device now supports:

1. 通过ROS2标准动作类型进行统一操作 / Unified operations through ROS2 standard action types
2. 完整的电池测试功能支持（通道状态、板状态、设备摘要等）/ Complete battery test function support (channel status, plate status, device summary, etc.)
3. TCP协议通信，支持720个通道 / TCP protocol communication supporting 720 channels
4. 实时状态监控和数据导出 / Real-time status monitoring and data export
5. 完善的错误处理和日志记录 / Comprehensive error handling and logging
6. 简化的操作流程和调试方法 / Simplified operation workflow and debugging methods

**重要提醒 / Important Reminder**: 请使用正确的动作类型，不同的动作使用不同的消息类型（EmptyIn、IntSingleInput、StrSingleInput、SendCmd）。

**Important Reminder**: Please use the correct action types, different actions use different message types (EmptyIn, IntSingleInput, StrSingleInput, SendCmd).