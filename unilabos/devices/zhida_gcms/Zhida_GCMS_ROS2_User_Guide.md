# 智达GCMS ROS2使用指南 / Zhida GCMS ROS2 User Guide

## 概述 / Overview

智达GCMS设备支持通过ROS2动作进行操作，包括CSV文件分析启动、设备状态查询等功能。

The Zhida GCMS device supports operations through ROS2 actions, including CSV file analysis startup, device status queries, and other functions.

## 主要功能 / Main Features

### 1. CSV文件分析启动 / CSV File Analysis Startup (`start_with_csv_file`)

- **功能 / Function**: 接收CSV文件路径，自动读取文件内容并启动分析 / Receives CSV file path, automatically reads file content and starts analysis
- **输入 / Input**: CSV文件的绝对路径 / Absolute path of CSV file
- **输出 / Output**: `{"return_info": str, "success": bool}`

### 2. 设备状态查询 / Device Status Query (`get_status`)

- **功能 / Function**: 获取设备当前运行状态 / Get current device running status
- **输出 / Output**: 设备状态字符串（如"RunSample"、"Idle"等）/ Device status string (e.g., "RunSample", "Idle", etc.)

### 3. 方法列表查询 / Method List Query (`get_methods`)

- **功能 / Function**: 获取设备支持的所有方法列表 / Get all method lists supported by the device
- **输出 / Output**: 方法列表字典 / Method list dictionary

### 4. 放盘操作 / Tray Operation (`put_tray`)

- **功能 / Function**: 控制设备准备样品托盘 / Control device to prepare sample tray
- **输出 / Output**: 操作结果信息 / Operation result information

### 5. 停止运行 / Stop Operation (`abort`)

- **功能 / Function**: 中止当前正在进行的分析任务 / Abort current analysis task in progress
- **输出 / Output**: 操作结果信息 / Operation result information

### 6. 获取版本信息 / Get Version Information (`get_version`)

- **功能 / Function**: 查询设备接口版本和固件版本信息 / Query device interface version and firmware version information
- **输出 / Output**: 版本信息字典 / Version information dictionary

## 使用方法 / Usage Methods

### ROS2命令行使用 / ROS2 Command Line Usage

### 1. 查询设备状态 / Query Device Status

```bash
ros2 action send_goal /devices/ZHIDA_GCMS_STATION/get_status unilabos_msgs/action/EmptyIn "{}"
```

### 2. 查询方法列表 / Query Method List

```bash
ros2 action send_goal /devices/ZHIDA_GCMS_STATION/get_methods unilabos_msgs/action/EmptyIn "{}"
```

### 3. 启动分析 / Start Analysis

使用CSV文件启动分析 / Start analysis using CSV file:
```bash
ros2 action send_goal /devices/ZHIDA_GCMS_STATION/start_with_csv_file unilabos_msgs/action/StrSingleInput "{string: 'D:/path/to/your/samples.csv'}"
```

ros2 action send_goal /devices/ZHIDA_GCMS_STATION/start_with_csv_file unilabos_msgs/action/StrSingleInput "{string: 'd:/UniLab/Uni-Lab-OS/unilabos/devices/zhida_gcms/zhida_gcms-test_3.csv'}"

使用Base64编码数据启动分析 / Start analysis using Base64 encoded data:
```bash
ros2 action send_goal /devices/ZHIDA_GCMS_STATION/start unilabos_msgs/action/StrSingleInput "{string: 'U2FtcGxlTmFtZSxBY3FNZXRob2QsUmFja0NvZGUsVmlhbFBvcyxTbXBsSW5qVm9sLE91dHB1dEZpbGU...'}"
```
ros2 action send_goal /devices/ZHIDA_GCMS_STATION/start unilabos_msgs/action/StrSingleInput "{string: 'U2FtcGxlTmFtZSxBY3FNZXRob2QsUmFja0NvZGUsVmlhbFBvcyxTbXBsSW5qVm9sLE91dHB1dEZpbGUKU2FtcGxlMDAxLDIwMjUwNjA0LXRlc3QsUmFjayAxLDEsMSwvQ2hyb21lbGVvbkxvY2FsL++/veixuO+/vcSyxLzvv73vv73vv73vv73vv73vv73vv73vv73vv70vMjAyNTA2MDQK'}"

### 4. 放盘操作 / Tray Operation

**注意 / Note**: 放盘操作是特殊场景下使用的功能，比如机械臂比较短需要让开位置，或者盘支架是可移动的时候，这个指令让进样器也去做相应动作。在当前配置中，空间足够，不需要这个额外的控制组件。

**Note**: The tray operation is used in special scenarios, such as when the robotic arm is relatively short and needs to make room, or when the tray bracket is movable, this command makes the injector perform corresponding actions. In the current configuration, the space is sufficient and this additional control component is not needed.

准备样品托盘 / Prepare sample tray:
```bash
ros2 action send_goal /devices/ZHIDA_GCMS_STATION/put_tray unilabos_msgs/action/EmptyIn "{}"
```

### 5. 停止运行 / Stop Operation

中止当前分析任务（注意！运行中发现任务运行中止，需要人工在InLab Solution 二次点击确认）/ Abort current analysis task (Note! If task abortion is detected during operation, manual confirmation is required by clicking twice in InLab Solution):
```bash
ros2 action send_goal /devices/ZHIDA_GCMS_STATION/abort unilabos_msgs/action/EmptyIn "{}"
```

### 6. 获取版本信息 / Get Version Information

查询设备版本 / Query device version:
```bash
ros2 action send_goal /devices/ZHIDA_GCMS_STATION/get_version unilabos_msgs/action/EmptyIn "{}"
```

### Python代码使用 / Python Code Usage

```python
from unilabos.devices.zhida_gcms.zhida import ZhidaClient

# 初始化客户端 / Initialize client
client = ZhidaClient(host='192.168.3.184', port=5792)
client.connect()

# 使用CSV文件启动分析 / Start analysis using CSV file
result = client.start_with_csv_file('/path/to/your/file.csv')
print(f"成功 / Success: {result['success']}")
print(f"信息 / Info: {result['return_info']}")

# 查询设备状态 / Query device status
status = client.get_status()
print(f"设备状态 / Device Status: {status}")

client.close()
```

## 使用注意事项 / Usage Notes

1. **文件路径 / File Path**: 必须使用绝对路径 / Must use absolute path
2. **文件格式 / File Format**: CSV文件必须是UTF-8编码 / CSV file must be UTF-8 encoded
3. **设备连接 / Device Connection**: 确保智达GCMS设备已连接并可访问 / Ensure Zhida GCMS device is connected and accessible
4. **权限 / Permissions**: 确保有读取CSV文件的权限 / Ensure you have permission to read CSV files

## 故障排除 / Troubleshooting

### 常见问题 / Common Issues

1. **文件路径错误 / File Path Error**: 确保使用绝对路径且文件存在 / Ensure using absolute path and file exists
2. **编码问题 / Encoding Issue**: 确保CSV文件是UTF-8编码 / Ensure CSV file is UTF-8 encoded
3. **设备连接 / Device Connection**: 检查网络连接和设备状态 / Check network connection and device status
4. **权限问题 / Permission Issue**: 确保有文件读取权限 / Ensure you have file read permissions

### 设备状态说明 / Device Status Description

- `"Idle"`: 设备空闲状态 / Device idle status
- `"RunSample"`: 正在运行样品分析 / Running sample analysis
- `"Error"`: 设备错误状态 / Device error status

## 总结 / Summary

智达GCMS设备现在支持 / Zhida GCMS device now supports:

1. 直接通过ROS2命令输入CSV文件路径启动分析 / Direct CSV file path input via ROS2 commands to start analysis
2. 按需查询设备状态和方法列表 / On-demand device status and method list queries
3. 完善的错误处理和日志记录 / Comprehensive error handling and logging
4. 简化的操作流程 / Simplified operation workflow