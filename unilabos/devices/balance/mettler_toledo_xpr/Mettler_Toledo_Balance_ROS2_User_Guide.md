# 梅特勒天平 ROS2 使用指南 / Mettler Toledo Balance ROS2 User Guide

## 概述 / Overview

梅特勒托利多XPR/XSR天平驱动支持通过ROS2动作进行操作，包括去皮、清零、读取重量等功能。

The Mettler Toledo XPR/XSR balance driver supports operations through ROS2 actions, including tare, zero, weight reading, and other functions.

## 主要功能 / Main Features

### 1. 去皮操作 / Tare Operation (`tare`)

- **功能 / Function**: 执行天平去皮操作 / Perform balance tare operation
- **输入 / Input**: `{"immediate": bool}` - 是否立即去皮 / Whether to tare immediately
- **输出 / Output**: `{"return_info": str, "success": bool}`

### 2. 清零操作 / Zero Operation (`zero`)

- **功能 / Function**: 执行天平清零操作 / Perform balance zero operation
- **输入 / Input**: `{"immediate": bool}` - 是否立即清零 / Whether to zero immediately
- **输出 / Output**: `{"return_info": str, "success": bool}`

### 3. 读取重量 / Read Weight (`read` / `get_weight`)

- **功能 / Function**: 读取当前天平重量 / Read current balance weight
- **输入 / Input**: 无参数 / No parameters
- **输出 / Output**: `{"return_info": str, "success": bool}` - 包含重量信息 / Contains weight information



## 使用方法 / Usage Methods

### ROS2命令行使用 / ROS2 Command Line Usage

### 1. 去皮操作 / Tare Operation

```bash
ros2 action send_goal /devices/BALANCE_STATION/send_cmd unilabos_msgs/action/SendCmd "{
  command: '{\"command\": \"tare\", \"params\": {\"immediate\": false}}'
}"
```

### 2. 清零操作 / Zero Operation

```bash
ros2 action send_goal /devices/BALANCE_STATION/send_cmd unilabos_msgs/action/SendCmd "{
  command: '{\"command\": \"zero\", \"params\": {\"immediate\": false}}'
}"
```

### 3. 读取重量 / Read Weight

```bash
ros2 action send_goal /devices/BALANCE_STATION/send_cmd unilabos_msgs/action/SendCmd "{
  command: '{\"command\": \"read\"}'
}"
```


### 4. 推荐的去皮读取流程 / Recommended Tare and Read Workflow

**步骤1: 去皮操作 / Step 1: Tare Operation**
```bash
# 放置空容器后执行去皮 / Execute tare after placing empty container
ros2 action send_goal /devices/BALANCE_STATION/send_cmd unilabos_msgs/action/SendCmd "{
  command: '{\"command\": \"tare\", \"params\": {\"immediate\": false}}'
}"
```

**步骤2: 读取净重 / Step 2: Read Net Weight**
```bash
# 添加物质后读取净重 / Read net weight after adding substance
ros2 action send_goal /devices/BALANCE_STATION/send_cmd unilabos_msgs/action/SendCmd "{
  command: '{\"command\": \"read\"}'
}"
```

**优势 / Advantages**:
- 可以在去皮和读取之间进行确认 / Can confirm between taring and reading
- 更好的错误处理和调试 / Better error handling and debugging
- 操作流程更加清晰 / Clearer operation workflow



## 命令格式说明 / Command Format Description

所有命令都使用JSON格式，包含以下字段 / All commands use JSON format with the following fields：

```json
{
  "command": "命令名称 / Command name",
  "params": {
    "参数名 / Parameter name": "参数值 / Parameter value"
  }
}
```

**注意事项 / Notes：**
1. JSON字符串需要正确转义引号 / JSON strings need proper quote escaping
2. 布尔值使用小写（true/false）/ Boolean values use lowercase (true/false)
3. 如果命令不需要参数，可以省略`params`字段 / If command doesn't need parameters, `params` field can be omitted

## 返回结果 / Return Results

所有命令都会返回包含以下字段的结果 / All commands return results with the following fields：

- `success`: 布尔值，表示操作是否成功 / Boolean value indicating operation success
- `return_info`: 字符串，包含操作结果的详细信息 / String containing detailed operation result information

## 成功执行示例 / Successful Execution Example

以下是一个成功执行读取重量命令的示例 / Here is an example of successfully executing a weight reading command：

```bash
ros2 action send_goal /devices/BALANCE_STATION/send_cmd unilabos_msgs/action/SendCmd "{
  command: '{\"command\": \"read\"}'
}"
```

**成功返回结果 / Successful Return Result：**
```
Waiting for an action server to become available...
Sending goal:
     command: '{"command": "read"}'

Goal accepted :)

Result:
    success: True
    return_info: Weight: 0.24866 Milligram

Goal finished with status: SUCCEEDED
```

### Python代码使用 / Python Code Usage

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from unilabos_msgs.action import SendCmd
import json

class BalanceController(Node):
    """梅特勒天平控制器 / Mettler Balance Controller"""
    def __init__(self):
        super().__init__('balance_controller')
        self._action_client = ActionClient(self, SendCmd, '/devices/BALANCE_STATION/send_cmd')
    
    def send_command(self, command, params=None):
        """发送命令到天平 / Send command to balance"""
        goal_msg = SendCmd.Goal()
        
        cmd_data = {'command': command}
        if params:
            cmd_data['params'] = params
            
        goal_msg.command = json.dumps(cmd_data)
        
        self._action_client.wait_for_server()
        future = self._action_client.send_goal_async(goal_msg)
        
        return future
    
    def tare_balance(self, immediate=False):
        """去皮操作 / Tare operation"""
        return self.send_command('tare', {'immediate': immediate})
    
    def zero_balance(self, immediate=False):
        """清零操作 / Zero operation"""
        return self.send_command('zero', {'immediate': immediate})
    
    def read_weight(self):
        """读取重量 / Read weight"""
        return self.send_command('read')
    



# 使用示例 / Usage Example
def main():
    rclpy.init()
    controller = BalanceController()
    
    # 去皮操作 / Tare operation
    future = controller.tare_balance(immediate=False)
    rclpy.spin_until_future_complete(controller, future)
    result = future.result().result
    print(f"去皮结果 / Tare result: {result.success}, 信息 / Info: {result.return_info}")
    
    # 读取重量 / Read weight
    future = controller.read_weight()
    rclpy.spin_until_future_complete(controller, future)
    result = future.result().result
    print(f"读取结果 / Read result: {result.success}, 信息 / Info: {result.return_info}")
    
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 使用注意事项 / Usage Notes

1. **设备连接 / Device Connection**: 确保梅特勒天平设备已连接并可访问 / Ensure Mettler balance device is connected and accessible
2. **命令格式 / Command Format**: JSON字符串需要正确转义引号 / JSON strings need proper quote escaping
3. **参数类型 / Parameter Types**: 布尔值使用小写（true/false）/ Boolean values use lowercase (true/false)
4. **权限 / Permissions**: 确保有操作天平的权限 / Ensure you have permission to operate the balance

## 故障排除 / Troubleshooting

### 常见问题 / Common Issues

1. **JSON格式错误 / JSON Format Error**: 确保JSON字符串格式正确且引号已转义 / Ensure JSON string format is correct and quotes are escaped
2. **未知命令名称 / Unknown Command Name**: 检查命令名称是否正确 / Check if command name is correct
3. **设备连接失败 / Device Connection Failed**: 检查网络连接和设备状态 / Check network connection and device status
4. **操作超时 / Operation Timeout**: 检查设备是否响应正常 / Check if device is responding normally

### 错误处理 / Error Handling

如果命令执行失败，返回结果中的`success`字段将为`false`，`return_info`字段将包含错误信息。

If command execution fails, the `success` field in the return result will be `false`, and the `return_info` field will contain error information.

### 调试技巧 / Debugging Tips

1. 检查设备节点是否正在运行 / Check if device node is running：
   ```bash
   ros2 node list | grep BALANCE
   ```

2. 查看可用的action / View available actions：
   ```bash
   ros2 action list | grep BALANCE
   ```

3. 检查action接口 / Check action interface：
   ```bash
   ros2 action info /devices/BALANCE_STATION/send_cmd
   ```

4. 查看节点日志 / View node logs：
   ```bash
   ros2 topic echo /rosout
   ```

## 总结 / Summary

梅特勒托利多天平设备现在支持 / Mettler Toledo balance device now supports:

1. 通过ROS2 SendCmd动作进行统一操作 / Unified operations through ROS2 SendCmd actions
2. 完整的天平功能支持（去皮、清零、读重等）/ Complete balance function support (tare, zero, weight reading, etc.)
3. 完善的错误处理和日志记录 / Comprehensive error handling and logging
4. 简化的操作流程和调试方法 / Simplified operation workflow and debugging methods