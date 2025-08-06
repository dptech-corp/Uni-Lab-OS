# Mettler Toledo XPR/XSR Balance Driver

## 概述

本驱动程序为梅特勒托利多XPR/XSR系列天平提供标准接口，支持去皮、清零和重量读取等操作。

## ⚠️ 重要说明 - WSDL文件配置

### 问题说明

本驱动程序需要使用梅特勒托利多官方提供的WSDL文件来与天平通信。由于该WSDL文件包含专有信息，不能随开源项目一起分发。

### 配置步骤

1. **获取WSDL文件**
   - 联系梅特勒托利多技术支持
   - 或从您的天平设备Web界面下载
   - 或从梅特勒托利多官方SDK获取

2. **安装WSDL文件**
   ```bash
   # 将获取的WSDL文件复制到驱动目录
   cp /path/to/your/MT.Laboratory.Balance.XprXsr.V03.wsdl \
      unilabos/devices/balance/mettler_toledo_xpr/
   ```

3. **验证安装**
   - 确保文件名为：`MT.Laboratory.Balance.XprXsr.V03.wsdl`
   - 确保文件包含Jinja2模板变量：`{{host}}`、`{{port}}`、`{{api_path}}`

### WSDL文件要求

- 文件必须是有效的WSDL格式
- 必须包含SessionService和WeighingService的定义
- 端点地址应使用模板变量以支持动态IP配置：
  ```xml
  <soap:address location="http://{{host}}:{{port}}/{{api_path}}/SessionService" />
  <soap:address location="http://{{host}}:{{port}}/{{api_path}}/WeighingService" />
  ```

### 文件结构

```
mettler_toledo_xpr/
├── MT.Laboratory.Balance.XprXsr.V03.wsdl          # 实际WSDL文件（用户提供）
├── MT.Laboratory.Balance.XprXsr.V03.wsdl.template # 模板文件（仅供参考）
├── mettler_toledo_xpr.py                          # 驱动程序
├── balance.yaml                                   # 设备配置
├── SendCmd_Usage_Guide.md                         # 使用指南
└── README.md                                      # 本文件
```

## 使用方法

### 基本配置

```python
from unilabos.devices.balance.mettler_toledo_xpr import MettlerToledoXPR

# 创建天平实例
balance = MettlerToledoXPR(
    ip="192.168.1.10",      # 天平IP地址
    port=81,                # 天平端口
    password="123456",      # 天平密码
    timeout=10              # 连接超时时间
)

# 执行操作
balance.tare()              # 去皮
balance.zero()              # 清零
weight = balance.get_weight()  # 读取重量
```

### ROS2 SendCmd Action

详细的ROS2使用方法请参考 [SendCmd_Usage_Guide.md](SendCmd_Usage_Guide.md)

## 故障排除

### 常见错误

1. **FileNotFoundError: WSDL template not found**
   - 确保WSDL文件已正确放置在驱动目录中
   - 检查文件名是否正确

2. **连接失败**
   - 检查天平IP地址和端口配置
   - 确保天平Web服务已启用
   - 验证网络连接

3. **认证失败**
   - 检查天平密码是否正确
   - 确保天平允许Web服务访问

### 调试模式

```python
import logging
logging.basicConfig(level=logging.DEBUG)

# 创建天平实例，将显示详细日志
balance = MettlerToledoXPR(ip="192.168.1.10")
```

## 支持的操作

- **去皮 (Tare)**: 将当前重量设为零点
- **清零 (Zero)**: 重新校准零点
- **读取重量 (Get Weight)**: 获取当前重量值
- **带去皮读取**: 先去皮再读取重量
- **连接管理**: 自动连接和断开

## 技术支持

如果您在配置WSDL文件时遇到问题，请：

1. 查看梅特勒托利多官方文档
2. 联系梅特勒托利多技术支持
3. 在项目GitHub页面提交Issue

## 许可证

本驱动程序遵循项目主许可证。WSDL文件的使用需遵循梅特勒托利多的许可条款。