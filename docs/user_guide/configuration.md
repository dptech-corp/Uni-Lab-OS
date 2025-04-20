# Uni-Lab 配置指南

Uni-Lab支持通过Python配置文件进行灵活的系统配置。本指南将帮助您理解配置选项并设置您的Uni-Lab环境。

## 配置文件格式

Uni-Lab支持Python格式的配置文件，它比YAML或JSON提供更多的灵活性，包括支持注释、条件逻辑和复杂数据结构。

### 基本配置示例

一个典型的配置文件包含以下部分：

```python
#!/usr/bin/env python
# coding=utf-8
"""Uni-Lab 配置文件"""

from dataclasses import dataclass

# 配置类定义

class MQConfig:
    """MQTT 配置类"""
    lab_id: str = "YOUR_LAB_ID"
    # 更多配置...

# 其他配置类...
```

## 配置选项说明

### MQTT配置 (MQConfig)

MQTT配置用于连接消息队列服务，是Uni-Lab与云端通信的主要方式。

```python

class MQConfig:
    """MQTT 配置类"""
    lab_id: str = "7AAEDBEA"  # 实验室唯一标识
    instance_id: str = "mqtt-cn-instance"
    access_key: str = "your-access-key"
    secret_key: str = "your-secret-key"
    group_id: str = "GID_labs"
    broker_url: str = "mqtt-cn-instance.mqtt.aliyuncs.com"
    port: int = 8883
    
    # 可以直接提供证书文件路径
    ca_file: str = "/path/to/ca.pem"  # 相对config.py所在目录的路径
    cert_file: str = "/path/to/cert.pem"  # 相对config.py所在目录的路径
    key_file: str = "/path/to/key.pem"  # 相对config.py所在目录的路径
    
    # 或者直接提供证书内容
    ca_content: str = ""
    cert_content: str = ""
    key_content: str = ""
```

#### 证书配置

MQTT连接支持两种方式配置证书：

1. **文件路径方式**（推荐）：指定证书文件的路径，系统会自动读取文件内容
2. **直接内容方式**：直接在配置中提供证书内容

推荐使用文件路径方式，便于证书的更新和管理。

### HTTP客户端配置 (HTTPConfig)

即将开放 Uni-Lab 云端实验室。

### ROS模块配置 (ROSConfig)

配置ROS消息转换器需要加载的模块：

```python

class ROSConfig:
    """ROS模块配置"""
    modules = [
        "std_msgs.msg",
        "geometry_msgs.msg",
        "control_msgs.msg",
        "control_msgs.action",
        "nav2_msgs.action",
        "unilabos_msgs.msg",
        "unilabos_msgs.action",
    ]
```

您可以根据需要添加其他ROS模块。

### 其他配置选项

- **OSSUploadConfig**: 对象存储上传配置

## 如何使用配置文件

启动Uni-Lab时通过`--config`参数指定配置文件路径：

```bash
unilab --config path/to/your/config.py
```

如果您不涉及多环境开发，可以在unilabos的安装路径中手动添加local_config.py的文件

# 启动Uni-Lab
python -m unilabos.app.main --config path/to/your/config.py
```
