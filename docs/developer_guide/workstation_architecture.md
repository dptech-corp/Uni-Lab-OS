# 工作站基础架构设计文档

## 1. 整体架构图

```mermaid
graph TB
    subgraph "工作站基础架构"
        WB[WorkstationBase]
        WB --> |继承| RPN[ROS2WorkstationNode]
        WB --> |组合| WCB[WorkstationCommunicationBase]
        WB --> |组合| MMB[MaterialManagementBase]
        WB --> |组合| WHS[WorkstationHTTPService]
    end
    
    subgraph "通信层实现"
        WCB --> |实现| PLC[PLCCommunication]
        WCB --> |实现| SER[SerialCommunication]
        WCB --> |实现| ETH[EthernetCommunication]
    end
    
    subgraph "物料管理实现"
        MMB --> |实现| PLR[PyLabRobotMaterialManager]
        MMB --> |实现| BIO[BioyondMaterialManager]
        MMB --> |实现| SIM[SimpleMaterialManager]
    end
    
    subgraph "HTTP服务"
        WHS --> |处理| LIMS[LIMS协议报送]
        WHS --> |处理| MAT[物料变更报送]
        WHS --> |处理| ERR[错误处理报送]
    end
    
    subgraph "具体工作站实现"
        WB --> |继承| WS1[PLCWorkstation]
        WB --> |继承| WS2[ReportingWorkstation]
        WB --> |继承| WS3[HybridWorkstation]
    end
    
    subgraph "外部系统"
        EXT1[PLC设备] --> |通信| PLC
        EXT2[外部工作站] --> |HTTP报送| WHS
        EXT3[LIMS系统] --> |HTTP报送| WHS
        EXT4[Bioyond物料系统] --> |查询| BIO
    end
```

## 2. 类关系图

```mermaid
classDiagram
    class WorkstationBase {
        <<abstract>>
        +device_id: str
        +communication: WorkstationCommunicationBase
        +material_management: MaterialManagementBase
        +http_service: WorkstationHTTPService
        +workflow_status: WorkflowStatus
        +supported_workflows: Dict
        
        +_create_communication_module()* 
        +_create_material_management_module()*
        +_register_supported_workflows()*
        
        +process_step_finish_report()
        +process_sample_finish_report()
        +process_order_finish_report()
        +process_material_change_report()
        +handle_external_error()
        
        +start_workflow()
        +stop_workflow()
        +get_workflow_status()
        +get_device_status()
    }
    
    class ROS2WorkstationNode {
        +sub_devices: Dict
        +protocol_names: List
        +execute_single_action()
        +create_ros_action_server()
        +initialize_device()
    }
    
    class WorkstationCommunicationBase {
        <<abstract>>
        +config: CommunicationConfig
        +is_connected: bool
        +connect()
        +disconnect()
        +start_workflow()*
        +stop_workflow()*
        +get_device_status()*
        +write_register()
        +read_register()
    }
    
    class MaterialManagementBase {
        <<abstract>>
        +device_id: str
        +deck_config: Dict
        +resource_tracker: DeviceNodeResourceTracker
        +plr_deck: Deck
        +find_materials_by_type()
        +update_material_location()
        +convert_to_unilab_format()
        +_create_resource_by_type()*
    }
    
    class WorkstationHTTPService {
        +workstation_instance: WorkstationBase
        +host: str
        +port: int
        +start()
        +stop()
        +_handle_step_finish_report()
        +_handle_material_change_report()
    }
    
    class PLCWorkstation {
        +plc_config: Dict
        +modbus_client: ModbusTCPClient
        +_create_communication_module()
        +_create_material_management_module()
        +_register_supported_workflows()
    }
    
    class ReportingWorkstation {
        +report_handlers: Dict
        +_create_communication_module()
        +_create_material_management_module()
        +_register_supported_workflows()
    }
    
    WorkstationBase --|> ROS2WorkstationNode
    WorkstationBase *-- WorkstationCommunicationBase
    WorkstationBase *-- MaterialManagementBase
    WorkstationBase *-- WorkstationHTTPService
    
    PLCWorkstation --|> WorkstationBase
    ReportingWorkstation --|> WorkstationBase
    
    WorkstationCommunicationBase <|-- PLCCommunication
    WorkstationCommunicationBase <|-- DummyCommunication
    
    MaterialManagementBase <|-- PyLabRobotMaterialManager
    MaterialManagementBase <|-- SimpleMaterialManager
```

## 3. 工作站启动时序图

```mermaid
sequenceDiagram
    participant APP as Application
    participant WS as WorkstationBase
    participant COMM as CommunicationModule
    participant MAT as MaterialManager
    participant HTTP as HTTPService
    participant ROS as ROS2WorkstationNode
    
    APP->>WS: 创建工作站实例
    WS->>ROS: 初始化ROS2WorkstationNode
    ROS->>ROS: 初始化子设备
    ROS->>ROS: 设置硬件接口代理
    
    WS->>COMM: _create_communication_module()
    COMM->>COMM: 初始化通信配置
    COMM->>COMM: 建立PLC/串口连接
    COMM-->>WS: 返回通信模块实例
    
    WS->>MAT: _create_material_management_module()
    MAT->>MAT: 创建PyLabRobot Deck
    MAT->>MAT: 初始化物料资源
    MAT->>MAT: 注册到ResourceTracker
    MAT-->>WS: 返回物料管理实例
    
    WS->>WS: _register_supported_workflows()
    WS->>WS: _create_workstation_services()
    WS->>HTTP: _start_http_service()
    HTTP->>HTTP: 创建HTTP服务器
    HTTP->>HTTP: 启动监听线程
    HTTP-->>WS: HTTP服务启动完成
    
    WS-->>APP: 工作站初始化完成
```

## 4. 工作流执行时序图

```mermaid
sequenceDiagram
    participant EXT as ExternalSystem
    participant WS as WorkstationBase
    participant COMM as CommunicationModule
    participant MAT as MaterialManager
    participant ROS as ROS2WorkstationNode
    participant DEV as SubDevice
    
    EXT->>WS: start_workflow(type, params)
    WS->>WS: 验证工作流类型
    WS->>COMM: start_workflow(type, params)
    COMM->>COMM: 发送启动命令到PLC
    COMM-->>WS: 启动成功
    
    WS->>WS: 更新workflow_status = RUNNING
    
    loop 工作流步骤执行
        WS->>ROS: execute_single_action(device_id, action, params)
        ROS->>DEV: 发送ROS Action请求
        DEV->>DEV: 执行设备动作
        DEV-->>ROS: 返回执行结果
        ROS-->>WS: 返回动作结果
        
        WS->>MAT: update_material_location(material_id, location)
        MAT->>MAT: 更新PyLabRobot资源状态
        MAT-->>WS: 更新完成
    end
    
    WS->>COMM: get_workflow_status()
    COMM->>COMM: 查询PLC状态寄存器
    COMM-->>WS: 返回状态信息
    
    WS->>WS: 更新workflow_status = COMPLETED
    WS-->>EXT: 工作流执行完成
```

## 5. HTTP报送处理时序图

```mermaid
sequenceDiagram
    participant EXT as ExternalWorkstation
    participant HTTP as HTTPService
    participant WS as WorkstationBase
    participant MAT as MaterialManager
    participant DB as DataStorage
    
    EXT->>HTTP: POST /report/step_finish
    HTTP->>HTTP: 解析请求数据
    HTTP->>HTTP: 验证LIMS协议字段
    HTTP->>WS: process_step_finish_report(request)
    
    WS->>WS: 增加接收计数
    WS->>WS: 记录步骤完成事件
    WS->>MAT: 更新相关物料状态
    MAT->>MAT: 更新PyLabRobot资源
    MAT-->>WS: 更新完成
    
    WS->>DB: 保存报送记录
    DB-->>WS: 保存完成
    
    WS-->>HTTP: 返回处理结果
    HTTP->>HTTP: 构造HTTP响应
    HTTP-->>EXT: 200 OK + acknowledgment_id
    
    Note over EXT,DB: 类似处理sample_finish, order_finish, material_change等报送
```

## 6. 错误处理时序图

```mermaid
sequenceDiagram
    participant DEV as Device
    participant WS as WorkstationBase
    participant COMM as CommunicationModule
    participant HTTP as HTTPService
    participant EXT as ExternalSystem
    
    DEV->>WS: 设备错误事件
    WS->>WS: handle_external_error(error_data)
    WS->>WS: 记录错误历史
    
    alt 关键错误
        WS->>COMM: emergency_stop()
        COMM->>COMM: 发送紧急停止命令
        WS->>WS: 更新workflow_status = ERROR
    else 普通错误
        WS->>WS: 标记动作失败
        WS->>WS: 触发重试逻辑
    end
    
    WS->>HTTP: 记录错误报送
    HTTP->>EXT: 主动通知错误状态
    
    WS-->>DEV: 错误处理完成
```

## 7. 典型工作站实现示例

### 7.1 PLC工作站实现

```python
class PLCWorkstation(WorkstationBase):
    def _create_communication_module(self):
        return PLCCommunication(self.communication_config)
    
    def _create_material_management_module(self):
        return PyLabRobotMaterialManager(
            self.device_id, 
            self.deck_config, 
            self.resource_tracker
        )
    
    def _register_supported_workflows(self):
        self.supported_workflows = {
            "battery_assembly": WorkflowInfo(...),
            "quality_check": WorkflowInfo(...)
        }
```

### 7.2 报送接收工作站实现

```python
class ReportingWorkstation(WorkstationBase):
    def _create_communication_module(self):
        return DummyCommunication(self.communication_config)
    
    def _create_material_management_module(self):
        return SimpleMaterialManager(
            self.device_id,
            self.deck_config,
            self.resource_tracker
        )
    
    def _register_supported_workflows(self):
        self.supported_workflows = {
            "data_collection": WorkflowInfo(...),
            "report_processing": WorkflowInfo(...)
        }
```

## 8. 核心接口说明

### 8.1 必须实现的抽象方法
- `_create_communication_module()`: 创建通信模块
- `_create_material_management_module()`: 创建物料管理模块
- `_register_supported_workflows()`: 注册支持的工作流

### 8.2 可重写的报送处理方法
- `process_step_finish_report()`: 步骤完成处理
- `process_sample_finish_report()`: 样本完成处理
- `process_order_finish_report()`: 订单完成处理
- `process_material_change_report()`: 物料变更处理
- `handle_external_error()`: 错误处理

### 8.3 工作流控制接口
- `start_workflow()`: 启动工作流
- `stop_workflow()`: 停止工作流
- `get_workflow_status()`: 获取状态

## 9. 配置参数说明

```python
workstation_config = {
    "communication_config": {
        "protocol": "modbus_tcp",
        "host": "192.168.1.100",
        "port": 502
    },
    "deck_config": {
        "size_x": 1000.0,
        "size_y": 1000.0,
        "size_z": 500.0
    },
    "http_service_config": {
        "enabled": True,
        "host": "127.0.0.1",
        "port": 8081
    },
    "communication_interfaces": {
        "logical_device_1": CommunicationInterface(...)
    }
}
```

这个架构设计支持：
1. **灵活的通信方式**: 通过CommunicationBase支持PLC、串口、以太网等
2. **多样的物料管理**: 支持PyLabRobot、Bioyond、简单物料系统
3. **统一的HTTP报送**: 基于LIMS协议的标准化报送接口
4. **完整的工作流控制**: 支持动态和静态工作流
5. **强大的错误处理**: 多层次的错误处理和恢复机制
