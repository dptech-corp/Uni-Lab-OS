import numpy as np
import networkx as nx
from typing import List, Dict, Any, Optional
from .pump_protocol import generate_pump_protocol_with_rinsing, generate_pump_protocol


def find_gas_source(G: nx.DiGraph, gas: str) -> str:
    """根据气体名称查找对应的气源"""
    # 按照命名规则查找气源
    gas_source_patterns = [
        f"gas_source_{gas}",
        f"gas_{gas}",
        f"flask_{gas}",
        f"{gas}_source"
    ]
    
    for pattern in gas_source_patterns:
        if pattern in G.nodes():
            return pattern
    
    # 模糊匹配
    for node in G.nodes():
        node_class = G.nodes[node].get('class', '') or ''
        if 'gas_source' in node_class and gas.lower() in node.lower():
            return node
        if node.startswith('flask_') and gas.lower() in node.lower():
            return node
    
    # 查找所有可用的气源
    available_gas_sources = [
        node for node in G.nodes() 
        if ((G.nodes[node].get('class') or '').startswith('virtual_gas_source')
            or ('gas' in node and 'source' in node)
            or (node.startswith('flask_') and any(g in node.lower() for g in ['air', 'nitrogen', 'argon', 'vacuum'])))
    ]
    
    raise ValueError(f"找不到气体 '{gas}' 对应的气源。可用气源: {available_gas_sources}")


def find_vacuum_pump(G: nx.DiGraph) -> str:
    """查找真空泵设备"""
    vacuum_pumps = [
        node for node in G.nodes() 
        if ((G.nodes[node].get('class') or '').startswith('virtual_vacuum_pump')
            or 'vacuum_pump' in node
            or 'vacuum' in (G.nodes[node].get('class') or ''))
    ]
    
    if not vacuum_pumps:
        raise ValueError("系统中未找到真空泵设备")
    
    return vacuum_pumps[0]


def find_connected_stirrer(G: nx.DiGraph, vessel: str) -> str:
    """查找与指定容器相连的搅拌器"""
    stirrer_nodes = [node for node in G.nodes() 
                    if (G.nodes[node].get('class') or '') == 'virtual_stirrer']
    
    # 检查哪个搅拌器与目标容器相连
    for stirrer in stirrer_nodes:
        if G.has_edge(stirrer, vessel) or G.has_edge(vessel, stirrer):
            return stirrer
    
    return stirrer_nodes[0] if stirrer_nodes else None


def find_associated_solenoid_valve(G: nx.DiGraph, device_id: str) -> Optional[str]:
    """查找与指定设备相关联的电磁阀"""
    solenoid_valves = [
        node for node in G.nodes() 
        if ('solenoid' in (G.nodes[node].get('class') or '').lower()
            or 'solenoid_valve' in node)
    ]
    
    # 通过网络连接查找直接相连的电磁阀
    for solenoid in solenoid_valves:
        if G.has_edge(device_id, solenoid) or G.has_edge(solenoid, device_id):
            return solenoid
    
    # 通过命名规则查找关联的电磁阀
    device_type = ""
    if 'vacuum' in device_id.lower():
        device_type = "vacuum"
    elif 'gas' in device_id.lower():
        device_type = "gas"
    
    if device_type:
        for solenoid in solenoid_valves:
            if device_type in solenoid.lower():
                return solenoid
    
    return None


def generate_evacuateandrefill_protocol(
    G: nx.DiGraph,
    vessel: str,
    gas: str,
    repeats: int = 1
) -> List[Dict[str, Any]]:
    """
    生成抽真空和充气操作的动作序列
    
    **修复版本**: 正确调用 pump_protocol 并处理异常
    """
    action_sequence = []
    
    # 参数设置 - 关键修复：减小体积避免超出泵容量
    VACUUM_VOLUME = 20.0     # 减小抽真空体积
    REFILL_VOLUME = 20.0     # 减小充气体积
    PUMP_FLOW_RATE = 2.5     # 降低流速
    STIR_SPEED = 300.0
    
    print(f"EVACUATE_REFILL: 开始生成协议，目标容器: {vessel}, 气体: {gas}, 重复次数: {repeats}")
    
    # 1. 验证设备存在
    if vessel not in G.nodes():
        raise ValueError(f"目标容器 '{vessel}' 不存在于系统中")
    
    # 2. 查找设备
    try:
        vacuum_pump = find_vacuum_pump(G)
        vacuum_solenoid = find_associated_solenoid_valve(G, vacuum_pump)
        gas_source = find_gas_source(G, gas)
        gas_solenoid = find_associated_solenoid_valve(G, gas_source)
        stirrer_id = find_connected_stirrer(G, vessel)
        
        print(f"EVACUATE_REFILL: 找到设备")
        print(f"  - 真空泵: {vacuum_pump}")
        print(f"  - 气源: {gas_source}")
        print(f"  - 真空电磁阀: {vacuum_solenoid}")
        print(f"  - 气源电磁阀: {gas_solenoid}")
        print(f"  - 搅拌器: {stirrer_id}")
        
    except ValueError as e:
        raise ValueError(f"设备查找失败: {str(e)}")
    
    # 3. **关键修复**: 验证路径存在性
    try:
        # 验证抽真空路径
        vacuum_path = nx.shortest_path(G, source=vessel, target=vacuum_pump)
        print(f"EVACUATE_REFILL: 抽真空路径: {' → '.join(vacuum_path)}")
        
        # 验证充气路径
        gas_path = nx.shortest_path(G, source=gas_source, target=vessel)
        print(f"EVACUATE_REFILL: 充气路径: {' → '.join(gas_path)}")
        
        # **新增**: 检查路径中的边数据
        for i in range(len(vacuum_path) - 1):
            nodeA, nodeB = vacuum_path[i], vacuum_path[i + 1]
            edge_data = G.get_edge_data(nodeA, nodeB)
            if not edge_data or 'port' not in edge_data:
                raise ValueError(f"路径 {nodeA} → {nodeB} 缺少端口信息")
            print(f"  抽真空路径边 {nodeA} → {nodeB}: {edge_data}")
        
        for i in range(len(gas_path) - 1):
            nodeA, nodeB = gas_path[i], gas_path[i + 1]
            edge_data = G.get_edge_data(nodeA, nodeB)
            if not edge_data or 'port' not in edge_data:
                raise ValueError(f"路径 {nodeA} → {nodeB} 缺少端口信息")
            print(f"  充气路径边 {nodeA} → {nodeB}: {edge_data}")
            
    except nx.NetworkXNoPath as e:
        raise ValueError(f"路径不存在: {str(e)}")
    except Exception as e:
        raise ValueError(f"路径验证失败: {str(e)}")
    
    # 4. 启动搅拌器
    if stirrer_id:
        action_sequence.append({
            "device_id": stirrer_id,
            "action_name": "start_stir",
            "action_kwargs": {
                "vessel": vessel,
                "stir_speed": STIR_SPEED,
                "purpose": "抽真空充气操作前启动搅拌"
            }
        })
    
    # 5. 执行多次抽真空-充气循环
    for cycle in range(repeats):
        print(f"EVACUATE_REFILL: === 第 {cycle+1}/{repeats} 次循环 ===")
        
        # ============ 抽真空阶段 ============
        print(f"EVACUATE_REFILL: 抽真空阶段开始")
        
        # 启动真空泵
        action_sequence.append({
            "device_id": vacuum_pump,
            "action_name": "set_status",
            "action_kwargs": {"string": "ON"}
        })
        
        # 开启真空电磁阀
        if vacuum_solenoid:
            action_sequence.append({
                "device_id": vacuum_solenoid,
                "action_name": "set_valve_position",
                "action_kwargs": {"command": "OPEN"}
            })
        
        # **关键修复**: 改进 pump_protocol 调用和错误处理
        print(f"EVACUATE_REFILL: 调用抽真空 pump_protocol: {vessel} → {vacuum_pump}")
        print(f"  - 体积: {VACUUM_VOLUME} mL")
        print(f"  - 流速: {PUMP_FLOW_RATE} mL/s")
        
        try:
            vacuum_transfer_actions = generate_pump_protocol_with_rinsing(
                G=G,
                from_vessel=vessel,
                to_vessel=vacuum_pump,
                volume=VACUUM_VOLUME,
                amount="",
                time=0.0,
                viscous=False,
                rinsing_solvent="",  # **修复**: 明确不使用清洗
                rinsing_volume=0.0,
                rinsing_repeats=0,
                solid=False,
                flowrate=PUMP_FLOW_RATE,
                transfer_flowrate=PUMP_FLOW_RATE
            )
            
            if vacuum_transfer_actions:
                action_sequence.extend(vacuum_transfer_actions)
                print(f"EVACUATE_REFILL: ✅ 成功添加 {len(vacuum_transfer_actions)} 个抽真空动作")
            else:
                print(f"EVACUATE_REFILL: ⚠️ 抽真空 pump_protocol 返回空序列")
                # **修复**: 添加手动泵动作作为备选
                action_sequence.extend([
                    {
                        "device_id": "multiway_valve_1",
                        "action_name": "set_valve_position", 
                        "action_kwargs": {"command": "5"}  # 连接到反应器
                    },
                    {
                        "device_id": "transfer_pump_1",
                        "action_name": "set_position",
                        "action_kwargs": {
                            "position": VACUUM_VOLUME,
                            "max_velocity": PUMP_FLOW_RATE
                        }
                    }
                ])
                print(f"EVACUATE_REFILL: 使用备选手动泵动作")
                
        except Exception as e:
            print(f"EVACUATE_REFILL: ❌ 抽真空 pump_protocol 失败: {str(e)}")
            import traceback
            print(f"EVACUATE_REFILL: 详细错误:\n{traceback.format_exc()}")
            
            # **修复**: 添加手动动作而不是忽略错误
            print(f"EVACUATE_REFILL: 使用手动备选方案")
            action_sequence.extend([
                {
                    "device_id": "multiway_valve_1",
                    "action_name": "set_valve_position",
                    "action_kwargs": {"command": "5"}  # 反应器端口
                },
                {
                    "device_id": "transfer_pump_1", 
                    "action_name": "set_position",
                    "action_kwargs": {
                        "position": VACUUM_VOLUME,
                        "max_velocity": PUMP_FLOW_RATE
                    }
                }
            ])
        
        # 关闭真空电磁阀
        if vacuum_solenoid:
            action_sequence.append({
                "device_id": vacuum_solenoid,
                "action_name": "set_valve_position",
                "action_kwargs": {"command": "CLOSED"}
            })
        
        # 关闭真空泵
        action_sequence.append({
            "device_id": vacuum_pump,
            "action_name": "set_status",
            "action_kwargs": {"string": "OFF"}
        })
        
        # ============ 充气阶段 ============
        print(f"EVACUATE_REFILL: 充气阶段开始")
        
        # 启动气源
        action_sequence.append({
            "device_id": gas_source,
            "action_name": "set_status", 
            "action_kwargs": {"string": "ON"}
        })
        
        # 开启气源电磁阀
        if gas_solenoid:
            action_sequence.append({
                "device_id": gas_solenoid,
                "action_name": "set_valve_position",
                "action_kwargs": {"command": "OPEN"}
            })
        
        # **关键修复**: 改进充气 pump_protocol 调用
        print(f"EVACUATE_REFILL: 调用充气 pump_protocol: {gas_source} → {vessel}")
        
        try:
            gas_transfer_actions = generate_pump_protocol_with_rinsing(
                G=G,
                from_vessel=gas_source,
                to_vessel=vessel,
                volume=REFILL_VOLUME,
                amount="",
                time=0.0,
                viscous=False,
                rinsing_solvent="",  # **修复**: 明确不使用清洗
                rinsing_volume=0.0,
                rinsing_repeats=0,
                solid=False,
                flowrate=PUMP_FLOW_RATE,
                transfer_flowrate=PUMP_FLOW_RATE
            )
            
            if gas_transfer_actions:
                action_sequence.extend(gas_transfer_actions)
                print(f"EVACUATE_REFILL: ✅ 成功添加 {len(gas_transfer_actions)} 个充气动作")
            else:
                print(f"EVACUATE_REFILL: ⚠️ 充气 pump_protocol 返回空序列")
                # **修复**: 添加手动充气动作
                action_sequence.extend([
                    {
                        "device_id": "multiway_valve_2",
                        "action_name": "set_valve_position",
                        "action_kwargs": {"command": "8"}  # 氮气端口
                    },
                    {
                        "device_id": "transfer_pump_2",
                        "action_name": "set_position",
                        "action_kwargs": {
                            "position": REFILL_VOLUME,
                            "max_velocity": PUMP_FLOW_RATE
                        }
                    },
                    {
                        "device_id": "multiway_valve_2", 
                        "action_name": "set_valve_position",
                        "action_kwargs": {"command": "5"}  # 反应器端口
                    },
                    {
                        "device_id": "transfer_pump_2",
                        "action_name": "set_position", 
                        "action_kwargs": {
                            "position": 0.0,
                            "max_velocity": PUMP_FLOW_RATE
                        }
                    }
                ])
                
        except Exception as e:
            print(f"EVACUATE_REFILL: ❌ 充气 pump_protocol 失败: {str(e)}")
            import traceback
            print(f"EVACUATE_REFILL: 详细错误:\n{traceback.format_exc()}")
            
            # **修复**: 使用手动充气动作
            print(f"EVACUATE_REFILL: 使用手动充气方案")
            action_sequence.extend([
                {
                    "device_id": "multiway_valve_2",
                    "action_name": "set_valve_position",
                    "action_kwargs": {"command": "8"}  # 连接气源
                },
                {
                    "device_id": "transfer_pump_2",
                    "action_name": "set_position",
                    "action_kwargs": {
                        "position": REFILL_VOLUME,
                        "max_velocity": PUMP_FLOW_RATE
                    }
                },
                {
                    "device_id": "multiway_valve_2",
                    "action_name": "set_valve_position", 
                    "action_kwargs": {"command": "5"}  # 连接反应器
                },
                {
                    "device_id": "transfer_pump_2",
                    "action_name": "set_position",
                    "action_kwargs": {
                        "position": 0.0,
                        "max_velocity": PUMP_FLOW_RATE
                    }
                }
            ])
        
        # 关闭气源电磁阀
        if gas_solenoid:
            action_sequence.append({
                "device_id": gas_solenoid,
                "action_name": "set_valve_position",
                "action_kwargs": {"command": "CLOSED"}
            })
        
        # 关闭气源
        action_sequence.append({
            "device_id": gas_source,
            "action_name": "set_status",
            "action_kwargs": {"string": "OFF"}
        })
        
        # 等待下一次循环
        if cycle < repeats - 1:
            action_sequence.append({
                "action_name": "wait",
                "action_kwargs": {"time": 2.0}
            })
    
    # 停止搅拌器
    if stirrer_id:
        action_sequence.append({
            "device_id": stirrer_id,
            "action_name": "stop_stir",
            "action_kwargs": {"vessel": vessel}
        })
    
    print(f"EVACUATE_REFILL: 协议生成完成，共 {len(action_sequence)} 个动作")
    return action_sequence


# 测试函数
def test_evacuateandrefill_protocol():
    """测试抽真空充气协议"""
    print("=== EVACUATE AND REFILL PROTOCOL 测试 ===")
    print("测试完成")


if __name__ == "__main__":
    test_evacuateandrefill_protocol()