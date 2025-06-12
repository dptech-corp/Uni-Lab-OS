from typing import List, Dict, Any
import networkx as nx

def generate_stir_protocol(
    G: nx.DiGraph,
    stir_time: float,
    stir_speed: float,
    settling_time: float
) -> List[Dict[str, Any]]:
    """
    生成搅拌操作的协议序列
    
    Args:
        G: 有向图，节点为设备和容器
        stir_time: 搅拌时间 (秒)
        stir_speed: 搅拌速度 (rpm)
        settling_time: 沉降时间 (秒)
    
    Returns:
        List[Dict[str, Any]]: 搅拌操作的动作序列
    
    Raises:
        ValueError: 当找不到搅拌设备时抛出异常
    
    Examples:
        stir_protocol = generate_stir_protocol(G, 300.0, 500.0, 60.0)
    """
    action_sequence = []
    
    # 查找搅拌设备
    stirrer_nodes = [node for node in G.nodes() 
                    if G.nodes[node].get('class') == 'virtual_stirrer']
    
    if not stirrer_nodes:
        raise ValueError("没有找到可用的搅拌设备")
    
    # 使用第一个可用的搅拌器
    stirrer_id = stirrer_nodes[0]
    
    # 执行搅拌操作
    action_sequence.append({
        "device_id": stirrer_id,
        "action_name": "stir",
        "action_kwargs": {
            "stir_time": stir_time,
            "stir_speed": stir_speed,
            "settling_time": settling_time
        }
    })
    
    return action_sequence


def generate_start_stir_protocol(
    G: nx.DiGraph,
    vessel: str,
    stir_speed: float,
    purpose: str
) -> List[Dict[str, Any]]:
    """
    生成开始搅拌操作的协议序列
    
    Args:
        G: 有向图，节点为设备和容器
        vessel: 搅拌容器
        stir_speed: 搅拌速度 (rpm)
        purpose: 搅拌目的
    
    Returns:
        List[Dict[str, Any]]: 开始搅拌操作的动作序列
    """
    action_sequence = []
    
    # 查找搅拌设备
    stirrer_nodes = [node for node in G.nodes() 
                    if G.nodes[node].get('class') == 'virtual_stirrer']
    
    if not stirrer_nodes:
        raise ValueError("没有找到可用的搅拌设备")
    
    stirrer_id = stirrer_nodes[0]
    
    # 验证容器是否存在
    if vessel not in G.nodes():
        raise ValueError(f"容器 {vessel} 不存在于图中")
    
    action_sequence.append({
        "device_id": stirrer_id,
        "action_name": "start_stir",
        "action_kwargs": {
            "vessel": vessel,
            "stir_speed": stir_speed,
            "purpose": purpose
        }
    })
    
    return action_sequence


def generate_stop_stir_protocol(
    G: nx.DiGraph,
    vessel: str
) -> List[Dict[str, Any]]:
    """
    生成停止搅拌操作的协议序列
    
    Args:
        G: 有向图，节点为设备和容器
        vessel: 搅拌容器
    
    Returns:
        List[Dict[str, Any]]: 停止搅拌操作的动作序列
    """
    action_sequence = []
    
    # 查找搅拌设备
    stirrer_nodes = [node for node in G.nodes() 
                    if G.nodes[node].get('class') == 'virtual_stirrer']
    
    if not stirrer_nodes:
        raise ValueError("没有找到可用的搅拌设备")
    
    stirrer_id = stirrer_nodes[0]
    
    # 验证容器是否存在
    if vessel not in G.nodes():
        raise ValueError(f"容器 {vessel} 不存在于图中")
    
    action_sequence.append({
        "device_id": stirrer_id,
        "action_name": "stop_stir",
        "action_kwargs": {
            "vessel": vessel
        }
    })
    
    return action_sequence