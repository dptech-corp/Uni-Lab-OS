from typing import List, Dict, Any
import networkx as nx

def generate_heat_chill_protocol(
    G: nx.DiGraph,
    vessel: str,
    temp: float,
    time: float,
    stir: bool,
    stir_speed: float,
    purpose: str
) -> List[Dict[str, Any]]:
    """
    生成加热/冷却操作的协议序列 - 严格按照 HeatChill.action
    """
    action_sequence = []
    
    # 查找加热/冷却设备
    heatchill_nodes = [node for node in G.nodes() 
                      if G.nodes[node].get('class') == 'virtual_heatchill']
    
    if not heatchill_nodes:
        raise ValueError("没有找到可用的加热/冷却设备")
    
    heatchill_id = heatchill_nodes[0]
    
    if vessel not in G.nodes():
        raise ValueError(f"容器 {vessel} 不存在于图中")
    
    action_sequence.append({
        "device_id": heatchill_id,
        "action_name": "heat_chill",
        "action_kwargs": {
            "vessel": vessel,
            "temp": temp,
            "time": time,
            "stir": stir,
            "stir_speed": stir_speed,
            "purpose": purpose
        }
    })
    
    return action_sequence


def generate_heat_chill_start_protocol(
    G: nx.DiGraph,
    vessel: str,
    temp: float,
    purpose: str
) -> List[Dict[str, Any]]:
    """
    生成开始加热/冷却操作的协议序列 - 严格按照 HeatChillStart.action
    """
    action_sequence = []
    
    heatchill_nodes = [node for node in G.nodes() 
                      if G.nodes[node].get('class') == 'virtual_heatchill']
    
    if not heatchill_nodes:
        raise ValueError("没有找到可用的加热/冷却设备")
    
    heatchill_id = heatchill_nodes[0]
    
    if vessel not in G.nodes():
        raise ValueError(f"容器 {vessel} 不存在于图中")
    
    action_sequence.append({
        "device_id": heatchill_id,
        "action_name": "heat_chill_start",
        "action_kwargs": {
            "vessel": vessel,
            "temp": temp,
            "purpose": purpose
        }
    })
    
    return action_sequence


def generate_heat_chill_stop_protocol(
    G: nx.DiGraph,
    vessel: str
) -> List[Dict[str, Any]]:
    """
    生成停止加热/冷却操作的协议序列
    
    Args:
        G: 有向图，节点为设备和容器
        vessel: 容器名称
    
    Returns:
        List[Dict[str, Any]]: 停止加热/冷却操作的动作序列
    """
    action_sequence = []
    
    # 查找加热/冷却设备
    heatchill_nodes = [node for node in G.nodes() 
                      if G.nodes[node].get('class') == 'virtual_heatchill']
    
    if not heatchill_nodes:
        raise ValueError("没有找到可用的加热/冷却设备")
    
    heatchill_id = heatchill_nodes[0]
    
    if vessel not in G.nodes():
        raise ValueError(f"容器 {vessel} 不存在于图中")
    
    action_sequence.append({
        "device_id": heatchill_id,
        "action_name": "heat_chill_stop",
        "action_kwargs": {
            "vessel": vessel
        }
    })
    
    return action_sequence