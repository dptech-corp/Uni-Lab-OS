from typing import List, Dict, Any
import networkx as nx

def generate_centrifuge_protocol(
    G: nx.DiGraph,
    vessel: str,
    speed: float,
    time: float,
    temp: float = 25.0
) -> List[Dict[str, Any]]:
    """
    生成离心操作的协议序列
    
    Args:
        G: 有向图，节点为设备和容器
        vessel: 离心容器名称
        speed: 离心速度 (rpm)
        time: 离心时间 (秒)
        temp: 温度 (摄氏度，可选)
    
    Returns:
        List[Dict[str, Any]]: 离心操作的动作序列
    
    Raises:
        ValueError: 当找不到离心机设备时抛出异常
    
    Examples:
        centrifuge_protocol = generate_centrifuge_protocol(G, "reactor", 5000, 300, 4.0)
    """
    action_sequence = []
    
    # 查找离心机设备
    centrifuge_nodes = [node for node in G.nodes() 
                       if G.nodes[node].get('class') == 'virtual_centrifuge']
    
    if not centrifuge_nodes:
        raise ValueError("没有找到可用的离心机设备")
    
    # 使用第一个可用的离心机
    centrifuge_id = centrifuge_nodes[0]
    
    # 验证容器是否存在
    if vessel not in G.nodes():
        raise ValueError(f"容器 {vessel} 不存在于图中")
    
    # 执行离心操作
    action_sequence.append({
        "device_id": centrifuge_id,
        "action_name": "centrifuge",
        "action_kwargs": {
            "vessel": vessel,
            "speed": speed,
            "time": time,
            "temp": temp
        }
    })
    
    return action_sequence


def generate_multi_step_centrifuge_protocol(
    G: nx.DiGraph,
    vessel: str,
    steps: List[Dict[str, Any]]
) -> List[Dict[str, Any]]:
    """
    生成多步骤离心操作的协议序列
    
    Args:
        G: 有向图，节点为设备和容器
        vessel: 离心容器名称
        steps: 离心步骤列表，每个步骤包含 speed, time, temp 参数
    
    Returns:
        List[Dict[str, Any]]: 多步骤离心操作的动作序列
    
    Examples:
        steps = [
            {"speed": 1000, "time": 60, "temp": 4.0},   # 低速预离心
            {"speed": 12000, "time": 600, "temp": 4.0}  # 高速离心
        ]
        protocol = generate_multi_step_centrifuge_protocol(G, "reactor", steps)
    """
    action_sequence = []
    
    # 查找离心机设备
    centrifuge_nodes = [node for node in G.nodes() 
                       if G.nodes[node].get('class') == 'virtual_centrifuge']
    
    if not centrifuge_nodes:
        raise ValueError("没有找到可用的离心机设备")
    
    centrifuge_id = centrifuge_nodes[0]
    
    # 验证容器是否存在
    if vessel not in G.nodes():
        raise ValueError(f"容器 {vessel} 不存在于图中")
    
    # 执行每个离心步骤
    for i, step in enumerate(steps):
        speed = step.get('speed', 5000)
        time = step.get('time', 300)
        temp = step.get('temp', 25.0)
        
        action_sequence.append({
            "device_id": centrifuge_id,
            "action_name": "centrifuge",
            "action_kwargs": {
                "vessel": vessel,
                "speed": speed,
                "time": time,
                "temp": temp
            }
        })
        
        # 步骤间等待时间（除了最后一步）
        if i < len(steps) - 1:
            action_sequence.append({
                "action_name": "wait", 
                "action_kwargs": {"time": 3}
            })
    
    return action_sequence