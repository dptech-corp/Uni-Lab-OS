from typing import List, Dict, Any
import networkx as nx

def generate_filter_protocol(
    G: nx.DiGraph,
    vessel: str,
    filtrate_vessel: str = "",
    stir: bool = False,
    stir_speed: float = 300.0,
    temp: float = 25.0,
    continue_heatchill: bool = False,
    volume: float = 0.0
) -> List[Dict[str, Any]]:
    """
    生成过滤操作的协议序列
    
    Args:
        G: 有向图，节点为设备和容器
        vessel: 过滤容器
        filtrate_vessel: 滤液容器（可选）
        stir: 是否搅拌
        stir_speed: 搅拌速度（可选）
        temp: 温度（可选，摄氏度）
        continue_heatchill: 是否继续加热冷却
        volume: 过滤体积（可选）
    
    Returns:
        List[Dict[str, Any]]: 过滤操作的动作序列
    
    Raises:
        ValueError: 当找不到过滤设备时抛出异常
    
    Examples:
        filter_protocol = generate_filter_protocol(G, "reactor", "filtrate_vessel", stir=True, volume=100.0)
    """
    action_sequence = []
    
    # 查找过滤设备
    filter_nodes = [node for node in G.nodes() 
                   if G.nodes[node].get('class') == 'virtual_filter']
    
    if not filter_nodes:
        raise ValueError("没有找到可用的过滤设备")
    
    # 使用第一个可用的过滤器
    filter_id = filter_nodes[0]
    
    # 验证容器是否存在
    if vessel not in G.nodes():
        raise ValueError(f"过滤容器 {vessel} 不存在于图中")
    
    if filtrate_vessel and filtrate_vessel not in G.nodes():
        raise ValueError(f"滤液容器 {filtrate_vessel} 不存在于图中")
    
    # 执行过滤操作
    action_sequence.append({
        "device_id": filter_id,
        "action_name": "filter_sample",
        "action_kwargs": {
            "vessel": vessel,
            "filtrate_vessel": filtrate_vessel,
            "stir": stir,
            "stir_speed": stir_speed,
            "temp": temp,
            "continue_heatchill": continue_heatchill,
            "volume": volume
        }
    })
    
    return action_sequence