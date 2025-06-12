from typing import List, Dict, Any
import networkx as nx

def generate_transfer_protocol(
    G: nx.DiGraph,
    from_vessel: str,
    to_vessel: str,
    volume: float,
    amount: str = "",
    time: float = 0,
    viscous: bool = False,
    rinsing_solvent: str = "",
    rinsing_volume: float = 0.0,
    rinsing_repeats: int = 0,
    solid: bool = False
) -> List[Dict[str, Any]]:
    """
    生成液体转移操作的协议序列
    
    Args:
        G: 有向图，节点为设备和容器
        from_vessel: 源容器
        to_vessel: 目标容器
        volume: 转移体积 (mL)
        amount: 数量描述 (可选)
        time: 转移时间 (秒，可选)
        viscous: 是否为粘性液体
        rinsing_solvent: 冲洗溶剂 (可选)
        rinsing_volume: 冲洗体积 (mL，可选)
        rinsing_repeats: 冲洗重复次数
        solid: 是否涉及固体
    
    Returns:
        List[Dict[str, Any]]: 转移操作的动作序列
    
    Raises:
        ValueError: 当找不到合适的转移设备时抛出异常
    
    Examples:
        transfer_protocol = generate_transfer_protocol(G, "flask_1", "reactor", 10.0)
    """
    action_sequence = []
    
    # 查找虚拟转移泵设备用于液体转移 - 修复：应该查找 virtual_transfer_pump
    pump_nodes = [node for node in G.nodes() 
                  if G.nodes[node].get('class') == 'virtual_transfer_pump']
    
    if not pump_nodes:
        raise ValueError("没有找到可用的转移泵设备进行液体转移")
    
    # 使用第一个可用的泵
    pump_id = pump_nodes[0]
    
    # 验证容器是否存在
    if from_vessel not in G.nodes():
        raise ValueError(f"源容器 {from_vessel} 不存在于图中")
    
    if to_vessel not in G.nodes():
        raise ValueError(f"目标容器 {to_vessel} 不存在于图中")
    
    # 执行液体转移操作 - 参数完全匹配Transfer.action
    action_sequence.append({
        "device_id": pump_id,
        "action_name": "transfer",
        "action_kwargs": {
            "from_vessel": from_vessel,
            "to_vessel": to_vessel,
            "volume": volume,
            "amount": amount,
            "time": time,
            "viscous": viscous,
            "rinsing_solvent": rinsing_solvent,
            "rinsing_volume": rinsing_volume,
            "rinsing_repeats": rinsing_repeats,
            "solid": solid
        }
    })
    
    return action_sequence