from typing import List, Dict, Any
import networkx as nx

def generate_run_column_protocol(
    G: nx.DiGraph,
    from_vessel: str,
    to_vessel: str,
    column: str
) -> List[Dict[str, Any]]:
    """
    生成柱层析分离的协议序列
    
    Args:
        G: 有向图，节点为设备和容器
        from_vessel: 源容器的名称，即样品起始所在的容器
        to_vessel: 目标容器的名称，分离后的样品要到达的容器
        column: 所使用的柱子的名称
    
    Returns:
        List[Dict[str, Any]]: 柱层析分离操作的动作序列
    
    Raises:
        ValueError: 当找不到必要的设备时抛出异常
    
    Examples:
        run_column_protocol = generate_run_column_protocol(G, "reactor", "collection_flask", "silica_column")
    """
    action_sequence = []
    
    # 验证容器是否存在
    if from_vessel not in G.nodes():
        raise ValueError(f"源容器 {from_vessel} 不存在于图中")
    
    if to_vessel not in G.nodes():
        raise ValueError(f"目标容器 {to_vessel} 不存在于图中")
    
    # 查找转移泵设备（用于样品转移）
    pump_nodes = [node for node in G.nodes() 
                  if G.nodes[node].get('class') == 'virtual_transfer_pump']
    
    if not pump_nodes:
        raise ValueError("没有找到可用的转移泵设备")
    
    pump_id = pump_nodes[0]
    
    # 查找柱层析设备
    column_nodes = [node for node in G.nodes() 
                   if G.nodes[node].get('class') == 'virtual_column']
    
    if not column_nodes:
        raise ValueError("没有找到可用的柱层析设备")
    
    column_id = column_nodes[0]
    
    # 步骤1：将样品从源容器转移到柱子上
    action_sequence.append({
        "device_id": pump_id,
        "action_name": "transfer",
        "action_kwargs": {
            "from_vessel": from_vessel,
            "to_vessel": column_id,  # 将样品转移到柱子设备
            "volume": 0.0,  # 转移所有液体，体积由系统确定
            "amount": f"样品上柱 - 使用 {column}",
            "time": 0.0,
            "viscous": False,
            "rinsing_solvent": "",
            "rinsing_volume": 0.0,
            "rinsing_repeats": 0,
            "solid": False
        }
    })
    
    # 步骤2：运行柱层析分离
    action_sequence.append({
        "device_id": column_id,
        "action_name": "run_column",
        "action_kwargs": {
            "from_vessel": from_vessel,
            "to_vessel": to_vessel,
            "column": column
        }
    })
    
    # 步骤3：将分离后的产物从柱子转移到目标容器
    action_sequence.append({
        "device_id": pump_id,
        "action_name": "transfer",
        "action_kwargs": {
            "from_vessel": column_id,  # 从柱子设备转移
            "to_vessel": to_vessel,
            "volume": 0.0,  # 转移所有液体，体积由系统确定
            "amount": f"收集分离产物 - 来自 {column}",
            "time": 0.0,
            "viscous": False,
            "rinsing_solvent": "",
            "rinsing_volume": 0.0,
            "rinsing_repeats": 0,
            "solid": False
        }
    })
    
    return action_sequence