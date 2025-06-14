from typing import List, Dict, Any
import networkx as nx

def generate_filter_through_protocol(
    G: nx.DiGraph,
    from_vessel: str,
    to_vessel: str,
    filter_through: str,
    eluting_solvent: str = "",
    eluting_volume: float = 0.0,
    eluting_repeats: int = 0,
    residence_time: float = 0.0
) -> List[Dict[str, Any]]:
    """
    生成通过过滤介质过滤的协议序列
    
    Args:
        G: 有向图，节点为设备和容器
        from_vessel: 源容器的名称，即物质起始所在的容器
        to_vessel: 目标容器的名称，物质过滤后要到达的容器
        filter_through: 过滤时所通过的介质，如滤纸、柱子等
        eluting_solvent: 洗脱溶剂的名称，可选参数
        eluting_volume: 洗脱溶剂的体积，可选参数
        eluting_repeats: 洗脱操作的重复次数，默认为 0
        residence_time: 物质在过滤介质中的停留时间，可选参数
    
    Returns:
        List[Dict[str, Any]]: 过滤操作的动作序列
    
    Raises:
        ValueError: 当找不到必要的设备时抛出异常
    
    Examples:
        filter_through_protocol = generate_filter_through_protocol(
            G, "reactor", "collection_flask", "celite", "ethanol", 50.0, 2, 60.0
        )
    """
    action_sequence = []
    
    # 验证容器是否存在
    if from_vessel not in G.nodes():
        raise ValueError(f"源容器 {from_vessel} 不存在于图中")
    
    if to_vessel not in G.nodes():
        raise ValueError(f"目标容器 {to_vessel} 不存在于图中")
    
    # 查找转移泵设备（用于液体转移）
    pump_nodes = [node for node in G.nodes() 
                  if G.nodes[node].get('class') == 'virtual_transfer_pump']
    
    if not pump_nodes:
        raise ValueError("没有找到可用的转移泵设备")
    
    pump_id = pump_nodes[0]
    
    # 查找过滤设备（可选，如果有专门的过滤设备）
    filter_nodes = [node for node in G.nodes() 
                   if G.nodes[node].get('class') == 'virtual_filter']
    
    filter_id = filter_nodes[0] if filter_nodes else None
    
    # 查找洗脱溶剂容器（如果需要洗脱）
    eluting_vessel = None
    if eluting_solvent and eluting_volume > 0:
        eluting_vessel = f"flask_{eluting_solvent}"
        if eluting_vessel not in G.nodes():
            # 查找可用的溶剂容器
            available_vessels = [node for node in G.nodes() 
                               if node.startswith('flask_') and 
                               G.nodes[node].get('type') == 'container']
            if available_vessels:
                eluting_vessel = available_vessels[0]
            else:
                raise ValueError(f"没有找到洗脱溶剂容器 {eluting_solvent}")
    
    # 步骤1：将样品从源容器转移到过滤装置（模拟通过过滤介质）
    # 这里我们将过滤过程分解为多个转移步骤来模拟通过介质的过程
    
    # 首先转移样品（模拟样品通过过滤介质）
    action_sequence.append({
        "device_id": pump_id,
        "action_name": "transfer",
        "action_kwargs": {
            "from_vessel": from_vessel,
            "to_vessel": to_vessel,
            "volume": 0.0,  # 转移所有液体，体积由系统确定
            "amount": f"通过 {filter_through} 过滤",
            "time": residence_time if residence_time > 0 else 0.0,
            "viscous": False,
            "rinsing_solvent": "",
            "rinsing_volume": 0.0,
            "rinsing_repeats": 0,
            "solid": True  # 通过过滤介质可能涉及固体分离
        }
    })
    
    # 步骤2：如果有专门的过滤设备，使用过滤设备处理
    if filter_id:
        action_sequence.append({
            "device_id": filter_id,
            "action_name": "filter_sample",
            "action_kwargs": {
                "vessel": to_vessel,
                "filtrate_vessel": to_vessel,
                "stir": False,
                "stir_speed": 0.0,
                "temp": 25.0,
                "continue_heatchill": False,
                "volume": 0.0
            }
        })
    
    # 步骤3：洗脱操作（如果指定了洗脱溶剂和重复次数）
    if eluting_solvent and eluting_volume > 0 and eluting_repeats > 0 and eluting_vessel:
        for repeat in range(eluting_repeats):
            # 添加洗脱溶剂
            action_sequence.append({
                "device_id": pump_id,
                "action_name": "transfer",
                "action_kwargs": {
                    "from_vessel": eluting_vessel,
                    "to_vessel": to_vessel,
                    "volume": eluting_volume,
                    "amount": f"洗脱溶剂 {eluting_solvent} - 第 {repeat + 1} 次",
                    "time": 0.0,
                    "viscous": False,
                    "rinsing_solvent": "",
                    "rinsing_volume": 0.0,
                    "rinsing_repeats": 0,
                    "solid": False
                }
            })
            
            # 如果有过滤设备，再次过滤洗脱液
            if filter_id:
                action_sequence.append({
                    "device_id": filter_id,
                    "action_name": "filter_sample",
                    "action_kwargs": {
                        "vessel": to_vessel,
                        "filtrate_vessel": to_vessel,
                        "stir": False,
                        "stir_speed": 0.0,
                        "temp": 25.0,
                        "continue_heatchill": False,
                        "volume": eluting_volume
                    }
                })
    
    return action_sequence