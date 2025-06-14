from typing import List, Dict, Any
import networkx as nx

def generate_clean_vessel_protocol(
    G: nx.DiGraph,
    vessel: str,
    solvent: str,
    volume: float,
    temp: float,
    repeats: int = 1
) -> List[Dict[str, Any]]:
    """
    生成容器清洗操作的协议序列，使用transfer操作实现清洗
    
    Args:
        G: 有向图，节点为设备和容器
        vessel: 要清洗的容器名称
        solvent: 用于清洗容器的溶剂名称
        volume: 清洗溶剂的体积
        temp: 清洗时的温度
        repeats: 清洗操作的重复次数，默认为 1
    
    Returns:
        List[Dict[str, Any]]: 容器清洗操作的动作序列
    
    Raises:
        ValueError: 当找不到必要的设备时抛出异常
    
    Examples:
        clean_vessel_protocol = generate_clean_vessel_protocol(G, "reactor", "water", 50.0, 25.0, 2)
    """
    action_sequence = []
    
    # 查找虚拟转移泵设备进行清洗操作
    pump_nodes = [node for node in G.nodes() 
                  if G.nodes[node].get('class') == 'virtual_transfer_pump']
    
    if not pump_nodes:
        raise ValueError("没有找到可用的转移泵设备进行容器清洗")
    
    pump_id = pump_nodes[0]
    
    # 验证容器是否存在
    if vessel not in G.nodes():
        raise ValueError(f"容器 {vessel} 不存在于图中")
    
    # 查找溶剂容器
    solvent_vessel = f"flask_{solvent}"
    if solvent_vessel not in G.nodes():
        raise ValueError(f"溶剂容器 {solvent_vessel} 不存在于图中")
    
    # 查找废液容器
    waste_vessel = "flask_waste"
    if waste_vessel not in G.nodes():
        raise ValueError(f"废液容器 {waste_vessel} 不存在于图中")
    
    # 查找加热设备（如果需要加热）
    heatchill_nodes = [node for node in G.nodes() 
                      if G.nodes[node].get('class') == 'virtual_heatchill']
    
    heatchill_id = heatchill_nodes[0] if heatchill_nodes else None
    
    # 执行清洗操作序列
    for repeat in range(repeats):
        # 1. 如果需要加热，先设置温度
        if temp > 25.0 and heatchill_id:
            action_sequence.append({
                "device_id": heatchill_id,
                "action_name": "heat_chill_start",
                "action_kwargs": {
                    "vessel": vessel,
                    "temp": temp,
                    "purpose": "cleaning"
                }
            })
        
        # 2. 使用transfer操作：从溶剂容器转移清洗溶剂到目标容器
        action_sequence.append({
            "device_id": pump_id,
            "action_name": "transfer",
            "action_kwargs": {
                "from_vessel": solvent_vessel,
                "to_vessel": vessel,
                "volume": volume,
                "amount": f"cleaning with {solvent} - cycle {repeat + 1}",
                "time": 0.0,
                "viscous": False,
                "rinsing_solvent": "",
                "rinsing_volume": 0.0,
                "rinsing_repeats": 0,
                "solid": False
            }
        })
        
        # 3. 等待清洗作用时间（可选，可以添加wait操作）
        # 这里省略wait操作，直接进行下一步
        
        # 4. 将清洗后的溶剂转移到废液容器
        action_sequence.append({
            "device_id": pump_id,
            "action_name": "transfer",
            "action_kwargs": {
                "from_vessel": vessel,
                "to_vessel": waste_vessel,
                "volume": volume,
                "amount": f"waste from cleaning {vessel} - cycle {repeat + 1}",
                "time": 0.0,
                "viscous": False,
                "rinsing_solvent": "",
                "rinsing_volume": 0.0,
                "rinsing_repeats": 0,
                "solid": False
            }
        })
        
        # 5. 如果加热了，停止加热
        if temp > 25.0 and heatchill_id:
            action_sequence.append({
                "device_id": heatchill_id,
                "action_name": "heat_chill_stop",
                "action_kwargs": {
                    "vessel": vessel
                }
            })
    
    return action_sequence