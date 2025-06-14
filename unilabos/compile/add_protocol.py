import networkx as nx  
from typing import List, Dict, Any  
  
def generate_add_protocol(  
    G: nx.DiGraph,  
    vessel: str,  
    reagent: str,  
    volume: float,  
    mass: float,  
    amount: str,  
    time: float,  
    stir: bool,  
    stir_speed: float,  
    viscous: bool,  
    purpose: str  
) -> List[Dict[str, Any]]:  
    """  
    生成添加试剂的协议序列 - 严格按照 Add.action
    """  
    action_sequence = []  
      
    # 如果指定了体积，执行液体转移  
    if volume > 0:  
        # 查找可用的试剂瓶
        available_flasks = [node for node in G.nodes() 
                           if node.startswith('flask_') 
                           and G.nodes[node].get('type') == 'container']
        
        if not available_flasks:
            raise ValueError("没有找到可用的试剂容器")
            
        reagent_vessel = available_flasks[0]
        
        # 查找泵设备
        pump_nodes = [node for node in G.nodes() 
                     if G.nodes[node].get('class') == 'virtual_pump']
        
        if pump_nodes:
            pump_id = pump_nodes[0]
            action_sequence.append({
                "device_id": pump_id,
                "action_name": "transfer",
                "action_kwargs": {
                    "from_vessel": reagent_vessel,
                    "to_vessel": vessel,
                    "volume": volume,
                    "amount": amount,
                    "time": time,
                    "viscous": viscous,
                    "rinsing_solvent": "",
                    "rinsing_volume": 0.0,
                    "rinsing_repeats": 0,
                    "solid": False
                }
            })
      
    # 如果需要搅拌，使用 StartStir 而不是 Stir  
    if stir:  
        stirrer_nodes = [node for node in G.nodes()   
                        if G.nodes[node].get('class') == 'virtual_stirrer']  
          
        if stirrer_nodes:  
            stirrer_id = stirrer_nodes[0]  
            action_sequence.append({  
                "device_id": stirrer_id,  
                "action_name": "start_stir",  # 使用 start_stir 而不是 stir
                "action_kwargs": {  
                    "vessel": vessel,
                    "stir_speed": stir_speed,
                    "purpose": f"添加 {reagent} 后搅拌"
                }  
            })  
      
    return action_sequence