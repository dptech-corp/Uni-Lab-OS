from typing import List, Dict, Any
import networkx as nx

def generate_dissolve_protocol(
    G: nx.DiGraph,
    vessel: str,
    solvent: str,
    volume: float,
    amount: str = "",
    temp: float = 25.0,
    time: float = 0.0,
    stir_speed: float = 0.0
) -> List[Dict[str, Any]]:
    """
    生成溶解操作的协议序列
    
    Args:
        G: 有向图，节点为设备和容器
        vessel: 装有要溶解物质的容器名称
        solvent: 用于溶解物质的溶剂名称
        volume: 溶剂的体积，可选参数
        amount: 要溶解物质的量，可选参数
        temp: 溶解时的温度，可选参数
        time: 溶解的时间，可选参数
        stir_speed: 搅拌速度，可选参数
    
    Returns:
        List[Dict[str, Any]]: 溶解操作的动作序列
    
    Raises:
        ValueError: 当找不到必要的设备时抛出异常
    
    Examples:
        dissolve_protocol = generate_dissolve_protocol(G, "reactor", "water", 100.0, "NaCl 5g", 60.0, 300.0, 500.0)
    """
    action_sequence = []
    
    # 验证容器是否存在
    if vessel not in G.nodes():
        raise ValueError(f"容器 {vessel} 不存在于图中")
    
    # 查找溶剂容器
    solvent_vessel = f"flask_{solvent}"
    if solvent_vessel not in G.nodes():
        # 如果没有找到特定溶剂容器，查找可用的源容器
        available_vessels = [node for node in G.nodes() 
                           if node.startswith('flask_') and 
                           G.nodes[node].get('type') == 'container']
        if available_vessels:
            solvent_vessel = available_vessels[0]
        else:
            raise ValueError(f"没有找到溶剂容器 {solvent}")
    
    # 查找转移泵设备
    pump_nodes = [node for node in G.nodes() 
                  if G.nodes[node].get('class') == 'virtual_transfer_pump']
    
    if not pump_nodes:
        raise ValueError("没有找到可用的转移泵设备")
    
    pump_id = pump_nodes[0]
    
    # 查找加热设备（如果需要加热）
    heatchill_nodes = [node for node in G.nodes() 
                      if G.nodes[node].get('class') == 'virtual_heatchill']
    
    heatchill_id = heatchill_nodes[0] if heatchill_nodes else None
    
    # 查找搅拌设备（如果需要搅拌）
    stirrer_nodes = [node for node in G.nodes() 
                    if G.nodes[node].get('class') == 'virtual_stirrer']
    
    stirrer_id = stirrer_nodes[0] if stirrer_nodes else None
    
    # 步骤1：如果需要加热，先设置温度
    if temp > 25.0 and heatchill_id:
        action_sequence.append({
            "device_id": heatchill_id,
            "action_name": "heat_chill_start",
            "action_kwargs": {
                "vessel": vessel,
                "temp": temp,
                "purpose": "dissolution"
            }
        })
    
    # 步骤2：添加溶剂到容器中
    if volume > 0:
        action_sequence.append({
            "device_id": pump_id,
            "action_name": "transfer",
            "action_kwargs": {
                "from_vessel": solvent_vessel,
                "to_vessel": vessel,
                "volume": volume,
                "amount": f"solvent {solvent} for dissolving {amount}",
                "time": 0.0,
                "viscous": False,
                "rinsing_solvent": "",
                "rinsing_volume": 0.0,
                "rinsing_repeats": 0,
                "solid": False
            }
        })
    
    # 步骤3：如果需要搅拌，开始搅拌
    if stir_speed > 0 and stirrer_id:
        action_sequence.append({
            "device_id": stirrer_id,
            "action_name": "start_stir",
            "action_kwargs": {
                "vessel": vessel,
                "stir_speed": stir_speed,
                "purpose": f"dissolving {amount} in {solvent}"
            }
        })
    
    # 步骤4：如果指定了溶解时间，等待溶解完成
    if time > 0:
        # 这里可以添加等待操作，或者使用搅拌操作来模拟溶解时间
        if stirrer_id and stir_speed > 0:
            # 停止之前的搅拌，使用定时搅拌
            action_sequence.append({
                "device_id": stirrer_id,
                "action_name": "stop_stir",
                "action_kwargs": {
                    "vessel": vessel
                }
            })
            
            # 开始定时搅拌
            action_sequence.append({
                "device_id": stirrer_id,
                "action_name": "stir",
                "action_kwargs": {
                    "stir_time": time,
                    "stir_speed": stir_speed,
                    "settling_time": 10.0  # 搅拌后静置10秒
                }
            })
    
    # 步骤5：如果加热了，停止加热
    if temp > 25.0 and heatchill_id:
        action_sequence.append({
            "device_id": heatchill_id,
            "action_name": "heat_chill_stop",
            "action_kwargs": {
                "vessel": vessel
            }
        })
    
    # 步骤6：如果还在搅拌，停止搅拌（除非已经用定时搅拌）
    if stir_speed > 0 and stirrer_id and time == 0:
        action_sequence.append({
            "device_id": stirrer_id,
            "action_name": "stop_stir",
            "action_kwargs": {
                "vessel": vessel
            }
        })
    
    return action_sequence