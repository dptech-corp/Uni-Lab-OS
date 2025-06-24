from typing import List, Dict, Any
import networkx as nx

def generate_wash_solid_protocol(
    G: nx.DiGraph,
    vessel: str,
    solvent: str,
    volume: float,
    filtrate_vessel: str = "",
    temp: float = 25.0,
    stir: bool = False,
    stir_speed: float = 0.0,
    time: float = 0.0,
    repeats: int = 1
) -> List[Dict[str, Any]]:
    """
    生成固体清洗的协议序列
    
    Args:
        G: 有向图，节点为设备和容器
        vessel: 装有固体物质的容器名称
        solvent: 用于清洗固体的溶剂名称
        volume: 清洗溶剂的体积
        filtrate_vessel: 滤液要收集到的容器名称，可选参数
        temp: 清洗时的温度，可选参数
        stir: 是否在清洗过程中搅拌，默认为 False
        stir_speed: 搅拌速度，可选参数
        time: 清洗的时间，可选参数
        repeats: 清洗操作的重复次数，默认为 1
    
    Returns:
        List[Dict[str, Any]]: 固体清洗操作的动作序列
    
    Raises:
        ValueError: 当找不到必要的设备时抛出异常
    
    Examples:
        wash_solid_protocol = generate_wash_solid_protocol(
            G, "reactor", "ethanol", 100.0, "waste_flask", 60.0, True, 300.0, 600.0, 3
        )
    """
    action_sequence = []
    
    # 验证容器是否存在
    if vessel not in G.nodes():
        raise ValueError(f"固体容器 {vessel} 不存在于图中")
    
    if filtrate_vessel and filtrate_vessel not in G.nodes():
        raise ValueError(f"滤液容器 {filtrate_vessel} 不存在于图中")
    
    # 查找转移泵设备（用于添加溶剂和转移滤液）
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
    
    # 查找过滤设备（用于分离固体和滤液）
    filter_nodes = [node for node in G.nodes() 
                   if G.nodes[node].get('class') == 'virtual_filter']
    
    filter_id = filter_nodes[0] if filter_nodes else None
    
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
    
    # 如果没有指定滤液容器，使用废液容器
    if not filtrate_vessel:
        waste_vessels = [node for node in G.nodes() 
                        if 'waste' in node.lower() and 
                        G.nodes[node].get('type') == 'container']
        filtrate_vessel = waste_vessels[0] if waste_vessels else "waste_flask"
    
    # 重复清洗操作
    for repeat in range(repeats):
        repeat_num = repeat + 1
        
        # 步骤1：如果需要加热，先设置温度
        if temp > 25.0 and heatchill_id:
            action_sequence.append({
                "device_id": heatchill_id,
                "action_name": "heat_chill_start",
                "action_kwargs": {
                    "vessel": vessel,
                    "temp": temp,
                    "purpose": f"固体清洗 - 第 {repeat_num} 次"
                }
            })
        
        # 步骤2：添加清洗溶剂到固体容器
        action_sequence.append({
            "device_id": pump_id,
            "action_name": "transfer",
            "action_kwargs": {
                "from_vessel": solvent_vessel,
                "to_vessel": vessel,
                "volume": volume,
                "amount": f"清洗溶剂 {solvent} - 第 {repeat_num} 次",
                "time": 0.0,
                "viscous": False,
                "rinsing_solvent": "",
                "rinsing_volume": 0.0,
                "rinsing_repeats": 0,
                "solid": False
            }
        })
        
        # 步骤3：如果需要搅拌，开始搅拌
        if stir and stir_speed > 0 and stirrer_id:
            if time > 0:
                # 定时搅拌
                action_sequence.append({
                    "device_id": stirrer_id,
                    "action_name": "stir",
                    "action_kwargs": {
                        "stir_time": time,
                        "stir_speed": stir_speed,
                        "settling_time": 30.0  # 搅拌后静置30秒
                    }
                })
            else:
                # 开始搅拌（需要手动停止）
                action_sequence.append({
                    "device_id": stirrer_id,
                    "action_name": "start_stir",
                    "action_kwargs": {
                        "vessel": vessel,
                        "stir_speed": stir_speed,
                        "purpose": f"固体清洗搅拌 - 第 {repeat_num} 次"
                    }
                })
        
        # 步骤4：如果指定了清洗时间但没有搅拌，等待清洗时间
        if time > 0 and (not stir or stir_speed == 0):
            # 这里可以添加等待操作，暂时跳过
            pass
        
        # 步骤5：如果有搅拌且没有定时，停止搅拌
        if stir and stir_speed > 0 and time == 0 and stirrer_id:
            action_sequence.append({
                "device_id": stirrer_id,
                "action_name": "stop_stir",
                "action_kwargs": {
                    "vessel": vessel
                }
            })
        
        # 步骤6：过滤分离固体和滤液
        if filter_id:
            action_sequence.append({
                "device_id": filter_id,
                "action_name": "filter_sample",
                "action_kwargs": {
                    "vessel": vessel,
                    "filtrate_vessel": filtrate_vessel,
                    "stir": False,
                    "stir_speed": 0.0,
                    "temp": temp,
                    "continue_heatchill": temp > 25.0,
                    "volume": volume
                }
            })
        else:
            # 没有专门的过滤设备，使用转移泵模拟过滤过程
            # 将滤液转移到滤液容器
            action_sequence.append({
                "device_id": pump_id,
                "action_name": "transfer",
                "action_kwargs": {
                    "from_vessel": vessel,
                    "to_vessel": filtrate_vessel,
                    "volume": volume,
                    "amount": f"转移滤液 - 第 {repeat_num} 次清洗",
                    "time": 0.0,
                    "viscous": False,
                    "rinsing_solvent": "",
                    "rinsing_volume": 0.0,
                    "rinsing_repeats": 0,
                    "solid": False
                }
            })
        
        # 步骤7：如果加热了，停止加热（在最后一次清洗后）
        if temp > 25.0 and heatchill_id and repeat_num == repeats:
            action_sequence.append({
                "device_id": heatchill_id,
                "action_name": "heat_chill_stop",
                "action_kwargs": {
                    "vessel": vessel
                }
            })
    
    return action_sequence