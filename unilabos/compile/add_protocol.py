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
    生成添加试剂的协议序列
    
    流程：
    1. 找到包含目标试剂的试剂瓶
    2. 配置八通阀门到试剂瓶位置
    3. 使用注射器泵吸取试剂
    4. 配置八通阀门到反应器位置  
    5. 使用注射器泵推送试剂到反应器
    6. 如果需要，启动搅拌
    """  
    action_sequence = []  
    
    # 验证目标容器存在
    if vessel not in G.nodes():
        raise ValueError(f"目标容器 {vessel} 不存在")
    
    # 如果指定了体积，执行液体转移  
    if volume > 0:  
        # 1. 查找注射器泵 (transfer pump)
        transfer_pump_nodes = [node for node in G.nodes() 
                              if G.nodes[node].get('class') == 'virtual_transfer_pump']
        
        if not transfer_pump_nodes:
            raise ValueError("没有找到可用的注射器泵 (virtual_transfer_pump)")
            
        transfer_pump_id = transfer_pump_nodes[0]
        
        # 2. 查找八通阀门
        multiway_valve_nodes = [node for node in G.nodes() 
                               if G.nodes[node].get('class') == 'virtual_multiway_valve']
        
        if not multiway_valve_nodes:
            raise ValueError("没有找到可用的八通阀门 (virtual_multiway_valve)")
            
        valve_id = multiway_valve_nodes[0]
        
        # 3. 查找包含指定试剂的试剂瓶
        reagent_vessel = None
        available_flasks = [node for node in G.nodes() 
                           if node.startswith('flask_') 
                           and G.nodes[node].get('type') == 'container']
        
        # 简化：使用第一个可用的试剂瓶，实际应该根据试剂名称匹配
        if available_flasks:
            reagent_vessel = available_flasks[0]
        else:
            raise ValueError("没有找到可用的试剂容器")
        
        # 4. 获取试剂瓶和反应器对应的阀门位置
        # 这需要根据实际连接图来确定，这里假设：
        reagent_valve_position = 1  # 试剂瓶连接到阀门位置1
        reactor_valve_position = 2  # 反应器连接到阀门位置2
        
        # 5. 执行添加操作序列
        
        # 5.1 设置阀门到试剂瓶位置
        action_sequence.append({
            "device_id": valve_id,
            "action_name": "set_position",
            "action_kwargs": {
                "position": reagent_valve_position
            }
        })
        
        # 5.2 使用注射器泵从试剂瓶吸取液体
        action_sequence.append({
            "device_id": transfer_pump_id,
            "action_name": "transfer",
            "action_kwargs": {
                "from_vessel": reagent_vessel,
                "to_vessel": transfer_pump_id,  # 吸入到注射器
                "volume": volume,
                "amount": amount,
                "time": time / 2,  # 吸取时间为总时间的一半
                "viscous": viscous,
                "rinsing_solvent": "",
                "rinsing_volume": 0.0,
                "rinsing_repeats": 0,
                "solid": False
            }
        })
        
        # 5.3 设置阀门到反应器位置
        action_sequence.append({
            "device_id": valve_id,
            "action_name": "set_position",
            "action_kwargs": {
                "position": reactor_valve_position
            }
        })
        
        # 5.4 使用注射器泵将液体推送到反应器
        action_sequence.append({
            "device_id": transfer_pump_id,
            "action_name": "transfer",
            "action_kwargs": {
                "from_vessel": transfer_pump_id,  # 从注射器推出
                "to_vessel": vessel,
                "volume": volume,
                "amount": amount,
                "time": time / 2,  # 推送时间为总时间的一半
                "viscous": viscous,
                "rinsing_solvent": "",
                "rinsing_volume": 0.0,
                "rinsing_repeats": 0,
                "solid": False
            }
        })
      
    # 6. 如果需要搅拌，启动搅拌器
    if stir:  
        stirrer_nodes = [node for node in G.nodes()   
                        if G.nodes[node].get('class') == 'virtual_stirrer']  
          
        if stirrer_nodes:  
            stirrer_id = stirrer_nodes[0]  
            action_sequence.append({  
                "device_id": stirrer_id,  
                "action_name": "start_stir",
                "action_kwargs": {  
                    "vessel": vessel,
                    "stir_speed": stir_speed,
                    "purpose": f"添加 {reagent} 后搅拌混合"
                }  
            })  
        else:
            print("警告：需要搅拌但未找到搅拌设备")
      
    return action_sequence


def find_valve_position_for_vessel(G: nx.DiGraph, valve_id: str, vessel_id: str) -> int:
    """
    根据连接图找到容器对应的阀门位置
    
    Args:
        G: 网络图
        valve_id: 阀门设备ID
        vessel_id: 容器ID
    
    Returns:
        int: 阀门位置编号 (1-8)
    """
    # 查找阀门到容器的连接
    edges = G.edges(data=True)
    
    for source, target, data in edges:
        if source == valve_id and target == vessel_id:
            # 从连接数据中提取端口信息
            port_info = data.get('port', {})
            valve_port = port_info.get(valve_id, '')
            
            # 解析端口名称获取位置编号
            if valve_port.startswith('multiway-valve-port-'):
                position = valve_port.split('-')[-1]
                return int(position)
    
    # 默认返回位置1
    return 1


def generate_add_with_autodiscovery(
    G: nx.DiGraph,  
    vessel: str,  
    reagent: str,  
    volume: float,  
    **kwargs
) -> List[Dict[str, Any]]:
    """
    智能添加协议生成器 - 自动发现设备连接关系
    """
    action_sequence = []
    
    # 查找必需的设备
    devices = {
        'transfer_pump': None,
        'multiway_valve': None,
        'stirrer': None
    }
    
    for node in G.nodes():
        node_class = G.nodes[node].get('class')
        if node_class == 'virtual_transfer_pump':
            devices['transfer_pump'] = node
        elif node_class == 'virtual_multiway_valve':
            devices['multiway_valve'] = node
        elif node_class == 'virtual_stirrer':
            devices['stirrer'] = node
    
    # 验证必需设备
    if not devices['transfer_pump']:
        raise ValueError("缺少注射器泵设备")
    if not devices['multiway_valve']:
        raise ValueError("缺少八通阀门设备")
    
    # 查找试剂容器
    reagent_vessels = [node for node in G.nodes() 
                      if node.startswith('flask_') 
                      and G.nodes[node].get('type') == 'container']
    
    if not reagent_vessels:
        raise ValueError("没有找到试剂容器")
    
    # 执行添加流程
    reagent_vessel = reagent_vessels[0]
    reagent_pos = find_valve_position_for_vessel(G, devices['multiway_valve'], reagent_vessel)
    reactor_pos = find_valve_position_for_vessel(G, devices['multiway_valve'], vessel)
    
    # 生成操作序列
    action_sequence.extend([
        # 切换到试剂瓶
        {
            "device_id": devices['multiway_valve'],
            "action_name": "set_position",
            "action_kwargs": {"position": reagent_pos}
        },
        # 吸取试剂
        {
            "device_id": devices['transfer_pump'],
            "action_name": "transfer",
            "action_kwargs": {
                "from_vessel": reagent_vessel,
                "to_vessel": devices['transfer_pump'],
                "volume": volume,
                "amount": kwargs.get('amount', ''),
                "time": kwargs.get('time', 10.0) / 2,
                "viscous": kwargs.get('viscous', False),
                "rinsing_solvent": "",
                "rinsing_volume": 0.0,
                "rinsing_repeats": 0,
                "solid": False
            }
        },
        # 切换到反应器
        {
            "device_id": devices['multiway_valve'],
            "action_name": "set_position",
            "action_kwargs": {"position": reactor_pos}
        },
        # 推送到反应器
        {
            "device_id": devices['transfer_pump'],
            "action_name": "transfer",
            "action_kwargs": {
                "from_vessel": devices['transfer_pump'],
                "to_vessel": vessel,
                "volume": volume,
                "amount": kwargs.get('amount', ''),
                "time": kwargs.get('time', 10.0) / 2,
                "viscous": kwargs.get('viscous', False),
                "rinsing_solvent": "",
                "rinsing_volume": 0.0,
                "rinsing_repeats": 0,
                "solid": False
            }
        }
    ])
    
    # 如果需要搅拌
    if kwargs.get('stir', False) and devices['stirrer']:
        action_sequence.append({
            "device_id": devices['stirrer'],
            "action_name": "start_stir",
            "action_kwargs": {
                "vessel": vessel,
                "stir_speed": kwargs.get('stir_speed', 300.0),
                "purpose": f"添加 {reagent} 后混合"
            }
        })
    
    return action_sequence