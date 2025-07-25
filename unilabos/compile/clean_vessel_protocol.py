from typing import List, Dict, Any
import networkx as nx
from .pump_protocol import generate_pump_protocol


def find_solvent_vessel(G: nx.DiGraph, solvent: str) -> str:
    """
    查找溶剂容器，支持多种匹配模式：
    1. 容器名称匹配（如 flask_water, reagent_bottle_1-DMF）
    2. 容器内液体类型匹配（如 liquid_type: "DMF", "ethanol"）
    """
    print(f"CLEAN_VESSEL: 正在查找溶剂 '{solvent}' 的容器...")
    
    # 第一步：通过容器名称匹配
    possible_names = [
        f"flask_{solvent}",           # flask_water, flask_ethanol
        f"bottle_{solvent}",          # bottle_water, bottle_ethanol  
        f"vessel_{solvent}",          # vessel_water, vessel_ethanol
        f"{solvent}_flask",           # water_flask, ethanol_flask
        f"{solvent}_bottle",          # water_bottle, ethanol_bottle
        f"{solvent}",                 # 直接用溶剂名
        f"solvent_{solvent}",         # solvent_water, solvent_ethanol
        f"reagent_bottle_{solvent}",  # reagent_bottle_DMF
    ]
    
    # 尝试名称匹配
    for vessel_name in possible_names:
        if vessel_name in G.nodes():
            print(f"CLEAN_VESSEL: 通过名称匹配找到容器: {vessel_name}")
            return vessel_name
    
    # 第二步：通过模糊名称匹配（名称中包含溶剂名）
    for node_id in G.nodes():
        if G.nodes[node_id].get('type') == 'container':
            # 检查节点ID或名称中是否包含溶剂名
            node_name = G.nodes[node_id].get('name', '').lower()
            if (solvent.lower() in node_id.lower() or 
                solvent.lower() in node_name):
                print(f"CLEAN_VESSEL: 通过模糊名称匹配找到容器: {node_id} (名称: {node_name})")
                return node_id
    
    # 第三步：通过液体类型匹配
    for node_id in G.nodes():
        if G.nodes[node_id].get('type') == 'container':
            vessel_data = G.nodes[node_id].get('data', {})
            liquids = vessel_data.get('liquid', [])
            
            for liquid in liquids:
                if isinstance(liquid, dict):
                    # 支持两种格式的液体类型字段
                    liquid_type = liquid.get('liquid_type') or liquid.get('name', '')
                    reagent_name = vessel_data.get('reagent_name', '')
                    config_reagent = G.nodes[node_id].get('config', {}).get('reagent', '')
                    
                    # 检查多个可能的字段
                    if (liquid_type.lower() == solvent.lower() or 
                        reagent_name.lower() == solvent.lower() or
                        config_reagent.lower() == solvent.lower()):
                        print(f"CLEAN_VESSEL: 通过液体类型匹配找到容器: {node_id}")
                        print(f"  - liquid_type: {liquid_type}")
                        print(f"  - reagent_name: {reagent_name}")
                        print(f"  - config.reagent: {config_reagent}")
                        return node_id
    
    # 第四步：列出所有可用的容器信息帮助调试
    available_containers = []
    for node_id in G.nodes():
        if G.nodes[node_id].get('type') == 'container':
            vessel_data = G.nodes[node_id].get('data', {})
            config_data = G.nodes[node_id].get('config', {})
            liquids = vessel_data.get('liquid', [])
            
            container_info = {
                'id': node_id,
                'name': G.nodes[node_id].get('name', ''),
                'liquid_types': [],
                'reagent_name': vessel_data.get('reagent_name', ''),
                'config_reagent': config_data.get('reagent', '')
            }
            
            for liquid in liquids:
                if isinstance(liquid, dict):
                    liquid_type = liquid.get('liquid_type') or liquid.get('name', '')
                    if liquid_type:
                        container_info['liquid_types'].append(liquid_type)
            
            available_containers.append(container_info)
    
    print(f"CLEAN_VESSEL: 可用容器列表:")
    for container in available_containers:
        print(f"  - {container['id']}: {container['name']}")
        print(f"    液体类型: {container['liquid_types']}")
        print(f"    试剂名称: {container['reagent_name']}")
        print(f"    配置试剂: {container['config_reagent']}")
    
    raise ValueError(f"未找到溶剂 '{solvent}' 的容器。尝试了名称匹配: {possible_names}")


def find_solvent_vessel_by_any_match(G: nx.DiGraph, solvent: str) -> str:
    """
    增强版溶剂容器查找，支持各种匹配方式的别名函数
    """
    return find_solvent_vessel(G, solvent)


def find_waste_vessel(G: nx.DiGraph) -> str:
    """
    查找废液容器
    """
    possible_waste_names = [
        "waste_workup",
        "flask_waste", 
        "bottle_waste",
        "waste",
        "waste_vessel",
        "waste_container"
    ]
    
    for waste_name in possible_waste_names:
        if waste_name in G.nodes():
            return waste_name
    
    raise ValueError(f"未找到废液容器。尝试了以下名称: {possible_waste_names}")


def find_connected_heatchill(G: nx.DiGraph, vessel: str) -> str:
    """
    查找与指定容器相连的加热冷却设备
    """
    # 查找所有加热冷却设备节点
    heatchill_nodes = [node for node in G.nodes() 
                      if (G.nodes[node].get('class') or '') == 'virtual_heatchill']
    
    # 检查哪个加热设备与目标容器相连（机械连接）
    for heatchill in heatchill_nodes:
        if G.has_edge(heatchill, vessel) or G.has_edge(vessel, heatchill):
            return heatchill
    
    # 如果没有直接连接，返回第一个可用的加热设备
    if heatchill_nodes:
        return heatchill_nodes[0]
    
    return None  # 没有加热设备也可以工作，只是不能加热


def generate_clean_vessel_protocol(
    G: nx.DiGraph,
    vessel: str,
    solvent: str,
    volume: float,
    temp: float,
    repeats: int = 1
) -> List[Dict[str, Any]]:
    """
    生成容器清洗操作的协议序列，复用 pump_protocol 的成熟算法
    
    清洗流程：
    1. 查找溶剂容器和废液容器
    2. 如果需要加热，启动加热设备
    3. 重复以下操作 repeats 次：
       a. 使用 pump_protocol 将溶剂从溶剂容器转移到目标容器
       b. (可选) 等待清洗作用时间
       c. 使用 pump_protocol 将清洗液从目标容器转移到废液容器
    4. 如果加热了，停止加热
    
    Args:
        G: 有向图，节点为设备和容器，边为流体管道
        vessel: 要清洗的容器名称
        solvent: 用于清洗的溶剂名称  
        volume: 每次清洗使用的溶剂体积
        temp: 清洗时的温度
        repeats: 清洗操作的重复次数，默认为 1
    
    Returns:
        List[Dict[str, Any]]: 容器清洗操作的动作序列
    
    Raises:
        ValueError: 当找不到必要的容器或设备时抛出异常
    
    Examples:
        clean_protocol = generate_clean_vessel_protocol(G, "main_reactor", "water", 100.0, 60.0, 2)
    """
    action_sequence = []
    
    print(f"CLEAN_VESSEL: 开始生成容器清洗协议")
    print(f"  - 目标容器: {vessel}")
    print(f"  - 清洗溶剂: {solvent}")
    print(f"  - 清洗体积: {volume} mL")
    print(f"  - 清洗温度: {temp}°C")
    print(f"  - 重复次数: {repeats}")
    
    # 验证目标容器存在
    if vessel not in G.nodes():
        raise ValueError(f"目标容器 '{vessel}' 不存在于系统中")
    
    # 查找溶剂容器
    try:
        solvent_vessel = find_solvent_vessel(G, solvent)
        print(f"CLEAN_VESSEL: 找到溶剂容器: {solvent_vessel}")
    except ValueError as e:
        raise ValueError(f"无法找到溶剂容器: {str(e)}")
    
    # 查找废液容器
    try:
        waste_vessel = find_waste_vessel(G)
        print(f"CLEAN_VESSEL: 找到废液容器: {waste_vessel}")
    except ValueError as e:
        raise ValueError(f"无法找到废液容器: {str(e)}")
    
    # 查找加热设备（可选）
    heatchill_id = find_connected_heatchill(G, vessel)
    if heatchill_id:
        print(f"CLEAN_VESSEL: 找到加热设备: {heatchill_id}")
    else:
        print(f"CLEAN_VESSEL: 未找到加热设备，将在室温下清洗")
    
    # 第一步：如果需要加热且有加热设备，启动加热
    if temp > 25.0 and heatchill_id:
        print(f"CLEAN_VESSEL: 启动加热至 {temp}°C")
        heatchill_start_action = {
            "device_id": heatchill_id,
            "action_name": "heat_chill_start",
            "action_kwargs": {
                "vessel": vessel,
                "temp": temp,
                "purpose": f"cleaning with {solvent}"
            }
        }
        action_sequence.append(heatchill_start_action)
        
        # 等待温度稳定
        wait_action = {
            "action_name": "wait", 
            "action_kwargs": {"time": 30}  # 等待30秒让温度稳定
        }
        action_sequence.append(wait_action)
    
    # 第二步：重复清洗操作
    for repeat in range(repeats):
        print(f"CLEAN_VESSEL: 执行第 {repeat + 1} 次清洗")
        
        # 2a. 使用 pump_protocol 将溶剂转移到目标容器
        print(f"CLEAN_VESSEL: 将 {volume} mL {solvent} 转移到 {vessel}")
        try:
            # 调用成熟的 pump_protocol 算法
            add_solvent_actions = generate_pump_protocol(
                G=G,
                from_vessel=solvent_vessel,
                to_vessel=vessel,
                volume=volume,
                flowrate=2.5,  # 适中的流速，避免飞溅
                transfer_flowrate=2.5
            )
            action_sequence.extend(add_solvent_actions)
        except Exception as e:
            raise ValueError(f"无法将溶剂转移到容器: {str(e)}")
        
        # 2b. 等待清洗作用时间（让溶剂充分清洗容器）
        cleaning_wait_time = 60 if temp > 50.0 else 30  # 高温下等待更久
        print(f"CLEAN_VESSEL: 等待清洗作用 {cleaning_wait_time} 秒")
        wait_action = {
            "action_name": "wait", 
            "action_kwargs": {"time": cleaning_wait_time}
        }
        action_sequence.append(wait_action)
        
        # 2c. 使用 pump_protocol 将清洗液转移到废液容器
        print(f"CLEAN_VESSEL: 将清洗液从 {vessel} 转移到废液容器")
        try:
            # 调用成熟的 pump_protocol 算法
            remove_waste_actions = generate_pump_protocol(
                G=G,
                from_vessel=vessel,
                to_vessel=waste_vessel,
                volume=volume,
                flowrate=2.5,  # 适中的流速
                transfer_flowrate=2.5
            )
            action_sequence.extend(remove_waste_actions)
        except Exception as e:
            raise ValueError(f"无法将清洗液转移到废液容器: {str(e)}")
        
        # 2d. 清洗循环间的短暂等待
        if repeat < repeats - 1:  # 不是最后一次清洗
            print(f"CLEAN_VESSEL: 清洗循环间等待")
            wait_action = {
                "action_name": "wait", 
                "action_kwargs": {"time": 10}
            }
            action_sequence.append(wait_action)
    
    # 第三步：如果加热了，停止加热
    if temp > 25.0 and heatchill_id:
        print(f"CLEAN_VESSEL: 停止加热")
        heatchill_stop_action = {
            "device_id": heatchill_id,
            "action_name": "heat_chill_stop",
            "action_kwargs": {
                "vessel": vessel
            }
        }
        action_sequence.append(heatchill_stop_action)
    
    print(f"CLEAN_VESSEL: 生成了 {len(action_sequence)} 个动作")
    print(f"CLEAN_VESSEL: 清洗协议生成完成")
    
    return action_sequence


# 便捷函数：常用清洗方案
def generate_quick_clean_protocol(
    G: nx.DiGraph, 
    vessel: str, 
    solvent: str = "water", 
    volume: float = 100.0
) -> List[Dict[str, Any]]:
    """快速清洗：室温，单次清洗"""
    return generate_clean_vessel_protocol(G, vessel, solvent, volume, 25.0, 1)


def generate_thorough_clean_protocol(
    G: nx.DiGraph, 
    vessel: str, 
    solvent: str = "water", 
    volume: float = 150.0,
    temp: float = 60.0
) -> List[Dict[str, Any]]:
    """深度清洗：加热，多次清洗"""
    return generate_clean_vessel_protocol(G, vessel, solvent, volume, temp, 3)


def generate_organic_clean_protocol(
    G: nx.DiGraph, 
    vessel: str, 
    volume: float = 100.0
) -> List[Dict[str, Any]]:
    """有机清洗：先用有机溶剂，再用水清洗"""
    action_sequence = []
    
    # 第一步：有机溶剂清洗
    try:
        organic_actions = generate_clean_vessel_protocol(
            G, vessel, "acetone", volume, 25.0, 2
        )
        action_sequence.extend(organic_actions)
    except ValueError:
        # 如果没有丙酮，尝试乙醇
        try:
            organic_actions = generate_clean_vessel_protocol(
                G, vessel, "ethanol", volume, 25.0, 2
            )
            action_sequence.extend(organic_actions)
        except ValueError:
            print("警告：未找到有机溶剂，跳过有机清洗步骤")
    
    # 第二步：水清洗
    water_actions = generate_clean_vessel_protocol(
        G, vessel, "water", volume, 25.0, 2
    )
    action_sequence.extend(water_actions)
    
    return action_sequence


def get_vessel_liquid_volume(G: nx.DiGraph, vessel: str) -> float:
    """获取容器中的液体体积（修复版）"""
    if vessel not in G.nodes():
        return 0.0
    
    vessel_data = G.nodes[vessel].get('data', {})
    liquids = vessel_data.get('liquid', [])
    
    total_volume = 0.0
    for liquid in liquids:
        if isinstance(liquid, dict):
            # 支持两种格式：新格式 (name, volume) 和旧格式 (liquid_type, liquid_volume)
            volume = liquid.get('volume') or liquid.get('liquid_volume', 0.0)
            total_volume += volume
    
    return total_volume


def get_vessel_liquid_types(G: nx.DiGraph, vessel: str) -> List[str]:
    """获取容器中所有液体的类型"""
    if vessel not in G.nodes():
        return []
    
    vessel_data = G.nodes[vessel].get('data', {})
    liquids = vessel_data.get('liquid', [])
    
    liquid_types = []
    for liquid in liquids:
        if isinstance(liquid, dict):
            # 支持两种格式的液体类型字段
            liquid_type = liquid.get('liquid_type') or liquid.get('name', '')
            if liquid_type:
                liquid_types.append(liquid_type)
    
    return liquid_types


def find_vessel_by_content(G: nx.DiGraph, content: str) -> List[str]:
    """
    根据内容物查找所有匹配的容器
    返回匹配容器的ID列表
    """
    matching_vessels = []
    
    for node_id in G.nodes():
        if G.nodes[node_id].get('type') == 'container':
            # 检查容器名称匹配
            node_name = G.nodes[node_id].get('name', '').lower()
            if content.lower() in node_id.lower() or content.lower() in node_name:
                matching_vessels.append(node_id)
                continue
            
            # 检查液体类型匹配
            vessel_data = G.nodes[node_id].get('data', {})
            liquids = vessel_data.get('liquid', [])
            config_data = G.nodes[node_id].get('config', {})
            
            # 检查 reagent_name 和 config.reagent
            reagent_name = vessel_data.get('reagent_name', '').lower()
            config_reagent = config_data.get('reagent', '').lower()
            
            if (content.lower() == reagent_name or 
                content.lower() == config_reagent):
                matching_vessels.append(node_id)
                continue
            
            # 检查液体列表
            for liquid in liquids:
                if isinstance(liquid, dict):
                    liquid_type = liquid.get('liquid_type') or liquid.get('name', '')
                    if liquid_type.lower() == content.lower():
                        matching_vessels.append(node_id)
                        break
    
    return matching_vessels