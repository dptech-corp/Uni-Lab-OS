from typing import List, Dict, Any
import networkx as nx
from .pump_protocol import generate_pump_protocol


def get_vessel_liquid_volume(G: nx.DiGraph, vessel: str) -> float:
    """获取容器中的液体体积"""
    if vessel not in G.nodes():
        return 0.0
    
    vessel_data = G.nodes[vessel].get('data', {})
    liquids = vessel_data.get('liquid', [])
    
    total_volume = 0.0
    for liquid in liquids:
        if isinstance(liquid, dict) and 'liquid_volume' in liquid:
            total_volume += liquid['liquid_volume']
    
    return total_volume


def find_filter_device(G: nx.DiGraph) -> str:
    """查找过滤器设备"""
    filter_nodes = [node for node in G.nodes() 
                   if (G.nodes[node].get('class') or '') == 'virtual_filter']
    
    if filter_nodes:
        return filter_nodes[0]
    
    raise ValueError("系统中未找到过滤器设备")


def find_filter_vessel(G: nx.DiGraph) -> str:
    """查找过滤器专用容器"""
    possible_names = [
        "filter_vessel",        # 标准过滤器容器
        "filtration_vessel",    # 备选名称
        "vessel_filter",        # 备选名称
        "filter_unit",          # 备选名称
        "filter"               # 简单名称
    ]
    
    for vessel_name in possible_names:
        if vessel_name in G.nodes():
            return vessel_name
    
    raise ValueError(f"未找到过滤器容器。尝试了以下名称: {possible_names}")


def find_filtrate_vessel(G: nx.DiGraph, filtrate_vessel: str = "") -> str:
    """查找滤液收集容器"""
    if filtrate_vessel and filtrate_vessel in G.nodes():
        return filtrate_vessel
    
    # 自动查找滤液容器
    possible_names = [
        "filtrate_vessel",
        "collection_bottle_1",
        "collection_bottle_2",
        "waste_workup"
    ]
    
    for vessel_name in possible_names:
        if vessel_name in G.nodes():
            return vessel_name
    
    raise ValueError(f"未找到滤液收集容器。尝试了以下名称: {possible_names}")


def find_connected_heatchill(G: nx.DiGraph, vessel: str) -> str:
    """查找与指定容器相连的加热搅拌器"""
    # 查找所有加热搅拌器节点
    heatchill_nodes = [node for node in G.nodes() 
                      if G.nodes[node].get('class') == 'virtual_heatchill']
    
    # 检查哪个加热器与目标容器相连
    for heatchill in heatchill_nodes:
        if G.has_edge(heatchill, vessel) or G.has_edge(vessel, heatchill):
            return heatchill
    
    # 如果没有直接连接，返回第一个可用的加热器
    if heatchill_nodes:
        return heatchill_nodes[0]
    
    raise ValueError(f"未找到与容器 {vessel} 相连的加热搅拌器")


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
    生成过滤操作的协议序列，复用 pump_protocol 的成熟算法
    
    过滤流程：
    1. 液体转移：将待过滤溶液从源容器转移到过滤器
    2. 启动加热搅拌：设置温度和搅拌
    3. 执行过滤：通过过滤器分离固液
    4. (可选) 继续或停止加热搅拌
    
    Args:
        G: 有向图，节点为设备和容器，边为流体管道
        vessel: 包含待过滤溶液的容器名称
        filtrate_vessel: 滤液收集容器（可选，自动查找）
        stir: 是否在过滤过程中搅拌
        stir_speed: 搅拌速度 (RPM)
        temp: 过滤温度 (°C)
        continue_heatchill: 过滤后是否继续加热搅拌
        volume: 预期过滤体积 (mL)，0表示全部过滤
    
    Returns:
        List[Dict[str, Any]]: 过滤操作的动作序列
    """
    action_sequence = []
    
    print(f"FILTER: 开始生成过滤协议")
    print(f"  - 源容器: {vessel}")
    print(f"  - 滤液容器: {filtrate_vessel}")
    print(f"  - 搅拌: {stir} ({stir_speed} RPM)" if stir else "  - 搅拌: 否")
    print(f"  - 过滤温度: {temp}°C")
    print(f"  - 预期过滤体积: {volume} mL" if volume > 0 else "  - 预期过滤体积: 全部")
    print(f"  - 继续加热搅拌: {continue_heatchill}")
    
    # 验证源容器存在
    if vessel not in G.nodes():
        raise ValueError(f"源容器 '{vessel}' 不存在于系统中")
    
    # 获取源容器中的液体体积
    source_volume = get_vessel_liquid_volume(G, vessel)
    print(f"FILTER: 源容器 {vessel} 中有 {source_volume} mL 液体")
    
    # 查找过滤器设备
    try:
        filter_id = find_filter_device(G)
        print(f"FILTER: 找到过滤器: {filter_id}")
    except ValueError as e:
        raise ValueError(f"无法找到过滤器: {str(e)}")
    
    # 查找过滤器容器
    try:
        filter_vessel_id = find_filter_vessel(G)
        print(f"FILTER: 找到过滤器容器: {filter_vessel_id}")
    except ValueError as e:
        raise ValueError(f"无法找到过滤器容器: {str(e)}")
    
    # 查找滤液收集容器
    try:
        actual_filtrate_vessel = find_filtrate_vessel(G, filtrate_vessel)
        print(f"FILTER: 找到滤液收集容器: {actual_filtrate_vessel}")
    except ValueError as e:
        raise ValueError(f"无法找到滤液收集容器: {str(e)}")
    
    # 查找加热搅拌器（如果需要温度控制或搅拌）
    heatchill_id = None
    if temp != 25.0 or stir or continue_heatchill:
        try:
            heatchill_id = find_connected_heatchill(G, filter_vessel_id)
            print(f"FILTER: 找到加热搅拌器: {heatchill_id}")
        except ValueError as e:
            print(f"FILTER: 警告 - {str(e)}")
    
    # === 简化的体积计算策略 ===
    if volume > 0:
        transfer_volume = min(volume, source_volume if source_volume > 0 else volume)
        print(f"FILTER: 指定过滤体积 {transfer_volume} mL")
    elif source_volume > 0:
        transfer_volume = source_volume * 0.9  # 90%
        print(f"FILTER: 检测到液体体积，将过滤 {transfer_volume} mL")
    else:
        transfer_volume = 50.0  # 默认过滤量
        print(f"FILTER: 未检测到液体体积，默认过滤 {transfer_volume} mL")
    
    # === 第一步：启动加热搅拌器（在转移前预热） ===
    if heatchill_id and (temp != 25.0 or stir):
        print(f"FILTER: 启动加热搅拌器，温度: {temp}°C，搅拌: {stir}")
        
        heatchill_action = {
            "device_id": heatchill_id,
            "action_name": "heat_chill_start",
            "action_kwargs": {
                "vessel": filter_vessel_id,
                "temp": temp,
                "purpose": f"过滤过程温度控制和搅拌"
            }
        }
        action_sequence.append(heatchill_action)
        
        # 等待温度稳定
        if temp != 25.0:
            wait_time = min(30, abs(temp - 25.0) * 1.0)  # 根据温差估算预热时间
            action_sequence.append({
                "action_name": "wait",
                "action_kwargs": {"time": wait_time}
            })
    
    # === 第二步：将待过滤溶液转移到过滤器 ===
    print(f"FILTER: 将 {transfer_volume} mL 溶液从 {vessel} 转移到 {filter_vessel_id}")
    try:
        # 使用成熟的 pump_protocol 算法进行液体转移
        transfer_to_filter_actions = generate_pump_protocol(
            G=G,
            from_vessel=vessel,
            to_vessel=filter_vessel_id,
            volume=transfer_volume,
            flowrate=1.0,  # 过滤转移用较慢速度，避免扰动
            transfer_flowrate=1.5
        )
        action_sequence.extend(transfer_to_filter_actions)
    except Exception as e:
        raise ValueError(f"无法将溶液转移到过滤器: {str(e)}")
    
    # 转移后等待
    action_sequence.append({
        "action_name": "wait",
        "action_kwargs": {"time": 5}
    })
    
    # === 第三步：执行过滤操作（完全按照 Filter.action 参数） ===
    print(f"FILTER: 执行过滤操作")
    filter_action = {
        "device_id": filter_id,
        "action_name": "filter",
        "action_kwargs": {
            "vessel": filter_vessel_id,
            "filtrate_vessel": actual_filtrate_vessel,
            "stir": stir,
            "stir_speed": stir_speed,
            "temp": temp,
            "continue_heatchill": continue_heatchill,
            "volume": transfer_volume
        }
    }
    action_sequence.append(filter_action)
    
    # 过滤后等待
    action_sequence.append({
        "action_name": "wait",
        "action_kwargs": {"time": 10}
    })
    
    # === 第四步：如果不继续加热搅拌，停止加热器 ===
    if heatchill_id and not continue_heatchill and (temp != 25.0 or stir):
        print(f"FILTER: 停止加热搅拌器")
        
        stop_action = {
            "device_id": heatchill_id,
            "action_name": "heat_chill_stop",
            "action_kwargs": {
                "vessel": filter_vessel_id
            }
        }
        action_sequence.append(stop_action)
    
    print(f"FILTER: 生成了 {len(action_sequence)} 个动作")
    print(f"FILTER: 过滤协议生成完成")
    
    return action_sequence


# 便捷函数：常用过滤方案
def generate_gravity_filter_protocol(
    G: nx.DiGraph,
    vessel: str,
    filtrate_vessel: str = ""
) -> List[Dict[str, Any]]:
    """重力过滤：室温，无搅拌"""
    return generate_filter_protocol(G, vessel, filtrate_vessel, False, 0.0, 25.0, False, 0.0)


def generate_hot_filter_protocol(
    G: nx.DiGraph,
    vessel: str,
    filtrate_vessel: str = "",
    temp: float = 60.0
) -> List[Dict[str, Any]]:
    """热过滤：高温过滤，防止结晶析出"""
    return generate_filter_protocol(G, vessel, filtrate_vessel, False, 0.0, temp, False, 0.0)


def generate_stirred_filter_protocol(
    G: nx.DiGraph,
    vessel: str,
    filtrate_vessel: str = "",
    stir_speed: float = 200.0
) -> List[Dict[str, Any]]:
    """搅拌过滤：低速搅拌，防止滤饼堵塞"""
    return generate_filter_protocol(G, vessel, filtrate_vessel, True, stir_speed, 25.0, False, 0.0)


def generate_hot_stirred_filter_protocol(
    G: nx.DiGraph,
    vessel: str,
    filtrate_vessel: str = "",
    temp: float = 60.0,
    stir_speed: float = 300.0
) -> List[Dict[str, Any]]:
    """热搅拌过滤：高温搅拌过滤"""
    return generate_filter_protocol(G, vessel, filtrate_vessel, True, stir_speed, temp, False, 0.0)