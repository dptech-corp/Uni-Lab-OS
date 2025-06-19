from typing import List, Dict, Any
import networkx as nx


def find_connected_stirrer(G: nx.DiGraph, vessel: str = None) -> str:
    """
    查找与指定容器相连的搅拌设备，或查找可用的搅拌设备
    """
    # 查找所有搅拌设备节点
    stirrer_nodes = [node for node in G.nodes() 
                    if (G.nodes[node].get('class') or '') == 'virtual_stirrer']
    
    if vessel:
        # 检查哪个搅拌设备与目标容器相连（机械连接）
        for stirrer in stirrer_nodes:
            if G.has_edge(stirrer, vessel) or G.has_edge(vessel, stirrer):
                return stirrer
    
    # 如果没有指定容器或没有直接连接，返回第一个可用的搅拌设备
    if stirrer_nodes:
        return stirrer_nodes[0]
    
    raise ValueError("系统中未找到可用的搅拌设备")


def generate_stir_protocol(
    G: nx.DiGraph,
    stir_time: float,
    stir_speed: float,
    settling_time: float
) -> List[Dict[str, Any]]:
    """
    生成搅拌操作的协议序列 - 定时搅拌 + 沉降
    """
    action_sequence = []
    
    print(f"STIR: 开始生成搅拌协议")
    print(f"  - 搅拌时间: {stir_time}秒")
    print(f"  - 搅拌速度: {stir_speed} RPM")
    print(f"  - 沉降时间: {settling_time}秒")
    
    # 查找搅拌设备
    try:
        stirrer_id = find_connected_stirrer(G)
        print(f"STIR: 找到搅拌设备: {stirrer_id}")
    except ValueError as e:
        raise ValueError(f"无法找到搅拌设备: {str(e)}")
    
    # 执行搅拌操作
    stir_action = {
        "device_id": stirrer_id,
        "action_name": "stir",
        "action_kwargs": {
            "stir_time": stir_time,
            "stir_speed": stir_speed,
            "settling_time": settling_time
        }
    }
    
    action_sequence.append(stir_action)
    
    print(f"STIR: 生成了 {len(action_sequence)} 个动作")
    return action_sequence


def generate_start_stir_protocol(
    G: nx.DiGraph,
    vessel: str,
    stir_speed: float,
    purpose: str
) -> List[Dict[str, Any]]:
    """
    生成开始搅拌操作的协议序列 - 持续搅拌
    """
    action_sequence = []
    
    print(f"START_STIR: 开始生成启动搅拌协议")
    print(f"  - 容器: {vessel}")
    print(f"  - 搅拌速度: {stir_speed} RPM")
    print(f"  - 目的: {purpose}")
    
    # 验证容器存在
    if vessel not in G.nodes():
        raise ValueError(f"容器 '{vessel}' 不存在于系统中")
    
    # 查找搅拌设备
    try:
        stirrer_id = find_connected_stirrer(G, vessel)
        print(f"START_STIR: 找到搅拌设备: {stirrer_id}")
    except ValueError as e:
        raise ValueError(f"无法找到搅拌设备: {str(e)}")
    
    # 执行开始搅拌操作
    start_stir_action = {
        "device_id": stirrer_id,
        "action_name": "start_stir",
        "action_kwargs": {
            "vessel": vessel,
            "stir_speed": stir_speed,
            "purpose": purpose
        }
    }
    
    action_sequence.append(start_stir_action)
    
    print(f"START_STIR: 生成了 {len(action_sequence)} 个动作")
    return action_sequence


def generate_stop_stir_protocol(
    G: nx.DiGraph,
    vessel: str
) -> List[Dict[str, Any]]:
    """
    生成停止搅拌操作的协议序列
    """
    action_sequence = []
    
    print(f"STOP_STIR: 开始生成停止搅拌协议")
    print(f"  - 容器: {vessel}")
    
    # 验证容器存在
    if vessel not in G.nodes():
        raise ValueError(f"容器 '{vessel}' 不存在于系统中")
    
    # 查找搅拌设备
    try:
        stirrer_id = find_connected_stirrer(G, vessel)
        print(f"STOP_STIR: 找到搅拌设备: {stirrer_id}")
    except ValueError as e:
        raise ValueError(f"无法找到搅拌设备: {str(e)}")
    
    # 执行停止搅拌操作
    stop_stir_action = {
        "device_id": stirrer_id,
        "action_name": "stop_stir",
        "action_kwargs": {
            "vessel": vessel
        }
    }
    
    action_sequence.append(stop_stir_action)
    
    print(f"STOP_STIR: 生成了 {len(action_sequence)} 个动作")
    return action_sequence


# 便捷函数
def generate_fast_stir_protocol(
    G: nx.DiGraph,
    time: float = 300.0,
    speed: float = 800.0,
    settling: float = 60.0
) -> List[Dict[str, Any]]:
    """快速搅拌的便捷函数"""
    return generate_stir_protocol(G, time, speed, settling)


def generate_gentle_stir_protocol(
    G: nx.DiGraph,
    time: float = 600.0,
    speed: float = 200.0,
    settling: float = 120.0
) -> List[Dict[str, Any]]:
    """温和搅拌的便捷函数"""
    return generate_stir_protocol(G, time, speed, settling)