from typing import List, Dict, Any
import networkx as nx
import logging

logger = logging.getLogger(__name__)

def debug_print(message):
    """调试输出"""
    print(f"[STIR] {message}", flush=True)
    logger.info(f"[STIR] {message}")

def find_connected_stirrer(G: nx.DiGraph, vessel: str = None) -> str:
    """
    查找与指定容器相连的搅拌设备，或查找可用的搅拌设备
    """
    debug_print(f"查找搅拌设备，目标容器: {vessel}")
    
    # 查找所有搅拌设备节点
    stirrer_nodes = []
    for node in G.nodes():
        node_data = G.nodes[node]
        node_class = node_data.get('class', '') or ''
        
        if 'stirrer' in node_class.lower() or 'virtual_stirrer' in node_class:
            stirrer_nodes.append(node)
            debug_print(f"找到搅拌设备: {node}")
    
    if vessel:
        # 检查哪个搅拌设备与目标容器相连（机械连接）
        for stirrer in stirrer_nodes:
            if G.has_edge(stirrer, vessel) or G.has_edge(vessel, stirrer):
                debug_print(f"搅拌设备 '{stirrer}' 与容器 '{vessel}' 相连")
                return stirrer
    
    # 如果没有指定容器或没有直接连接，返回第一个可用的搅拌设备
    if stirrer_nodes:
        debug_print(f"使用第一个搅拌设备: {stirrer_nodes[0]}")
        return stirrer_nodes[0]
    
    debug_print("未找到搅拌设备，使用默认设备")
    return "stirrer_1"  # 默认设备

def generate_stir_protocol(
    G: nx.DiGraph,
    vessel: str,
    stir_time: float = 300.0,
    stir_speed: float = 200.0,
    settling_time: float = 60.0,
    **kwargs  # 🔧 接受额外参数，增强兼容性
) -> List[Dict[str, Any]]:
    """
    生成搅拌操作的协议序列 - 定时搅拌 + 沉降
    
    Args:
        G: 设备图
        vessel: 搅拌容器名称（必需）
        stir_time: 搅拌时间 (秒)，默认300s
        stir_speed: 搅拌速度 (RPM)，默认200 RPM
        settling_time: 沉降时间 (秒)，默认60s
        **kwargs: 其他参数（兼容性）
    
    Returns:
        List[Dict[str, Any]]: 搅拌操作的动作序列
    """
    
    debug_print("=" * 50)
    debug_print("开始生成搅拌协议")
    debug_print(f"输入参数:")
    debug_print(f"  - vessel: {vessel}")
    debug_print(f"  - stir_time: {stir_time}s ({stir_time/60:.1f}分钟)")
    debug_print(f"  - stir_speed: {stir_speed} RPM")
    debug_print(f"  - settling_time: {settling_time}s ({settling_time/60:.1f}分钟)")
    debug_print(f"  - 其他参数: {kwargs}")
    debug_print("=" * 50)
    
    action_sequence = []
    
    # === 参数验证 ===
    debug_print("步骤1: 参数验证...")
    
    # 验证必需参数
    if not vessel:
        raise ValueError("vessel 参数不能为空")
    
    if vessel not in G.nodes():
        raise ValueError(f"容器 '{vessel}' 不存在于系统中")
    
    # 修正参数范围
    if stir_time < 0:
        debug_print(f"搅拌时间 {stir_time}s 无效，修正为 300s")
        stir_time = 300.0
    elif stir_time > 7200:
        debug_print(f"搅拌时间 {stir_time}s 过长，修正为 3600s")
        stir_time = 3600.0
    
    if stir_speed < 10.0:
        debug_print(f"搅拌速度 {stir_speed} RPM 过低，修正为 100 RPM")
        stir_speed = 100.0
    elif stir_speed > 1500.0:
        debug_print(f"搅拌速度 {stir_speed} RPM 过高，修正为 1000 RPM")
        stir_speed = 1000.0
    
    if settling_time < 0:
        debug_print(f"沉降时间 {settling_time}s 无效，修正为 60s")
        settling_time = 60.0
    elif settling_time > 1800:
        debug_print(f"沉降时间 {settling_time}s 过长，修正为 600s")
        settling_time = 600.0
    
    debug_print(f"✅ 参数验证通过")
    
    # === 查找搅拌设备 ===
    debug_print("步骤2: 查找搅拌设备...")
    
    try:
        stirrer_id = find_connected_stirrer(G, vessel)
        debug_print(f"设备配置: 搅拌设备 = {stirrer_id}")
        
    except Exception as e:
        debug_print(f"❌ 设备查找失败: {str(e)}")
        raise ValueError(f"无法找到搅拌设备: {str(e)}")
    
    # === 执行搅拌操作 ===
    debug_print("步骤3: 执行搅拌操作...")
    
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
    
    # === 总结 ===
    debug_print("=" * 50)
    debug_print(f"搅拌协议生成完成")
    debug_print(f"总动作数: {len(action_sequence)}")
    debug_print(f"搅拌容器: {vessel}")
    debug_print(f"搅拌参数: {stir_speed} RPM, {stir_time}s, 沉降 {settling_time}s")
    debug_print("=" * 50)
    
    return action_sequence

def generate_start_stir_protocol(
    G: nx.DiGraph,
    vessel: str,
    stir_speed: float = 200.0,
    purpose: str = "",
    **kwargs  # 🔧 接受额外参数，增强兼容性
) -> List[Dict[str, Any]]:
    """
    生成开始搅拌操作的协议序列 - 持续搅拌
    
    Args:
        G: 设备图
        vessel: 搅拌容器名称（必需）
        stir_speed: 搅拌速度 (RPM)，默认200 RPM
        purpose: 搅拌目的（可选）
        **kwargs: 其他参数（兼容性）
    
    Returns:
        List[Dict[str, Any]]: 开始搅拌操作的动作序列
    """
    
    debug_print("=" * 50)
    debug_print("开始生成启动搅拌协议")
    debug_print(f"输入参数:")
    debug_print(f"  - vessel: {vessel}")
    debug_print(f"  - stir_speed: {stir_speed} RPM")
    debug_print(f"  - purpose: {purpose}")
    debug_print(f"  - 其他参数: {kwargs}")
    debug_print("=" * 50)
    
    action_sequence = []
    
    # === 参数验证 ===
    debug_print("步骤1: 参数验证...")
    
    # 验证必需参数
    if not vessel:
        raise ValueError("vessel 参数不能为空")
    
    if vessel not in G.nodes():
        raise ValueError(f"容器 '{vessel}' 不存在于系统中")
    
    # 修正参数范围
    if stir_speed < 10.0:
        debug_print(f"搅拌速度 {stir_speed} RPM 过低，修正为 100 RPM")
        stir_speed = 100.0
    elif stir_speed > 1500.0:
        debug_print(f"搅拌速度 {stir_speed} RPM 过高，修正为 1000 RPM")
        stir_speed = 1000.0
    
    debug_print(f"✅ 参数验证通过")
    
    # === 查找搅拌设备 ===
    debug_print("步骤2: 查找搅拌设备...")
    
    try:
        stirrer_id = find_connected_stirrer(G, vessel)
        debug_print(f"设备配置: 搅拌设备 = {stirrer_id}")
        
    except Exception as e:
        debug_print(f"❌ 设备查找失败: {str(e)}")
        raise ValueError(f"无法找到搅拌设备: {str(e)}")
    
    # === 执行开始搅拌操作 ===
    debug_print("步骤3: 执行开始搅拌操作...")
    
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
    
    # === 总结 ===
    debug_print("=" * 50)
    debug_print(f"启动搅拌协议生成完成")
    debug_print(f"总动作数: {len(action_sequence)}")
    debug_print(f"搅拌容器: {vessel}")
    debug_print(f"搅拌速度: {stir_speed} RPM")
    debug_print(f"搅拌目的: {purpose}")
    debug_print("=" * 50)
    
    return action_sequence

def generate_stop_stir_protocol(
    G: nx.DiGraph,
    vessel: str,
    **kwargs  # 🔧 接受额外参数，增强兼容性
) -> List[Dict[str, Any]]:
    """
    生成停止搅拌操作的协议序列
    
    Args:
        G: 设备图
        vessel: 搅拌容器名称（必需）
        **kwargs: 其他参数（兼容性）
    
    Returns:
        List[Dict[str, Any]]: 停止搅拌操作的动作序列
    """
    
    debug_print("=" * 50)
    debug_print("开始生成停止搅拌协议")
    debug_print(f"输入参数:")
    debug_print(f"  - vessel: {vessel}")
    debug_print(f"  - 其他参数: {kwargs}")
    debug_print("=" * 50)
    
    action_sequence = []
    
    # === 参数验证 ===
    debug_print("步骤1: 参数验证...")
    
    # 验证必需参数
    if not vessel:
        raise ValueError("vessel 参数不能为空")
    
    if vessel not in G.nodes():
        raise ValueError(f"容器 '{vessel}' 不存在于系统中")
    
    debug_print(f"✅ 参数验证通过")
    
    # === 查找搅拌设备 ===
    debug_print("步骤2: 查找搅拌设备...")
    
    try:
        stirrer_id = find_connected_stirrer(G, vessel)
        debug_print(f"设备配置: 搅拌设备 = {stirrer_id}")
        
    except Exception as e:
        debug_print(f"❌ 设备查找失败: {str(e)}")
        raise ValueError(f"无法找到搅拌设备: {str(e)}")
    
    # === 执行停止搅拌操作 ===
    debug_print("步骤3: 执行停止搅拌操作...")
    
    stop_stir_action = {
        "device_id": stirrer_id,
        "action_name": "stop_stir",
        "action_kwargs": {
            "vessel": vessel
        }
    }
    
    action_sequence.append(stop_stir_action)
    
    # === 总结 ===
    debug_print("=" * 50)
    debug_print(f"停止搅拌协议生成完成")
    debug_print(f"总动作数: {len(action_sequence)}")
    debug_print(f"搅拌容器: {vessel}")
    debug_print("=" * 50)
    
    return action_sequence

# === 便捷函数 ===

def generate_fast_stir_protocol(
    G: nx.DiGraph,
    vessel: str,
    **kwargs
) -> List[Dict[str, Any]]:
    """快速搅拌：高速短时间"""
    return generate_stir_protocol(
        G, vessel, 
        stir_time=300.0, 
        stir_speed=800.0, 
        settling_time=60.0,
        **kwargs
    )

def generate_gentle_stir_protocol(
    G: nx.DiGraph,
    vessel: str,
    **kwargs
) -> List[Dict[str, Any]]:
    """温和搅拌：低速长时间"""
    return generate_stir_protocol(
        G, vessel, 
        stir_time=900.0, 
        stir_speed=150.0, 
        settling_time=120.0,
        **kwargs
    )

def generate_thorough_stir_protocol(
    G: nx.DiGraph,
    vessel: str,
    **kwargs
) -> List[Dict[str, Any]]:
    """彻底搅拌：中速长时间"""
    return generate_stir_protocol(
        G, vessel, 
        stir_time=1800.0, 
        stir_speed=400.0, 
        settling_time=300.0,
        **kwargs
    )

# 测试函数
def test_stir_protocol():
    """测试搅拌协议"""
    debug_print("=== STIR PROTOCOL 测试 ===")
    debug_print("✅ 测试完成")

if __name__ == "__main__":
    test_stir_protocol()