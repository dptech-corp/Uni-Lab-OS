from typing import List, Dict, Any
import networkx as nx
import logging
import sys

logger = logging.getLogger(__name__)

def debug_print(message):
    """调试输出"""
    print(f"[WASH_SOLID] {message}", flush=True)
    logger.info(f"[WASH_SOLID] {message}")

def find_solvent_source(G: nx.DiGraph, solvent: str) -> str:
    """查找溶剂源容器"""
    debug_print(f"查找溶剂 '{solvent}' 的源容器...")
    
    # 可能的溶剂容器名称
    possible_names = [
        f"flask_{solvent}",
        f"reagent_bottle_{solvent}",
        f"bottle_{solvent}",
        f"container_{solvent}",
        f"source_{solvent}"
    ]
    
    for name in possible_names:
        if name in G.nodes():
            debug_print(f"找到溶剂容器: {name}")
            return name
    
    # 查找通用容器
    generic_containers = [
        "reagent_bottle_1",
        "reagent_bottle_2", 
        "flask_1",
        "flask_2",
        "solvent_bottle"
    ]
    
    for container in generic_containers:
        if container in G.nodes():
            debug_print(f"使用通用容器: {container}")
            return container
    
    debug_print("未找到溶剂容器，使用默认容器")
    return f"flask_{solvent}"

def find_filtrate_vessel(G: nx.DiGraph, filtrate_vessel: str = "") -> str:
    """查找滤液收集容器"""
    debug_print(f"查找滤液收集容器，指定容器: '{filtrate_vessel}'")
    
    # 如果指定了容器且存在，直接使用
    if filtrate_vessel and filtrate_vessel.strip():
        if filtrate_vessel in G.nodes():
            debug_print(f"使用指定的滤液容器: {filtrate_vessel}")
            return filtrate_vessel
        else:
            debug_print(f"指定的滤液容器 '{filtrate_vessel}' 不存在，查找默认容器")
    
    # 自动查找滤液容器
    possible_names = [
        "waste_workup",         # 废液收集
        "filtrate_vessel",      # 标准滤液容器
        "collection_bottle_1",  # 收集瓶
        "collection_bottle_2",  # 收集瓶
        "rotavap",              # 旋蒸仪
        "waste_flask",          # 废液瓶
        "flask_1",              # 通用烧瓶
        "flask_2"               # 通用烧瓶
    ]
    
    for vessel_name in possible_names:
        if vessel_name in G.nodes():
            debug_print(f"找到滤液收集容器: {vessel_name}")
            return vessel_name
    
    debug_print("未找到滤液收集容器，使用默认容器")
    return "waste_workup"

def find_pump_device(G: nx.DiGraph) -> str:
    """查找转移泵设备"""
    debug_print("查找转移泵设备...")
    
    pump_devices = []
    for node in G.nodes():
        node_data = G.nodes[node]
        node_class = node_data.get('class', '') or ''
        
        if 'transfer_pump' in node_class or 'virtual_transfer_pump' in node_class:
            pump_devices.append(node)
            debug_print(f"找到转移泵设备: {node}")
    
    if pump_devices:
        return pump_devices[0]
    
    debug_print("未找到转移泵设备，使用默认设备")
    return "transfer_pump_1"

def find_filter_device(G: nx.DiGraph) -> str:
    """查找过滤器设备"""
    debug_print("查找过滤器设备...")
    
    filter_devices = []
    for node in G.nodes():
        node_data = G.nodes[node]
        node_class = node_data.get('class', '') or ''
        
        if 'filter' in node_class.lower() or 'virtual_filter' in node_class:
            filter_devices.append(node)
            debug_print(f"找到过滤器设备: {node}")
    
    if filter_devices:
        return filter_devices[0]
    
    debug_print("未找到过滤器设备，使用默认设备")
    return "filter_1"

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
    repeats: int = 1,
    **kwargs  # 🔧 接受额外参数，增强兼容性
) -> List[Dict[str, Any]]:
    """
    生成固体清洗操作的协议序列 - 简化版本
    
    Args:
        G: 设备图
        vessel: 装有固体的容器名称（必需）
        solvent: 清洗溶剂名称（必需）
        volume: 清洗溶剂体积（必需）
        filtrate_vessel: 滤液收集容器（可选，自动查找）
        temp: 清洗温度，默认25°C
        stir: 是否搅拌，默认False
        stir_speed: 搅拌速度，默认0
        time: 清洗时间，默认0
        repeats: 重复次数，默认1
        **kwargs: 其他参数（兼容性）
    
    Returns:
        List[Dict[str, Any]]: 固体清洗操作的动作序列
    """
    
    debug_print("=" * 50)
    debug_print("开始生成固体清洗协议")
    debug_print(f"输入参数:")
    debug_print(f"  - vessel: {vessel}")
    debug_print(f"  - solvent: {solvent}")
    debug_print(f"  - volume: {volume}mL")
    debug_print(f"  - filtrate_vessel: {filtrate_vessel}")
    debug_print(f"  - temp: {temp}°C")
    debug_print(f"  - stir: {stir}")
    debug_print(f"  - stir_speed: {stir_speed} RPM")
    debug_print(f"  - time: {time}s")
    debug_print(f"  - repeats: {repeats}")
    debug_print(f"  - 其他参数: {kwargs}")
    debug_print("=" * 50)
    
    action_sequence = []
    
    # === 参数验证 ===
    debug_print("步骤1: 参数验证...")
    
    # 验证必需参数
    if not vessel:
        raise ValueError("vessel 参数不能为空")
    
    if not solvent:
        raise ValueError("solvent 参数不能为空")
    
    if volume <= 0:
        raise ValueError("volume 必须大于0")
    
    if vessel not in G.nodes():
        raise ValueError(f"容器 '{vessel}' 不存在于系统中")
    
    # 修正参数范围
    if temp < 0 or temp > 200:
        debug_print(f"温度 {temp}°C 超出范围，修正为 25°C")
        temp = 25.0
    
    if stir_speed < 0 or stir_speed > 500:
        debug_print(f"搅拌速度 {stir_speed} RPM 超出范围，修正为 0")
        stir_speed = 0.0
    
    if time < 0:
        debug_print(f"时间 {time}s 无效，修正为 0")
        time = 0.0
    
    if repeats < 1:
        debug_print(f"重复次数 {repeats} 无效，修正为 1")
        repeats = 1
    elif repeats > 10:
        debug_print(f"重复次数 {repeats} 过多，修正为 10")
        repeats = 10
    
    debug_print(f"✅ 参数验证通过")
    
    # === 查找设备 ===
    debug_print("步骤2: 查找设备...")
    
    try:
        solvent_source = find_solvent_source(G, solvent)
        actual_filtrate_vessel = find_filtrate_vessel(G, filtrate_vessel)
        pump_device = find_pump_device(G)
        filter_device = find_filter_device(G)
        
        debug_print(f"设备配置:")
        debug_print(f"  - 溶剂源: {solvent_source}")
        debug_print(f"  - 滤液容器: {actual_filtrate_vessel}")
        debug_print(f"  - 转移泵: {pump_device}")
        debug_print(f"  - 过滤器: {filter_device}")
        
    except Exception as e:
        debug_print(f"❌ 设备查找失败: {str(e)}")
        raise ValueError(f"设备查找失败: {str(e)}")
    
    # === 执行清洗循环 ===
    debug_print("步骤3: 执行清洗循环...")
    
    for cycle in range(repeats):
        debug_print(f"=== 第 {cycle+1}/{repeats} 次清洗 ===")
        
        # 添加清洗溶剂
        debug_print(f"添加清洗溶剂: {solvent_source} -> {vessel}")
        
        wash_action = {
            "device_id": filter_device,
            "action_name": "wash_solid",
            "action_kwargs": {
                "vessel": vessel,
                "solvent": solvent,
                "volume": volume,
                "filtrate_vessel": actual_filtrate_vessel,
                "temp": temp,
                "stir": stir,
                "stir_speed": stir_speed,
                "time": time,
                "repeats": 1  # 每次循环只做1次
            }
        }
        action_sequence.append(wash_action)
        
        # 等待清洗完成
        action_sequence.append({
            "action_name": "wait",
            "action_kwargs": {"time": max(10.0, time * 0.1)}
        })
    
    # === 总结 ===
    debug_print("=" * 50)
    debug_print(f"固体清洗协议生成完成")
    debug_print(f"总动作数: {len(action_sequence)}")
    debug_print(f"清洗容器: {vessel}")
    debug_print(f"使用溶剂: {solvent}")
    debug_print(f"清洗体积: {volume}mL")
    debug_print(f"重复次数: {repeats}")
    debug_print("=" * 50)
    
    return action_sequence

# === 便捷函数 ===

def generate_quick_wash_protocol(
    G: nx.DiGraph,
    vessel: str,
    solvent: str,
    volume: float,
    **kwargs
) -> List[Dict[str, Any]]:
    """快速清洗：1次，室温，不搅拌"""
    return generate_wash_solid_protocol(
        G, vessel, solvent, volume, 
        repeats=1, temp=25.0, stir=False, **kwargs
    )

def generate_thorough_wash_protocol(
    G: nx.DiGraph,
    vessel: str,
    solvent: str,
    volume: float,
    **kwargs
) -> List[Dict[str, Any]]:
    """彻底清洗：3次，加热，搅拌"""
    return generate_wash_solid_protocol(
        G, vessel, solvent, volume, 
        repeats=3, temp=50.0, stir=True, stir_speed=200.0, time=300.0, **kwargs
    )

def generate_gentle_wash_protocol(
    G: nx.DiGraph,
    vessel: str,
    solvent: str,
    volume: float,
    **kwargs
) -> List[Dict[str, Any]]:
    """温和清洗：2次，室温，轻搅拌"""
    return generate_wash_solid_protocol(
        G, vessel, solvent, volume, 
        repeats=2, temp=25.0, stir=True, stir_speed=100.0, time=180.0, **kwargs
    )

# 测试函数
def test_wash_solid_protocol():
    """测试固体清洗协议"""
    debug_print("=== WASH SOLID PROTOCOL 测试 ===")
    debug_print("✅ 测试完成")

if __name__ == "__main__":
    test_wash_solid_protocol()