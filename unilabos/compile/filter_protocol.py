from typing import List, Dict, Any
import networkx as nx
from .pump_protocol import generate_pump_protocol
import logging
import sys

logger = logging.getLogger(__name__)

def debug_print(message):
    """调试输出"""
    print(f"[FILTER] {message}", flush=True)
    logger.info(f"[FILTER] {message}")

def get_vessel_liquid_volume(G: nx.DiGraph, vessel: str) -> float:
    """获取容器中的液体体积"""
    debug_print(f"检查容器 '{vessel}' 的液体体积...")
    
    if vessel not in G.nodes():
        debug_print(f"容器 '{vessel}' 不存在")
        return 0.0
    
    vessel_data = G.nodes[vessel].get('data', {})
    
    # 检查多种体积字段
    volume_keys = ['total_volume', 'volume', 'liquid_volume', 'current_volume']
    for key in volume_keys:
        if key in vessel_data:
            try:
                volume = float(vessel_data[key])
                debug_print(f"从 '{key}' 读取到体积: {volume}mL")
                return volume
            except (ValueError, TypeError):
                continue
    
    # 检查liquid数组
    liquids = vessel_data.get('liquid', [])
    if isinstance(liquids, list):
        total_volume = 0.0
        for liquid in liquids:
            if isinstance(liquid, dict):
                for vol_key in ['liquid_volume', 'volume', 'amount']:
                    if vol_key in liquid:
                        try:
                            vol = float(liquid[vol_key])
                            total_volume += vol
                            debug_print(f"从液体数据 '{vol_key}' 读取: {vol}mL")
                        except (ValueError, TypeError):
                            continue
        if total_volume > 0:
            return total_volume
    
    debug_print(f"未检测到液体体积，返回 0.0")
    return 0.0

def find_filter_device(G: nx.DiGraph) -> str:
    """查找过滤器设备"""
    debug_print("查找过滤器设备...")
    
    # 查找过滤器设备
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
    return "filter_1"  # 默认设备

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
        "filtrate_vessel",      # 标准名称
        "collection_bottle_1",  # 收集瓶
        "collection_bottle_2",  # 收集瓶
        "waste_workup",         # 废液收集
        "rotavap",              # 旋蒸仪
        "flask_1",              # 通用烧瓶
        "flask_2"               # 通用烧瓶
    ]
    
    for vessel_name in possible_names:
        if vessel_name in G.nodes():
            debug_print(f"找到滤液收集容器: {vessel_name}")
            return vessel_name
    
    debug_print("未找到滤液收集容器，使用默认容器")
    return "filtrate_vessel"  # 默认容器

def generate_filter_protocol(
    G: nx.DiGraph,
    vessel: str,
    filtrate_vessel: str = "",
    **kwargs  # 🔧 接受额外参数，增强兼容性
) -> List[Dict[str, Any]]:
    """
    生成过滤操作的协议序列 - 简化版本
    
    Args:
        G: 设备图
        vessel: 过滤容器名称（必需）
        filtrate_vessel: 滤液容器名称（可选，自动查找）
        **kwargs: 其他参数（兼容性）
    
    Returns:
        List[Dict[str, Any]]: 过滤操作的动作序列
    """
    
    debug_print("=" * 50)
    debug_print("开始生成过滤协议")
    debug_print(f"输入参数:")
    debug_print(f"  - vessel: {vessel}")
    debug_print(f"  - filtrate_vessel: {filtrate_vessel}")
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
    
    # === 查找设备 ===
    debug_print("步骤2: 查找设备...")
    
    try:
        filter_device = find_filter_device(G)
        actual_filtrate_vessel = find_filtrate_vessel(G, filtrate_vessel)
        
        debug_print(f"设备配置:")
        debug_print(f"  - 过滤器设备: {filter_device}")
        debug_print(f"  - 滤液收集容器: {actual_filtrate_vessel}")
        
    except Exception as e:
        debug_print(f"❌ 设备查找失败: {str(e)}")
        raise ValueError(f"设备查找失败: {str(e)}")
    
    # === 体积检测 ===
    debug_print("步骤3: 体积检测...")
    
    source_volume = get_vessel_liquid_volume(G, vessel)
    
    if source_volume > 0:
        transfer_volume = source_volume
        debug_print(f"检测到液体体积: {transfer_volume}mL")
    else:
        transfer_volume = 50.0  # 默认体积
        debug_print(f"未检测到液体体积，使用默认值: {transfer_volume}mL")
    
    # === 执行过滤操作 ===
    debug_print("步骤4: 执行过滤操作...")
    
    # 过滤动作（直接调用过滤器）
    debug_print(f"执行过滤: {vessel} -> {actual_filtrate_vessel}")
    
    filter_action = {
        "device_id": filter_device,
        "action_name": "filter",
        "action_kwargs": {
            "vessel": vessel,
            "filtrate_vessel": actual_filtrate_vessel,
            "stir": False,           # 🔧 使用默认值
            "stir_speed": 0.0,       # 🔧 使用默认值
            "temp": 25.0,            # 🔧 使用默认值
            "continue_heatchill": False,  # 🔧 使用默认值
            "volume": transfer_volume     # 🔧 使用检测到的体积
        }
    }
    action_sequence.append(filter_action)
    
    # 过滤后等待
    action_sequence.append({
        "action_name": "wait",
        "action_kwargs": {"time": 10.0}
    })
    
    # === 总结 ===
    debug_print("=" * 50)
    debug_print(f"过滤协议生成完成")
    debug_print(f"总动作数: {len(action_sequence)}")
    debug_print(f"过滤容器: {vessel}")
    debug_print(f"滤液容器: {actual_filtrate_vessel}")
    debug_print(f"处理体积: {transfer_volume}mL")
    debug_print("=" * 50)
    
    return action_sequence

# 测试函数
def test_filter_protocol():
    """测试过滤协议"""
    debug_print("=== FILTER PROTOCOL 测试 ===")
    debug_print("✅ 测试完成")

if __name__ == "__main__":
    test_filter_protocol()