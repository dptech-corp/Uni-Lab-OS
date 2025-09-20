from typing import List, Dict, Any, Union
import networkx as nx
import logging
import re

from .utils.unit_parser import parse_time_input

logger = logging.getLogger(__name__)

def debug_print(message):
    """调试输出"""
    logger.info(f"[STIR] {message}")


def find_connected_stirrer(G: nx.DiGraph, vessel: str = None) -> str:
    """查找与指定容器相连的搅拌设备"""
    debug_print(f"🔍 查找搅拌设备，目标容器: {vessel} 🥽")
    
    # 🔧 查找所有搅拌设备
    stirrer_nodes = []
    for node in G.nodes():
        node_data = G.nodes[node]
        node_class = node_data.get('class', '') or ''
        
        if 'stirrer' in node_class.lower() or 'virtual_stirrer' in node_class:
            stirrer_nodes.append(node)
            debug_print(f"🎉 找到搅拌设备: {node} 🌪️")
    
    # 🔗 检查连接
    if vessel and stirrer_nodes:
        for stirrer in stirrer_nodes:
            if G.has_edge(stirrer, vessel) or G.has_edge(vessel, stirrer):
                debug_print(f"✅ 搅拌设备 '{stirrer}' 与容器 '{vessel}' 相连 🔗")
                return stirrer
    
    # 🎯 使用第一个可用设备
    if stirrer_nodes:
        selected = stirrer_nodes[0]
        debug_print(f"🔧 使用第一个搅拌设备: {selected} 🌪️")
        return selected
    
    # 🆘 默认设备
    debug_print("⚠️ 未找到搅拌设备，使用默认设备 🌪️")
    return "stirrer_1"

def validate_and_fix_params(stir_time: float, stir_speed: float, settling_time: float) -> tuple:
    """验证和修正参数"""
    # ⏰ 搅拌时间验证
    if stir_time < 0:
        debug_print(f"⚠️ 搅拌时间 {stir_time}s 无效，修正为 100s 🕐")
        stir_time = 100.0
    elif stir_time > 100:  # 限制为100s
        debug_print(f"⚠️ 搅拌时间 {stir_time}s 过长，仿真运行时，修正为 100s 🕐")
        stir_time = 100.0
    else:
        debug_print(f"✅ 搅拌时间 {stir_time}s ({stir_time/60:.1f}分钟) 有效 ⏰")
    
    # 🌪️ 搅拌速度验证
    if stir_speed < 10.0 or stir_speed > 1500.0:
        debug_print(f"⚠️ 搅拌速度 {stir_speed} RPM 超出范围，修正为 300 RPM 🌪️")
        stir_speed = 300.0
    else:
        debug_print(f"✅ 搅拌速度 {stir_speed} RPM 在正常范围内 🌪️")
    
    # ⏱️ 沉降时间验证
    if settling_time < 0 or settling_time > 600:  # 限制为10分钟
        debug_print(f"⚠️ 沉降时间 {settling_time}s 超出范围，修正为 60s ⏱️")
        settling_time = 60.0
    else:
        debug_print(f"✅ 沉降时间 {settling_time}s 在正常范围内 ⏱️")
    
    return stir_time, stir_speed, settling_time

def extract_vessel_id(vessel: Union[str, dict]) -> str:
    """
    从vessel参数中提取vessel_id
    
    Args:
        vessel: vessel字典或vessel_id字符串
        
    Returns:
        str: vessel_id
    """
    if isinstance(vessel, dict):
        vessel_id = list(vessel.values())[0].get("id", "")
        debug_print(f"🔧 从vessel字典提取ID: {vessel_id}")
        return vessel_id
    elif isinstance(vessel, str):
        debug_print(f"🔧 vessel参数为字符串: {vessel}")
        return vessel
    else:
        debug_print(f"⚠️ 无效的vessel参数类型: {type(vessel)}")
        return ""

def get_vessel_display_info(vessel: Union[str, dict]) -> str:
    """
    获取容器的显示信息（用于日志）
    
    Args:
        vessel: vessel字典或vessel_id字符串
        
    Returns:
        str: 显示信息
    """
    if isinstance(vessel, dict):
        vessel_id = vessel.get("id", "unknown")
        vessel_name = vessel.get("name", "")
        if vessel_name:
            return f"{vessel_id} ({vessel_name})"
        else:
            return vessel_id
    else:
        return str(vessel)

def generate_stir_protocol(
    G: nx.DiGraph,
    vessel: Union[str, dict],  # 支持vessel字典或字符串
    time: Union[str, float, int] = "300",
    stir_time: Union[str, float, int] = "0",
    time_spec: str = "",
    event: str = "",
    stir_speed: float = 300.0,
    settling_time: Union[str, float] = "60",
    **kwargs
) -> List[Dict[str, Any]]:
    """生成搅拌操作的协议序列 - 修复vessel参数传递"""
    
    # 🔧 核心修改：正确处理vessel参数
    vessel_id = extract_vessel_id(vessel)
    vessel_display = get_vessel_display_info(vessel)
    
    # 🔧 关键修复：确保vessel_resource是完整的Resource对象
    if isinstance(vessel, dict):
        vessel_resource = vessel  # 已经是完整的Resource字典
        debug_print(f"✅ 使用传入的vessel Resource对象")
    else:
        # 如果只是字符串，构建一个基本的Resource对象
        vessel_resource = {
            "id": vessel,
            "name": "",
            "category": "",
            "children": [],
            "config": "",
            "data": "",
            "parent": "",
            "pose": {
                "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
                "position": {"x": 0.0, "y": 0.0, "z": 0.0}
            },
            "sample_id": "",
            "type": ""
        }
        debug_print(f"🔧 构建了基本的vessel Resource对象: {vessel}")
    
    debug_print("🌪️" * 20)
    debug_print("🚀 开始生成搅拌协议（支持vessel字典）✨")
    debug_print(f"📝 输入参数:")
    debug_print(f"  🥽 vessel: {vessel_display} (ID: {vessel_id})")
    debug_print(f"  ⏰ time: {time}")
    debug_print(f"  🕐 stir_time: {stir_time}")
    debug_print(f"  🎯 time_spec: {time_spec}")
    debug_print(f"  🌪️ stir_speed: {stir_speed} RPM")
    debug_print(f"  ⏱️ settling_time: {settling_time}")
    debug_print("🌪️" * 20)
    
    # 📋 参数验证
    debug_print("📍 步骤1: 参数验证... 🔧")
    if not vessel_id:  # 🔧 使用 vessel_id
        debug_print("❌ vessel 参数不能为空! 😱")
        raise ValueError("vessel 参数不能为空")
    
    if vessel_id not in G.nodes():  # 🔧 使用 vessel_id
        debug_print(f"❌ 容器 '{vessel_id}' 不存在于系统中! 😞")
        raise ValueError(f"容器 '{vessel_id}' 不存在于系统中")
    
    debug_print("✅ 基础参数验证通过 🎯")
    
    # 🔄 参数解析
    debug_print("📍 步骤2: 参数解析... ⚡")
    
    # 确定实际时间（优先级：time_spec > stir_time > time）
    if time_spec:
        parsed_time = parse_time_input(time_spec)
        debug_print(f"🎯 使用time_spec: '{time_spec}' → {parsed_time}s")
    elif stir_time not in ["0", 0, 0.0]:
        parsed_time = parse_time_input(stir_time)
        debug_print(f"🎯 使用stir_time: {stir_time} → {parsed_time}s")
    else:
        parsed_time = parse_time_input(time)
        debug_print(f"🎯 使用time: {time} → {parsed_time}s")
    
    # 解析沉降时间
    parsed_settling_time = parse_time_input(settling_time)
    
    # 🕐 模拟运行时间优化
    debug_print("  ⏱️ 检查模拟运行时间限制...")
    original_stir_time = parsed_time
    original_settling_time = parsed_settling_time
    
    # 搅拌时间限制为60秒
    stir_time_limit = 60.0
    if parsed_time > stir_time_limit:
        parsed_time = stir_time_limit
        debug_print(f"  🎮 搅拌时间优化: {original_stir_time}s → {parsed_time}s ⚡")
    
    # 沉降时间限制为30秒
    settling_time_limit = 30.0
    if parsed_settling_time > settling_time_limit:
        parsed_settling_time = settling_time_limit
        debug_print(f"  🎮 沉降时间优化: {original_settling_time}s → {parsed_settling_time}s ⚡")
    
    # 参数修正
    parsed_time, stir_speed, parsed_settling_time = validate_and_fix_params(
        parsed_time, stir_speed, parsed_settling_time
    )
    
    debug_print(f"🎯 最终参数: time={parsed_time}s, speed={stir_speed}RPM, settling={parsed_settling_time}s")
    
    # 🔍 查找设备
    debug_print("📍 步骤3: 查找搅拌设备... 🔍")
    try:
        stirrer_id = find_connected_stirrer(G, vessel_id)  # 🔧 使用 vessel_id
        debug_print(f"🎉 使用搅拌设备: {stirrer_id} ✨")
    except Exception as e:
        debug_print(f"❌ 设备查找失败: {str(e)} 😭")
        raise ValueError(f"无法找到搅拌设备: {str(e)}")
    
    # 🚀 生成动作
    debug_print("📍 步骤4: 生成搅拌动作... 🌪️")
    
    action_sequence = []
    stir_action = {
        "device_id": stirrer_id,
        "action_name": "stir",
        "action_kwargs": {
            # 🔧 关键修复：传递vessel_id字符串，而不是完整的Resource对象
            "vessel": {"id": vessel_id},  # 传递字符串ID，不是Resource对象
            "time": str(time),
            "event": event,
            "time_spec": time_spec,
            "stir_time": float(parsed_time),
            "stir_speed": float(stir_speed),
            "settling_time": float(parsed_settling_time)
        }
    }
    action_sequence.append(stir_action)
    debug_print("✅ 搅拌动作已添加 🌪️✨")
    
    # 显示时间优化信息
    if original_stir_time != parsed_time or original_settling_time != parsed_settling_time:
        debug_print(f"  🎭 模拟优化说明:")
        debug_print(f"    搅拌时间: {original_stir_time/60:.1f}分钟 → {parsed_time/60:.1f}分钟")
        debug_print(f"    沉降时间: {original_settling_time/60:.1f}分钟 → {parsed_settling_time/60:.1f}分钟")
    
    # 🎊 总结
    debug_print("🎊" * 20)
    debug_print(f"🎉 搅拌协议生成完成! ✨")
    debug_print(f"📊 总动作数: {len(action_sequence)} 个")
    debug_print(f"🥽 搅拌容器: {vessel_display}")
    debug_print(f"🌪️ 搅拌参数: {stir_speed} RPM, {parsed_time}s, 沉降 {parsed_settling_time}s")
    debug_print(f"⏱️ 预计总时间: {(parsed_time + parsed_settling_time)/60:.1f} 分钟 ⌛")
    debug_print("🎊" * 20)
    
    return action_sequence

def generate_start_stir_protocol(
    G: nx.DiGraph,
    vessel: Union[str, dict],
    stir_speed: float = 300.0,
    purpose: str = "",
    **kwargs
) -> List[Dict[str, Any]]:
    """生成开始搅拌操作的协议序列 - 修复vessel参数传递"""
    
    # 🔧 核心修改：正确处理vessel参数
    vessel_id = extract_vessel_id(vessel)
    vessel_display = get_vessel_display_info(vessel)
    
    # 🔧 关键修复：确保vessel_resource是完整的Resource对象
    if isinstance(vessel, dict):
        vessel_resource = vessel  # 已经是完整的Resource字典
        debug_print(f"✅ 使用传入的vessel Resource对象")
    else:
        # 如果只是字符串，构建一个基本的Resource对象
        vessel_resource = {
            "id": vessel,
            "name": "",
            "category": "",
            "children": [],
            "config": "",
            "data": "",
            "parent": "",
            "pose": {
                "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
                "position": {"x": 0.0, "y": 0.0, "z": 0.0}
            },
            "sample_id": "",
            "type": ""
        }
        debug_print(f"🔧 构建了基本的vessel Resource对象: {vessel}")
    
    debug_print("🔄 开始生成启动搅拌协议（修复vessel参数）✨")
    debug_print(f"🥽 vessel: {vessel_display} (ID: {vessel_id})")
    debug_print(f"🌪️ speed: {stir_speed} RPM")
    debug_print(f"🎯 purpose: {purpose}")
    
    # 基础验证
    if not vessel_id or vessel_id not in G.nodes():
        debug_print("❌ 容器验证失败!")
        raise ValueError("vessel 参数无效")
    
    # 参数修正
    if stir_speed < 10.0 or stir_speed > 1500.0:
        debug_print(f"⚠️ 搅拌速度修正: {stir_speed} → 300 RPM 🌪️")
        stir_speed = 300.0
    
    # 查找设备
    stirrer_id = find_connected_stirrer(G, vessel_id)
    
    # 🔧 关键修复：传递vessel_id字符串
    action_sequence = [{
        "device_id": stirrer_id,
        "action_name": "start_stir",
        "action_kwargs": {
            # 🔧 关键修复：传递vessel_id字符串，而不是完整的Resource对象
            "vessel": {"id": vessel_id},  # 传递字符串ID，不是Resource对象
            "stir_speed": stir_speed,
            "purpose": purpose or f"启动搅拌 {stir_speed} RPM"
        }
    }]
    
    debug_print(f"✅ 启动搅拌协议生成完成 🎯")
    return action_sequence

def generate_stop_stir_protocol(
    G: nx.DiGraph,
    vessel: Union[str, dict],
    **kwargs
) -> List[Dict[str, Any]]:
    """生成停止搅拌操作的协议序列 - 修复vessel参数传递"""
    
    # 🔧 核心修改：正确处理vessel参数
    vessel_id = extract_vessel_id(vessel)
    vessel_display = get_vessel_display_info(vessel)
    
    # 🔧 关键修复：确保vessel_resource是完整的Resource对象
    if isinstance(vessel, dict):
        vessel_resource = vessel  # 已经是完整的Resource字典
        debug_print(f"✅ 使用传入的vessel Resource对象")
    else:
        # 如果只是字符串，构建一个基本的Resource对象
        vessel_resource = {
            "id": vessel,
            "name": "",
            "category": "",
            "children": [],
            "config": "",
            "data": "",
            "parent": "",
            "pose": {
                "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
                "position": {"x": 0.0, "y": 0.0, "z": 0.0}
            },
            "sample_id": "",
            "type": ""
        }
        debug_print(f"🔧 构建了基本的vessel Resource对象: {vessel}")
    
    debug_print("🛑 开始生成停止搅拌协议（修复vessel参数）✨")
    debug_print(f"🥽 vessel: {vessel_display} (ID: {vessel_id})")
    
    # 基础验证
    if not vessel_id or vessel_id not in G.nodes():
        debug_print("❌ 容器验证失败!")
        raise ValueError("vessel 参数无效")
    
    # 查找设备
    stirrer_id = find_connected_stirrer(G, vessel_id)
    
    # 🔧 关键修复：传递vessel_id字符串
    action_sequence = [{
        "device_id": stirrer_id,
        "action_name": "stop_stir",
        "action_kwargs": {
            # 🔧 关键修复：传递vessel_id字符串，而不是完整的Resource对象
            "vessel": {"id": vessel_id},  # 传递字符串ID，不是Resource对象
        }
    }]
    
    debug_print(f"✅ 停止搅拌协议生成完成 🎯")
    return action_sequence

# 🔧 新增：便捷函数
def stir_briefly(G: nx.DiGraph, vessel: Union[str, dict], 
                speed: float = 300.0) -> List[Dict[str, Any]]:
    """短时间搅拌（30秒）"""
    vessel_display = get_vessel_display_info(vessel)
    debug_print(f"⚡ 短时间搅拌: {vessel_display} @ {speed}RPM (30s)")
    return generate_stir_protocol(G, vessel, time="30", stir_speed=speed)

def stir_slowly(G: nx.DiGraph, vessel: Union[str, dict], 
               time: Union[str, float] = "10 min") -> List[Dict[str, Any]]:
    """慢速搅拌"""
    vessel_display = get_vessel_display_info(vessel)
    debug_print(f"🐌 慢速搅拌: {vessel_display} @ 150RPM")
    return generate_stir_protocol(G, vessel, time=time, stir_speed=150.0)

def stir_vigorously(G: nx.DiGraph, vessel: Union[str, dict], 
                   time: Union[str, float] = "5 min") -> List[Dict[str, Any]]:
    """剧烈搅拌"""
    vessel_display = get_vessel_display_info(vessel)
    debug_print(f"💨 剧烈搅拌: {vessel_display} @ 800RPM")
    return generate_stir_protocol(G, vessel, time=time, stir_speed=800.0)

def stir_for_reaction(G: nx.DiGraph, vessel: Union[str, dict], 
                     time: Union[str, float] = "1 h") -> List[Dict[str, Any]]:
    """反应搅拌（标准速度，长时间）"""
    vessel_display = get_vessel_display_info(vessel)
    debug_print(f"🧪 反应搅拌: {vessel_display} @ 400RPM")
    return generate_stir_protocol(G, vessel, time=time, stir_speed=400.0)

def stir_for_dissolution(G: nx.DiGraph, vessel: Union[str, dict], 
                        time: Union[str, float] = "15 min") -> List[Dict[str, Any]]:
    """溶解搅拌（中等速度）"""
    vessel_display = get_vessel_display_info(vessel)
    debug_print(f"💧 溶解搅拌: {vessel_display} @ 500RPM")
    return generate_stir_protocol(G, vessel, time=time, stir_speed=500.0)

def stir_gently(G: nx.DiGraph, vessel: Union[str, dict], 
               time: Union[str, float] = "30 min") -> List[Dict[str, Any]]:
    """温和搅拌"""
    vessel_display = get_vessel_display_info(vessel)
    debug_print(f"🍃 温和搅拌: {vessel_display} @ 200RPM")
    return generate_stir_protocol(G, vessel, time=time, stir_speed=200.0)

def stir_overnight(G: nx.DiGraph, vessel: Union[str, dict]) -> List[Dict[str, Any]]:
    """过夜搅拌（模拟时缩短为2小时）"""
    vessel_display = get_vessel_display_info(vessel)
    debug_print(f"🌙 过夜搅拌（模拟2小时）: {vessel_display} @ 300RPM")
    return generate_stir_protocol(G, vessel, time="2 h", stir_speed=300.0)

def start_continuous_stirring(G: nx.DiGraph, vessel: Union[str, dict], 
                             speed: float = 300.0, purpose: str = "continuous stirring") -> List[Dict[str, Any]]:
    """开始连续搅拌"""
    vessel_display = get_vessel_display_info(vessel)
    debug_print(f"🔄 开始连续搅拌: {vessel_display} @ {speed}RPM")
    return generate_start_stir_protocol(G, vessel, stir_speed=speed, purpose=purpose)

def stop_all_stirring(G: nx.DiGraph, vessel: Union[str, dict]) -> List[Dict[str, Any]]:
    """停止所有搅拌"""
    vessel_display = get_vessel_display_info(vessel)
    debug_print(f"🛑 停止搅拌: {vessel_display}")
    return generate_stop_stir_protocol(G, vessel)

# 测试函数
def test_stir_protocol():
    """测试搅拌协议"""
    debug_print("🧪 === STIR PROTOCOL 测试 === ✨")
    
    # 测试vessel参数处理
    debug_print("🔧 测试vessel参数处理...")
    
    # 测试字典格式
    vessel_dict = {"id": "flask_1", "name": "反应瓶1"}
    vessel_id = extract_vessel_id(vessel_dict)
    vessel_display = get_vessel_display_info(vessel_dict)
    debug_print(f"  字典格式: {vessel_dict} → ID: {vessel_id}, 显示: {vessel_display}")
    
    # 测试字符串格式
    vessel_str = "flask_2"
    vessel_id = extract_vessel_id(vessel_str)
    vessel_display = get_vessel_display_info(vessel_str)
    debug_print(f"  字符串格式: {vessel_str} → ID: {vessel_id}, 显示: {vessel_display}")
    
    debug_print("✅ 测试完成 🎉")

if __name__ == "__main__":
    test_stir_protocol()
