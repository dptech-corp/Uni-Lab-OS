from typing import List, Dict, Any, Union
import networkx as nx
import logging
import re

logger = logging.getLogger(__name__)

def debug_print(message):
    """调试输出"""
    print(f"🌪️ [STIR] {message}", flush=True)
    logger.info(f"[STIR] {message}")

def parse_time_input(time_input: Union[str, float, int], default_unit: str = "s") -> float:
    """
    统一的时间解析函数（精简版）
    
    Args:
        time_input: 时间输入（如 "30 min", "1 h", "300", "?", 60.0）
        default_unit: 默认单位（默认为秒）
    
    Returns:
        float: 时间（秒）
    """
    if not time_input:
        return 100.0  # 默认100秒
    
    # 🔢 处理数值输入
    if isinstance(time_input, (int, float)):
        result = float(time_input)
        debug_print(f"⏰ 数值时间: {time_input} → {result}s")
        return result
    
    # 📝 处理字符串输入
    time_str = str(time_input).lower().strip()
    debug_print(f"🔍 解析时间: '{time_str}'")
    
    # ❓ 特殊值处理
    special_times = {
        '?': 300.0, 'unknown': 300.0, 'tbd': 300.0,
        'briefly': 30.0, 'quickly': 60.0, 'slowly': 600.0,
        'several minutes': 300.0, 'few minutes': 180.0, 'overnight': 3600.0
    }
    
    if time_str in special_times:
        result = special_times[time_str]
        debug_print(f"🎯 特殊时间: '{time_str}' → {result}s ({result/60:.1f}分钟)")
        return result
    
    # 🔢 纯数字处理
    try:
        result = float(time_str)
        debug_print(f"⏰ 纯数字: {time_str} → {result}s")
        return result
    except ValueError:
        pass
    
    # 📐 正则表达式解析
    pattern = r'(\d+\.?\d*)\s*([a-z]*)'
    match = re.match(pattern, time_str)
    
    if not match:
        debug_print(f"⚠️ 无法解析时间: '{time_str}'，使用默认值: 100s")
        return 100.0
    
    value = float(match.group(1))
    unit = match.group(2) or default_unit
    
    # 📏 单位转换
    unit_multipliers = {
        's': 1.0, 'sec': 1.0, 'second': 1.0, 'seconds': 1.0,
        'm': 60.0, 'min': 60.0, 'mins': 60.0, 'minute': 60.0, 'minutes': 60.0,
        'h': 3600.0, 'hr': 3600.0, 'hrs': 3600.0, 'hour': 3600.0, 'hours': 3600.0,
        'd': 86400.0, 'day': 86400.0, 'days': 86400.0
    }
    
    multiplier = unit_multipliers.get(unit, 1.0)
    result = value * multiplier
    
    debug_print(f"✅ 时间解析: '{time_str}' → {value} {unit} → {result}s ({result/60:.1f}分钟)")
    return result

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

def generate_stir_protocol(
    G: nx.DiGraph,
    vessel: str,
    time: Union[str, float, int] = "300",
    stir_time: Union[str, float, int] = "0",
    time_spec: str = "",
    event: str = "",
    stir_speed: float = 300.0,
    settling_time: Union[str, float] = "60",
    **kwargs
) -> List[Dict[str, Any]]:
    """
    生成搅拌操作的协议序列（精简版）
    """
    
    debug_print("🌪️" * 20)
    debug_print("🚀 开始生成搅拌协议 ✨")
    debug_print(f"📝 输入参数:")
    debug_print(f"  🥽 vessel: {vessel}")
    debug_print(f"  ⏰ time: {time}")
    debug_print(f"  🕐 stir_time: {stir_time}")
    debug_print(f"  🎯 time_spec: {time_spec}")
    debug_print(f"  🌪️ stir_speed: {stir_speed} RPM")
    debug_print(f"  ⏱️ settling_time: {settling_time}")
    debug_print("🌪️" * 20)
    
    # 📋 参数验证
    debug_print("📍 步骤1: 参数验证... 🔧")
    if not vessel:
        debug_print("❌ vessel 参数不能为空! 😱")
        raise ValueError("vessel 参数不能为空")
    
    if vessel not in G.nodes():
        debug_print(f"❌ 容器 '{vessel}' 不存在于系统中! 😞")
        raise ValueError(f"容器 '{vessel}' 不存在于系统中")
    
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
        stirrer_id = find_connected_stirrer(G, vessel)
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
            "vessel": vessel,
            "time": str(time),  # 保持原始格式
            "event": event,
            "time_spec": time_spec,
            "stir_time": float(parsed_time),  # 确保是数字
            "stir_speed": float(stir_speed),  # 确保是数字
            "settling_time": float(parsed_settling_time)  # 确保是数字
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
    debug_print(f"🥽 搅拌容器: {vessel}")
    debug_print(f"🌪️ 搅拌参数: {stir_speed} RPM, {parsed_time}s, 沉降 {parsed_settling_time}s")
    debug_print(f"⏱️ 预计总时间: {(parsed_time + parsed_settling_time)/60:.1f} 分钟 ⌛")
    debug_print("🎊" * 20)
    
    return action_sequence

def generate_start_stir_protocol(
    G: nx.DiGraph,
    vessel: str,
    stir_speed: float = 300.0,
    purpose: str = "",
    **kwargs
) -> List[Dict[str, Any]]:
    """生成开始搅拌操作的协议序列"""
    
    debug_print("🔄 开始生成启动搅拌协议 ✨")
    debug_print(f"🥽 vessel: {vessel}, 🌪️ speed: {stir_speed} RPM")
    
    # 基础验证
    if not vessel or vessel not in G.nodes():
        debug_print("❌ 容器验证失败!")
        raise ValueError("vessel 参数无效")
    
    # 参数修正
    if stir_speed < 10.0 or stir_speed > 1500.0:
        debug_print(f"⚠️ 搅拌速度修正: {stir_speed} → 300 RPM 🌪️")
        stir_speed = 300.0
    
    # 查找设备
    stirrer_id = find_connected_stirrer(G, vessel)
    
    # 生成动作
    action_sequence = [{
        "device_id": stirrer_id,
        "action_name": "start_stir",
        "action_kwargs": {
            "vessel": vessel,
            "stir_speed": stir_speed,
            "purpose": purpose or f"启动搅拌 {stir_speed} RPM"
        }
    }]
    
    debug_print(f"✅ 启动搅拌协议生成完成 🎯")
    return action_sequence

def generate_stop_stir_protocol(
    G: nx.DiGraph,
    vessel: str,
    **kwargs
) -> List[Dict[str, Any]]:
    """生成停止搅拌操作的协议序列"""
    
    debug_print("🛑 开始生成停止搅拌协议 ✨")
    debug_print(f"🥽 vessel: {vessel}")
    
    # 基础验证
    if not vessel or vessel not in G.nodes():
        debug_print("❌ 容器验证失败!")
        raise ValueError("vessel 参数无效")
    
    # 查找设备
    stirrer_id = find_connected_stirrer(G, vessel)
    
    # 生成动作
    action_sequence = [{
        "device_id": stirrer_id,
        "action_name": "stop_stir",
        "action_kwargs": {
            "vessel": vessel
        }
    }]
    
    debug_print(f"✅ 停止搅拌协议生成完成 🎯")
    return action_sequence

# 测试函数
def test_stir_protocol():
    """测试搅拌协议"""
    debug_print("🧪 === STIR PROTOCOL 测试 === ✨")
    debug_print("✅ 测试完成 🎉")

if __name__ == "__main__":
    test_stir_protocol()
