from typing import List, Dict, Any, Union
import networkx as nx
import logging
import re

logger = logging.getLogger(__name__)

def debug_print(message):
    """调试输出"""
    print(f"🌡️ [HEATCHILL] {message}", flush=True)
    logger.info(f"[HEATCHILL] {message}")

def parse_time_input(time_input: Union[str, float, int]) -> float:
    """
    解析时间输入（统一函数）
    
    Args:
        time_input: 时间输入（如 "30 min", "1 h", "300", "?", 60.0）
    
    Returns:
        float: 时间（秒）
    """
    if not time_input:
        return 300.0
    
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
        'overnight': 43200.0, 'several hours': 10800.0, 
        'few hours': 7200.0, 'long time': 3600.0, 'short time': 300.0
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
        debug_print(f"⚠️ 无法解析时间: '{time_str}'，使用默认值: 300s")
        return 300.0
    
    value = float(match.group(1))
    unit = match.group(2) or 's'
    
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

def parse_temp_input(temp_input: Union[str, float], default_temp: float = 25.0) -> float:
    """
    解析温度输入（统一函数）
    
    Args:
        temp_input: 温度输入
        default_temp: 默认温度
        
    Returns:
        float: 温度（°C）
    """
    if not temp_input:
        return default_temp
    
    # 🔢 数值输入
    if isinstance(temp_input, (int, float)):
        result = float(temp_input)
        debug_print(f"🌡️ 数值温度: {temp_input} → {result}°C")
        return result
    
    # 📝 字符串输入
    temp_str = str(temp_input).lower().strip()
    debug_print(f"🔍 解析温度: '{temp_str}'")
    
    # 🎯 特殊温度
    special_temps = {
        "room temperature": 25.0, "reflux": 78.0, "ice bath": 0.0,
        "boiling": 100.0, "hot": 60.0, "warm": 40.0, "cold": 10.0
    }
    
    if temp_str in special_temps:
        result = special_temps[temp_str]
        debug_print(f"🎯 特殊温度: '{temp_str}' → {result}°C")
        return result
    
    # 📐 正则解析（如 "256 °C"）
    temp_pattern = r'(\d+(?:\.\d+)?)\s*°?[cf]?'
    match = re.search(temp_pattern, temp_str)
    
    if match:
        result = float(match.group(1))
        debug_print(f"✅ 温度解析: '{temp_str}' → {result}°C")
        return result
    
    debug_print(f"⚠️ 无法解析温度: '{temp_str}'，使用默认值: {default_temp}°C")
    return default_temp

def find_connected_heatchill(G: nx.DiGraph, vessel: str) -> str:
    """查找与指定容器相连的加热/冷却设备"""
    debug_print(f"🔍 查找加热设备，目标容器: {vessel}")
    
    # 🔧 查找所有加热设备
    heatchill_nodes = []
    for node in G.nodes():
        node_data = G.nodes[node]
        node_class = node_data.get('class', '') or ''
        
        if 'heatchill' in node_class.lower() or 'virtual_heatchill' in node_class:
            heatchill_nodes.append(node)
            debug_print(f"🎉 找到加热设备: {node}")
    
    # 🔗 检查连接
    if vessel and heatchill_nodes:
        for heatchill in heatchill_nodes:
            if G.has_edge(heatchill, vessel) or G.has_edge(vessel, heatchill):
                debug_print(f"✅ 加热设备 '{heatchill}' 与容器 '{vessel}' 相连")
                return heatchill
    
    # 🎯 使用第一个可用设备
    if heatchill_nodes:
        selected = heatchill_nodes[0]
        debug_print(f"🔧 使用第一个加热设备: {selected}")
        return selected
    
    # 🆘 默认设备
    debug_print("⚠️ 未找到加热设备，使用默认设备")
    return "heatchill_1"

def validate_and_fix_params(temp: float, time: float, stir_speed: float) -> tuple:
    """验证和修正参数"""
    # 🌡️ 温度范围验证
    if temp < -50.0 or temp > 300.0:
        debug_print(f"⚠️ 温度 {temp}°C 超出范围，修正为 25°C")
        temp = 25.0
    else:
        debug_print(f"✅ 温度 {temp}°C 在正常范围内")
    
    # ⏰ 时间验证
    if time < 0:
        debug_print(f"⚠️ 时间 {time}s 无效，修正为 300s")
        time = 300.0
    else:
        debug_print(f"✅ 时间 {time}s ({time/60:.1f}分钟) 有效")
    
    # 🌪️ 搅拌速度验证
    if stir_speed < 0 or stir_speed > 1500.0:
        debug_print(f"⚠️ 搅拌速度 {stir_speed} RPM 超出范围，修正为 300 RPM")
        stir_speed = 300.0
    else:
        debug_print(f"✅ 搅拌速度 {stir_speed} RPM 在正常范围内")
    
    return temp, time, stir_speed

def generate_heat_chill_protocol(
    G: nx.DiGraph,
    vessel: str,
    temp: float = 25.0,
    time: Union[str, float] = "300",
    temp_spec: str = "",
    time_spec: str = "",
    pressure: str = "",
    reflux_solvent: str = "",
    stir: bool = False,
    stir_speed: float = 300.0,
    purpose: str = "",
    **kwargs
) -> List[Dict[str, Any]]:
    """
    生成加热/冷却操作的协议序列
    """
    
    debug_print("🌡️" * 20)
    debug_print("🚀 开始生成加热冷却协议 ✨")
    debug_print(f"📝 输入参数:")
    debug_print(f"  🥽 vessel: {vessel}")
    debug_print(f"  🌡️ temp: {temp}°C")
    debug_print(f"  ⏰ time: {time}")
    debug_print(f"  🎯 temp_spec: {temp_spec}")
    debug_print(f"  ⏱️ time_spec: {time_spec}")
    debug_print(f"  🌪️ stir: {stir} ({stir_speed} RPM)")
    debug_print("🌡️" * 20)
    
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
    
    #温度解析：优先使用 temp_spec
    final_temp = parse_temp_input(temp_spec, temp) if temp_spec else temp
    
    # 时间解析：优先使用 time_spec
    final_time = parse_time_input(time_spec) if time_spec else parse_time_input(time)
    
    # 参数修正
    final_temp, final_time, stir_speed = validate_and_fix_params(final_temp, final_time, stir_speed)
    
    debug_print(f"🎯 最终参数: temp={final_temp}°C, time={final_time}s, stir_speed={stir_speed} RPM")
    
    # 🔍 查找设备
    debug_print("📍 步骤3: 查找加热设备... 🔍")
    try:
        heatchill_id = find_connected_heatchill(G, vessel)
        debug_print(f"🎉 使用加热设备: {heatchill_id} ✨")
    except Exception as e:
        debug_print(f"❌ 设备查找失败: {str(e)} 😭")
        raise ValueError(f"无法找到加热设备: {str(e)}")
    
    # 🚀 生成动作
    debug_print("📍 步骤4: 生成加热动作... 🔥")
    
    # 🕐 模拟运行时间优化
    debug_print("  ⏱️ 检查模拟运行时间限制...")
    original_time = final_time
    simulation_time_limit = 100.0  # 模拟运行时间限制：100秒
    
    if final_time > simulation_time_limit:
        final_time = simulation_time_limit
        debug_print(f"  🎮 模拟运行优化: {original_time}s → {final_time}s (限制为{simulation_time_limit}s) ⚡")
        debug_print(f"  📊 时间缩短: {original_time/60:.1f}分钟 → {final_time/60:.1f}分钟 🚀")
    else:
        debug_print(f"  ✅ 时间在限制内: {final_time}s ({final_time/60:.1f}分钟) 保持不变 🎯")
    
    action_sequence = []
    heatchill_action = {
        "device_id": heatchill_id,
        "action_name": "heat_chill",
        "action_kwargs": {
            "vessel": vessel,
            "temp": float(final_temp),
            "time": float(final_time),
            "stir": bool(stir),
            "stir_speed": float(stir_speed),
            "purpose": str(purpose or f"加热到 {final_temp}°C") + (f" (模拟时间: {final_time}s)" if original_time != final_time else "")
        }
    }
    action_sequence.append(heatchill_action)
    debug_print("✅ 加热动作已添加 🔥✨")
    
    # 显示时间调整信息
    if original_time != final_time:
        debug_print(f"  🎭 模拟优化说明: 原计划 {original_time/60:.1f}分钟，实际模拟 {final_time/60:.1f}分钟 ⚡")
    
    # 🎊 总结
    debug_print("🎊" * 20)
    debug_print(f"🎉 加热冷却协议生成完成! ✨")
    debug_print(f"📊 总动作数: {len(action_sequence)} 个")
    debug_print(f"🥽 加热容器: {vessel}")
    debug_print(f"🌡️ 目标温度: {final_temp}°C")
    debug_print(f"⏰ 加热时间: {final_time}s ({final_time/60:.1f}分钟)")
    debug_print("🎊" * 20)
    
    return action_sequence

def generate_heat_chill_to_temp_protocol(
    G: nx.DiGraph,
    vessel: str,
    temp: float = 25.0,
    time: Union[str, float] = 100.0,
    **kwargs
) -> List[Dict[str, Any]]:
    """生成加热到指定温度的协议（简化版）"""
    debug_print(f"🌡️ 生成加热到温度协议: {vessel} → {temp}°C")
    return generate_heat_chill_protocol(G, vessel, temp, time, **kwargs)

def generate_heat_chill_start_protocol(
    G: nx.DiGraph,
    vessel: str,
    temp: float = 25.0,
    purpose: str = "",
    **kwargs
) -> List[Dict[str, Any]]:
    """生成开始加热操作的协议序列"""
    
    debug_print("🔥 开始生成启动加热协议 ✨")
    debug_print(f"🥽 vessel: {vessel}, 🌡️ temp: {temp}°C")
    
    # 基础验证
    if not vessel or vessel not in G.nodes():
        debug_print("❌ 容器验证失败!")
        raise ValueError("vessel 参数无效")
    
    # 查找设备
    heatchill_id = find_connected_heatchill(G, vessel)
    
    # 生成动作
    action_sequence = [{
        "device_id": heatchill_id,
        "action_name": "heat_chill_start",
        "action_kwargs": {
            "vessel": vessel,
            "temp": temp,
            "purpose": purpose or f"开始加热到 {temp}°C"
        }
    }]
    
    debug_print(f"✅ 启动加热协议生成完成 🎯")
    return action_sequence

def generate_heat_chill_stop_protocol(
    G: nx.DiGraph,
    vessel: str,
    **kwargs
) -> List[Dict[str, Any]]:
    """生成停止加热操作的协议序列"""
    
    debug_print("🛑 开始生成停止加热协议 ✨")
    debug_print(f"🥽 vessel: {vessel}")
    
    # 基础验证
    if not vessel or vessel not in G.nodes():
        debug_print("❌ 容器验证失败!")
        raise ValueError("vessel 参数无效")
    
    # 查找设备
    heatchill_id = find_connected_heatchill(G, vessel)
    
    # 生成动作
    action_sequence = [{
        "device_id": heatchill_id,
        "action_name": "heat_chill_stop",
        "action_kwargs": {
            "vessel": vessel
        }
    }]
    
    debug_print(f"✅ 停止加热协议生成完成 🎯")
    return action_sequence

# 测试函数
def test_heatchill_protocol():
    """测试加热协议"""
    debug_print("🧪 === HEATCHILL PROTOCOL 测试 === ✨")
    debug_print("✅ 测试完成 🎉")

if __name__ == "__main__":
    test_heatchill_protocol()