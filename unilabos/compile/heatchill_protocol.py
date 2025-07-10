from typing import List, Dict, Any, Union
import networkx as nx
import logging
import re

logger = logging.getLogger(__name__)

def debug_print(message):
    """调试输出"""
    print(f"[HEATCHILL] {message}", flush=True)
    logger.info(f"[HEATCHILL] {message}")

def parse_time_with_units(time_input: Union[str, float, int], default_unit: str = "s") -> float:
    """
    解析带单位的时间输入
    
    Args:
        time_input: 时间输入（如 "30 min", "1 h", "300", "?", 60.0）
        default_unit: 默认单位（默认为秒）
    
    Returns:
        float: 时间（秒）
    """
    if not time_input:
        return 0.0
    
    # 处理数值输入
    if isinstance(time_input, (int, float)):
        result = float(time_input)
        debug_print(f"数值时间输入: {time_input} → {result}s（默认单位）")
        return result
    
    # 处理字符串输入
    time_str = str(time_input).lower().strip()
    debug_print(f"解析时间字符串: '{time_str}'")
    
    # 处理特殊值
    if time_str in ['?', 'unknown', 'tbd', 'to be determined']:
        default_time = 300.0  # 5分钟默认值
        debug_print(f"检测到未知时间，使用默认值: {default_time}s")
        return default_time
    
    # 如果是纯数字，使用默认单位
    try:
        value = float(time_str)
        if default_unit == "s":
            result = value
        elif default_unit in ["min", "minute"]:
            result = value * 60.0
        elif default_unit in ["h", "hour"]:
            result = value * 3600.0
        else:
            result = value  # 默认秒
        debug_print(f"纯数字输入: {time_str} → {result}s（单位: {default_unit}）")
        return result
    except ValueError:
        pass
    
    # 使用正则表达式匹配数字和单位
    pattern = r'(\d+\.?\d*)\s*([a-z]*)'
    match = re.match(pattern, time_str)
    
    if not match:
        debug_print(f"⚠️ 无法解析时间: '{time_str}'，使用默认值: 60s")
        return 60.0
    
    value = float(match.group(1))
    unit = match.group(2) or default_unit
    
    # 单位转换映射
    unit_multipliers = {
        # 秒
        's': 1.0,
        'sec': 1.0,
        'second': 1.0,
        'seconds': 1.0,
        
        # 分钟
        'm': 60.0,
        'min': 60.0,
        'mins': 60.0,
        'minute': 60.0,
        'minutes': 60.0,
        
        # 小时
        'h': 3600.0,
        'hr': 3600.0,
        'hrs': 3600.0,
        'hour': 3600.0,
        'hours': 3600.0,
        
        # 天
        'd': 86400.0,
        'day': 86400.0,
        'days': 86400.0,
    }
    
    multiplier = unit_multipliers.get(unit, 1.0)
    result = value * multiplier
    
    debug_print(f"时间解析: '{time_str}' → {value} {unit} → {result}s")
    return result

def parse_temp_spec(temp_spec: str) -> float:
    """解析温度规格为具体温度"""
    if not temp_spec:
        return 25.0
    
    temp_spec = temp_spec.strip().lower()
    
    # 特殊温度规格
    special_temps = {
        "room temperature": 25.0,      # 室温
        "reflux": 78.0,                 # 默认回流温度
        "ice bath": 0.0,                # 冰浴
        "boiling": 100.0,               # 沸腾
        "hot": 60.0,                    # 热
        "warm": 40.0,                   # 温热
        "cold": 10.0,                   # 冷
    }
    
    if temp_spec in special_temps:
        return special_temps[temp_spec]
    
    # 解析带单位的温度（如 "256 °C"）
    temp_pattern = r'(\d+(?:\.\d+)?)\s*°?[cf]?'
    match = re.search(temp_pattern, temp_spec)
    
    if match:
        return float(match.group(1))
    
    return 25.0

def parse_time_spec(time_spec: str) -> float:
    """解析时间规格为秒数"""
    if not time_spec:
        return 300.0
    
    time_spec = time_spec.strip().lower()
    
    # 特殊时间规格
    special_times = {
        "overnight": 43200.0,           # 12小时
        "several hours": 10800.0,       # 3小时
        "few hours": 7200.0,            # 2小时
        "long time": 3600.0,            # 1小时
        "short time": 300.0,            # 5分钟
    }
    
    if time_spec in special_times:
        return special_times[time_spec]
    
    # 解析带单位的时间（如 "2 h"）
    time_pattern = r'(\d+(?:\.\d+)?)\s*([a-zA-Z]+)'
    match = re.search(time_pattern, time_spec)
    
    if match:
        value = float(match.group(1))
        unit = match.group(2).lower()
        
        unit_multipliers = {
            's': 1.0,
            'sec': 1.0,
            'min': 60.0,
            'minute': 60.0,
            'minutes': 60.0,
            'h': 3600.0,
            'hr': 3600.0,
            'hour': 3600.0,
            'hours': 3600.0,
        }
        
        multiplier = unit_multipliers.get(unit, 3600.0)
        return value * multiplier
    
    return 300.0

def find_connected_heatchill(G: nx.DiGraph, vessel: str) -> str:
    """查找与指定容器相连的加热/冷却设备"""
    debug_print(f"查找加热设备，目标容器: {vessel}")
    
    # 查找所有加热/冷却设备节点
    heatchill_nodes = []
    for node in G.nodes():
        node_data = G.nodes[node]
        node_class = node_data.get('class', '') or ''
        
        if 'heatchill' in node_class.lower() or 'virtual_heatchill' in node_class:
            heatchill_nodes.append(node)
            debug_print(f"找到加热设备: {node}")
    
    if vessel:
        # 检查哪个加热设备与目标容器相连
        for heatchill in heatchill_nodes:
            if G.has_edge(heatchill, vessel) or G.has_edge(vessel, heatchill):
                debug_print(f"加热设备 '{heatchill}' 与容器 '{vessel}' 相连")
                return heatchill
    
    # 如果没有指定容器或没有直接连接，返回第一个可用的加热设备
    if heatchill_nodes:
        debug_print(f"使用第一个加热设备: {heatchill_nodes[0]}")
        return heatchill_nodes[0]
    
    debug_print("未找到加热设备，使用默认设备")
    return "heatchill_1"

def generate_heat_chill_protocol(
    G: nx.DiGraph,
    vessel: str,
    temp: float = 25.0,
    time: Union[str, float] = "300",     # 🔧 修改：支持字符串时间
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
    生成加热/冷却操作的协议序列 - 支持单位
    """
    
    debug_print("=" * 50)
    debug_print("开始生成加热冷却协议（支持单位）")
    debug_print(f"输入参数:")
    debug_print(f"  - vessel: {vessel}")
    debug_print(f"  - temp: {temp}°C")
    debug_print(f"  - time: {time} (类型: {type(time)})")
    debug_print(f"  - temp_spec: {temp_spec}")
    debug_print(f"  - time_spec: {time_spec}")
    debug_print(f"  - pressure: {pressure}")
    debug_print(f"  - reflux_solvent: {reflux_solvent}")
    debug_print(f"  - stir: {stir}")
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
    
    # === 🔧 新增：单位解析处理 ===
    debug_print("步骤2: 单位解析处理...")
    
    # 温度解析：优先使用 temp_spec，然后是 temp
    final_temp = temp
    if temp_spec:
        final_temp = parse_temp_spec(temp_spec)
        debug_print(f"温度解析: '{temp_spec}' → {final_temp}°C")
    
    # 时间解析：优先使用 time_spec，然后是 time
    if time_spec:
        final_time = parse_time_spec(time_spec)  # 使用现有的time_spec解析
        debug_print(f"时间解析: '{time_spec}' → {final_time}s")
    else:
        final_time = parse_time_with_units(time, "s")
        debug_print(f"时间解析: {time} → {final_time}s ({final_time/60:.1f}分钟)")
    
    # 参数范围验证
    if final_temp < -50.0 or final_temp > 300.0:
        debug_print(f"温度 {final_temp}°C 超出范围，修正为 25°C")
        final_temp = 25.0
    
    if final_time < 0:
        debug_print(f"时间 {final_time}s 无效，修正为 300s")
        final_time = 300.0
    
    if stir_speed < 0 or stir_speed > 1500.0:
        debug_print(f"搅拌速度 {stir_speed} RPM 超出范围，修正为 300 RPM")
        stir_speed = 300.0
    
    debug_print(f"✅ 单位解析和参数验证通过")
    
    # === 查找加热设备 ===
    debug_print("步骤3: 查找加热设备...")
    
    try:
        heatchill_id = find_connected_heatchill(G, vessel)
        debug_print(f"设备配置: 加热设备 = {heatchill_id}")
        
    except Exception as e:
        debug_print(f"❌ 设备查找失败: {str(e)}")
        raise ValueError(f"无法找到加热设备: {str(e)}")
    
    # === 执行加热操作 ===
    debug_print("步骤4: 执行加热操作...")
    
    heatchill_action = {
        "device_id": heatchill_id,
        "action_name": "heat_chill",
        "action_kwargs": {
            "vessel": vessel,
            "temp": float(final_temp),      # 🔧 确保是浮点数
            "time": float(final_time),      # 🔧 确保是浮点数
            "stir": bool(stir),             # 🔧 确保是布尔值
            "stir_speed": float(stir_speed), # 🔧 确保是浮点数
            "purpose": str(purpose or f"加热到 {final_temp}°C")  # 🔧 确保是字符串
        }
    }
    
    action_sequence.append(heatchill_action)
    
    # === 总结 ===
    debug_print("=" * 50)
    debug_print(f"加热冷却协议生成完成（支持单位）")
    debug_print(f"总动作数: {len(action_sequence)}")
    debug_print(f"加热容器: {vessel}")
    debug_print(f"目标温度: {final_temp}°C")
    debug_print(f"加热时间: {final_time}s ({final_time/60:.1f}分钟)")
    if pressure:
        debug_print(f"压力参数: {pressure} (已接收，不做特殊处理)")
    if reflux_solvent:
        debug_print(f"回流溶剂: {reflux_solvent} (已接收，不做特殊处理)")
    debug_print("=" * 50)
    
    return action_sequence

def generate_heat_chill_to_temp_protocol(
        G: nx.DiGraph,
        vessel: str,
        temp: float = 25.0,
        time: Union[str, float] = 300.0,  # 🔧 也支持字符串
        temp_spec: str = "",
        time_spec: str = "",
        pressure: str = "",
        reflux_solvent: str = "",
        stir: bool = False,
        stir_speed: float = 300.0,
        purpose: str = "",
        **kwargs  # 🔧 接受额外参数，增强兼容性
) -> List[Dict[str, Any]]:
    """
    生成加热/冷却操作的协议序列

    Args:
        G: 设备图
        vessel: 加热容器名称（必需）
        temp: 目标温度 (°C)
        time: 加热时间（支持字符串和数字）
        temp_spec: 温度规格（如 'room temperature', 'reflux'）
        time_spec: 时间规格（如 'overnight', '2 h'）
        pressure: 压力规格（如 '1 mbar'），不做特殊处理
        reflux_solvent: 回流溶剂名称，不做特殊处理
        stir: 是否搅拌
        stir_speed: 搅拌速度 (RPM)
        purpose: 操作目的
        **kwargs: 其他参数（兼容性）

    Returns:
        List[Dict[str, Any]]: 加热操作的动作序列
    """

    debug_print("=" * 50)
    debug_print("开始生成加热冷却协议")
    debug_print(f"输入参数:")
    debug_print(f"  - vessel: {vessel}")
    debug_print(f"  - temp: {temp}°C")
    debug_print(f"  - time: {time} (类型: {type(time)})")
    debug_print(f"  - temp_spec: {temp_spec}")
    debug_print(f"  - time_spec: {time_spec}")
    debug_print(f"  - pressure: {pressure}")
    debug_print(f"  - reflux_solvent: {reflux_solvent}")
    debug_print(f"  - stir: {stir}")
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

    # 温度解析：优先使用 temp_spec，然后是 temp
    final_temp = temp
    if temp_spec:
        final_temp = parse_temp_spec(temp_spec)
        debug_print(f"温度解析: '{temp_spec}' → {final_temp}°C")

    # 🔧 修复：时间解析，支持字符串输入
    if time_spec:
        final_time = parse_time_spec(time_spec)
        debug_print(f"时间解析: '{time_spec}' → {final_time}s ({final_time / 60:.1f}分钟)")
    else:
        final_time = parse_time_with_units(time, "s")
        debug_print(f"时间解析: {time} → {final_time}s ({final_time/60:.1f}分钟)")

    # 参数范围验证
    if final_temp < -50.0 or final_temp > 300.0:
        debug_print(f"温度 {final_temp}°C 超出范围，修正为 25°C")
        final_temp = 25.0

    if final_time < 0:
        debug_print(f"时间 {final_time}s 无效，修正为 300s")
        final_time = 300.0

    if stir_speed < 0 or stir_speed > 1500.0:
        debug_print(f"搅拌速度 {stir_speed} RPM 超出范围，修正为 300 RPM")
        stir_speed = 300.0

    debug_print(f"✅ 参数验证通过")

    # === 查找加热设备 ===
    debug_print("步骤2: 查找加热设备...")

    try:
        heatchill_id = find_connected_heatchill(G, vessel)
        debug_print(f"设备配置: 加热设备 = {heatchill_id}")

    except Exception as e:
        debug_print(f"❌ 设备查找失败: {str(e)}")
        raise ValueError(f"无法找到加热设备: {str(e)}")

    # === 执行加热操作 ===
    debug_print("步骤3: 执行加热操作...")

    heatchill_action = {
        "device_id": heatchill_id,
        "action_name": "heat_chill",
        "action_kwargs": {
            "vessel": vessel,
            "temp": float(final_temp),      # 🔧 确保是浮点数
            "time": float(final_time),      # 🔧 确保是浮点数
            "stir": bool(stir),             # 🔧 确保是布尔值
            "stir_speed": float(stir_speed), # 🔧 确保是浮点数
            "purpose": str(purpose or f"加热到 {final_temp}°C")  # 🔧 确保是字符串
        }
    }

    action_sequence.append(heatchill_action)

    # === 总结 ===
    debug_print("=" * 50)
    debug_print(f"加热冷却协议生成完成")
    debug_print(f"总动作数: {len(action_sequence)}")
    debug_print(f"加热容器: {vessel}")
    debug_print(f"目标温度: {final_temp}°C")
    debug_print(f"加热时间: {final_time}s ({final_time / 60:.1f}分钟)")
    if pressure:
        debug_print(f"压力参数: {pressure} (已接收，不做特殊处理)")
    if reflux_solvent:
        debug_print(f"回流溶剂: {reflux_solvent} (已接收，不做特殊处理)")
    debug_print("=" * 50)

    return action_sequence


def generate_heat_chill_start_protocol(
    G: nx.DiGraph,
    vessel: str,
    temp: float = 25.0,
    purpose: str = "",
    **kwargs
) -> List[Dict[str, Any]]:
    """生成开始加热操作的协议序列"""
    
    debug_print("=" * 50)
    debug_print("开始生成启动加热协议")
    debug_print(f"输入参数:")
    debug_print(f"  - vessel: {vessel}")
    debug_print(f"  - temp: {temp}°C")
    debug_print(f"  - purpose: {purpose}")
    debug_print("=" * 50)
    
    action_sequence = []
    
    # 验证参数
    if not vessel:
        raise ValueError("vessel 参数不能为空")
    
    if vessel not in G.nodes():
        raise ValueError(f"容器 '{vessel}' 不存在于系统中")
    
    # 查找加热设备
    try:
        heatchill_id = find_connected_heatchill(G, vessel)
        debug_print(f"设备配置: 加热设备 = {heatchill_id}")
    except Exception as e:
        debug_print(f"❌ 设备查找失败: {str(e)}")
        raise ValueError(f"无法找到加热设备: {str(e)}")
    
    # 执行开始加热操作
    start_action = {
        "device_id": heatchill_id,
        "action_name": "heat_chill_start",
        "action_kwargs": {
            "vessel": vessel,
            "temp": temp,
            "purpose": purpose or f"开始加热到 {temp}°C"
        }
    }
    
    action_sequence.append(start_action)
    
    debug_print(f"启动加热协议生成完成，动作数: {len(action_sequence)}")
    return action_sequence

def generate_heat_chill_stop_protocol(
    G: nx.DiGraph,
    vessel: str,
    **kwargs
) -> List[Dict[str, Any]]:
    """生成停止加热操作的协议序列"""
    
    debug_print("=" * 50)
    debug_print("开始生成停止加热协议")
    debug_print(f"输入参数:")
    debug_print(f"  - vessel: {vessel}")
    debug_print("=" * 50)
    
    action_sequence = []
    
    # 验证参数
    if not vessel:
        raise ValueError("vessel 参数不能为空")
    
    if vessel not in G.nodes():
        raise ValueError(f"容器 '{vessel}' 不存在于系统中")
    
    # 查找加热设备
    try:
        heatchill_id = find_connected_heatchill(G, vessel)
        debug_print(f"设备配置: 加热设备 = {heatchill_id}")
    except Exception as e:
        debug_print(f"❌ 设备查找失败: {str(e)}")
        raise ValueError(f"无法找到加热设备: {str(e)}")
    
    # 执行停止加热操作
    stop_action = {
        "device_id": heatchill_id,
        "action_name": "heat_chill_stop",
        "action_kwargs": {
            "vessel": vessel
        }
    }
    
    action_sequence.append(stop_action)
    
    debug_print(f"停止加热协议生成完成，动作数: {len(action_sequence)}")
    return action_sequence

# 测试函数
def test_heatchill_protocol():
    """测试加热协议"""
    debug_print("=== HEATCHILL PROTOCOL 测试 ===")
    debug_print("✅ 测试完成")

if __name__ == "__main__":
    test_heatchill_protocol()