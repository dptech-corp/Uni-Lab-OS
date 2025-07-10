from typing import List, Dict, Any, Union
import networkx as nx
import logging
import re

logger = logging.getLogger(__name__)

def debug_print(message):
    """调试输出"""
    print(f"[STIR] {message}", flush=True)
    logger.info(f"[STIR] {message}")

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

def parse_time_spec(time_spec: str) -> float:
    """
    解析时间规格字符串为秒数
    
    Args:
        time_spec: 时间规格字符串（如 "several minutes", "overnight", "few hours"）
    
    Returns:
        float: 时间（秒）
    """
    if not time_spec:
        return 0.0
    
    time_spec = time_spec.lower().strip()
    
    # 预定义的时间规格映射
    time_spec_map = {
        # 几分钟
        "several minutes": 5.0 * 60,      # 5分钟
        "few minutes": 3.0 * 60,          # 3分钟
        "couple of minutes": 2.0 * 60,    # 2分钟
        "a few minutes": 3.0 * 60,        # 3分钟
        "some minutes": 5.0 * 60,         # 5分钟
        
        # 几小时
        "several hours": 3.0 * 3600,      # 3小时
        "few hours": 2.0 * 3600,          # 2小时
        "couple of hours": 2.0 * 3600,    # 2小时
        "a few hours": 3.0 * 3600,        # 3小时
        "some hours": 4.0 * 3600,         # 4小时
        
        # 特殊时间
        "overnight": 12.0 * 3600,         # 12小时
        "over night": 12.0 * 3600,        # 12小时
        "morning": 4.0 * 3600,            # 4小时
        "afternoon": 6.0 * 3600,          # 6小时
        "evening": 4.0 * 3600,            # 4小时
        
        # 短时间
        "briefly": 30.0,                  # 30秒
        "momentarily": 10.0,              # 10秒
        "quickly": 60.0,                  # 1分钟
        "slowly": 10.0 * 60,              # 10分钟
        
        # 长时间
        "extended": 6.0 * 3600,           # 6小时
        "prolonged": 8.0 * 3600,          # 8小时
        "extensively": 12.0 * 3600,       # 12小时
    }
    
    # 直接匹配
    if time_spec in time_spec_map:
        result = time_spec_map[time_spec]
        debug_print(f"时间规格解析: '{time_spec}' → {result/60:.1f}分钟")
        return result
    
    # 模糊匹配
    for spec, value in time_spec_map.items():
        if spec in time_spec or time_spec in spec:
            result = value
            debug_print(f"时间规格模糊匹配: '{time_spec}' → '{spec}' → {result/60:.1f}分钟")
            return result
    
    # 如果无法识别，返回默认值
    default_time = 5.0 * 60  # 5分钟
    debug_print(f"⚠️ 无法识别时间规格: '{time_spec}'，使用默认值: {default_time/60:.1f}分钟")
    return default_time

def parse_time_string(time_str: str) -> float:
    """
    解析时间字符串为秒数，支持多种单位
    
    Args:
        time_str: 时间字符串（如 "0.5 h", "30 min", "120 s", "2.5"）
    
    Returns:
        float: 时间（秒）
    """
    if not time_str:
        return 0.0
    
    # 如果是纯数字，默认单位为秒
    try:
        return float(time_str)
    except ValueError:
        pass
    
    # 清理字符串
    time_str = time_str.lower().strip()
    
    # 使用正则表达式匹配数字和单位
    pattern = r'(\d+\.?\d*)\s*([a-z]*)'
    match = re.match(pattern, time_str)
    
    if not match:
        debug_print(f"⚠️ 无法解析时间字符串: '{time_str}'，使用默认值: 60秒")
        return 60.0
    
    value = float(match.group(1))
    unit = match.group(2)
    
    # 单位转换映射
    unit_map = {
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
        
        # 如果没有单位，默认为秒
        '': 1.0,
    }
    
    multiplier = unit_map.get(unit, 1.0)
    result = value * multiplier
    
    debug_print(f"时间字符串解析: '{time_str}' → {value} {unit or 'seconds'} → {result}秒")
    return result

def parse_time_input(time_input: Union[str, float, int], time_spec: str = "") -> float:
    """
    统一的时间输入解析函数
    
    Args:
        time_input: 时间输入（可以是字符串、浮点数或整数）
        time_spec: 时间规格字符串（优先级高于time_input）
    
    Returns:
        float: 时间（秒）
    """
    # 优先处理 time_spec
    if time_spec:
        return parse_time_spec(time_spec)
    
    # 处理 time_input
    if isinstance(time_input, (int, float)):
        # 数字默认单位为秒
        result = float(time_input)
        debug_print(f"数字时间输入: {time_input} → {result}秒")
        return result
    
    if isinstance(time_input, str):
        return parse_time_string(time_input)
    
    # 默认值
    debug_print(f"⚠️ 无法处理时间输入: {time_input}，使用默认值: 60秒")
    return 60.0

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
    time: Union[str, float, int] = "300",        # 🔧 修改：默认为字符串
    stir_time: Union[str, float, int] = "0",     # 🔧 修改：支持字符串
    time_spec: str = "",
    event: str = "",
    stir_speed: float = 200.0,
    settling_time: Union[str, float] = "60",     # 🔧 修改：支持字符串
    **kwargs
) -> List[Dict[str, Any]]:
    """
    生成搅拌操作的协议序列 - 支持单位
    
    Args:
        G: 设备图
        vessel: 搅拌容器名称（必需）
        time: 搅拌时间（支持 "5 min", "300", "0.5 h" 等）
        stir_time: 搅拌时间（与time等效，支持单位）
        time_spec: 时间规格（优先级最高）
        event: 事件标识
        stir_speed: 搅拌速度 (RPM)，默认200 RPM
        settling_time: 沉降时间（支持单位，默认60秒）
        **kwargs: 其他参数（兼容性）
    
    Returns:
        List[Dict[str, Any]]: 搅拌操作的动作序列
    """
    
    debug_print("=" * 50)
    debug_print("开始生成搅拌协议（支持单位）")
    debug_print(f"输入参数:")
    debug_print(f"  - vessel: {vessel}")
    debug_print(f"  - time: {time}")
    debug_print(f"  - stir_time: {stir_time}")
    debug_print(f"  - time_spec: {time_spec}")
    debug_print(f"  - event: {event}")
    debug_print(f"  - stir_speed: {stir_speed}")
    debug_print(f"  - settling_time: {settling_time}")
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
    
    # === 🔧 新增：单位解析处理 ===
    debug_print("步骤2: 单位解析处理...")
    
    # 确定实际使用的时间值（stir_time优先）
    actual_time_input = stir_time if stir_time not in ["0", 0, 0.0] else time
    
    # 解析时间（time_spec > actual_time_input）
    if time_spec:
        parsed_time = parse_time_spec(time_spec)  # 使用现有的time_spec解析
        debug_print(f"使用time_spec: '{time_spec}' → {parsed_time}s")
    else:
        parsed_time = parse_time_with_units(actual_time_input, "s")
        debug_print(f"解析时间: {actual_time_input} → {parsed_time}s")
    
    # 解析沉降时间
    parsed_settling_time = parse_time_with_units(settling_time, "s")
    debug_print(f"解析沉降时间: {settling_time} → {parsed_settling_time}s")
    
    debug_print(f"时间解析结果:")
    debug_print(f"  - 原始输入: time={time}, stir_time={stir_time}")
    debug_print(f"  - 时间规格: {time_spec}")
    debug_print(f"  - 最终搅拌时间: {parsed_time}s ({parsed_time/60:.1f}分钟)")
    debug_print(f"  - 最终沉降时间: {parsed_settling_time}s ({parsed_settling_time/60:.1f}分钟)")
    
    # 修正参数范围
    if parsed_time < 0:
        debug_print(f"搅拌时间 {parsed_time}s 无效，修正为 300s")
        parsed_time = 300.0
    elif parsed_time > 7200:
        debug_print(f"搅拌时间 {parsed_time}s 过长，修正为 3600s")
        parsed_time = 3600.0
    
    if stir_speed < 10.0:
        debug_print(f"搅拌速度 {stir_speed} RPM 过低，修正为 100 RPM")
        stir_speed = 100.0
    elif stir_speed > 1500.0:
        debug_print(f"搅拌速度 {stir_speed} RPM 过高，修正为 1000 RPM")
        stir_speed = 1000.0
    
    if parsed_settling_time < 0:
        debug_print(f"沉降时间 {parsed_settling_time}s 无效，修正为 60s")
        parsed_settling_time = 60.0
    elif parsed_settling_time > 1800:
        debug_print(f"沉降时间 {parsed_settling_time}s 过长，修正为 600s")
        parsed_settling_time = 600.0
    
    # === 查找搅拌设备 ===
    debug_print("步骤3: 查找搅拌设备...")
    
    try:
        stirrer_id = find_connected_stirrer(G, vessel)
        debug_print(f"设备配置: 搅拌设备 = {stirrer_id}")
        
    except Exception as e:
        debug_print(f"❌ 设备查找失败: {str(e)}")
        raise ValueError(f"无法找到搅拌设备: {str(e)}")
    
    # === 执行搅拌操作 ===
    debug_print("步骤4: 执行搅拌操作...")
    
    # 构建搅拌动作参数
    stir_kwargs = {
        "vessel": vessel,
        "time": str(time),                    # 保持原始字符串格式
        "event": event,
        "time_spec": time_spec,
        "stir_time": parsed_time,             # 解析后的时间（秒）
        "stir_speed": stir_speed,
        "settling_time": parsed_settling_time # 解析后的沉降时间（秒）
    }
    
    debug_print(f"搅拌参数: {stir_kwargs}")
    
    stir_action = {
        "device_id": stirrer_id,
        "action_name": "stir",
        "action_kwargs": stir_kwargs
    }
    
    action_sequence.append(stir_action)
    
    # === 总结 ===
    debug_print("=" * 50)
    debug_print(f"搅拌协议生成完成（支持单位）")
    debug_print(f"总动作数: {len(action_sequence)}")
    debug_print(f"搅拌容器: {vessel}")
    debug_print(f"搅拌参数: {stir_speed} RPM, {parsed_time}s, 沉降 {parsed_settling_time}s")
    debug_print("=" * 50)
    
    return action_sequence

def generate_start_stir_protocol(
    G: nx.DiGraph,
    vessel: str,
    stir_speed: float = 200.0,
    purpose: str = "",
    **kwargs
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
    **kwargs
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
