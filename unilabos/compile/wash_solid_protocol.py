from typing import List, Dict, Any, Union
import networkx as nx
import logging
import re

logger = logging.getLogger(__name__)

def debug_print(message):
    """调试输出"""
    print(f"[WASH_SOLID] {message}", flush=True)
    logger.info(f"[WASH_SOLID] {message}")

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

def parse_volume_with_units(volume_input: Union[str, float, int], default_unit: str = "mL") -> float:
    """
    解析带单位的体积输入
    
    Args:
        volume_input: 体积输入（如 "100 mL", "2.5 L", "500", "?", 100.0）
        default_unit: 默认单位（默认为毫升）
    
    Returns:
        float: 体积（毫升）
    """
    if not volume_input:
        return 0.0
    
    # 处理数值输入
    if isinstance(volume_input, (int, float)):
        result = float(volume_input)
        debug_print(f"数值体积输入: {volume_input} → {result}mL（默认单位）")
        return result
    
    # 处理字符串输入
    volume_str = str(volume_input).lower().strip()
    debug_print(f"解析体积字符串: '{volume_str}'")
    
    # 处理特殊值
    if volume_str in ['?', 'unknown', 'tbd', 'to be determined']:
        default_volume = 50.0  # 50mL默认值
        debug_print(f"检测到未知体积，使用默认值: {default_volume}mL")
        return default_volume
    
    # 如果是纯数字，使用默认单位
    try:
        value = float(volume_str)
        if default_unit.lower() in ["ml", "milliliter"]:
            result = value
        elif default_unit.lower() in ["l", "liter"]:
            result = value * 1000.0
        elif default_unit.lower() in ["μl", "ul", "microliter"]:
            result = value / 1000.0
        else:
            result = value  # 默认mL
        debug_print(f"纯数字输入: {volume_str} → {result}mL（单位: {default_unit}）")
        return result
    except ValueError:
        pass
    
    # 移除空格并提取数字和单位
    volume_clean = re.sub(r'\s+', '', volume_str)
    
    # 匹配数字和单位的正则表达式
    match = re.match(r'([0-9]*\.?[0-9]+)\s*(ml|l|μl|ul|microliter|milliliter|liter)?', volume_clean)
    
    if not match:
        debug_print(f"⚠️ 无法解析体积: '{volume_str}'，使用默认值: 50mL")
        return 50.0
    
    value = float(match.group(1))
    unit = match.group(2) or default_unit.lower()
    
    # 转换为毫升
    if unit in ['l', 'liter']:
        volume = value * 1000.0  # L -> mL
    elif unit in ['μl', 'ul', 'microliter']:
        volume = value / 1000.0  # μL -> mL
    else:  # ml, milliliter 或默认
        volume = value  # 已经是mL
    
    debug_print(f"体积解析: '{volume_str}' → {value} {unit} → {volume}mL")
    return volume

def parse_volume_spec(volume_spec: str) -> float:
    """
    解析体积规格字符串为毫升数
    
    Args:
        volume_spec: 体积规格字符串（如 "small volume", "large volume"）
    
    Returns:
        float: 体积（毫升）
    """
    if not volume_spec:
        return 0.0
    
    volume_spec = volume_spec.lower().strip()
    
    # 预定义的体积规格映射
    volume_spec_map = {
        # 小体积
        "small volume": 10.0,
        "small amount": 10.0,
        "minimal volume": 5.0,
        "tiny volume": 5.0,
        "little volume": 15.0,
        
        # 中等体积
        "medium volume": 50.0,
        "moderate volume": 50.0,
        "normal volume": 50.0,
        "standard volume": 50.0,
        
        # 大体积
        "large volume": 100.0,
        "big volume": 100.0,
        "substantial volume": 150.0,
        "generous volume": 200.0,
        
        # 极端体积
        "minimum": 5.0,
        "maximum": 500.0,
        "excess": 200.0,
        "plenty": 100.0,
    }
    
    # 直接匹配
    if volume_spec in volume_spec_map:
        result = volume_spec_map[volume_spec]
        debug_print(f"体积规格解析: '{volume_spec}' → {result}mL")
        return result
    
    # 模糊匹配
    for spec, value in volume_spec_map.items():
        if spec in volume_spec or volume_spec in spec:
            result = value
            debug_print(f"体积规格模糊匹配: '{volume_spec}' → '{spec}' → {result}mL")
            return result
    
    # 如果无法识别，返回默认值
    default_volume = 50.0
    debug_print(f"⚠️ 无法识别体积规格: '{volume_spec}'，使用默认值: {default_volume}mL")
    return default_volume

def parse_repeats_spec(repeats_spec: str) -> int:
    """
    解析重复次数规格字符串为整数
    
    Args:
        repeats_spec: 重复次数规格字符串（如 "several", "many"）
    
    Returns:
        int: 重复次数
    """
    if not repeats_spec:
        return 1
    
    repeats_spec = repeats_spec.lower().strip()
    
    # 预定义的重复次数映射
    repeats_spec_map = {
        # 少数次
        "once": 1,
        "twice": 2,
        "few": 3,
        "couple": 2,
        "several": 4,
        "some": 3,
        
        # 多次
        "many": 5,
        "multiple": 4,
        "numerous": 6,
        "repeated": 3,
        "extensively": 5,
        "thoroughly": 4,
        
        # 极端情况
        "minimal": 1,
        "maximum": 10,
        "excess": 8,
    }
    
    # 直接匹配
    if repeats_spec in repeats_spec_map:
        result = repeats_spec_map[repeats_spec]
        debug_print(f"重复次数解析: '{repeats_spec}' → {result}次")
        return result
    
    # 模糊匹配
    for spec, value in repeats_spec_map.items():
        if spec in repeats_spec or repeats_spec in spec:
            result = value
            debug_print(f"重复次数模糊匹配: '{repeats_spec}' → '{spec}' → {result}次")
            return result
    
    # 如果无法识别，返回默认值
    default_repeats = 3
    debug_print(f"⚠️ 无法识别重复次数规格: '{repeats_spec}'，使用默认值: {default_repeats}次")
    return default_repeats

def parse_mass_to_volume(mass: str) -> float:
    """
    将质量字符串转换为体积（简化假设：密度约为1 g/mL）
    
    Args:
        mass: 质量字符串（如 "10 g", "2.5g", "100mg"）
    
    Returns:
        float: 体积（毫升）
    """
    if not mass or not mass.strip():
        return 0.0
    
    mass = mass.lower().strip()
    debug_print(f"解析质量字符串: '{mass}'")
    
    # 移除空格并提取数字和单位
    mass_clean = re.sub(r'\s+', '', mass)
    
    # 匹配数字和单位的正则表达式
    match = re.match(r'([0-9]*\.?[0-9]+)\s*(g|mg|kg|gram|milligram|kilogram)?', mass_clean)
    
    if not match:
        debug_print(f"⚠️ 无法解析质量字符串: '{mass}'，返回0.0mL")
        return 0.0
    
    value = float(match.group(1))
    unit = match.group(2) or 'g'  # 默认单位为克
    
    # 转换为毫升（假设密度为1 g/mL）
    if unit in ['mg', 'milligram']:
        volume = value / 1000.0  # mg -> g -> mL
    elif unit in ['kg', 'kilogram']:
        volume = value * 1000.0  # kg -> g -> mL
    else:  # g, gram 或默认
        volume = value  # g -> mL (密度=1)
    
    debug_print(f"质量转换: {value}{unit} → {volume}mL")
    return volume

def parse_volume_string(volume_str: str) -> float:
    """
    解析体积字符串，支持带单位的输入
    
    Args:
        volume_str: 体积字符串（如 "10", "10 mL", "2.5L", "500μL", "?"）
    
    Returns:
        float: 体积（毫升）
    """
    if not volume_str or not volume_str.strip():
        return 0.0
    
    volume_str = volume_str.lower().strip()
    debug_print(f"解析体积字符串: '{volume_str}'")
    
    # 🔧 新增：处理未知体积符号
    if volume_str in ['?', 'unknown', 'tbd', 'to be determined', 'unspecified']:
        default_unknown_volume = 50.0  # 未知体积时的默认值
        debug_print(f"检测到未知体积符号 '{volume_str}'，使用默认值: {default_unknown_volume}mL")
        return default_unknown_volume
    
    # 移除空格并提取数字和单位
    volume_clean = re.sub(r'\s+', '', volume_str)
    
    # 匹配数字和单位的正则表达式
    match = re.match(r'([0-9]*\.?[0-9]+)\s*(ml|l|μl|ul|microliter|milliliter|liter)?', volume_clean)
    
    if not match:
        debug_print(f"⚠️ 无法解析体积字符串: '{volume_str}'，返回0.0mL")
        return 0.0
    
    value = float(match.group(1))
    unit = match.group(2) or 'ml'  # 默认单位为毫升
    
    # 转换为毫升
    if unit in ['l', 'liter']:
        volume = value * 1000.0  # L -> mL
    elif unit in ['μl', 'ul', 'microliter']:
        volume = value / 1000.0  # μL -> mL
    else:  # ml, milliliter 或默认
        volume = value  # 已经是mL
    
    debug_print(f"体积转换: {value}{unit} → {volume}mL")
    return volume

def parse_volume_input(volume: Union[float, str], volume_spec: str = "", mass: str = "") -> float:
    """
    统一的体积输入解析函数 - 增强版
    
    Args:
        volume: 体积数值或字符串
        volume_spec: 体积规格字符串（优先级最高）
        mass: 质量字符串（优先级第二）
    
    Returns:
        float: 体积（毫升）
    """
    debug_print(f"解析体积输入: volume={volume}, volume_spec='{volume_spec}', mass='{mass}'")
    
    # 优先级1：volume_spec
    if volume_spec and volume_spec.strip():
        result = parse_volume_spec(volume_spec)
        debug_print(f"使用volume_spec: {result}mL")
        return result
    
    # 优先级2：mass（质量转体积）
    if mass and mass.strip():
        result = parse_mass_to_volume(mass)
        if result > 0:
            debug_print(f"使用mass转换: {result}mL")
            return result
    
    # 优先级3：volume
    if volume:
        if isinstance(volume, str):
            # 字符串形式的体积
            result = parse_volume_string(volume)
            if result > 0:
                debug_print(f"使用volume字符串: {result}mL")
                return result
        elif isinstance(volume, (int, float)) and volume > 0:
            # 数值形式的体积
            result = float(volume)
            debug_print(f"使用volume数值: {result}mL")
            return result
    
    # 默认值
    default_volume = 50.0
    debug_print(f"⚠️ 所有体积输入无效，使用默认值: {default_volume}mL")
    return default_volume

def parse_repeats_input(repeats: int, repeats_spec: str = "") -> int:
    """
    统一的重复次数输入解析函数
    
    Args:
        repeats: 重复次数数值
        repeats_spec: 重复次数规格字符串（优先级高于repeats）
    
    Returns:
        int: 重复次数
    """
    # 优先处理 repeats_spec
    if repeats_spec:
        return parse_repeats_spec(repeats_spec)
    
    # 处理 repeats
    if repeats > 0:
        return repeats
    
    # 默认值
    debug_print(f"⚠️ 无法处理重复次数输入: repeats={repeats}, repeats_spec='{repeats_spec}'，使用默认值: 1次")
    return 1

def find_solvent_source(G: nx.DiGraph, solvent: str) -> str:
    """查找溶剂源容器"""
    debug_print(f"查找溶剂 '{solvent}' 的源容器...")
    
    # 可能的溶剂容器名称
    possible_names = [
        f"flask_{solvent}",
        f"reagent_bottle_{solvent}",
        f"bottle_{solvent}",
        f"container_{solvent}",
        f"source_{solvent}",
        f"liquid_reagent_bottle_{solvent}"
    ]
    
    for name in possible_names:
        if name in G.nodes():
            debug_print(f"找到溶剂容器: {name}")
            return name
    
    # 查找通用容器
    generic_containers = [
        "liquid_reagent_bottle_1",
        "liquid_reagent_bottle_2",
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

def generate_wash_solid_protocol(
    G: nx.DiGraph,
    vessel: str,
    solvent: str,
    volume: Union[float, str] = "50",        # 🔧 修改：默认为字符串
    filtrate_vessel: str = "",
    temp: float = 25.0,
    stir: bool = False,
    stir_speed: float = 0.0,
    time: Union[str, float] = "0",           # 🔧 修改：支持字符串时间
    repeats: int = 1,
    # === 现有参数保持不变 ===
    volume_spec: str = "",
    repeats_spec: str = "",
    mass: str = "",
    event: str = "",
    **kwargs
) -> List[Dict[str, Any]]:
    """
    生成固体清洗操作的协议序列 - 增强版（支持单位）
    
    支持多种输入方式：
    1. volume: "100 mL", "50", "2.5 L", "?"
    2. time: "5 min", "300", "0.5 h", "?"
    3. volume_spec: "small volume", "large volume" 等
    4. mass: "10 g", "2.5 kg", "500 mg" 等（转换为体积）
    """
    
    debug_print("=" * 60)
    debug_print("开始生成固体清洗协议（支持单位）")
    debug_print(f"输入参数:")
    debug_print(f"  - vessel: {vessel}")
    debug_print(f"  - solvent: {solvent}")
    debug_print(f"  - volume: {volume} (类型: {type(volume)})")
    debug_print(f"  - time: {time} (类型: {type(time)})")
    debug_print(f"  - volume_spec: '{volume_spec}'")
    debug_print(f"  - mass: '{mass}'")
    debug_print(f"  - filtrate_vessel: '{filtrate_vessel}'")
    debug_print(f"  - temp: {temp}°C")
    debug_print(f"  - stir: {stir}")
    debug_print(f"  - stir_speed: {stir_speed} RPM")
    debug_print(f"  - repeats: {repeats}")
    debug_print(f"  - repeats_spec: '{repeats_spec}'")
    debug_print(f"  - event: '{event}'")
    debug_print("=" * 60)
    
    action_sequence = []
    
    # === 参数验证 ===
    debug_print("步骤1: 参数验证...")
    
    # 验证必需参数
    if not vessel:
        raise ValueError("vessel 参数不能为空")
    
    if not solvent:
        raise ValueError("solvent 参数不能为空")
    
    if vessel not in G.nodes():
        raise ValueError(f"容器 '{vessel}' 不存在于系统中")
    
    debug_print(f"✅ 必需参数验证通过")
    
    # === 🔧 新增：单位解析处理 ===
    debug_print("步骤2: 单位解析处理...")
    
    # 解析体积（优先级：volume_spec > mass > volume）
    if volume_spec and volume_spec.strip():
        final_volume = parse_volume_spec(volume_spec)
        debug_print(f"使用volume_spec: {final_volume}mL")
    elif mass and mass.strip():
        final_volume = parse_mass_to_volume(mass)
        if final_volume > 0:
            debug_print(f"使用mass转换: {final_volume}mL")
        else:
            final_volume = parse_volume_with_units(volume, "mL")
            debug_print(f"mass转换失败，使用volume: {final_volume}mL")
    else:
        final_volume = parse_volume_with_units(volume, "mL")
        debug_print(f"使用volume: {final_volume}mL")
    
    # 解析时间
    final_time = parse_time_with_units(time, "s")
    debug_print(f"解析时间: {time} → {final_time}s ({final_time/60:.1f}min)")
    
    # 处理重复次数参数（repeats_spec优先）
    final_repeats = parse_repeats_input(repeats, repeats_spec)
    debug_print(f"最终重复次数: {final_repeats}次")
    
    # 修正参数范围
    if temp < 0 or temp > 200:
        debug_print(f"温度 {temp}°C 超出范围，修正为 25°C")
        temp = 25.0
    
    if stir_speed < 0 or stir_speed > 500:
        debug_print(f"搅拌速度 {stir_speed} RPM 超出范围，修正为 200 RPM")
        stir_speed = 200.0 if stir else 0.0
    
    if final_time < 0:
        debug_print(f"时间 {final_time}s 无效，修正为 0")
        final_time = 0.0
    
    if final_repeats < 1:
        debug_print(f"重复次数 {final_repeats} 无效，修正为 1")
        final_repeats = 1
    elif final_repeats > 10:
        debug_print(f"重复次数 {final_repeats} 过多，修正为 10")
        final_repeats = 10
    
    debug_print(f"✅ 单位解析和参数处理完成")
    
    # === 查找设备（保持原有逻辑）===
    debug_print("步骤3: 查找设备...")
    
    try:
        # 查找溶剂源
        solvent_source = find_solvent_source(G, solvent)
        
        # 查找滤液收集容器
        actual_filtrate_vessel = find_filtrate_vessel(G, filtrate_vessel)
        
        # 查找过滤器（用于过滤操作）
        filter_device = None
        for node in G.nodes():
            node_data = G.nodes[node]
            node_class = node_data.get('class', '') or ''
            if 'filter' in node_class.lower():
                filter_device = node
                break
        
        if not filter_device:
            filter_device = "filter_1"  # 默认过滤器
        
        # 查找转移泵（用于转移溶剂）
        transfer_pump = None
        for node in G.nodes():
            node_data = G.nodes[node]
            node_class = node_data.get('class', '') or ''
            if 'transfer' in node_class.lower() and 'pump' in node_class.lower():
                transfer_pump = node
                break
        
        if not transfer_pump:
            transfer_pump = "transfer_pump_1"  # 默认转移泵
        
        # 查找搅拌器（如果需要搅拌）
        stirrer_device = None
        if stir:
            for node in G.nodes():
                node_data = G.nodes[node]
                node_class = node_data.get('class', '') or ''
                if 'stirrer' in node_class.lower():
                    stirrer_device = node
                    break
            
            if not stirrer_device:
                stirrer_device = "stirrer_1"  # 默认搅拌器
        
        debug_print(f"设备配置:")
        debug_print(f"  - 溶剂源: {solvent_source}")
        debug_print(f"  - 转移泵: {transfer_pump}")
        debug_print(f"  - 过滤器: {filter_device}")
        debug_print(f"  - 搅拌器: {stirrer_device}")
        debug_print(f"  - 滤液容器: {actual_filtrate_vessel}")
        
    except Exception as e:
        debug_print(f"❌ 设备查找失败: {str(e)}")
        raise ValueError(f"设备查找失败: {str(e)}")
    
    # === 执行清洗循环（保持原有逻辑，使用解析后的参数）===
    debug_print("步骤4: 执行清洗循环...")
    
    for cycle in range(final_repeats):
        debug_print(f"=== 第 {cycle+1}/{final_repeats} 次清洗 ===")
        
        # 1. 加入清洗溶剂
        debug_print(f"  步骤 {cycle+1}.1: 加入清洗溶剂")
        try:
            from .pump_protocol import generate_pump_protocol_with_rinsing
            
            transfer_actions = generate_pump_protocol_with_rinsing(
                G=G,
                from_vessel=solvent_source,
                to_vessel=vessel,
                volume=final_volume,      # 使用解析后的体积
                amount="",
                time=0.0,
                viscous=False,
                rinsing_solvent="",
                rinsing_volume=0.0,
                rinsing_repeats=0,
                solid=False,
                flowrate=2.5,
                transfer_flowrate=0.5,
                rate_spec="",
                event=event,
                through=""
            )
            
            if transfer_actions:
                action_sequence.extend(transfer_actions)
                debug_print(f"✅ 添加了 {len(transfer_actions)} 个转移动作")
            else:
                debug_print("⚠️ 转移协议返回空序列")
                
        except Exception as e:
            debug_print(f"❌ 转移失败: {str(e)}")
        
        # 2. 搅拌混合（如果需要）
        if stir and stirrer_device:
            debug_print(f"  步骤 {cycle+1}.2: 搅拌混合")
            stir_time = max(final_time, 30.0) if final_time > 0 else 60.0  # 使用解析后的时间
            
            stir_action = {
                "device_id": stirrer_device,
                "action_name": "stir",
                "action_kwargs": {
                    "vessel": vessel,
                    "time": str(time),        # 保持原始字符串格式
                    "event": event,
                    "time_spec": "",
                    "stir_time": stir_time,   # 解析后的时间（秒）
                    "stir_speed": stir_speed,
                    "settling_time": 30.0
                }
            }
            action_sequence.append(stir_action)
        
        # 3. 过滤分离
        debug_print(f"  步骤 {cycle+1}.3: 过滤分离")
        filter_action = {
            "device_id": filter_device,
            "action_name": "filter",
            "action_kwargs": {
                "vessel": vessel,
                "filtrate_vessel": actual_filtrate_vessel,
                "stir": False,  # 过滤时不搅拌
                "stir_speed": 0.0,
                "temp": temp,
                "continue_heatchill": False,
                "volume": final_volume    # 使用解析后的体积
            }
        }
        action_sequence.append(filter_action)
        
        # 4. 等待完成
        wait_time = 10.0
        action_sequence.append({
            "action_name": "wait",
            "action_kwargs": {"time": wait_time}
        })
    
    # === 总结 ===
    debug_print("=" * 60)
    debug_print(f"固体清洗协议生成完成（支持单位）")
    debug_print(f"总动作数: {len(action_sequence)}")
    debug_print(f"清洗容器: {vessel}")
    debug_print(f"使用溶剂: {solvent}")
    debug_print(f"清洗体积: {final_volume}mL")
    debug_print(f"清洗时间: {final_time}s ({final_time/60:.1f}min)")
    debug_print(f"重复次数: {final_repeats}")
    debug_print(f"滤液收集: {actual_filtrate_vessel}")
    debug_print(f"事件标识: {event}")
    debug_print("=" * 60)
    
    return action_sequence

# 删除不需要的函数，简化代码
def find_wash_solid_device(G: nx.DiGraph) -> str:
    """
    🗑️ 已弃用：WashSolid不再作为单一设备动作
    现在分解为基础动作序列：transfer + stir + filter
    """
    debug_print("⚠️ find_wash_solid_device 已弃用，使用基础动作序列")
    return "OrganicSynthesisStation"  # 兼容性返回

# === 便捷函数 ===

def generate_water_wash_protocol(
    G: nx.DiGraph,
    vessel: str,
    volume: float = 50.0,
    **kwargs
) -> List[Dict[str, Any]]:
    """水洗协议：用水清洗固体"""
    return generate_wash_solid_protocol(
        G, vessel, "water", volume, **kwargs
    )

def generate_organic_wash_protocol(
    G: nx.DiGraph,
    vessel: str,
    solvent: str,
    volume: float = 30.0,
    **kwargs
) -> List[Dict[str, Any]]:
    """有机溶剂清洗协议：用有机溶剂清洗固体"""
    return generate_wash_solid_protocol(
        G, vessel, solvent, volume, **kwargs
    )

def generate_thorough_wash_protocol(
    G: nx.DiGraph,
    vessel: str,
    solvent: str,
    volume: float = 100.0,
    **kwargs
) -> List[Dict[str, Any]]:
    """彻底清洗协议：多次清洗，搅拌，加热"""
    return generate_wash_solid_protocol(
        G, vessel, solvent, volume, 
        repeats=4, temp=50.0, stir=True, stir_speed=200.0, time=300.0, **kwargs
    )