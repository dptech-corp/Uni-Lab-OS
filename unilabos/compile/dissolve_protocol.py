import networkx as nx
import re
import logging
from typing import List, Dict, Any, Union
from .pump_protocol import generate_pump_protocol_with_rinsing

logger = logging.getLogger(__name__)

def debug_print(message):
    """调试输出"""
    print(f"[DISSOLVE] {message}", flush=True)
    logger.info(f"[DISSOLVE] {message}")

def parse_volume_input(volume_input: Union[str, float]) -> float:
    """
    解析体积输入，支持带单位的字符串
    
    Args:
        volume_input: 体积输入（如 "10 mL", "?", 10.0）
    
    Returns:
        float: 体积（毫升）
    """
    if isinstance(volume_input, (int, float)):
        return float(volume_input)
    
    if not volume_input or not str(volume_input).strip():
        return 0.0
    
    volume_str = str(volume_input).lower().strip()
    debug_print(f"解析体积输入: '{volume_str}'")
    
    # 处理未知体积
    if volume_str in ['?', 'unknown', 'tbd', 'to be determined']:
        default_volume = 50.0  # 默认50mL
        debug_print(f"检测到未知体积，使用默认值: {default_volume}mL")
        return default_volume
    
    # 移除空格并提取数字和单位
    volume_clean = re.sub(r'\s+', '', volume_str)
    
    # 匹配数字和单位的正则表达式
    match = re.match(r'([0-9]*\.?[0-9]+)\s*(ml|l|μl|ul|microliter|milliliter|liter)?', volume_clean)
    
    if not match:
        debug_print(f"⚠️ 无法解析体积: '{volume_str}'，使用默认值50mL")
        return 50.0
    
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

def parse_mass_input(mass_input: Union[str, float]) -> float:
    """
    解析质量输入，支持带单位的字符串
    
    Args:
        mass_input: 质量输入（如 "2.9 g", "?", 2.5）
    
    Returns:
        float: 质量（克）
    """
    if isinstance(mass_input, (int, float)):
        return float(mass_input)
    
    if not mass_input or not str(mass_input).strip():
        return 0.0
    
    mass_str = str(mass_input).lower().strip()
    debug_print(f"解析质量输入: '{mass_str}'")
    
    # 处理未知质量
    if mass_str in ['?', 'unknown', 'tbd', 'to be determined']:
        default_mass = 1.0  # 默认1g
        debug_print(f"检测到未知质量，使用默认值: {default_mass}g")
        return default_mass
    
    # 移除空格并提取数字和单位
    mass_clean = re.sub(r'\s+', '', mass_str)
    
    # 匹配数字和单位的正则表达式
    match = re.match(r'([0-9]*\.?[0-9]+)\s*(g|mg|kg|gram|milligram|kilogram)?', mass_clean)
    
    if not match:
        debug_print(f"⚠️ 无法解析质量: '{mass_str}'，返回0.0g")
        return 0.0
    
    value = float(match.group(1))
    unit = match.group(2) or 'g'  # 默认单位为克
    
    # 转换为克
    if unit in ['mg', 'milligram']:
        mass = value / 1000.0  # mg -> g
    elif unit in ['kg', 'kilogram']:
        mass = value * 1000.0  # kg -> g
    else:  # g, gram 或默认
        mass = value  # 已经是g
    
    debug_print(f"质量转换: {value}{unit} → {mass}g")
    return mass

def parse_time_input(time_input: Union[str, float]) -> float:
    """
    解析时间输入，支持带单位的字符串
    
    Args:
        time_input: 时间输入（如 "30 min", "1 h", "?", 60.0）
    
    Returns:
        float: 时间（秒）
    """
    if isinstance(time_input, (int, float)):
        return float(time_input)
    
    if not time_input or not str(time_input).strip():
        return 0.0
    
    time_str = str(time_input).lower().strip()
    debug_print(f"解析时间输入: '{time_str}'")
    
    # 处理未知时间
    if time_str in ['?', 'unknown', 'tbd']:
        default_time = 600.0  # 默认10分钟
        debug_print(f"检测到未知时间，使用默认值: {default_time}s")
        return default_time
    
    # 移除空格并提取数字和单位
    time_clean = re.sub(r'\s+', '', time_str)
    
    # 匹配数字和单位的正则表达式
    match = re.match(r'([0-9]*\.?[0-9]+)\s*(s|sec|second|min|minute|h|hr|hour|d|day)?', time_clean)
    
    if not match:
        debug_print(f"⚠️ 无法解析时间: '{time_str}'，返回0s")
        return 0.0
    
    value = float(match.group(1))
    unit = match.group(2) or 's'  # 默认单位为秒
    
    # 转换为秒
    if unit in ['min', 'minute']:
        time_sec = value * 60.0  # min -> s
    elif unit in ['h', 'hr', 'hour']:
        time_sec = value * 3600.0  # h -> s
    elif unit in ['d', 'day']:
        time_sec = value * 86400.0  # d -> s
    else:  # s, sec, second 或默认
        time_sec = value  # 已经是s
    
    debug_print(f"时间转换: {value}{unit} → {time_sec}s")
    return time_sec

def parse_temperature_input(temp_input: Union[str, float]) -> float:
    """
    解析温度输入，支持带单位的字符串
    
    Args:
        temp_input: 温度输入（如 "60 °C", "room temperature", "?", 25.0）
    
    Returns:
        float: 温度（摄氏度）
    """
    if isinstance(temp_input, (int, float)):
        return float(temp_input)
    
    if not temp_input or not str(temp_input).strip():
        return 25.0  # 默认室温
    
    temp_str = str(temp_input).lower().strip()
    debug_print(f"解析温度输入: '{temp_str}'")
    
    # 处理特殊温度描述
    temp_aliases = {
        'room temperature': 25.0,
        'rt': 25.0,
        'ambient': 25.0,
        'cold': 4.0,
        'ice': 0.0,
        'reflux': 80.0,  # 默认回流温度
        '?': 25.0,
        'unknown': 25.0
    }
    
    if temp_str in temp_aliases:
        result = temp_aliases[temp_str]
        debug_print(f"温度别名解析: '{temp_str}' → {result}°C")
        return result
    
    # 移除空格并提取数字和单位
    temp_clean = re.sub(r'\s+', '', temp_str)
    
    # 匹配数字和单位的正则表达式
    match = re.match(r'([0-9]*\.?[0-9]+)\s*(°c|c|celsius|°f|f|fahrenheit|k|kelvin)?', temp_clean)
    
    if not match:
        debug_print(f"⚠️ 无法解析温度: '{temp_str}'，使用默认值25°C")
        return 25.0
    
    value = float(match.group(1))
    unit = match.group(2) or 'c'  # 默认单位为摄氏度
    
    # 转换为摄氏度
    if unit in ['°f', 'f', 'fahrenheit']:
        temp_c = (value - 32) * 5/9  # F -> C
    elif unit in ['k', 'kelvin']:
        temp_c = value - 273.15  # K -> C
    else:  # °c, c, celsius 或默认
        temp_c = value  # 已经是C
    
    debug_print(f"温度转换: {value}{unit} → {temp_c}°C")
    return temp_c

def find_solvent_vessel(G: nx.DiGraph, solvent: str) -> str:
    """增强版溶剂容器查找"""
    debug_print(f"查找溶剂 '{solvent}' 的容器...")
    
    # 🔧 方法1：直接搜索 data.reagent_name 和 config.reagent
    for node in G.nodes():
        node_data = G.nodes[node].get('data', {})
        node_type = G.nodes[node].get('type', '')
        config_data = G.nodes[node].get('config', {})
        
        # 只搜索容器类型的节点
        if node_type == 'container':
            reagent_name = node_data.get('reagent_name', '').lower()
            config_reagent = config_data.get('reagent', '').lower()
            
            # 精确匹配
            if reagent_name == solvent.lower() or config_reagent == solvent.lower():
                debug_print(f"✅ 通过reagent字段找到容器: {node}")
                return node
            
            # 模糊匹配
            if (solvent.lower() in reagent_name and reagent_name) or \
               (solvent.lower() in config_reagent and config_reagent):
                debug_print(f"✅ 通过reagent字段模糊匹配到容器: {node}")
                return node
    
    # 🔧 方法2：常见的容器命名规则
    solvent_clean = solvent.lower().replace(' ', '_').replace('-', '_')
    possible_names = [
        f"flask_{solvent_clean}",
        f"bottle_{solvent_clean}",
        f"vessel_{solvent_clean}",
        f"{solvent_clean}_flask",
        f"{solvent_clean}_bottle",
        f"solvent_{solvent_clean}",
        f"reagent_{solvent_clean}",
        f"reagent_bottle_{solvent_clean}"
    ]
    
    for name in possible_names:
        if name in G.nodes():
            node_type = G.nodes[name].get('type', '')
            if node_type == 'container':
                debug_print(f"✅ 通过命名规则找到容器: {name}")
                return name
    
    # 🔧 方法3：使用第一个试剂瓶作为备选
    for node_id in G.nodes():
        node_data = G.nodes[node_id]
        if (node_data.get('type') == 'container' and 
            ('reagent' in node_id.lower() or 'bottle' in node_id.lower() or 'flask' in node_id.lower())):
            debug_print(f"⚠️ 未找到专用容器，使用备选容器: {node_id}")
            return node_id
    
    raise ValueError(f"找不到溶剂 '{solvent}' 对应的容器")

def find_connected_heatchill(G: nx.DiGraph, vessel: str) -> str:
    """查找连接到指定容器的加热搅拌器"""
    heatchill_nodes = []
    for node in G.nodes():
        node_class = G.nodes[node].get('class', '').lower()
        if 'heatchill' in node_class:
            heatchill_nodes.append(node)
    
    # 查找连接到容器的加热器
    for heatchill in heatchill_nodes:
        if G.has_edge(heatchill, vessel) or G.has_edge(vessel, heatchill):
            debug_print(f"找到连接的加热器: {heatchill}")
            return heatchill
    
    # 返回第一个加热器
    if heatchill_nodes:
        debug_print(f"使用第一个加热器: {heatchill_nodes[0]}")
        return heatchill_nodes[0]
    
    return ""

def find_connected_stirrer(G: nx.DiGraph, vessel: str) -> str:
    """查找连接到指定容器的搅拌器"""
    stirrer_nodes = []
    for node in G.nodes():
        node_class = G.nodes[node].get('class', '').lower()
        if 'stirrer' in node_class:
            stirrer_nodes.append(node)
    
    # 查找连接到容器的搅拌器
    for stirrer in stirrer_nodes:
        if G.has_edge(stirrer, vessel) or G.has_edge(vessel, stirrer):
            debug_print(f"找到连接的搅拌器: {stirrer}")
            return stirrer
    
    # 返回第一个搅拌器
    if stirrer_nodes:
        debug_print(f"使用第一个搅拌器: {stirrer_nodes[0]}")
        return stirrer_nodes[0]
    
    return ""

def find_solid_dispenser(G: nx.DiGraph) -> str:
    """查找固体加样器"""
    for node in G.nodes():
        node_class = G.nodes[node].get('class', '').lower()
        if 'solid_dispenser' in node_class or 'dispenser' in node_class:
            debug_print(f"找到固体加样器: {node}")
            return node
    
    debug_print("⚠️ 未找到固体加样器")
    return ""

def generate_dissolve_protocol(
    G: nx.DiGraph,
    vessel: str,
    # 🔧 修复：按照checklist.md的DissolveProtocol参数
    solvent: str = "",
    volume: Union[str, float] = 0.0,
    amount: str = "",
    temp: Union[str, float] = 25.0,
    time: Union[str, float] = 0.0,
    stir_speed: float = 300.0,
    # 🔧 关键修复：添加缺失的参数，防止"unexpected keyword argument"错误
    mass: Union[str, float] = 0.0,  # 这个参数在action文件中存在，必须包含
    mol: str = "",                  # 这个参数在action文件中存在，必须包含
    reagent: str = "",              # 这个参数在action文件中存在，必须包含
    event: str = "",                # 这个参数在action文件中存在，必须包含
    **kwargs                        # 🔧 关键：接受所有其他参数，防止unexpected keyword错误
) -> List[Dict[str, Any]]:
    """
    生成溶解操作的协议序列 - 修复版
    
    🔧 修复要点：
    1. 添加action文件中的所有参数（mass, mol, reagent, event）
    2. 使用 **kwargs 接受所有额外参数，防止 unexpected keyword argument 错误
    3. 支持固体溶解和液体溶解两种模式
    
    支持两种溶解模式：
    1. 液体溶解：指定 solvent + volume，使用pump protocol转移溶剂
    2. 固体溶解：指定 mass/mol + reagent，使用固体加样器添加固体试剂
    
    支持所有XDL参数和单位：
    - volume: "10 mL", "?" 或数值
    - mass: "2.9 g", "?" 或数值  
    - temp: "60 °C", "room temperature", "?" 或数值
    - time: "30 min", "1 h", "?" 或数值
    - mol: "0.12 mol", "16.2 mmol"
    """
    
    debug_print("=" * 60)
    debug_print("开始生成溶解协议 - 修复版")
    debug_print(f"原始参数:")
    debug_print(f"  - vessel: '{vessel}'")
    debug_print(f"  - solvent: '{solvent}'")
    debug_print(f"  - volume: {volume} (类型: {type(volume)})")
    debug_print(f"  - mass: {mass} (类型: {type(mass)})")
    debug_print(f"  - temp: {temp} (类型: {type(temp)})")
    debug_print(f"  - time: {time} (类型: {type(time)})")
    debug_print(f"  - reagent: '{reagent}'")
    debug_print(f"  - mol: '{mol}'")
    debug_print(f"  - event: '{event}'")
    debug_print(f"  - kwargs: {kwargs}")  # 显示额外参数
    debug_print("=" * 60)
    
    action_sequence = []
    
    # === 参数验证 ===
    debug_print("步骤1: 参数验证...")
    
    if not vessel:
        raise ValueError("vessel 参数不能为空")
    
    if vessel not in G.nodes():
        raise ValueError(f"容器 '{vessel}' 不存在于系统中")
    
    debug_print("✅ 基本参数验证通过")
    
    # === 🔧 关键修复：参数解析 ===
    debug_print("步骤2: 参数解析...")
    
    # 解析各种参数为数值
    final_volume = parse_volume_input(volume)
    final_mass = parse_mass_input(mass)
    final_temp = parse_temperature_input(temp)
    final_time = parse_time_input(time)
    
    debug_print(f"解析结果:")
    debug_print(f"  - 体积: {final_volume}mL")
    debug_print(f"  - 质量: {final_mass}g")
    debug_print(f"  - 温度: {final_temp}°C")
    debug_print(f"  - 时间: {final_time}s")
    debug_print(f"  - 试剂: '{reagent}'")
    debug_print(f"  - 摩尔: '{mol}'")
    debug_print(f"  - 事件: '{event}'")
    
    # === 判断溶解类型 ===
    debug_print("步骤3: 判断溶解类型...")
    
    # 判断是固体溶解还是液体溶解
    is_solid_dissolve = (final_mass > 0 or (mol and mol.strip() != "") or (reagent and reagent.strip() != ""))
    is_liquid_dissolve = (final_volume > 0 and solvent and solvent.strip() != "")
    
    if not is_solid_dissolve and not is_liquid_dissolve:
        # 默认为液体溶解，50mL
        is_liquid_dissolve = True
        final_volume = 50.0
        if not solvent:
            solvent = "water"  # 默认溶剂
        debug_print("⚠️ 未明确指定溶解参数，默认为50mL水溶解")
    
    debug_print(f"溶解类型: {'固体溶解' if is_solid_dissolve else '液体溶解'}")
    
    # === 查找设备 ===
    debug_print("步骤4: 查找设备...")
    
    # 查找加热搅拌器
    heatchill_id = find_connected_heatchill(G, vessel)
    stirrer_id = find_connected_stirrer(G, vessel)
    
    # 优先使用加热搅拌器，否则使用独立搅拌器
    stir_device_id = heatchill_id or stirrer_id
    
    debug_print(f"设备映射:")
    debug_print(f"  - 加热器: '{heatchill_id}'")
    debug_print(f"  - 搅拌器: '{stirrer_id}'")
    debug_print(f"  - 使用设备: '{stir_device_id}'")
    
    # === 执行溶解流程 ===
    debug_print("步骤5: 执行溶解流程...")
    
    try:
        # 步骤5.1: 启动加热搅拌（如果需要）
        if stir_device_id and (final_temp > 25.0 or final_time > 0 or stir_speed > 0):
            debug_print(f"5.1: 启动加热搅拌，温度: {final_temp}°C")
            
            if heatchill_id and (final_temp > 25.0 or final_time > 0):
                # 使用加热搅拌器
                heatchill_action = {
                    "device_id": heatchill_id,
                    "action_name": "heat_chill_start",
                    "action_kwargs": {
                        "vessel": vessel,
                        "temp": final_temp,
                        "purpose": f"溶解准备 - {event}" if event else "溶解准备"
                    }
                }
                action_sequence.append(heatchill_action)
                
                # 等待温度稳定
                if final_temp > 25.0:
                    wait_time = min(60, abs(final_temp - 25.0) * 1.5)
                    action_sequence.append({
                        "action_name": "wait",
                        "action_kwargs": {"time": wait_time}
                    })
            
            elif stirrer_id:
                # 使用独立搅拌器
                stir_action = {
                    "device_id": stirrer_id,
                    "action_name": "start_stir",
                    "action_kwargs": {
                        "vessel": vessel,
                        "stir_speed": stir_speed,
                        "purpose": f"溶解搅拌 - {event}" if event else "溶解搅拌"
                    }
                }
                action_sequence.append(stir_action)
                
                # 等待搅拌稳定
                action_sequence.append({
                    "action_name": "wait",
                    "action_kwargs": {"time": 5}
                })
        
        if is_solid_dissolve:
            # === 固体溶解路径 ===
            debug_print(f"5.2: 使用固体溶解路径")
            
            solid_dispenser = find_solid_dispenser(G)
            if solid_dispenser:
                # 固体加样
                add_kwargs = {
                    "vessel": vessel,
                    "reagent": reagent or amount or "solid reagent",
                    "purpose": f"溶解固体试剂 - {event}" if event else "溶解固体试剂",
                    "event": event
                }
                
                if final_mass > 0:
                    add_kwargs["mass"] = str(final_mass)
                if mol and mol.strip():
                    add_kwargs["mol"] = mol
                
                action_sequence.append({
                    "device_id": solid_dispenser,
                    "action_name": "add_solid",
                    "action_kwargs": add_kwargs
                })
                
                debug_print(f"✅ 固体加样完成")
            else:
                debug_print("⚠️ 未找到固体加样器，跳过固体添加")
        
        elif is_liquid_dissolve:
            # === 液体溶解路径 ===
            debug_print(f"5.3: 使用液体溶解路径")
            
            # 查找溶剂容器
            try:
                solvent_vessel = find_solvent_vessel(G, solvent)
            except ValueError as e:
                debug_print(f"⚠️ {str(e)}，跳过溶剂添加")
                solvent_vessel = None
            
            if solvent_vessel:
                # 计算流速 - 溶解时通常用较慢的速度，避免飞溅
                flowrate = 1.0  # 较慢的注入速度
                transfer_flowrate = 0.5  # 较慢的转移速度
                
                # 调用pump protocol
                pump_actions = generate_pump_protocol_with_rinsing(
                    G=G,
                    from_vessel=solvent_vessel,
                    to_vessel=vessel,
                    volume=final_volume,
                    amount=amount,
                    time=0.0,  # 不在pump level控制时间
                    viscous=False,
                    rinsing_solvent="",
                    rinsing_volume=0.0,
                    rinsing_repeats=0,
                    solid=False,
                    flowrate=flowrate,
                    transfer_flowrate=transfer_flowrate,
                    rate_spec="",
                    event=event,
                    through="",
                    **kwargs
                )
                action_sequence.extend(pump_actions)
                debug_print(f"✅ 溶剂转移完成，添加了 {len(pump_actions)} 个动作")
                
                # 溶剂添加后等待
                action_sequence.append({
                    "action_name": "wait",
                    "action_kwargs": {"time": 5}
                })
        
        # 步骤5.4: 等待溶解完成
        if final_time > 0:
            debug_print(f"5.4: 等待溶解完成 - {final_time}s")
            
            if heatchill_id:
                # 使用定时加热搅拌
                dissolve_action = {
                    "device_id": heatchill_id,
                    "action_name": "heat_chill",
                    "action_kwargs": {
                        "vessel": vessel,
                        "temp": final_temp,
                        "time": final_time,
                        "stir": True,
                        "stir_speed": stir_speed,
                        "purpose": f"溶解等待 - {event}" if event else "溶解等待"
                    }
                }
                action_sequence.append(dissolve_action)
            
            elif stirrer_id:
                # 使用定时搅拌
                stir_action = {
                    "device_id": stirrer_id,
                    "action_name": "stir",
                    "action_kwargs": {
                        "vessel": vessel,
                        "stir_time": final_time,
                        "stir_speed": stir_speed,
                        "settling_time": 0,
                        "purpose": f"溶解搅拌 - {event}" if event else "溶解搅拌"
                    }
                }
                action_sequence.append(stir_action)
            
            else:
                # 简单等待
                action_sequence.append({
                    "action_name": "wait",
                    "action_kwargs": {"time": final_time}
                })
        
        # 步骤5.5: 停止加热搅拌（如果需要）
        if heatchill_id and final_time == 0 and final_temp > 25.0:
            debug_print(f"5.5: 停止加热器")
            
            stop_action = {
                "device_id": heatchill_id,
                "action_name": "heat_chill_stop",
                "action_kwargs": {
                    "vessel": vessel
                }
            }
            action_sequence.append(stop_action)
        
    except Exception as e:
        debug_print(f"⚠️ 溶解流程执行失败: {str(e)}")
        # 添加错误日志
        action_sequence.append({
            "device_id": "system",
            "action_name": "log_message",
            "action_kwargs": {
                "message": f"溶解失败: {str(e)}"
            }
        })
    
    # === 最终结果 ===
    debug_print("=" * 60)
    debug_print(f"✅ 溶解协议生成完成")
    debug_print(f"📊 总动作数: {len(action_sequence)}")
    debug_print(f"📋 处理总结:")
    debug_print(f"  - 容器: {vessel}")
    debug_print(f"  - 溶解类型: {'固体溶解' if is_solid_dissolve else '液体溶解'}")
    if is_liquid_dissolve:
        debug_print(f"  - 溶剂: {solvent} ({final_volume}mL)")
    if is_solid_dissolve:
        debug_print(f"  - 试剂: {reagent}")
        debug_print(f"  - 质量: {final_mass}g")
        debug_print(f"  - 摩尔: {mol}")
    debug_print(f"  - 温度: {final_temp}°C")
    debug_print(f"  - 时间: {final_time}s")
    debug_print("=" * 60)
    
    return action_sequence

# === 便捷函数 ===

def dissolve_solid_by_mass(G: nx.DiGraph, vessel: str, reagent: str, mass: Union[str, float], 
                          temp: Union[str, float] = 25.0, time: Union[str, float] = "10 min") -> List[Dict[str, Any]]:
    """按质量溶解固体"""
    return generate_dissolve_protocol(
        G, vessel, 
        mass=mass, 
        reagent=reagent,
        temp=temp, 
        time=time
    )

def dissolve_solid_by_moles(G: nx.DiGraph, vessel: str, reagent: str, mol: str, 
                           temp: Union[str, float] = 25.0, time: Union[str, float] = "10 min") -> List[Dict[str, Any]]:
    """按摩尔数溶解固体"""
    return generate_dissolve_protocol(
        G, vessel, 
        mol=mol, 
        reagent=reagent,
        temp=temp, 
        time=time
    )

def dissolve_with_solvent(G: nx.DiGraph, vessel: str, solvent: str, volume: Union[str, float], 
                         temp: Union[str, float] = 25.0, time: Union[str, float] = "5 min") -> List[Dict[str, Any]]:
    """用溶剂溶解"""
    return generate_dissolve_protocol(
        G, vessel, 
        solvent=solvent, 
        volume=volume,
        temp=temp, 
        time=time
    )

def dissolve_at_room_temp(G: nx.DiGraph, vessel: str, solvent: str, volume: Union[str, float]) -> List[Dict[str, Any]]:
    """室温溶解"""
    return generate_dissolve_protocol(
        G, vessel, 
        solvent=solvent, 
        volume=volume,
        temp="room temperature", 
        time="5 min"
    )

def dissolve_with_heating(G: nx.DiGraph, vessel: str, solvent: str, volume: Union[str, float], 
                         temp: Union[str, float] = "60 °C", time: Union[str, float] = "15 min") -> List[Dict[str, Any]]:
    """加热溶解"""
    return generate_dissolve_protocol(
        G, vessel, 
        solvent=solvent, 
        volume=volume,
        temp=temp, 
        time=time
    )

# 测试函数
def test_dissolve_protocol():
    """测试溶解协议的各种参数解析"""
    print("=== DISSOLVE PROTOCOL 修复版测试 ===")
    
    # 测试体积解析
    volumes = ["10 mL", "?", 10.0, "1 L", "500 μL"]
    for vol in volumes:
        result = parse_volume_input(vol)
        print(f"体积解析: {vol} → {result}mL")
    
    # 测试质量解析
    masses = ["2.9 g", "?", 2.5, "500 mg"]
    for mass in masses:
        result = parse_mass_input(mass)
        print(f"质量解析: {mass} → {result}g")
    
    # 测试温度解析
    temps = ["60 °C", "room temperature", "?", 25.0, "reflux"]
    for temp in temps:
        result = parse_temperature_input(temp)
        print(f"温度解析: {temp} → {result}°C")
    
    # 测试时间解析
    times = ["30 min", "1 h", "?", 60.0]
    for time in times:
        result = parse_time_input(time)
        print(f"时间解析: {time} → {result}s")
    
    print("✅ 测试完成")

if __name__ == "__main__":
    test_dissolve_protocol()