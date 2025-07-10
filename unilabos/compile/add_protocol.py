import networkx as nx
import re
import logging
from typing import List, Dict, Any, Union
from .pump_protocol import generate_pump_protocol_with_rinsing

logger = logging.getLogger(__name__)

def debug_print(message):
    """调试输出"""
    print(f"[ADD] {message}", flush=True)
    logger.info(f"[ADD] {message}")

def parse_volume_input(volume_input: Union[str, float]) -> float:
    """
    解析体积输入，支持带单位的字符串
    
    Args:
        volume_input: 体积输入（如 "2.7 mL", "2.67 mL", "?", 10.0）
    
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
        default_volume = 10.0  # 默认10mL
        debug_print(f"检测到未知体积，使用默认值: {default_volume}mL")
        return default_volume
    
    # 移除空格并提取数字和单位
    volume_clean = re.sub(r'\s+', '', volume_str)
    
    # 匹配数字和单位的正则表达式
    match = re.match(r'([0-9]*\.?[0-9]+)\s*(ml|l|μl|ul|microliter|milliliter|liter)?', volume_clean)
    
    if not match:
        debug_print(f"⚠️ 无法解析体积: '{volume_str}'，使用默认值10mL")
        return 10.0
    
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
        mass_input: 质量输入（如 "19.3 g", "4.5 g", 2.5）
    
    Returns:
        float: 质量（克）
    """
    if isinstance(mass_input, (int, float)):
        return float(mass_input)
    
    if not mass_input or not str(mass_input).strip():
        return 0.0
    
    mass_str = str(mass_input).lower().strip()
    debug_print(f"解析质量输入: '{mass_str}'")
    
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
        time_input: 时间输入（如 "1 h", "20 min", "30 s", 60.0）
    
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
        default_time = 60.0  # 默认1分钟
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

def find_reagent_vessel(G: nx.DiGraph, reagent: str) -> str:
    """增强版试剂容器查找，支持固体和液体"""
    debug_print(f"查找试剂 '{reagent}' 的容器...")
    
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
            if reagent_name == reagent.lower() or config_reagent == reagent.lower():
                debug_print(f"✅ 通过reagent字段找到容器: {node}")
                return node
            
            # 模糊匹配
            if (reagent.lower() in reagent_name and reagent_name) or \
               (reagent.lower() in config_reagent and config_reagent):
                debug_print(f"✅ 通过reagent字段模糊匹配到容器: {node}")
                return node
    
    # 🔧 方法2：常见的容器命名规则
    reagent_clean = reagent.lower().replace(' ', '_').replace('-', '_')
    possible_names = [
        reagent_clean,
        f"flask_{reagent_clean}",
        f"bottle_{reagent_clean}",
        f"vessel_{reagent_clean}", 
        f"{reagent_clean}_flask",
        f"{reagent_clean}_bottle",
        f"reagent_{reagent_clean}",
        f"reagent_bottle_{reagent_clean}",
        f"solid_reagent_bottle_{reagent_clean}",
        f"reagent_bottle_1",  # 通用试剂瓶
        f"reagent_bottle_2",
        f"reagent_bottle_3"
    ]
    
    for name in possible_names:
        if name in G.nodes():
            node_type = G.nodes[name].get('type', '')
            if node_type == 'container':
                debug_print(f"✅ 通过命名规则找到容器: {name}")
                return name
    
    # 🔧 方法3：节点名称模糊匹配
    for node_id in G.nodes():
        node_data = G.nodes[node_id]
        if node_data.get('type') == 'container':
            # 检查节点名称是否包含试剂名称
            if reagent_clean in node_id.lower():
                debug_print(f"✅ 通过节点名称模糊匹配到容器: {node_id}")
                return node_id
            
            # 检查液体类型匹配
            vessel_data = node_data.get('data', {})
            liquids = vessel_data.get('liquid', [])
            for liquid in liquids:
                if isinstance(liquid, dict):
                    liquid_type = liquid.get('liquid_type') or liquid.get('name', '')
                    if liquid_type.lower() == reagent.lower():
                        debug_print(f"✅ 通过液体类型匹配到容器: {node_id}")
                        return node_id
    
    # 🔧 方法4：使用第一个试剂瓶作为备选
    for node_id in G.nodes():
        node_data = G.nodes[node_id]
        if (node_data.get('type') == 'container' and 
            ('reagent' in node_id.lower() or 'bottle' in node_id.lower())):
            debug_print(f"⚠️ 未找到专用容器，使用备选试剂瓶: {node_id}")
            return node_id
    
    raise ValueError(f"找不到试剂 '{reagent}' 对应的容器")

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

def generate_add_protocol(
    G: nx.DiGraph,
    vessel: str,
    reagent: str,
    # 🔧 修复：所有参数都用 Union 类型，支持字符串和数值
    volume: Union[str, float] = 0.0,
    mass: Union[str, float] = 0.0,
    amount: str = "",
    time: Union[str, float] = 0.0,
    stir: bool = False,
    stir_speed: float = 300.0,
    viscous: bool = False,
    purpose: str = "添加试剂",
    # XDL扩展参数
    mol: str = "",
    event: str = "",
    rate_spec: str = "",
    equiv: str = "",
    ratio: str = "",
    **kwargs
) -> List[Dict[str, Any]]:
    """
    生成添加试剂协议 - 修复版
    
    支持所有XDL参数和单位：
    - volume: "2.7 mL", "2.67 mL", "?" 或数值
    - mass: "19.3 g", "4.5 g" 或数值
    - time: "1 h", "20 min" 或数值（秒）
    - mol: "0.28 mol", "16.2 mmol", "25.2 mmol"
    - rate_spec: "portionwise", "dropwise"
    - event: "A", "B"
    - equiv: "1.1"
    - ratio: "?", "1:1"
    """
    
    debug_print("=" * 60)
    debug_print("开始生成添加试剂协议")
    debug_print(f"原始参数:")
    debug_print(f"  - vessel: '{vessel}'")
    debug_print(f"  - reagent: '{reagent}'")
    debug_print(f"  - volume: {volume} (类型: {type(volume)})")
    debug_print(f"  - mass: {mass} (类型: {type(mass)})")
    debug_print(f"  - time: {time} (类型: {type(time)})")
    debug_print(f"  - mol: '{mol}'")
    debug_print(f"  - event: '{event}'")
    debug_print(f"  - rate_spec: '{rate_spec}'")
    debug_print("=" * 60)
    
    action_sequence = []
    
    # === 参数验证 ===
    debug_print("步骤1: 参数验证...")
    
    if not vessel:
        raise ValueError("vessel 参数不能为空")
    if not reagent:
        raise ValueError("reagent 参数不能为空")
    
    if vessel not in G.nodes():
        raise ValueError(f"容器 '{vessel}' 不存在于系统中")
    
    debug_print("✅ 基本参数验证通过")
    
    # === 🔧 关键修复：参数解析 ===
    debug_print("步骤2: 参数解析...")
    
    # 解析各种参数为数值
    final_volume = parse_volume_input(volume)
    final_mass = parse_mass_input(mass)
    final_time = parse_time_input(time)
    
    debug_print(f"解析结果:")
    debug_print(f"  - 体积: {final_volume}mL")
    debug_print(f"  - 质量: {final_mass}g")
    debug_print(f"  - 时间: {final_time}s")
    debug_print(f"  - 摩尔: '{mol}'")
    debug_print(f"  - 事件: '{event}'")
    debug_print(f"  - 速率: '{rate_spec}'")
    
    # === 判断添加类型 ===
    debug_print("步骤3: 判断添加类型...")
    
    # 🔧 修复：现在使用解析后的数值进行比较
    is_solid = (final_mass > 0 or (mol and mol.strip() != ""))
    is_liquid = (final_volume > 0)
    
    if not is_solid and not is_liquid:
        # 默认为液体，10mL
        is_liquid = True
        final_volume = 10.0
        debug_print("⚠️ 未指定体积或质量，默认为10mL液体")
    
    debug_print(f"添加类型: {'固体' if is_solid else '液体'}")
    
    # === 执行添加流程 ===
    debug_print("步骤4: 执行添加流程...")
    
    try:
        if is_solid:
            # === 固体添加路径 ===
            debug_print(f"使用固体添加路径")
            
            solid_dispenser = find_solid_dispenser(G)
            if solid_dispenser:
                # 启动搅拌
                if stir:
                    stirrer_id = find_connected_stirrer(G, vessel)
                    if stirrer_id:
                        action_sequence.append({
                            "device_id": stirrer_id,
                            "action_name": "start_stir",
                            "action_kwargs": {
                                "vessel": vessel,
                                "stir_speed": stir_speed,
                                "purpose": f"准备添加固体 {reagent}"
                            }
                        })
                        # 等待搅拌稳定
                        action_sequence.append({
                            "action_name": "wait",
                            "action_kwargs": {"time": 3}
                        })
                
                # 固体加样
                add_kwargs = {
                    "vessel": vessel,
                    "reagent": reagent,
                    "purpose": purpose,
                    "event": event,
                    "rate_spec": rate_spec
                }
                
                if final_mass > 0:
                    add_kwargs["mass"] = str(final_mass)
                if mol and mol.strip():
                    add_kwargs["mol"] = mol
                if equiv and equiv.strip():
                    add_kwargs["equiv"] = equiv
                
                action_sequence.append({
                    "device_id": solid_dispenser,
                    "action_name": "add_solid",
                    "action_kwargs": add_kwargs
                })
                
                # 添加后等待
                if final_time > 0:
                    action_sequence.append({
                        "action_name": "wait",
                        "action_kwargs": {"time": final_time}
                    })
                    
                debug_print(f"✅ 固体添加完成")
            else:
                debug_print("⚠️ 未找到固体加样器，跳过固体添加")
        
        else:
            # === 液体添加路径 ===
            debug_print(f"使用液体添加路径")
            
            # 查找试剂容器
            reagent_vessel = find_reagent_vessel(G, reagent)
            
            # 启动搅拌
            if stir:
                stirrer_id = find_connected_stirrer(G, vessel)
                if stirrer_id:
                    action_sequence.append({
                        "device_id": stirrer_id,
                        "action_name": "start_stir",
                        "action_kwargs": {
                            "vessel": vessel,
                            "stir_speed": stir_speed,
                            "purpose": f"准备添加液体 {reagent}"
                        }
                    })
                    # 等待搅拌稳定
                    action_sequence.append({
                        "action_name": "wait",
                        "action_kwargs": {"time": 5}
                    })
            
            # 计算流速
            if final_time > 0:
                flowrate = final_volume / final_time * 60  # mL/min
                transfer_flowrate = flowrate
            else:
                if rate_spec == "dropwise":
                    flowrate = 0.5  # 滴加，很慢
                    transfer_flowrate = 0.2
                elif viscous:
                    flowrate = 1.0  # 粘性液体
                    transfer_flowrate = 0.3
                else:
                    flowrate = 2.5  # 正常流速
                    transfer_flowrate = 0.5
            
            debug_print(f"流速设置: {flowrate} mL/min")
            
            # 调用pump protocol
            pump_actions = generate_pump_protocol_with_rinsing(
                G=G,
                from_vessel=reagent_vessel,
                to_vessel=vessel,
                volume=final_volume,
                amount=amount,
                time=final_time,
                viscous=viscous,
                rinsing_solvent="",
                rinsing_volume=0.0,
                rinsing_repeats=0,
                solid=False,
                flowrate=flowrate,
                transfer_flowrate=transfer_flowrate,
                rate_spec=rate_spec,
                event=event,
                through="",
                **kwargs
            )
            action_sequence.extend(pump_actions)
            debug_print(f"✅ 液体转移完成，添加了 {len(pump_actions)} 个动作")
            
    except Exception as e:
        debug_print(f"⚠️ 试剂添加失败: {str(e)}")
        # 添加错误日志
        action_sequence.append({
            "device_id": "system",
            "action_name": "log_message",
            "action_kwargs": {
                "message": f"试剂 '{reagent}' 添加失败: {str(e)}"
            }
        })
    
    # === 最终结果 ===
    debug_print("=" * 60)
    debug_print(f"✅ 添加试剂协议生成完成")
    debug_print(f"📊 总动作数: {len(action_sequence)}")
    debug_print(f"📋 处理总结:")
    debug_print(f"  - 试剂: {reagent}")
    debug_print(f"  - 添加类型: {'固体' if is_solid else '液体'}")
    debug_print(f"  - 目标容器: {vessel}")
    if is_liquid:
        debug_print(f"  - 体积: {final_volume}mL")
    if is_solid:
        debug_print(f"  - 质量: {final_mass}g")
        debug_print(f"  - 摩尔: {mol}")
    debug_print("=" * 60)
    
    return action_sequence

# === 便捷函数 ===

def add_liquid_volume(G: nx.DiGraph, vessel: str, reagent: str, volume: Union[str, float], 
                     time: Union[str, float] = 0.0, rate_spec: str = "") -> List[Dict[str, Any]]:
    """添加指定体积的液体试剂"""
    return generate_add_protocol(
        G, vessel, reagent, 
        volume=volume, 
        time=time, 
        rate_spec=rate_spec
    )

def add_solid_mass(G: nx.DiGraph, vessel: str, reagent: str, mass: Union[str, float], 
                   event: str = "") -> List[Dict[str, Any]]:
    """添加指定质量的固体试剂"""
    return generate_add_protocol(
        G, vessel, reagent, 
        mass=mass, 
        event=event
    )

def add_solid_moles(G: nx.DiGraph, vessel: str, reagent: str, mol: str, 
                    event: str = "") -> List[Dict[str, Any]]:
    """按摩尔数添加固体试剂"""
    return generate_add_protocol(
        G, vessel, reagent, 
        mol=mol, 
        event=event
    )

def add_dropwise_liquid(G: nx.DiGraph, vessel: str, reagent: str, volume: Union[str, float], 
                        time: Union[str, float] = "20 min", event: str = "") -> List[Dict[str, Any]]:
    """滴加液体试剂"""
    return generate_add_protocol(
        G, vessel, reagent, 
        volume=volume, 
        time=time, 
        rate_spec="dropwise", 
        event=event
    )

def add_portionwise_solid(G: nx.DiGraph, vessel: str, reagent: str, mass: Union[str, float], 
                          time: Union[str, float] = "1 h", event: str = "") -> List[Dict[str, Any]]:
    """分批添加固体试剂"""
    return generate_add_protocol(
        G, vessel, reagent, 
        mass=mass, 
        time=time, 
        rate_spec="portionwise", 
        event=event
    )

# 测试函数
def test_add_protocol():
    """测试添加协议的各种参数解析"""
    print("=== ADD PROTOCOL 增强版测试 ===")
    
    # 测试体积解析
    volumes = ["2.7 mL", "2.67 mL", "?", 10.0, "1 L", "500 μL"]
    for vol in volumes:
        result = parse_volume_input(vol)
        print(f"体积解析: {vol} → {result}mL")
    
    # 测试质量解析
    masses = ["19.3 g", "4.5 g", 2.5, "500 mg", "1 kg"]
    for mass in masses:
        result = parse_mass_input(mass)
        print(f"质量解析: {mass} → {result}g")
    
    # 测试时间解析
    times = ["1 h", "20 min", "30 s", 60.0, "?"]
    for time in times:
        result = parse_time_input(time)
        print(f"时间解析: {time} → {result}s")
    
    print("✅ 测试完成")

if __name__ == "__main__":
    test_add_protocol()