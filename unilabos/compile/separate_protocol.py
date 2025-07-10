import networkx as nx
import re
import logging
from typing import List, Dict, Any, Union
from .pump_protocol import generate_pump_protocol_with_rinsing

logger = logging.getLogger(__name__)

def debug_print(message):
    """调试输出"""
    print(f"[SEPARATE] {message}", flush=True)
    logger.info(f"[SEPARATE] {message}")

def parse_volume_input(volume_input: Union[str, float]) -> float:
    """
    解析体积输入，支持带单位的字符串
    
    Args:
        volume_input: 体积输入（如 "200 mL", "?", 50.0）
    
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
        default_volume = 100.0  # 默认100mL
        debug_print(f"检测到未知体积，使用默认值: {default_volume}mL")
        return default_volume
    
    # 移除空格并提取数字和单位
    volume_clean = re.sub(r'\s+', '', volume_str)
    
    # 匹配数字和单位的正则表达式
    match = re.match(r'([0-9]*\.?[0-9]+)\s*(ml|l|μl|ul|microliter|milliliter|liter)?', volume_clean)
    
    if not match:
        debug_print(f"⚠️ 无法解析体积: '{volume_str}'，使用默认值100mL")
        return 100.0
    
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

def find_solvent_vessel(G: nx.DiGraph, solvent: str) -> str:
    """查找溶剂容器"""
    if not solvent or not solvent.strip():
        return ""
    
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
            ('reagent' in node_id.lower() or 'bottle' in node_id.lower())):
            debug_print(f"⚠️ 未找到专用容器，使用备选容器: {node_id}")
            return node_id
    
    debug_print(f"⚠️ 未找到溶剂 '{solvent}' 的容器")
    return ""

def find_separator_device(G: nx.DiGraph, vessel: str) -> str:
    """查找分离器设备"""
    debug_print(f"查找容器 '{vessel}' 对应的分离器设备...")
    
    # 方法1：查找连接到容器的分离器设备
    for node in G.nodes():
        node_class = G.nodes[node].get('class', '').lower()
        if 'separator' in node_class:
            # 检查是否连接到目标容器
            if G.has_edge(node, vessel) or G.has_edge(vessel, node):
                debug_print(f"✅ 找到连接的分离器: {node}")
                return node
    
    # 方法2：根据命名规则查找
    possible_names = [
        f"{vessel}_controller",
        f"{vessel}_separator",
        vessel,  # 容器本身可能就是分离器
        "separator_1",
        "virtual_separator"
    ]
    
    for name in possible_names:
        if name in G.nodes():
            node_class = G.nodes[name].get('class', '').lower()
            if 'separator' in node_class:
                debug_print(f"✅ 通过命名规则找到分离器: {name}")
                return name
    
    # 方法3：查找第一个分离器设备
    for node in G.nodes():
        node_class = G.nodes[node].get('class', '').lower()
        if 'separator' in node_class:
            debug_print(f"⚠️ 使用第一个分离器设备: {node}")
            return node
    
    debug_print(f"⚠️ 未找到分离器设备")
    return ""

def generate_separate_protocol(
    G: nx.DiGraph,
    # 🔧 基础参数，支持XDL的vessel参数
    vessel: str = "",                    # XDL: 分离容器
    purpose: str = "separate",           # 分离目的
    product_phase: str = "top",          # 产物相
    # 🔧 可选的详细参数
    from_vessel: str = "",               # 源容器（通常在separate前已经transfer了）
    separation_vessel: str = "",         # 分离容器（与vessel同义）
    to_vessel: str = "",                 # 目标容器（可选）
    waste_phase_to_vessel: str = "",     # 废相目标容器
    product_vessel: str = "",            # XDL: 产物容器（与to_vessel同义）
    waste_vessel: str = "",              # XDL: 废液容器（与waste_phase_to_vessel同义）
    # 🔧 溶剂相关参数
    solvent: str = "",                   # 溶剂名称
    solvent_volume: Union[str, float] = 0.0,  # 溶剂体积
    volume: Union[str, float] = 0.0,     # XDL: 体积（与solvent_volume同义）
    # 🔧 操作参数
    through: str = "",                   # 通过材料
    repeats: int = 1,                    # 重复次数
    stir_time: float = 30.0,             # 搅拌时间（秒）
    stir_speed: float = 300.0,           # 搅拌速度
    settling_time: float = 300.0,        # 沉降时间（秒）
    **kwargs
) -> List[Dict[str, Any]]:
    """
    生成分离操作的协议序列 - 修复版
    
    支持XDL参数格式：
    - vessel: 分离容器（必需）
    - purpose: "wash", "extract", "separate"
    - product_phase: "top", "bottom"
    - product_vessel: 产物收集容器
    - waste_vessel: 废液收集容器
    - solvent: 溶剂名称
    - volume: "200 mL", "?" 或数值
    - repeats: 重复次数
    
    分离流程：
    1. （可选）添加溶剂到分离容器
    2. 搅拌混合
    3. 静置分层
    4. 收集指定相到目标容器
    5. 重复指定次数
    """
    
    debug_print("=" * 60)
    debug_print("开始生成分离协议 - 修复版")
    debug_print(f"原始参数:")
    debug_print(f"  - vessel: '{vessel}'")
    debug_print(f"  - purpose: '{purpose}'")
    debug_print(f"  - product_phase: '{product_phase}'")
    debug_print(f"  - solvent: '{solvent}'")
    debug_print(f"  - volume: {volume} (类型: {type(volume)})")
    debug_print(f"  - repeats: {repeats}")
    debug_print(f"  - product_vessel: '{product_vessel}'")
    debug_print(f"  - waste_vessel: '{waste_vessel}'")
    debug_print("=" * 60)
    
    action_sequence = []
    
    # === 参数验证和标准化 ===
    debug_print("步骤1: 参数验证和标准化...")
    
    # 统一容器参数
    final_vessel = vessel or separation_vessel
    if not final_vessel:
        raise ValueError("必须指定分离容器 (vessel 或 separation_vessel)")
    
    final_to_vessel = to_vessel or product_vessel
    final_waste_vessel = waste_phase_to_vessel or waste_vessel
    
    # 统一体积参数
    final_volume = parse_volume_input(volume or solvent_volume)
    
    # 🔧 修复：确保repeats至少为1
    if repeats <= 0:
        repeats = 1
        debug_print(f"⚠️ repeats参数 <= 0，自动设置为1")
    
    debug_print(f"标准化参数:")
    debug_print(f"  - 分离容器: '{final_vessel}'")
    debug_print(f"  - 产物容器: '{final_to_vessel}'")
    debug_print(f"  - 废液容器: '{final_waste_vessel}'")
    debug_print(f"  - 溶剂体积: {final_volume}mL")
    debug_print(f"  - 重复次数: {repeats}")
    
    # 验证必需参数
    if not purpose:
        purpose = "separate"
    if not product_phase:
        product_phase = "top"
    if purpose not in ["wash", "extract", "separate"]:
        debug_print(f"⚠️ 未知的分离目的 '{purpose}'，使用默认值 'separate'")
        purpose = "separate"
    if product_phase not in ["top", "bottom"]:
        debug_print(f"⚠️ 未知的产物相 '{product_phase}'，使用默认值 'top'")
        product_phase = "top"
    
    debug_print("✅ 参数验证通过")
    
    # === 查找设备 ===
    debug_print("步骤2: 查找设备...")
    
    # 查找分离器设备
    separator_device = find_separator_device(G, final_vessel)
    if not separator_device:
        debug_print("⚠️ 未找到分离器设备，可能无法执行分离操作")
    
    # 查找溶剂容器（如果需要）
    solvent_vessel = ""
    if solvent and solvent.strip():
        solvent_vessel = find_solvent_vessel(G, solvent)
    
    debug_print(f"设备映射:")
    debug_print(f"  - 分离器设备: '{separator_device}'")
    debug_print(f"  - 溶剂容器: '{solvent_vessel}'")
    
    # === 执行分离流程 ===
    debug_print("步骤3: 执行分离流程...")
    
    try:
        for repeat_idx in range(repeats):
            debug_print(f"3.{repeat_idx+1}: 第 {repeat_idx+1}/{repeats} 次分离")
            
            # 步骤3.1: 添加溶剂（如果需要）
            if solvent_vessel and final_volume > 0:
                debug_print(f"3.{repeat_idx+1}.1: 添加溶剂 {solvent} ({final_volume}mL)")
                
                # 使用pump protocol添加溶剂
                pump_actions = generate_pump_protocol_with_rinsing(
                    G=G,
                    from_vessel=solvent_vessel,
                    to_vessel=final_vessel,
                    volume=final_volume,
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
                    event="",
                    through="",
                    **kwargs
                )
                action_sequence.extend(pump_actions)
                debug_print(f"✅ 溶剂添加完成，添加了 {len(pump_actions)} 个动作")
            
            # 步骤3.2: 执行分离操作
            if separator_device:
                debug_print(f"3.{repeat_idx+1}.2: 执行分离操作")
                
                # 调用分离器设备的separate方法
                separate_action = {
                    "device_id": separator_device,
                    "action_name": "separate",
                    "action_kwargs": {
                        "purpose": purpose,
                        "product_phase": product_phase,
                        "from_vessel": from_vessel or final_vessel,
                        "separation_vessel": final_vessel,
                        "to_vessel": final_to_vessel or final_vessel,
                        "waste_phase_to_vessel": final_waste_vessel or final_vessel,
                        "solvent": solvent,
                        "solvent_volume": final_volume,
                        "through": through,
                        "repeats": 1,  # 每次调用只做一次分离
                        "stir_time": stir_time,
                        "stir_speed": stir_speed,
                        "settling_time": settling_time
                    }
                }
                action_sequence.append(separate_action)
                debug_print(f"✅ 分离操作添加完成")
            
            else:
                debug_print(f"3.{repeat_idx+1}.2: 无分离器设备，跳过分离操作")
                # 添加等待时间模拟分离
                action_sequence.append({
                    "action_name": "wait",
                    "action_kwargs": {"time": stir_time + settling_time}
                })
            
            # 等待间隔（除了最后一次）
            if repeat_idx < repeats - 1:
                action_sequence.append({
                    "action_name": "wait",
                    "action_kwargs": {"time": 5}
                })
    
    except Exception as e:
        debug_print(f"⚠️ 分离流程执行失败: {str(e)}")
        # 添加错误日志
        action_sequence.append({
            "device_id": "system",
            "action_name": "log_message",
            "action_kwargs": {
                "message": f"分离操作失败: {str(e)}"
            }
        })
    
    # === 最终结果 ===
    debug_print("=" * 60)
    debug_print(f"✅ 分离协议生成完成")
    debug_print(f"📊 总动作数: {len(action_sequence)}")
    debug_print(f"📋 处理总结:")
    debug_print(f"  - 分离容器: {final_vessel}")
    debug_print(f"  - 分离目的: {purpose}")
    debug_print(f"  - 产物相: {product_phase}")
    debug_print(f"  - 重复次数: {repeats}")
    if solvent:
        debug_print(f"  - 溶剂: {solvent} ({final_volume}mL)")
    if final_to_vessel:
        debug_print(f"  - 产物容器: {final_to_vessel}")
    if final_waste_vessel:
        debug_print(f"  - 废液容器: {final_waste_vessel}")
    debug_print("=" * 60)
    
    return action_sequence

# === 便捷函数 ===

def separate_phases_only(G: nx.DiGraph, vessel: str, product_phase: str = "top", 
                        product_vessel: str = "", waste_vessel: str = "") -> List[Dict[str, Any]]:
    """仅进行相分离（不添加溶剂）"""
    return generate_separate_protocol(
        G, vessel=vessel, 
        purpose="separate", 
        product_phase=product_phase,
        product_vessel=product_vessel,
        waste_vessel=waste_vessel
    )

def wash_with_solvent(G: nx.DiGraph, vessel: str, solvent: str, volume: Union[str, float],
                     product_phase: str = "top", repeats: int = 1) -> List[Dict[str, Any]]:
    """用溶剂洗涤"""
    return generate_separate_protocol(
        G, vessel=vessel,
        purpose="wash",
        product_phase=product_phase,
        solvent=solvent,
        volume=volume,
        repeats=repeats
    )

def extract_with_solvent(G: nx.DiGraph, vessel: str, solvent: str, volume: Union[str, float],
                        product_phase: str = "bottom", repeats: int = 3) -> List[Dict[str, Any]]:
    """用溶剂萃取"""
    return generate_separate_protocol(
        G, vessel=vessel,
        purpose="extract",
        product_phase=product_phase,
        solvent=solvent,
        volume=volume,
        repeats=repeats
    )

def separate_aqueous_organic(G: nx.DiGraph, vessel: str, organic_phase: str = "top",
                            product_vessel: str = "", waste_vessel: str = "") -> List[Dict[str, Any]]:
    """水-有机相分离"""
    return generate_separate_protocol(
        G, vessel=vessel,
        purpose="separate",
        product_phase=organic_phase,
        product_vessel=product_vessel,
        waste_vessel=waste_vessel
    )

# 测试函数
def test_separate_protocol():
    """测试分离协议的各种参数解析"""
    print("=== SEPARATE PROTOCOL 增强版测试 ===")
    
    # 测试体积解析
    volumes = ["200 mL", "?", 100.0, "1 L", "500 μL"]
    for vol in volumes:
        result = parse_volume_input(vol)
        print(f"体积解析: {vol} → {result}mL")
    
    print("✅ 测试完成")

if __name__ == "__main__":
    test_separate_protocol()
