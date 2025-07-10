import networkx as nx
import re
from typing import List, Dict, Any, Tuple, Union
from .pump_protocol import generate_pump_protocol_with_rinsing


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
        print(f"RECRYSTALLIZE: 数值体积输入: {volume_input} → {result}mL（默认单位）")
        return result
    
    # 处理字符串输入
    volume_str = str(volume_input).lower().strip()
    print(f"RECRYSTALLIZE: 解析体积字符串: '{volume_str}'")
    
    # 处理特殊值
    if volume_str in ['?', 'unknown', 'tbd', 'to be determined']:
        default_volume = 50.0  # 50mL默认值
        print(f"RECRYSTALLIZE: 检测到未知体积，使用默认值: {default_volume}mL")
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
        print(f"RECRYSTALLIZE: 纯数字输入: {volume_str} → {result}mL（单位: {default_unit}）")
        return result
    except ValueError:
        pass
    
    # 移除空格并提取数字和单位
    volume_clean = re.sub(r'\s+', '', volume_str)
    
    # 匹配数字和单位的正则表达式
    match = re.match(r'([0-9]*\.?[0-9]+)\s*(ml|l|μl|ul|microliter|milliliter|liter)?', volume_clean)
    
    if not match:
        print(f"RECRYSTALLIZE: ⚠️ 无法解析体积: '{volume_str}'，使用默认值: 50mL")
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
    
    print(f"RECRYSTALLIZE: 体积解析: '{volume_str}' → {value} {unit} → {volume}mL")
    return volume


def parse_ratio(ratio_str: str) -> Tuple[float, float]:
    """
    解析比例字符串，支持多种格式
    
    Args:
        ratio_str: 比例字符串（如 "1:1", "3:7", "50:50"）
    
    Returns:
        Tuple[float, float]: 比例元组 (ratio1, ratio2)
    """
    try:
        # 处理 "1:1", "3:7", "50:50" 等格式
        if ":" in ratio_str:
            parts = ratio_str.split(":")
            if len(parts) == 2:
                ratio1 = float(parts[0])
                ratio2 = float(parts[1])
                return ratio1, ratio2
        
        # 处理 "1-1", "3-7" 等格式
        if "-" in ratio_str:
            parts = ratio_str.split("-")
            if len(parts) == 2:
                ratio1 = float(parts[0])
                ratio2 = float(parts[1])
                return ratio1, ratio2
        
        # 处理 "1,1", "3,7" 等格式
        if "," in ratio_str:
            parts = ratio_str.split(",")
            if len(parts) == 2:
                ratio1 = float(parts[0])
                ratio2 = float(parts[1])
                return ratio1, ratio2
        
        # 默认 1:1
        print(f"RECRYSTALLIZE: 无法解析比例 '{ratio_str}'，使用默认比例 1:1")
        return 1.0, 1.0
    
    except ValueError:
        print(f"RECRYSTALLIZE: 比例解析错误 '{ratio_str}'，使用默认比例 1:1")
        return 1.0, 1.0


def find_solvent_vessel(G: nx.DiGraph, solvent: str) -> str:
    """
    查找溶剂容器
    
    Args:
        G: 网络图
        solvent: 溶剂名称
    
    Returns:
        str: 溶剂容器ID
    """
    print(f"RECRYSTALLIZE: 正在查找溶剂 '{solvent}' 的容器...")
    
    # 构建可能的容器名称
    possible_names = [
        f"flask_{solvent}",
        f"bottle_{solvent}",
        f"reagent_{solvent}",
        f"reagent_bottle_{solvent}",
        f"{solvent}_flask",
        f"{solvent}_bottle",
        f"{solvent}",
        f"vessel_{solvent}",
    ]
    
    # 第一步：通过容器名称匹配
    for vessel_name in possible_names:
        if vessel_name in G.nodes():
            print(f"RECRYSTALLIZE: 通过名称匹配找到容器: {vessel_name}")
            return vessel_name
    
    # 第二步：通过模糊匹配
    for node_id in G.nodes():
        if G.nodes[node_id].get('type') == 'container':
            node_name = G.nodes[node_id].get('name', '').lower()
            
            if solvent.lower() in node_id.lower() or solvent.lower() in node_name:
                print(f"RECRYSTALLIZE: 通过模糊匹配找到容器: {node_id}")
                return node_id
    
    # 第三步：通过液体类型匹配
    for node_id in G.nodes():
        if G.nodes[node_id].get('type') == 'container':
            vessel_data = G.nodes[node_id].get('data', {})
            liquids = vessel_data.get('liquid', [])
            
            for liquid in liquids:
                if isinstance(liquid, dict):
                    liquid_type = (liquid.get('liquid_type') or liquid.get('name', '')).lower()
                    reagent_name = vessel_data.get('reagent_name', '').lower()
                    
                    if solvent.lower() in liquid_type or solvent.lower() in reagent_name:
                        print(f"RECRYSTALLIZE: 通过液体类型匹配找到容器: {node_id}")
                        return node_id
    
    raise ValueError(f"找不到溶剂 '{solvent}' 对应的容器")


def generate_recrystallize_protocol(
    G: nx.DiGraph,
    ratio: str,
    solvent1: str,
    solvent2: str,
    vessel: str,
    volume: Union[str, float],  # 🔧 修改：支持字符串和数值
    **kwargs
) -> List[Dict[str, Any]]:
    """
    生成重结晶协议序列 - 支持单位
    
    Args:
        G: 有向图，节点为容器和设备
        ratio: 溶剂比例（如 "1:1", "3:7"）
        solvent1: 第一种溶剂名称
        solvent2: 第二种溶剂名称
        vessel: 目标容器
        volume: 总体积（支持 "100 mL", "50", "2.5 L" 等）
        **kwargs: 其他可选参数
    
    Returns:
        List[Dict[str, Any]]: 动作序列
    """
    action_sequence = []
    
    print(f"RECRYSTALLIZE: 开始生成重结晶协议（支持单位）")
    print(f"  - 比例: {ratio}")
    print(f"  - 溶剂1: {solvent1}")
    print(f"  - 溶剂2: {solvent2}")
    print(f"  - 容器: {vessel}")
    print(f"  - 总体积: {volume} (类型: {type(volume)})")
    
    # 1. 验证目标容器存在
    if vessel not in G.nodes():
        raise ValueError(f"目标容器 '{vessel}' 不存在于系统中")
    
    # 2. 🔧 新增：解析体积（支持单位）
    final_volume = parse_volume_with_units(volume, "mL")
    print(f"RECRYSTALLIZE: 解析体积: {volume} → {final_volume}mL")
    
    # 3. 解析比例
    ratio1, ratio2 = parse_ratio(ratio)
    total_ratio = ratio1 + ratio2
    
    # 4. 计算各溶剂体积
    volume1 = final_volume * (ratio1 / total_ratio)
    volume2 = final_volume * (ratio2 / total_ratio)
    
    print(f"RECRYSTALLIZE: 解析比例: {ratio1}:{ratio2}")
    print(f"RECRYSTALLIZE: {solvent1} 体积: {volume1:.2f} mL")
    print(f"RECRYSTALLIZE: {solvent2} 体积: {volume2:.2f} mL")
    
    # 5. 查找溶剂容器
    try:
        solvent1_vessel = find_solvent_vessel(G, solvent1)
        print(f"RECRYSTALLIZE: 找到溶剂1容器: {solvent1_vessel}")
    except ValueError as e:
        raise ValueError(f"无法找到溶剂1 '{solvent1}': {str(e)}")
    
    try:
        solvent2_vessel = find_solvent_vessel(G, solvent2)
        print(f"RECRYSTALLIZE: 找到溶剂2容器: {solvent2_vessel}")
    except ValueError as e:
        raise ValueError(f"无法找到溶剂2 '{solvent2}': {str(e)}")
    
    # 6. 验证路径存在
    try:
        path1 = nx.shortest_path(G, source=solvent1_vessel, target=vessel)
        print(f"RECRYSTALLIZE: 溶剂1路径: {' → '.join(path1)}")
    except nx.NetworkXNoPath:
        raise ValueError(f"从溶剂1容器 '{solvent1_vessel}' 到目标容器 '{vessel}' 没有可用路径")
    
    try:
        path2 = nx.shortest_path(G, source=solvent2_vessel, target=vessel)
        print(f"RECRYSTALLIZE: 溶剂2路径: {' → '.join(path2)}")
    except nx.NetworkXNoPath:
        raise ValueError(f"从溶剂2容器 '{solvent2_vessel}' 到目标容器 '{vessel}' 没有可用路径")
    
    # 7. 添加第一种溶剂
    print(f"RECRYSTALLIZE: 开始添加溶剂1 {volume1:.2f} mL")
    
    try:
        pump_actions1 = generate_pump_protocol_with_rinsing(
            G=G,
            from_vessel=solvent1_vessel,
            to_vessel=vessel,
            volume=volume1,             # 使用解析后的体积
            amount="",
            time=0.0,
            viscous=False,
            rinsing_solvent="",  # 重结晶不需要清洗
            rinsing_volume=0.0,
            rinsing_repeats=0,
            solid=False,
            flowrate=2.0,  # 正常流速
            transfer_flowrate=0.5
        )
        
        action_sequence.extend(pump_actions1)
        
    except Exception as e:
        raise ValueError(f"生成溶剂1泵协议时出错: {str(e)}")
    
    # 8. 等待溶剂1稳定
    action_sequence.append({
        "action_name": "wait",
        "action_kwargs": {
            "time": 10.0,
            "description": f"等待溶剂1 {solvent1} 稳定"
        }
    })
    
    # 9. 添加第二种溶剂
    print(f"RECRYSTALLIZE: 开始添加溶剂2 {volume2:.2f} mL")
    
    try:
        pump_actions2 = generate_pump_protocol_with_rinsing(
            G=G,
            from_vessel=solvent2_vessel,
            to_vessel=vessel,
            volume=volume2,             # 使用解析后的体积
            amount="",
            time=0.0,
            viscous=False,
            rinsing_solvent="",  # 重结晶不需要清洗
            rinsing_volume=0.0,
            rinsing_repeats=0,
            solid=False,
            flowrate=2.0,  # 正常流速
            transfer_flowrate=0.5
        )
        
        action_sequence.extend(pump_actions2)
        
    except Exception as e:
        raise ValueError(f"生成溶剂2泵协议时出错: {str(e)}")
    
    # 10. 等待溶剂2稳定
    action_sequence.append({
        "action_name": "wait",
        "action_kwargs": {
            "time": 10.0,
            "description": f"等待溶剂2 {solvent2} 稳定"
        }
    })
    
    # 11. 等待重结晶完成
    action_sequence.append({
        "action_name": "wait",
        "action_kwargs": {
            "time": 600.0,  # 等待10分钟进行重结晶
            "description": f"等待重结晶完成（{solvent1}:{solvent2} = {ratio}，总体积 {final_volume}mL）"
        }
    })
    
    print(f"RECRYSTALLIZE: 协议生成完成，共 {len(action_sequence)} 个动作")
    print(f"RECRYSTALLIZE: 预计总时间: {620/60:.1f} 分钟")
    print(f"RECRYSTALLIZE: 总体积: {final_volume}mL")
    
    return action_sequence


# 测试函数
def test_recrystallize_protocol():
    """测试重结晶协议"""
    print("=== RECRYSTALLIZE PROTOCOL 测试 ===")
    
    # 测试比例解析
    test_ratios = ["1:1", "3:7", "50:50", "1-1", "2,8", "invalid"]
    for ratio in test_ratios:
        r1, r2 = parse_ratio(ratio)
        print(f"比例 '{ratio}' -> {r1}:{r2}")
    
    print("测试完成")


if __name__ == "__main__":
    test_recrystallize_protocol()