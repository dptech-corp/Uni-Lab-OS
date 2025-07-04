import networkx as nx
from typing import List, Dict, Any
from .pump_protocol import generate_pump_protocol_with_rinsing


def find_acid_base_vessel(G: nx.DiGraph, reagent: str) -> str:
    """
    查找酸碱试剂容器，支持多种匹配模式
    
    Args:
        G: 网络图
        reagent: 试剂名称（如 "hydrochloric acid", "sodium hydroxide"）
    
    Returns:
        str: 试剂容器ID
    """
    print(f"ADJUST_PH: 正在查找试剂 '{reagent}' 的容器...")
    
    # 常见酸碱试剂的别名映射
    reagent_aliases = {
        "hydrochloric acid": ["HCl", "hydrochloric_acid", "hcl", "muriatic_acid"],
        "sodium hydroxide": ["NaOH", "sodium_hydroxide", "naoh", "caustic_soda"],
        "sulfuric acid": ["H2SO4", "sulfuric_acid", "h2so4"],
        "nitric acid": ["HNO3", "nitric_acid", "hno3"],
        "acetic acid": ["CH3COOH", "acetic_acid", "glacial_acetic_acid"],
        "ammonia": ["NH3", "ammonium_hydroxide", "nh3"],
        "potassium hydroxide": ["KOH", "potassium_hydroxide", "koh"]
    }
    
    # 构建搜索名称列表
    search_names = [reagent.lower()]
    
    # 添加别名
    for base_name, aliases in reagent_aliases.items():
        if reagent.lower() in base_name.lower() or base_name.lower() in reagent.lower():
            search_names.extend([alias.lower() for alias in aliases])
    
    # 构建可能的容器名称
    possible_names = []
    for name in search_names:
        name_clean = name.replace(" ", "_").replace("-", "_")
        possible_names.extend([
            f"flask_{name_clean}",
            f"bottle_{name_clean}",
            f"reagent_{name_clean}",
            f"acid_{name_clean}" if "acid" in name else f"base_{name_clean}",
            f"{name_clean}_bottle",
            f"{name_clean}_flask",
            name_clean
        ])
    
    # 第一步：通过容器名称匹配
    for vessel_name in possible_names:
        if vessel_name in G.nodes():
            print(f"ADJUST_PH: 通过名称匹配找到容器: {vessel_name}")
            return vessel_name
    
    # 第二步：通过模糊匹配
    for node_id in G.nodes():
        if G.nodes[node_id].get('type') == 'container':
            node_name = G.nodes[node_id].get('name', '').lower()
            
            # 检查是否包含任何搜索名称
            for search_name in search_names:
                if search_name in node_id.lower() or search_name in node_name:
                    print(f"ADJUST_PH: 通过模糊匹配找到容器: {node_id}")
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
                    
                    for search_name in search_names:
                        if search_name in liquid_type or search_name in reagent_name:
                            print(f"ADJUST_PH: 通过液体类型匹配找到容器: {node_id}")
                            return node_id
    
    # 列出可用容器帮助调试
    available_containers = []
    for node_id in G.nodes():
        if G.nodes[node_id].get('type') == 'container':
            vessel_data = G.nodes[node_id].get('data', {})
            liquids = vessel_data.get('liquid', [])
            liquid_types = [liquid.get('liquid_type', '') or liquid.get('name', '') 
                           for liquid in liquids if isinstance(liquid, dict)]
            
            available_containers.append({
                'id': node_id,
                'name': G.nodes[node_id].get('name', ''),
                'liquids': liquid_types,
                'reagent_name': vessel_data.get('reagent_name', '')
            })
    
    print(f"ADJUST_PH: 可用容器列表:")
    for container in available_containers:
        print(f"  - {container['id']}: {container['name']}")
        print(f"    液体: {container['liquids']}")
        print(f"    试剂: {container['reagent_name']}")
    
    raise ValueError(f"找不到试剂 '{reagent}' 对应的容器。尝试了: {possible_names}")


def find_connected_stirrer(G: nx.DiGraph, vessel: str) -> str:
    """查找与容器相连的搅拌器"""
    stirrer_nodes = [node for node in G.nodes() 
                    if (G.nodes[node].get('class') or '') == 'virtual_stirrer']
    
    for stirrer in stirrer_nodes:
        if G.has_edge(stirrer, vessel) or G.has_edge(vessel, stirrer):
            return stirrer
    
    return stirrer_nodes[0] if stirrer_nodes else None


def calculate_reagent_volume(target_ph_value: float, reagent: str, vessel_volume: float = 100.0) -> float:  # 改为 target_ph_value
    """
    估算需要的试剂体积来调节pH
    
    Args:
        target_ph_value: 目标pH值  # 改为 target_ph_value
        reagent: 试剂名称
        vessel_volume: 容器体积 (mL)
    
    Returns:
        float: 估算的试剂体积 (mL)
    """
    # 简化的pH调节体积估算（实际应用中需要更精确的计算）
    if "acid" in reagent.lower() or "hcl" in reagent.lower():
        # 酸性试剂：pH越低需要的体积越大
        if target_ph_value < 3:  # 改为 target_ph_value
            return vessel_volume * 0.05  # 5%
        elif target_ph_value < 5:  # 改为 target_ph_value
            return vessel_volume * 0.02  # 2%
        else:
            return vessel_volume * 0.01  # 1%
    
    elif "hydroxide" in reagent.lower() or "naoh" in reagent.lower():
        # 碱性试剂：pH越高需要的体积越大
        if target_ph_value > 11:  # 改为 target_ph_value
            return vessel_volume * 0.05  # 5%
        elif target_ph_value > 9:  # 改为 target_ph_value
            return vessel_volume * 0.02  # 2%
        else:
            return vessel_volume * 0.01  # 1%
    
    else:
        # 未知试剂，使用默认值
        return vessel_volume * 0.01


def generate_adjust_ph_protocol(
    G: nx.DiGraph,
    vessel: str,
    ph_value: float,  # 改为 ph_value
    reagent: str,
    **kwargs
) -> List[Dict[str, Any]]:
    """
    生成调节pH的协议序列
    
    Args:
        G: 有向图，节点为容器和设备
        vessel: 目标容器（需要调节pH的容器）
        ph_value: 目标pH值（从XDL传入）  # 改为 ph_value
        reagent: 酸碱试剂名称（从XDL传入）
        **kwargs: 其他可选参数，使用默认值
    
    Returns:
        List[Dict[str, Any]]: 动作序列
    """
    action_sequence = []
    
    # 从kwargs中获取可选参数，如果没有则使用默认值
    volume = kwargs.get('volume', 0.0)           # 自动估算体积
    stir = kwargs.get('stir', True)              # 默认搅拌
    stir_speed = kwargs.get('stir_speed', 300.0) # 默认搅拌速度
    stir_time = kwargs.get('stir_time', 60.0)    # 默认搅拌时间
    settling_time = kwargs.get('settling_time', 30.0)  # 默认平衡时间
    
    print(f"ADJUST_PH: 开始生成pH调节协议")
    print(f"  - 目标容器: {vessel}")
    print(f"  - 目标pH: {ph_value}")  # 改为 ph_value
    print(f"  - 试剂: {reagent}")
    print(f"  - 使用默认参数: 体积=自动估算, 搅拌=True, 搅拌速度=300RPM")
    
    # 1. 验证目标容器存在
    if vessel not in G.nodes():
        raise ValueError(f"目标容器 '{vessel}' 不存在于系统中")
    
    # 2. 查找酸碱试剂容器
    try:
        reagent_vessel = find_acid_base_vessel(G, reagent)
        print(f"ADJUST_PH: 找到试剂容器: {reagent_vessel}")
    except ValueError as e:
        raise ValueError(f"无法找到试剂 '{reagent}': {str(e)}")
    
    # 3. 如果未指定体积，自动估算
    if volume <= 0:
        # 获取目标容器的体积信息
        vessel_data = G.nodes[vessel].get('data', {})
        vessel_volume = vessel_data.get('max_volume', 100.0)  # 默认100mL
        
        estimated_volume = calculate_reagent_volume(ph_value, reagent, vessel_volume)  # 改为 ph_value
        volume = estimated_volume
        print(f"ADJUST_PH: 自动估算试剂体积: {volume:.2f} mL")
    
    # 4. 验证路径存在
    try:
        path = nx.shortest_path(G, source=reagent_vessel, target=vessel)
        print(f"ADJUST_PH: 找到路径: {' → '.join(path)}")
    except nx.NetworkXNoPath:
        raise ValueError(f"从试剂容器 '{reagent_vessel}' 到目标容器 '{vessel}' 没有可用路径")
    
    # 5. 先启动搅拌（如果需要）
    stirrer_id = None
    if stir:
        try:
            stirrer_id = find_connected_stirrer(G, vessel)
            
            if stirrer_id:
                print(f"ADJUST_PH: 找到搅拌器 {stirrer_id}，启动搅拌")
                action_sequence.append({
                    "device_id": stirrer_id,
                    "action_name": "start_stir",
                    "action_kwargs": {
                        "vessel": vessel,
                        "stir_speed": stir_speed,
                        "purpose": f"pH调节: 启动搅拌，准备添加 {reagent}"
                    }
                })
                
                # 等待搅拌稳定
                action_sequence.append({
                    "action_name": "wait",
                    "action_kwargs": {"time": 5}
                })
            else:
                print(f"ADJUST_PH: 警告 - 未找到搅拌器，继续执行")
        
        except Exception as e:
            print(f"ADJUST_PH: 搅拌器配置出错: {str(e)}")
    
    # 6. 缓慢添加试剂 - 使用pump_protocol
    print(f"ADJUST_PH: 开始添加试剂 {volume:.2f} mL")
    
    # 计算添加时间（pH调节需要缓慢添加）
    addition_time = max(30.0, volume * 2.0)  # 至少30秒，每mL需要2秒
    
    try:
        pump_actions = generate_pump_protocol_with_rinsing(
            G=G,
            from_vessel=reagent_vessel,
            to_vessel=vessel,
            volume=volume,
            amount="",
            time=addition_time,
            viscous=False,
            rinsing_solvent="",  # pH调节不需要清洗
            rinsing_volume=0.0,
            rinsing_repeats=0,
            solid=False,
            flowrate=0.5  # 缓慢注入
        )
        
        action_sequence.extend(pump_actions)
        
    except Exception as e:
        raise ValueError(f"生成泵协议时出错: {str(e)}")
    
    # 7. 持续搅拌以混合和平衡
    if stir and stirrer_id:
        print(f"ADJUST_PH: 持续搅拌 {stir_time} 秒以混合试剂")
        action_sequence.append({
            "device_id": stirrer_id,
            "action_name": "stir",
            "action_kwargs": {
                "stir_time": stir_time,
                "stir_speed": stir_speed,
                "settling_time": settling_time,
                "purpose": f"pH调节: 混合试剂，目标pH={ph_value}"  # 改为 ph_value
            }
        })
    
    # 8. 等待反应平衡
    action_sequence.append({
        "action_name": "wait",
        "action_kwargs": {
            "time": settling_time,
            "description": f"等待pH平衡到目标值 {ph_value}"  # 改为 ph_value
        }
    })
    
    print(f"ADJUST_PH: 协议生成完成，共 {len(action_sequence)} 个动作")
    print(f"ADJUST_PH: 预计总时间: {addition_time + stir_time + settling_time:.0f} 秒")
    
    return action_sequence


def generate_adjust_ph_protocol_stepwise(
    G: nx.DiGraph,
    vessel: str,
    ph_value: float,
    reagent: str,
    max_volume: float = 10.0,
    steps: int = 3
) -> List[Dict[str, Any]]:
    """
    分步调节pH的协议（更安全，避免过度调节）
    
    Args:
        G: 网络图
        vessel: 目标容器
        pH: 目标pH值
        reagent: 酸碱试剂
        max_volume: 最大试剂体积
        steps: 分步数量
    
    Returns:
        List[Dict[str, Any]]: 动作序列
    """
    action_sequence = []
    
    print(f"ADJUST_PH: 开始分步pH调节（{steps}步）")
    
    # 每步添加的体积
    step_volume = max_volume / steps
    
    for i in range(steps):
        print(f"ADJUST_PH: 第 {i+1}/{steps} 步，添加 {step_volume} mL")
        
        # 生成单步协议
        step_actions = generate_adjust_ph_protocol(
            G=G,
            vessel=vessel,
            ph_value=ph_value,
            reagent=reagent,
            volume=step_volume,
            stir=True,
            stir_speed=300.0,
            stir_time=30.0,
            settling_time=20.0
        )
        
        action_sequence.extend(step_actions)
        
        # 步骤间等待
        if i < steps - 1:
            action_sequence.append({
                "action_name": "wait",
                "action_kwargs": {
                    "time": 30,
                    "description": f"pH调节第{i+1}步完成，等待下一步"
                }
            })
    
    print(f"ADJUST_PH: 分步pH调节完成")
    return action_sequence


# 便捷函数：常用pH调节
def generate_acidify_protocol(
    G: nx.DiGraph,
    vessel: str,
    target_ph: float = 2.0,
    acid: str = "hydrochloric acid"
) -> List[Dict[str, Any]]:
    """酸化协议"""
    return generate_adjust_ph_protocol(
        G, vessel, target_ph, acid, 0.0, True, 300.0, 120.0, 60.0
    )


def generate_basify_protocol(
    G: nx.DiGraph,
    vessel: str,
    target_ph: float = 12.0,
    base: str = "sodium hydroxide"
) -> List[Dict[str, Any]]:
    """碱化协议"""
    return generate_adjust_ph_protocol(
        G, vessel, target_ph, base, 0.0, True, 300.0, 120.0, 60.0
    )


def generate_neutralize_protocol(
    G: nx.DiGraph,
    vessel: str,
    reagent: str = "sodium hydroxide"
) -> List[Dict[str, Any]]:
    """中和协议（pH=7）"""
    return generate_adjust_ph_protocol(
        G, vessel, 7.0, reagent, 0.0, True, 350.0, 180.0, 90.0
    )


# 测试函数
def test_adjust_ph_protocol():
    """测试pH调节协议"""
    print("=== ADJUST PH PROTOCOL 测试 ===")
    print("测试完成")


if __name__ == "__main__":
    test_adjust_ph_protocol()