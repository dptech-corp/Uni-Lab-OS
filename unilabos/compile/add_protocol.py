import networkx as nx
from typing import List, Dict, Any
from .pump_protocol import generate_pump_protocol_with_rinsing


def find_reagent_vessel(G: nx.DiGraph, reagent: str) -> str:
    """
    根据试剂名称查找对应的试剂瓶
    
    Args:
        G: 网络图
        reagent: 试剂名称
    
    Returns:
        str: 试剂瓶的vessel ID
    
    Raises:
        ValueError: 如果找不到对应的试剂瓶
    """
    # 按照pump_protocol的命名规则查找试剂瓶
    reagent_vessel_id = f"flask_{reagent}"
    
    if reagent_vessel_id in G.nodes():
        return reagent_vessel_id
    
    # 如果直接匹配失败，尝试模糊匹配
    for node in G.nodes():
        if node.startswith('flask_') and reagent.lower() in node.lower():
            return node
    
    # 如果还是找不到，列出所有可用的试剂瓶
    available_flasks = [node for node in G.nodes() 
                       if node.startswith('flask_') 
                       and G.nodes[node].get('type') == 'container']
    
    raise ValueError(f"找不到试剂 '{reagent}' 对应的试剂瓶。可用试剂瓶: {available_flasks}")


def find_connected_stirrer(G: nx.DiGraph, vessel: str) -> str:
    """
    查找与指定容器相连的搅拌器
    
    Args:
        G: 网络图
        vessel: 容器ID
    
    Returns:
        str: 搅拌器ID，如果找不到则返回None
    """
    # 查找所有搅拌器节点
    stirrer_nodes = [node for node in G.nodes() 
                    if G.nodes[node].get('class') == 'virtual_stirrer']
    
    # 检查哪个搅拌器与目标容器相连
    for stirrer in stirrer_nodes:
        if G.has_edge(stirrer, vessel) or G.has_edge(vessel, stirrer):
            return stirrer
    
    # 如果没有直接连接，返回第一个可用的搅拌器
    return stirrer_nodes[0] if stirrer_nodes else None


def generate_add_protocol(
    G: nx.DiGraph,
    vessel: str,
    reagent: str,
    volume: float,
    mass: float = 0.0,
    amount: str = "",
    time: float = 0.0,
    stir: bool = False,
    stir_speed: float = 300.0,
    viscous: bool = False,
    purpose: str = "添加试剂"
) -> List[Dict[str, Any]]:
    """
    生成添加试剂的协议序列
    
    基于pump_protocol的成熟算法，实现试剂添加功能：
    1. 自动查找试剂瓶
    2. **先启动搅拌，再进行转移** - 确保试剂添加更均匀
    3. 使用pump_protocol实现液体转移
    
    Args:
        G: 有向图，节点为容器和设备，边为连接关系
        vessel: 目标容器（要添加试剂的容器）
        reagent: 试剂名称（用于查找对应的试剂瓶）
        volume: 要添加的体积 (mL)
        mass: 要添加的质量 (g) - 暂时未使用，预留接口
        amount: 其他数量描述
        time: 添加时间 (s)，如果指定则计算流速
        stir: 是否启用搅拌
        stir_speed: 搅拌速度 (RPM)
        viscous: 是否为粘稠液体
        purpose: 添加目的描述
    
    Returns:
        List[Dict[str, Any]]: 动作序列
    
    Raises:
        ValueError: 当找不到必要的设备或容器时
    """
    action_sequence = []
    
    # 1. 验证目标容器存在
    if vessel not in G.nodes():
        raise ValueError(f"目标容器 '{vessel}' 不存在于系统中")
    
    # 2. 查找试剂瓶
    try:
        reagent_vessel = find_reagent_vessel(G, reagent)
    except ValueError as e:
        raise ValueError(f"无法找到试剂 '{reagent}': {str(e)}")
    
    # 3. 验证是否存在从试剂瓶到目标容器的路径
    try:
        path = nx.shortest_path(G, source=reagent_vessel, target=vessel)
        print(f"ADD_PROTOCOL: 找到路径 {reagent_vessel} -> {vessel}: {path}")
    except nx.NetworkXNoPath:
        raise ValueError(f"从试剂瓶 '{reagent_vessel}' 到目标容器 '{vessel}' 没有可用路径")
    
    # 4. **先启动搅拌** - 关键改进！
    if stir:
        try:
            stirrer_id = find_connected_stirrer(G, vessel)
            
            if stirrer_id:
                print(f"ADD_PROTOCOL: 找到搅拌器 {stirrer_id}，将在添加前启动搅拌")
                
                # 先启动搅拌
                stir_action = {
                    "device_id": stirrer_id,
                    "action_name": "start_stir",
                    "action_kwargs": {
                        "vessel": vessel,
                        "stir_speed": stir_speed,
                        "purpose": f"{purpose}: 启动搅拌，准备添加 {reagent}"
                    }
                }
                
                action_sequence.append(stir_action)
                print(f"ADD_PROTOCOL: 已添加搅拌动作，速度 {stir_speed} RPM")
            else:
                print(f"ADD_PROTOCOL: 警告 - 需要搅拌但未找到与容器 {vessel} 相连的搅拌器")
        
        except Exception as e:
            print(f"ADD_PROTOCOL: 搅拌器配置出错: {str(e)}")
    
    # 5. 如果指定了体积，执行液体转移
    if volume > 0:
        # 5.1 计算流速参数
        if time > 0:
            # 根据时间计算流速
            transfer_flowrate = volume / time
            flowrate = transfer_flowrate
        else:
            # 使用默认流速
            if viscous:
                transfer_flowrate = 0.3  # 粘稠液体用较慢速度
                flowrate = 1.0
            else:
                transfer_flowrate = 0.5  # 普通液体默认速度
                flowrate = 2.5
        
        print(f"ADD_PROTOCOL: 准备转移 {volume} mL 从 {reagent_vessel} 到 {vessel}")
        print(f"ADD_PROTOCOL: 转移流速={transfer_flowrate} mL/s, 注入流速={flowrate} mL/s")
        
        # 5.2 使用pump_protocol的核心算法实现液体转移
        try:
            pump_actions = generate_pump_protocol_with_rinsing(
                G=G,
                from_vessel=reagent_vessel,
                to_vessel=vessel,
                volume=volume,
                amount=amount,
                time=time,
                viscous=viscous,
                rinsing_solvent="",  # 添加试剂通常不需要清洗
                rinsing_volume=0.0,
                rinsing_repeats=0,
                solid=False,
                flowrate=flowrate,
                transfer_flowrate=transfer_flowrate
            )
            
            # 添加pump actions到序列中
            action_sequence.extend(pump_actions)
            
        except Exception as e:
            raise ValueError(f"生成泵协议时出错: {str(e)}")
    
    print(f"ADD_PROTOCOL: 生成了 {len(action_sequence)} 个动作")
    return action_sequence


def generate_add_protocol_with_cleaning(
    G: nx.DiGraph,
    vessel: str,
    reagent: str,
    volume: float,
    mass: float = 0.0,
    amount: str = "",
    time: float = 0.0,
    stir: bool = False,
    stir_speed: float = 300.0,
    viscous: bool = False,
    purpose: str = "添加试剂",
    cleaning_solvent: str = "air",
    cleaning_volume: float = 5.0,
    cleaning_repeats: int = 1
) -> List[Dict[str, Any]]:
    """
    生成带清洗的添加试剂协议
    
    与普通添加协议的区别是会在添加后进行管道清洗
    
    Args:
        G: 有向图
        vessel: 目标容器
        reagent: 试剂名称
        volume: 添加体积
        mass: 添加质量（预留）
        amount: 其他数量描述
        time: 添加时间
        stir: 是否搅拌
        stir_speed: 搅拌速度
        viscous: 是否粘稠
        purpose: 添加目的
        cleaning_solvent: 清洗溶剂（"air"表示空气清洗）
        cleaning_volume: 清洗体积
        cleaning_repeats: 清洗重复次数
    
    Returns:
        List[Dict[str, Any]]: 动作序列
    """
    action_sequence = []
    
    # 1. 查找试剂瓶
    reagent_vessel = find_reagent_vessel(G, reagent)
    
    # 2. **先启动搅拌**
    if stir:
        stirrer_id = find_connected_stirrer(G, vessel)
        if stirrer_id:
            action_sequence.append({
                "device_id": stirrer_id,
                "action_name": "start_stir",
                "action_kwargs": {
                    "vessel": vessel,
                    "stir_speed": stir_speed,
                    "purpose": f"{purpose}: 启动搅拌，准备添加 {reagent}"
                }
            })
    
    # 3. 计算流速
    if time > 0:
        transfer_flowrate = volume / time
        flowrate = transfer_flowrate
    else:
        if viscous:
            transfer_flowrate = 0.3
            flowrate = 1.0
        else:
            transfer_flowrate = 0.5
            flowrate = 2.5
    
    # 4. 使用带清洗的pump_protocol
    pump_actions = generate_pump_protocol_with_rinsing(
        G=G,
        from_vessel=reagent_vessel,
        to_vessel=vessel,
        volume=volume,
        amount=amount,
        time=time,
        viscous=viscous,
        rinsing_solvent=cleaning_solvent,
        rinsing_volume=cleaning_volume,
        rinsing_repeats=cleaning_repeats,
        solid=False,
        flowrate=flowrate,
        transfer_flowrate=transfer_flowrate
    )
    
    action_sequence.extend(pump_actions)
    
    return action_sequence


def generate_sequential_add_protocol(
    G: nx.DiGraph,
    vessel: str,
    reagents: List[Dict[str, Any]],
    stir_between_additions: bool = True,
    final_stir: bool = True,
    final_stir_speed: float = 400.0,
    final_stir_time: float = 300.0
) -> List[Dict[str, Any]]:
    """
    生成连续添加多种试剂的协议
    
    Args:
        G: 网络图
        vessel: 目标容器
        reagents: 试剂列表，每个元素包含试剂添加参数
        stir_between_additions: 是否在每次添加之间搅拌
        final_stir: 是否在所有添加完成后进行最终搅拌
        final_stir_speed: 最终搅拌速度
        final_stir_time: 最终搅拌时间
    
    Returns:
        List[Dict[str, Any]]: 完整的动作序列
    
    Example:
        reagents = [
            {
                "reagent": "DMF",
                "volume": 10.0,
                "viscous": False,
                "stir_speed": 300.0
            },
            {
                "reagent": "ethyl_acetate", 
                "volume": 5.0,
                "viscous": False,
                "stir_speed": 350.0
            }
        ]
    """
    action_sequence = []
    
    for i, reagent_params in enumerate(reagents):
        print(f"ADD_PROTOCOL: 处理第 {i+1}/{len(reagents)} 个试剂: {reagent_params.get('reagent')}")
        
        # 生成单个试剂的添加协议
        add_actions = generate_add_protocol(
            G=G,
            vessel=vessel,
            reagent=reagent_params.get('reagent'),
            volume=reagent_params.get('volume', 0.0),
            mass=reagent_params.get('mass', 0.0),
            amount=reagent_params.get('amount', ''),
            time=reagent_params.get('time', 0.0),
            stir=stir_between_additions,
            stir_speed=reagent_params.get('stir_speed', 300.0),
            viscous=reagent_params.get('viscous', False),
            purpose=reagent_params.get('purpose', f'添加试剂 {i+1}')
        )
        
        action_sequence.extend(add_actions)
        
        # 在添加之间加入等待时间
        if i < len(reagents) - 1:  # 不是最后一个试剂
            action_sequence.append({
                "action_name": "wait",
                "action_kwargs": {"time": 2}
            })
    
    # 最终搅拌
    if final_stir:
        stirrer_id = find_connected_stirrer(G, vessel)
        if stirrer_id:
            action_sequence.extend([
                {
                    "action_name": "wait",
                    "action_kwargs": {"time": final_stir_time}
                }
            ])
    
    print(f"ADD_PROTOCOL: 连续添加协议生成完成，共 {len(action_sequence)} 个动作")
    return action_sequence


# 使用示例和测试函数
def test_add_protocol():
    """测试添加协议的示例"""
    print("=== ADD PROTOCOL 测试 ===")
    print("测试完成")


if __name__ == "__main__":
    test_add_protocol()