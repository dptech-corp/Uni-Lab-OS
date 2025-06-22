import networkx as nx
from typing import List, Dict, Any
from .pump_protocol import generate_pump_protocol_with_rinsing


def find_reagent_vessel(G: nx.DiGraph, reagent: str) -> str:
    """
    根据试剂名称查找对应的试剂瓶，支持多种匹配模式：
    1. 容器名称匹配（如 flask_DMF, reagent_bottle_1-DMF）
    2. 容器内液体类型匹配（如 liquid_type: "DMF", name: "ethanol"）
    3. 试剂名称匹配（如 reagent_name: "DMF", config.reagent: "ethyl_acetate"）
    
    Args:
        G: 网络图
        reagent: 试剂名称
    
    Returns:
        str: 试剂瓶的vessel ID
    
    Raises:
        ValueError: 如果找不到对应的试剂瓶
    """
    print(f"ADD_PROTOCOL: 正在查找试剂 '{reagent}' 的容器...")
    
    # 第一步：通过容器名称匹配
    possible_names = [
        f"flask_{reagent}",           # flask_DMF, flask_ethanol
        f"bottle_{reagent}",          # bottle_DMF, bottle_ethanol  
        f"vessel_{reagent}",          # vessel_DMF, vessel_ethanol
        f"{reagent}_flask",           # DMF_flask, ethanol_flask
        f"{reagent}_bottle",          # DMF_bottle, ethanol_bottle
        f"{reagent}",                 # 直接用试剂名
        f"reagent_{reagent}",         # reagent_DMF, reagent_ethanol
        f"reagent_bottle_{reagent}",  # reagent_bottle_DMF
    ]
    
    # 尝试名称匹配
    for vessel_name in possible_names:
        if vessel_name in G.nodes():
            print(f"ADD_PROTOCOL: 通过名称匹配找到容器: {vessel_name}")
            return vessel_name
    
    # 第二步：通过模糊名称匹配（名称中包含试剂名）
    for node_id in G.nodes():
        if G.nodes[node_id].get('type') == 'container':
            # 检查节点ID或名称中是否包含试剂名
            node_name = G.nodes[node_id].get('name', '').lower()
            if (reagent.lower() in node_id.lower() or 
                reagent.lower() in node_name):
                print(f"ADD_PROTOCOL: 通过模糊名称匹配找到容器: {node_id} (名称: {node_name})")
                return node_id
    
    # 第三步：通过液体类型匹配
    for node_id in G.nodes():
        if G.nodes[node_id].get('type') == 'container':
            vessel_data = G.nodes[node_id].get('data', {})
            liquids = vessel_data.get('liquid', [])
            
            for liquid in liquids:
                if isinstance(liquid, dict):
                    # 支持两种格式的液体类型字段
                    liquid_type = liquid.get('liquid_type') or liquid.get('name', '')
                    reagent_name = vessel_data.get('reagent_name', '')
                    config_reagent = G.nodes[node_id].get('config', {}).get('reagent', '')
                    
                    # 检查多个可能的字段
                    if (liquid_type.lower() == reagent.lower() or 
                        reagent_name.lower() == reagent.lower() or
                        config_reagent.lower() == reagent.lower()):
                        print(f"ADD_PROTOCOL: 通过液体类型匹配找到容器: {node_id}")
                        print(f"  - liquid_type: {liquid_type}")
                        print(f"  - reagent_name: {reagent_name}")
                        print(f"  - config.reagent: {config_reagent}")
                        return node_id
    
    # 第四步：列出所有可用的容器信息帮助调试
    available_containers = []
    for node_id in G.nodes():
        if G.nodes[node_id].get('type') == 'container':
            vessel_data = G.nodes[node_id].get('data', {})
            config_data = G.nodes[node_id].get('config', {})
            liquids = vessel_data.get('liquid', [])
            
            container_info = {
                'id': node_id,
                'name': G.nodes[node_id].get('name', ''),
                'liquid_types': [],
                'reagent_name': vessel_data.get('reagent_name', ''),
                'config_reagent': config_data.get('reagent', '')
            }
            
            for liquid in liquids:
                if isinstance(liquid, dict):
                    liquid_type = liquid.get('liquid_type') or liquid.get('name', '')
                    if liquid_type:
                        container_info['liquid_types'].append(liquid_type)
            
            available_containers.append(container_info)
    
    print(f"ADD_PROTOCOL: 可用容器列表:")
    for container in available_containers:
        print(f"  - {container['id']}: {container['name']}")
        print(f"    液体类型: {container['liquid_types']}")
        print(f"    试剂名称: {container['reagent_name']}")
        print(f"    配置试剂: {container['config_reagent']}")
    
    raise ValueError(f"找不到试剂 '{reagent}' 对应的试剂瓶。尝试了名称匹配: {possible_names}")


def find_reagent_vessel_by_any_match(G: nx.DiGraph, reagent: str) -> str:
    """
    增强版试剂容器查找，支持各种匹配方式的别名函数
    """
    return find_reagent_vessel(G, reagent)


def get_vessel_reagent_volume(G: nx.DiGraph, vessel: str) -> float:
    """获取容器中的试剂体积"""
    if vessel not in G.nodes():
        return 0.0
    
    vessel_data = G.nodes[vessel].get('data', {})
    liquids = vessel_data.get('liquid', [])
    
    total_volume = 0.0
    for liquid in liquids:
        if isinstance(liquid, dict):
            # 支持两种格式：新格式 (name, volume) 和旧格式 (liquid_type, liquid_volume)
            volume = liquid.get('volume') or liquid.get('liquid_volume', 0.0)
            total_volume += volume
    
    return total_volume


def get_vessel_reagent_types(G: nx.DiGraph, vessel: str) -> List[str]:
    """获取容器中所有试剂的类型"""
    if vessel not in G.nodes():
        return []
    
    vessel_data = G.nodes[vessel].get('data', {})
    liquids = vessel_data.get('liquid', [])
    
    reagent_types = []
    for liquid in liquids:
        if isinstance(liquid, dict):
            # 支持两种格式的试剂类型字段
            reagent_type = liquid.get('liquid_type') or liquid.get('name', '')
            if reagent_type:
                reagent_types.append(reagent_type)
    
    # 同时检查配置中的试剂信息
    config_reagent = G.nodes[vessel].get('config', {}).get('reagent', '')
    reagent_name = vessel_data.get('reagent_name', '')
    
    if config_reagent and config_reagent not in reagent_types:
        reagent_types.append(config_reagent)
    if reagent_name and reagent_name not in reagent_types:
        reagent_types.append(reagent_name)
    
    return reagent_types


def find_vessels_by_reagent(G: nx.DiGraph, reagent: str) -> List[str]:
    """
    根据试剂类型查找所有匹配的容器
    返回匹配容器的ID列表
    """
    matching_vessels = []
    
    for node_id in G.nodes():
        if G.nodes[node_id].get('type') == 'container':
            # 检查容器名称匹配
            node_name = G.nodes[node_id].get('name', '').lower()
            if reagent.lower() in node_id.lower() or reagent.lower() in node_name:
                matching_vessels.append(node_id)
                continue
            
            # 检查试剂类型匹配
            vessel_data = G.nodes[node_id].get('data', {})
            liquids = vessel_data.get('liquid', [])
            config_data = G.nodes[node_id].get('config', {})
            
            # 检查 reagent_name 和 config.reagent
            reagent_name = vessel_data.get('reagent_name', '').lower()
            config_reagent = config_data.get('reagent', '').lower()
            
            if (reagent.lower() == reagent_name or 
                reagent.lower() == config_reagent):
                matching_vessels.append(node_id)
                continue
            
            # 检查液体列表
            for liquid in liquids:
                if isinstance(liquid, dict):
                    liquid_type = liquid.get('liquid_type') or liquid.get('name', '')
                    if liquid_type.lower() == reagent.lower():
                        matching_vessels.append(node_id)
                        break
    
    return matching_vessels


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
                    if (G.nodes[node].get('class') or '') == 'virtual_stirrer']
    
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
    生成添加试剂的协议序列，支持智能试剂匹配
    
    基于pump_protocol的成熟算法，实现试剂添加功能：
    1. 智能查找试剂瓶（支持名称匹配、液体类型匹配、试剂配置匹配）
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
    
    print(f"ADD_PROTOCOL: 开始生成添加试剂协议")
    print(f"  - 目标容器: {vessel}")
    print(f"  - 试剂: {reagent}")
    print(f"  - 体积: {volume} mL")
    print(f"  - 质量: {mass} g")
    print(f"  - 搅拌: {stir} (速度: {stir_speed} RPM)")
    print(f"  - 粘稠: {viscous}")
    print(f"  - 目的: {purpose}")
    
    # 1. 验证目标容器存在
    if vessel not in G.nodes():
        raise ValueError(f"目标容器 '{vessel}' 不存在于系统中")
    
    # 2. 智能查找试剂瓶
    try:
        reagent_vessel = find_reagent_vessel(G, reagent)
        print(f"ADD_PROTOCOL: 找到试剂容器: {reagent_vessel}")
    except ValueError as e:
        raise ValueError(f"无法找到试剂 '{reagent}': {str(e)}")
    
    # 3. 验证试剂容器中的试剂体积
    available_volume = get_vessel_reagent_volume(G, reagent_vessel)
    print(f"ADD_PROTOCOL: 试剂容器 {reagent_vessel} 中有 {available_volume} mL 试剂")
    
    if available_volume < volume:
        print(f"ADD_PROTOCOL: 警告 - 试剂容器中的试剂不足！需要 {volume} mL，可用 {available_volume} mL")
    
    # 4. 验证是否存在从试剂瓶到目标容器的路径
    try:
        path = nx.shortest_path(G, source=reagent_vessel, target=vessel)
        print(f"ADD_PROTOCOL: 找到路径 {reagent_vessel} -> {vessel}: {path}")
    except nx.NetworkXNoPath:
        raise ValueError(f"从试剂瓶 '{reagent_vessel}' 到目标容器 '{vessel}' 没有可用路径")
    
    # 5. **先启动搅拌** - 关键改进！
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
                
                # 等待搅拌稳定
                action_sequence.append({
                    "action_name": "wait",
                    "action_kwargs": {"time": 5}
                })
            else:
                print(f"ADD_PROTOCOL: 警告 - 需要搅拌但未找到与容器 {vessel} 相连的搅拌器")
        
        except Exception as e:
            print(f"ADD_PROTOCOL: 搅拌器配置出错: {str(e)}")
    
    # 6. 如果指定了体积，执行液体转移
    if volume > 0:
        # 6.1 计算流速参数
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
        
        # 6.2 使用pump_protocol的核心算法实现液体转移
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
    print(f"ADD_PROTOCOL: 添加试剂协议生成完成")
    
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
    生成带清洗的添加试剂协议，支持智能试剂匹配
    
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
    
    # 1. 智能查找试剂瓶
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
            
            # 等待搅拌稳定
            action_sequence.append({
                "action_name": "wait",
                "action_kwargs": {"time": 5}
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
    生成连续添加多种试剂的协议，支持智能试剂匹配
    
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
                "reagent": "DMF",           # 会匹配 reagent_bottle_1 (reagent_name: "DMF")
                "volume": 10.0,
                "viscous": False,
                "stir_speed": 300.0
            },
            {
                "reagent": "ethyl_acetate", # 会匹配 reagent_bottle_2 (reagent_name: "ethyl_acetate")
                "volume": 5.0,
                "viscous": False,
                "stir_speed": 350.0
            }
        ]
    """
    action_sequence = []
    
    print(f"ADD_PROTOCOL: 开始连续添加 {len(reagents)} 种试剂到容器 {vessel}")
    
    for i, reagent_params in enumerate(reagents):
        reagent_name = reagent_params.get('reagent')
        print(f"ADD_PROTOCOL: 处理第 {i+1}/{len(reagents)} 个试剂: {reagent_name}")
        
        # 生成单个试剂的添加协议
        add_actions = generate_add_protocol(
            G=G,
            vessel=vessel,
            reagent=reagent_name,
            volume=reagent_params.get('volume', 0.0),
            mass=reagent_params.get('mass', 0.0),
            amount=reagent_params.get('amount', ''),
            time=reagent_params.get('time', 0.0),
            stir=stir_between_additions,
            stir_speed=reagent_params.get('stir_speed', 300.0),
            viscous=reagent_params.get('viscous', False),
            purpose=reagent_params.get('purpose', f'添加试剂 {reagent_name} ({i+1}/{len(reagents)})')
        )
        
        action_sequence.extend(add_actions)
        
        # 在添加之间加入等待时间
        if i < len(reagents) - 1:  # 不是最后一个试剂
            action_sequence.append({
                "action_name": "wait",
                "action_kwargs": {"time": 10}  # 试剂混合时间
            })
    
    # 最终搅拌
    if final_stir:
        stirrer_id = find_connected_stirrer(G, vessel)
        if stirrer_id:
            print(f"ADD_PROTOCOL: 添加最终搅拌动作，速度 {final_stir_speed} RPM，时间 {final_stir_time} 秒")
            action_sequence.extend([
                {
                    "device_id": stirrer_id,
                    "action_name": "stir",
                    "action_kwargs": {
                        "stir_time": final_stir_time,
                        "stir_speed": final_stir_speed,
                        "settling_time": 30.0
                    }
                }
            ])
    
    print(f"ADD_PROTOCOL: 连续添加协议生成完成，共 {len(action_sequence)} 个动作")
    return action_sequence


# 便捷函数：常用添加方案
def generate_organic_add_protocol(
    G: nx.DiGraph,
    vessel: str,
    organic_reagent: str,
    volume: float,
    stir_speed: float = 400.0
) -> List[Dict[str, Any]]:
    """有机试剂添加：慢速、搅拌"""
    return generate_add_protocol(
        G, vessel, organic_reagent, volume, 0.0, "", 0.0, 
        True, stir_speed, False, f"添加有机试剂 {organic_reagent}"
    )


def generate_viscous_add_protocol(
    G: nx.DiGraph,
    vessel: str,
    viscous_reagent: str,
    volume: float,
    addition_time: float = 120.0
) -> List[Dict[str, Any]]:
    """粘稠试剂添加：慢速、长时间"""
    return generate_add_protocol(
        G, vessel, viscous_reagent, volume, 0.0, "", addition_time, 
        True, 250.0, True, f"缓慢添加粘稠试剂 {viscous_reagent}"
    )


def generate_solvent_add_protocol(
    G: nx.DiGraph,
    vessel: str,
    solvent: str,
    volume: float
) -> List[Dict[str, Any]]:
    """溶剂添加：快速、无需特殊处理"""
    return generate_add_protocol(
        G, vessel, solvent, volume, 0.0, "", 0.0, 
        False, 300.0, False, f"添加溶剂 {solvent}"
    )


# 使用示例和测试函数
def test_add_protocol():
    """测试添加协议的示例"""
    print("=== ADD PROTOCOL 智能匹配测试 ===")
    print("测试完成")


if __name__ == "__main__":
    test_add_protocol()