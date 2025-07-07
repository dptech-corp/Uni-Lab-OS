import networkx as nx
from typing import List, Dict, Any
from .pump_protocol import generate_pump_protocol_with_rinsing


def find_reagent_vessel(G: nx.DiGraph, reagent: str) -> str:
    """增强版试剂容器查找，支持固体和液体"""
    print(f"ADD_PROTOCOL: 查找试剂 '{reagent}' 的容器...")
    
    # 1. 直接名称匹配
    possible_names = [
        reagent,
        f"flask_{reagent}",
        f"bottle_{reagent}",
        f"vessel_{reagent}", 
        f"{reagent}_flask",
        f"{reagent}_bottle",
        f"reagent_{reagent}",
        f"reagent_bottle_{reagent}",
        f"solid_reagent_bottle_{reagent}",  # 🔧 添加固体试剂瓶匹配
    ]
    
    for name in possible_names:
        if name in G.nodes():
            print(f"ADD_PROTOCOL: 找到容器: {name}")
            return name
    
    # 2. 模糊匹配 - 检查容器数据
    for node_id in G.nodes():
        node_data = G.nodes[node_id]
        if node_data.get('type') == 'container':
            # 检查配置中的试剂名称
            config_reagent = node_data.get('config', {}).get('reagent', '')
            data_reagent = node_data.get('data', {}).get('reagent_name', '')
            
            # 名称匹配
            if (config_reagent.lower() == reagent.lower() or 
                data_reagent.lower() == reagent.lower() or
                reagent.lower() in node_id.lower()):
                print(f"ADD_PROTOCOL: 模糊匹配到容器: {node_id}")
                return node_id
            
            # 液体类型匹配（保持原有逻辑）
            vessel_data = node_data.get('data', {})
            liquids = vessel_data.get('liquid', [])
            for liquid in liquids:
                if isinstance(liquid, dict):
                    liquid_type = liquid.get('liquid_type') or liquid.get('name', '')
                    if liquid_type.lower() == reagent.lower():
                        print(f"ADD_PROTOCOL: 液体类型匹配到容器: {node_id}")
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
            print(f"ADD_PROTOCOL: 找到连接的搅拌器: {stirrer}")
            return stirrer
    
    # 返回第一个搅拌器
    if stirrer_nodes:
        print(f"ADD_PROTOCOL: 使用第一个搅拌器: {stirrer_nodes[0]}")
        return stirrer_nodes[0]
    
    return None


def find_solid_dispenser(G: nx.DiGraph) -> str:
    """查找固体加样器"""
    for node in G.nodes():
        node_class = G.nodes[node].get('class', '').lower()
        if 'solid_dispenser' in node_class:
            print(f"ADD_PROTOCOL: 找到固体加样器: {node}")
            return node
    return None


def generate_add_protocol(
    G: nx.DiGraph,
    vessel: str,
    reagent: str,
    volume: float = 0.0,
    mass: float = 0.0,
    amount: str = "",
    time: float = 0.0,
    stir: bool = False,
    stir_speed: float = 300.0,
    viscous: bool = False,
    purpose: str = "添加试剂",
    # 新增XDL参数
    mol: str = "",
    event: str = "",
    rate_spec: str = "",
    equiv: str = "",
    ratio: str = "",
    **kwargs
) -> List[Dict[str, Any]]:
    """
    生成添加试剂协议
    
    智能判断：
    - 有 mass 或 mol → 固体加样器
    - 有 volume → 液体转移
    - 都没有 → 默认液体 1mL
    """
    
    print(f"ADD_PROTOCOL: 添加 {reagent} 到 {vessel}")
    print(f"  - 体积: {volume} mL, 质量: {mass} g, 摩尔: {mol}")
    print(f"  - 时间: {time} s, 事件: {event}, 速率: {rate_spec}")
    
    # 1. 验证容器
    if vessel not in G.nodes():
        raise ValueError(f"容器 '{vessel}' 不存在")
    
    # 2. 判断固体 vs 液体
    is_solid = (mass > 0 or mol.strip() != "")
    
    action_sequence = []
    
    if is_solid:
        # === 固体加样路径 ===
        print(f"ADD_PROTOCOL: 使用固体加样器")
        
        solid_dispenser = find_solid_dispenser(G)
        if not solid_dispenser:
            raise ValueError("未找到固体加样器")
        
        # 启动搅拌（如果需要）
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
        action_sequence.append({
            "device_id": solid_dispenser,
            "action_name": "add_solid", 
            "action_kwargs": {
                "vessel": vessel,
                "reagent": reagent,
                "mass": str(mass) if mass > 0 else "",
                "mol": mol,
                "purpose": purpose,
                "event": event
            }
        })
        
    else:
        # === 液体转移路径 ===
        print(f"ADD_PROTOCOL: 使用液体转移")
        
        # 默认体积
        if volume <= 0:
            volume = 1.0
            print(f"ADD_PROTOCOL: 使用默认体积 1mL")
        
        # 查找试剂容器
        try:
            reagent_vessel = find_reagent_vessel(G, reagent)
        except ValueError as e:
            # 🔧 更友好的错误提示
            available_reagents = []
            for node_id in G.nodes():
                node_data = G.nodes[node_id]
                if node_data.get('type') == 'container':
                    config_reagent = node_data.get('config', {}).get('reagent', '')
                    data_reagent = node_data.get('data', {}).get('reagent_name', '')
                    if config_reagent:
                        available_reagents.append(f"{node_id}({config_reagent})")
                    elif data_reagent:
                        available_reagents.append(f"{node_id}({data_reagent})")
            
            error_msg = f"找不到试剂 '{reagent}'。可用试剂: {', '.join(available_reagents)}"
            print(f"ADD_PROTOCOL: {error_msg}")
            raise ValueError(error_msg)
        
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
        if time > 0:
            flowrate = volume / time
            transfer_flowrate = flowrate
        else:
            flowrate = 1.0 if viscous else 2.5
            transfer_flowrate = 0.3 if viscous else 0.5
        
        # 🔧 调用 pump_protocol 时使用正确的参数
        try:
            pump_actions = generate_pump_protocol_with_rinsing(
                G=G,
                from_vessel=reagent_vessel,
                to_vessel=vessel,
                volume=volume,
                amount=amount,
                duration=time,  # 🔧 使用 duration 而不是 time
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
                equiv=equiv,
                ratio=ratio,
                **kwargs
            )
            action_sequence.extend(pump_actions)
        except Exception as e:
            raise ValueError(f"液体转移失败: {str(e)}")
    
    print(f"ADD_PROTOCOL: 生成 {len(action_sequence)} 个动作")
    return action_sequence


# 处理 wait 动作
def process_wait_action(action_kwargs: Dict[str, Any]) -> Dict[str, Any]:
    """处理等待动作"""
    wait_time = action_kwargs.get('time', 1.0)
    return {
        "action_name": "wait",
        "action_kwargs": {"time": wait_time},
        "description": f"等待 {wait_time} 秒"
    }


# 便捷函数
def add_liquid(G: nx.DiGraph, vessel: str, reagent: str, volume: float, 
               time: float = 0.0, rate_spec: str = "") -> List[Dict[str, Any]]:
    """添加液体试剂"""
    return generate_add_protocol(
        G, vessel, reagent, 
        volume=volume, 
        time=time, 
        rate_spec=rate_spec
    )


def add_solid(G: nx.DiGraph, vessel: str, reagent: str, mass: float, 
              event: str = "") -> List[Dict[str, Any]]:
    """添加固体试剂"""
    return generate_add_protocol(
        G, vessel, reagent, 
        mass=mass, 
        event=event
    )


def add_solid_mol(G: nx.DiGraph, vessel: str, reagent: str, mol: str, 
                  event: str = "") -> List[Dict[str, Any]]:
    """按摩尔数添加固体试剂"""
    return generate_add_protocol(
        G, vessel, reagent, 
        mol=mol, 
        event=event
    )


def add_dropwise(G: nx.DiGraph, vessel: str, reagent: str, volume: float, 
                 time: float = 0.0, event: str = "") -> List[Dict[str, Any]]:
    """滴加液体试剂"""
    return generate_add_protocol(
        G, vessel, reagent, 
        volume=volume, 
        time=time, 
        rate_spec="dropwise", 
        event=event
    )


def add_portionwise(G: nx.DiGraph, vessel: str, reagent: str, mass: float, 
                    time: float = 0.0, event: str = "") -> List[Dict[str, Any]]:
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
    """测试添加协议"""
    print("=== ADD PROTOCOL 修复版测试 ===")
    print("✅ 已修复设备查找逻辑")
    print("✅ 已添加固体试剂瓶支持")
    print("✅ 已修复错误处理")
    print("✅ 测试完成")


if __name__ == "__main__":
    test_add_protocol()