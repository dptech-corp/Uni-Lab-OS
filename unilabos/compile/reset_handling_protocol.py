import networkx as nx
from typing import List, Dict, Any
from .pump_protocol import generate_pump_protocol_with_rinsing


def find_solvent_vessel(G: nx.DiGraph, solvent: str) -> str:
    """
    查找溶剂容器，支持多种匹配模式
    
    Args:
        G: 网络图
        solvent: 溶剂名称（如 "methanol", "ethanol", "water"）
    
    Returns:
        str: 溶剂容器ID
    """
    print(f"RESET_HANDLING: 正在查找溶剂 '{solvent}' 的容器...")
    
    # 构建可能的容器名称
    possible_names = [
        f"flask_{solvent}",           # flask_methanol
        f"bottle_{solvent}",          # bottle_methanol
        f"reagent_{solvent}",         # reagent_methanol
        f"reagent_bottle_{solvent}",  # reagent_bottle_methanol
        f"{solvent}_flask",           # methanol_flask
        f"{solvent}_bottle",          # methanol_bottle
        f"{solvent}",                 # methanol
        f"vessel_{solvent}",          # vessel_methanol
    ]
    
    # 第一步：通过容器名称匹配
    for vessel_name in possible_names:
        if vessel_name in G.nodes():
            print(f"RESET_HANDLING: 通过名称匹配找到容器: {vessel_name}")
            return vessel_name
    
    # 第二步：通过模糊匹配
    for node_id in G.nodes():
        if G.nodes[node_id].get('type') == 'container':
            node_name = G.nodes[node_id].get('name', '').lower()
            
            # 检查是否包含溶剂名称
            if solvent.lower() in node_id.lower() or solvent.lower() in node_name:
                print(f"RESET_HANDLING: 通过模糊匹配找到容器: {node_id}")
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
                        print(f"RESET_HANDLING: 通过液体类型匹配找到容器: {node_id}")
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
    
    print(f"RESET_HANDLING: 可用容器列表:")
    for container in available_containers:
        print(f"  - {container['id']}: {container['name']}")
        print(f"    液体: {container['liquids']}")
        print(f"    试剂: {container['reagent_name']}")
    
    raise ValueError(f"找不到溶剂 '{solvent}' 对应的容器。尝试了: {possible_names}")


def generate_reset_handling_protocol(
    G: nx.DiGraph,
    solvent: str,
    **kwargs  # 接收其他可能的参数但不使用
) -> List[Dict[str, Any]]:
    """
    生成重置处理协议序列
    
    Args:
        G: 有向图，节点为容器和设备
        solvent: 溶剂名称（从XDL传入）
        **kwargs: 其他可选参数，但不使用
    
    Returns:
        List[Dict[str, Any]]: 动作序列
    """
    action_sequence = []
    
    # 固定参数
    target_vessel = "main_reactor"  # 默认目标容器
    volume = 100.0  # 默认体积 100 mL
    
    print(f"RESET_HANDLING: 开始生成重置处理协议")
    print(f"  - 溶剂: {solvent}")
    print(f"  - 目标容器: {target_vessel}")
    print(f"  - 体积: {volume} mL")
    
    # 1. 验证目标容器存在
    if target_vessel not in G.nodes():
        raise ValueError(f"目标容器 '{target_vessel}' 不存在于系统中")
    
    # 2. 查找溶剂容器
    try:
        solvent_vessel = find_solvent_vessel(G, solvent)
        print(f"RESET_HANDLING: 找到溶剂容器: {solvent_vessel}")
    except ValueError as e:
        raise ValueError(f"无法找到溶剂 '{solvent}': {str(e)}")
    
    # 3. 验证路径存在
    try:
        path = nx.shortest_path(G, source=solvent_vessel, target=target_vessel)
        print(f"RESET_HANDLING: 找到路径: {' → '.join(path)}")
    except nx.NetworkXNoPath:
        raise ValueError(f"从溶剂容器 '{solvent_vessel}' 到目标容器 '{target_vessel}' 没有可用路径")
    
    # 4. 使用pump_protocol转移溶剂
    print(f"RESET_HANDLING: 开始转移溶剂 {volume} mL")
    
    try:
        pump_actions = generate_pump_protocol_with_rinsing(
            G=G,
            from_vessel=solvent_vessel,
            to_vessel=target_vessel,
            volume=volume,
            amount="",
            time=0.0,
            viscous=False,
            rinsing_solvent="",  # 重置处理不需要清洗
            rinsing_volume=0.0,
            rinsing_repeats=0,
            solid=False,
            flowrate=2.5,  # 正常流速
            transfer_flowrate=0.5  # 正常转移流速
        )
        
        action_sequence.extend(pump_actions)
        
    except Exception as e:
        raise ValueError(f"生成泵协议时出错: {str(e)}")
    
    # 5. 等待溶剂稳定
    action_sequence.append({
        "action_name": "wait",
        "action_kwargs": {
            "time": 10.0,
            "description": f"等待溶剂 {solvent} 稳定"
        }
    })
    
    print(f"RESET_HANDLING: 协议生成完成，共 {len(action_sequence)} 个动作")
    print(f"RESET_HANDLING: 已添加 {volume} mL {solvent} 到 {target_vessel}")
    
    return action_sequence


# 测试函数
def test_reset_handling_protocol():
    """测试重置处理协议"""
    print("=== RESET HANDLING PROTOCOL 测试 ===")
    print("测试完成")


if __name__ == "__main__":
    test_reset_handling_protocol()