import networkx as nx
from typing import List, Dict, Any
from .pump_protocol import generate_pump_protocol_with_rinsing


def debug_print(message):
    """调试输出"""
    print(f"🔄 [RESET_HANDLING] {message}", flush=True)


def find_solvent_vessel(G: nx.DiGraph, solvent: str) -> str:
    """
    查找溶剂容器，支持多种匹配模式
    
    Args:
        G: 网络图
        solvent: 溶剂名称（如 "methanol", "ethanol", "water"）
    
    Returns:
        str: 溶剂容器ID
    """
    debug_print(f"🔍 正在查找溶剂 '{solvent}' 的容器... 🧪")
    
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
    
    debug_print(f"📋 候选容器名称: {possible_names[:3]}... (共{len(possible_names)}个) 📝")
    
    # 第一步：通过容器名称匹配
    debug_print("  🎯 步骤1: 精确名称匹配...")
    for vessel_name in possible_names:
        if vessel_name in G.nodes():
            debug_print(f"  🎉 通过名称匹配找到容器: {vessel_name} ✨")
            return vessel_name
    debug_print("  😞 精确名称匹配失败，尝试模糊匹配... 🔍")
    
    # 第二步：通过模糊匹配
    debug_print("  🔍 步骤2: 模糊名称匹配...")
    for node_id in G.nodes():
        if G.nodes[node_id].get('type') == 'container':
            node_name = G.nodes[node_id].get('name', '').lower()
            
            # 检查是否包含溶剂名称
            if solvent.lower() in node_id.lower() or solvent.lower() in node_name:
                debug_print(f"  🎉 通过模糊匹配找到容器: {node_id} ✨")
                return node_id
    debug_print("  😞 模糊匹配失败，尝试液体类型匹配... 🧪")
    
    # 第三步：通过液体类型匹配
    debug_print("  🧪 步骤3: 液体类型匹配...")
    for node_id in G.nodes():
        if G.nodes[node_id].get('type') == 'container':
            vessel_data = G.nodes[node_id].get('data', {})
            liquids = vessel_data.get('liquid', [])
            
            for liquid in liquids:
                if isinstance(liquid, dict):
                    liquid_type = (liquid.get('liquid_type') or liquid.get('name', '')).lower()
                    reagent_name = vessel_data.get('reagent_name', '').lower()
                    
                    if solvent.lower() in liquid_type or solvent.lower() in reagent_name:
                        debug_print(f"  🎉 通过液体类型匹配找到容器: {node_id} ✨")
                        return node_id
    
    # 列出可用容器帮助调试
    debug_print("  📊 显示可用容器信息...")
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
    
    debug_print(f"  📋 可用容器列表 (共{len(available_containers)}个):")
    for i, container in enumerate(available_containers[:5]):  # 只显示前5个
        debug_print(f"    {i+1}. 🥽 {container['id']}: {container['name']}")
        debug_print(f"       💧 液体: {container['liquids']}")
        debug_print(f"       🧪 试剂: {container['reagent_name']}")
    
    if len(available_containers) > 5:
        debug_print(f"    ... 还有 {len(available_containers)-5} 个容器 📦")
    
    debug_print(f"❌ 找不到溶剂 '{solvent}' 对应的容器 😭")
    raise ValueError(f"找不到溶剂 '{solvent}' 对应的容器。尝试了: {possible_names[:3]}...")


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
    volume = 50.0  # 默认体积 50 mL

    debug_print("🔄" * 20)
    debug_print("🚀 开始生成重置处理协议 ✨")
    debug_print(f"📝 输入参数:")
    debug_print(f"  🧪 溶剂: {solvent}")
    debug_print(f"  🥽 目标容器: {target_vessel}")
    debug_print(f"  💧 体积: {volume} mL")
    debug_print(f"  ⚙️ 其他参数: {kwargs}")
    debug_print("🔄" * 20)
    
    # 1. 验证目标容器存在
    debug_print("📍 步骤1: 验证目标容器... 🔧")
    if target_vessel not in G.nodes():
        debug_print(f"❌ 目标容器 '{target_vessel}' 不存在于系统中! 😱")
        raise ValueError(f"目标容器 '{target_vessel}' 不存在于系统中")
    debug_print(f"✅ 目标容器 '{target_vessel}' 验证通过 🎯")
    
    # 2. 查找溶剂容器
    debug_print("📍 步骤2: 查找溶剂容器... 🔍")
    try:
        solvent_vessel = find_solvent_vessel(G, solvent)
        debug_print(f"🎉 找到溶剂容器: {solvent_vessel} ✨")
    except ValueError as e:
        debug_print(f"❌ 溶剂容器查找失败: {str(e)} 😭")
        raise ValueError(f"无法找到溶剂 '{solvent}': {str(e)}")
    
    # 3. 验证路径存在
    debug_print("📍 步骤3: 验证传输路径... 🛤️")
    try:
        path = nx.shortest_path(G, source=solvent_vessel, target=target_vessel)
        debug_print(f"🛤️ 找到路径: {' → '.join(path)} ✅")
    except nx.NetworkXNoPath:
        debug_print(f"❌ 路径不可达: {solvent_vessel} → {target_vessel} 😞")
        raise ValueError(f"从溶剂容器 '{solvent_vessel}' 到目标容器 '{target_vessel}' 没有可用路径")
    
    # 4. 使用pump_protocol转移溶剂
    debug_print("📍 步骤4: 转移溶剂... 🚰")
    debug_print(f"  🚛 开始转移: {solvent_vessel} → {target_vessel}")
    debug_print(f"  💧 转移体积: {volume} mL")
    
    try:
        debug_print("  🔄 生成泵送协议...")
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
        debug_print(f"  ✅ 泵送协议已添加: {len(pump_actions)} 个动作 🚰✨")
        
    except Exception as e:
        debug_print(f"  ❌ 泵送协议生成失败: {str(e)} 😭")
        raise ValueError(f"生成泵协议时出错: {str(e)}")
    
    # 5. 等待溶剂稳定
    debug_print("📍 步骤5: 等待溶剂稳定... ⏳")
    
    # 🕐 模拟运行时间优化
    debug_print("  ⏱️ 检查模拟运行时间限制...")
    original_wait_time = 10.0  # 原始等待时间
    simulation_time_limit = 5.0  # 模拟运行时间限制：5秒
    
    final_wait_time = min(original_wait_time, simulation_time_limit)
    
    if original_wait_time > simulation_time_limit:
        debug_print(f"  🎮 模拟运行优化: {original_wait_time}s → {final_wait_time}s ⚡")
        debug_print(f"  📊 时间缩短: {original_wait_time}s → {final_wait_time}s 🚀")
    else:
        debug_print(f"  ✅ 时间在限制内: {final_wait_time}s 保持不变 🎯")
    
    action_sequence.append({
        "action_name": "wait",
        "action_kwargs": {
            "time": final_wait_time,
            "description": f"等待溶剂 {solvent} 稳定" + (f" (模拟时间)" if original_wait_time != final_wait_time else "")
        }
    })
    debug_print(f"  ✅ 稳定等待已添加: {final_wait_time}s ⏰✨")
    
    # 显示时间调整信息
    if original_wait_time != final_wait_time:
        debug_print(f"  🎭 模拟优化说明: 原计划 {original_wait_time}s，实际模拟 {final_wait_time}s ⚡")
    
    # 🎊 总结
    debug_print("🔄" * 20)
    debug_print(f"🎉 重置处理协议生成完成! ✨")
    debug_print(f"📊 总动作数: {len(action_sequence)} 个")
    debug_print(f"🧪 溶剂: {solvent}")
    debug_print(f"🥽 源容器: {solvent_vessel}")
    debug_print(f"🥽 目标容器: {target_vessel}")
    debug_print(f"💧 转移体积: {volume} mL")
    debug_print(f"⏱️ 预计总时间: {(final_wait_time + 5):.0f} 秒 ⌛")
    debug_print(f"🎯 已添加 {volume} mL {solvent} 到 {target_vessel} 🚰✨")
    debug_print("🔄" * 20)
    
    return action_sequence


# 测试函数
def test_reset_handling_protocol():
    """测试重置处理协议"""
    debug_print("🧪 === RESET HANDLING PROTOCOL 测试 === ✨")
    
    # 测试溶剂名称
    debug_print("🧪 测试常用溶剂名称...")
    test_solvents = ["methanol", "ethanol", "water", "acetone", "dmso"]
    for solvent in test_solvents:
        debug_print(f"  🔍 测试溶剂: {solvent}")
    
    debug_print("✅ 测试完成 🎉")


if __name__ == "__main__":
    test_reset_handling_protocol()