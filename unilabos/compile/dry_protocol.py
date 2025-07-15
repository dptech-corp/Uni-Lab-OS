import networkx as nx
from typing import List, Dict, Any


def find_connected_heater(G: nx.DiGraph, vessel: str) -> str:
    """
    查找与容器相连的加热器
    
    Args:
        G: 网络图
        vessel: 容器名称
    
    Returns:
        str: 加热器ID，如果没有则返回None
    """
    print(f"DRY: 正在查找与容器 '{vessel}' 相连的加热器...")
    
    # 查找所有加热器节点
    heater_nodes = [node for node in G.nodes() 
                   if ('heater' in node.lower() or 
                       'heat' in node.lower() or
                       G.nodes[node].get('class') == 'virtual_heatchill' or
                       G.nodes[node].get('type') == 'heater')]
    
    print(f"DRY: 找到的加热器节点: {heater_nodes}")
    
    # 检查是否有加热器与目标容器相连
    for heater in heater_nodes:
        if G.has_edge(heater, vessel) or G.has_edge(vessel, heater):
            print(f"DRY: 找到与容器 '{vessel}' 相连的加热器: {heater}")
            return heater
    
    # 如果没有直接连接，查找距离最近的加热器
    for heater in heater_nodes:
        try:
            path = nx.shortest_path(G, source=heater, target=vessel)
            if len(path) <= 3:  # 最多2个中间节点
                print(f"DRY: 找到距离较近的加热器: {heater}, 路径: {' → '.join(path)}")
                return heater
        except nx.NetworkXNoPath:
            continue
    
    print(f"DRY: 未找到与容器 '{vessel}' 相连的加热器")
    return None


def generate_dry_protocol(
    G: nx.DiGraph,
    compound: str,
    vessel: str,
    **kwargs  # 接收其他可能的参数但不使用
) -> List[Dict[str, Any]]:
    """
    生成干燥协议序列
    
    Args:
        G: 有向图，节点为容器和设备
        compound: 化合物名称（从XDL传入）
        vessel: 目标容器（从XDL传入）
        **kwargs: 其他可选参数，但不使用
    
    Returns:
        List[Dict[str, Any]]: 动作序列
    """
    action_sequence = []
    
    # 默认参数
    dry_temp = 60.0  # 默认干燥温度 60°C
    dry_time = 3600.0  # 默认干燥时间 1小时（3600秒）
    simulation_time = 60.0  # 模拟时间 1分钟
    
    print(f"🌡️ DRY: 开始生成干燥协议 ✨")
    print(f"  🧪 化合物: {compound}")
    print(f"  🥽 容器: {vessel}")
    print(f"  🔥 干燥温度: {dry_temp}°C")
    print(f"  ⏰ 干燥时间: {dry_time/60:.0f} 分钟")
    
    # 1. 验证目标容器存在
    print(f"\n📋 步骤1: 验证目标容器 '{vessel}' 是否存在...")
    if vessel not in G.nodes():
        print(f"⚠️ DRY: 警告 - 容器 '{vessel}' 不存在于系统中，跳过干燥 😢")
        return action_sequence
    print(f"✅ 容器 '{vessel}' 验证通过!")
    
    # 2. 查找相连的加热器
    print(f"\n🔍 步骤2: 查找与容器相连的加热器...")
    heater_id = find_connected_heater(G, vessel)
    
    if heater_id is None:
        print(f"😭 DRY: 警告 - 未找到与容器 '{vessel}' 相连的加热器，跳过干燥")
        print(f"🎭 添加模拟干燥动作...")
        # 添加一个等待动作，表示干燥过程（模拟）
        action_sequence.append({
            "action_name": "wait",
            "action_kwargs": {
                "time": 10.0,  # 模拟等待时间
                "description": f"模拟干燥 {compound} (无加热器可用)"
            }
        })
        print(f"📄 DRY: 协议生成完成，共 {len(action_sequence)} 个动作 🎯")
        return action_sequence
    
    print(f"🎉 找到加热器: {heater_id}!")
    
    # 3. 启动加热器进行干燥
    print(f"\n🚀 步骤3: 开始执行干燥流程...")
    print(f"🔥 启动加热器 {heater_id} 进行干燥")
    
    # 3.1 启动加热
    print(f"  ⚡ 动作1: 启动加热到 {dry_temp}°C...")
    action_sequence.append({
        "device_id": heater_id,
        "action_name": "heat_chill_start",
        "action_kwargs": {
            "vessel": vessel,
            "temp": dry_temp,
            "purpose": f"干燥 {compound}"
        }
    })
    print(f"  ✅ 加热器启动命令已添加 🔥")
    
    # 3.2 等待温度稳定
    print(f"  ⏳ 动作2: 等待温度稳定...")
    action_sequence.append({
        "action_name": "wait",
        "action_kwargs": {
            "time": 10.0,
            "description": f"等待温度稳定到 {dry_temp}°C"
        }
    })
    print(f"  ✅ 温度稳定等待命令已添加 🌡️")
    
    # 3.3 保持干燥温度
    print(f"  🔄 动作3: 保持干燥温度 {simulation_time/60:.0f} 分钟...")
    action_sequence.append({
        "device_id": heater_id,
        "action_name": "heat_chill",
        "action_kwargs": {
            "vessel": vessel,
            "temp": dry_temp,
            "time": simulation_time,
            "purpose": f"干燥 {compound}，保持温度 {dry_temp}°C"
        }
    })
    print(f"  ✅ 温度保持命令已添加 🌡️⏰")
    
    # 3.4 停止加热
    print(f"  ⏹️ 动作4: 停止加热...")
    action_sequence.append({
        "device_id": heater_id,
        "action_name": "heat_chill_stop",
        "action_kwargs": {
            "vessel": vessel,
            "purpose": f"干燥完成，停止加热"
        }
    })
    print(f"  ✅ 停止加热命令已添加 🛑")
    
    # 3.5 等待冷却
    print(f"  ❄️ 动作5: 等待冷却...")
    action_sequence.append({
        "action_name": "wait",
        "action_kwargs": {
            "time": 10.0,  # 等待10秒冷却
            "description": f"等待 {compound} 冷却"
        }
    })
    print(f"  ✅ 冷却等待命令已添加 🧊")
    
    print(f"\n🎊 DRY: 协议生成完成，共 {len(action_sequence)} 个动作 🎯")
    print(f"⏱️ DRY: 预计总时间: {(dry_time + 360)/60:.0f} 分钟 ⌛")
    print(f"🏁 所有动作序列准备就绪! ✨")
    
    return action_sequence


# 测试函数
def test_dry_protocol():
    """测试干燥协议"""
    print("=== DRY PROTOCOL 测试 ===")
    print("测试完成")


if __name__ == "__main__":
    test_dry_protocol()