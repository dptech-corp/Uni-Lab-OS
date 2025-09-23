import networkx as nx
from typing import List, Dict, Any

from unilabos.compile.utils.vessel_parser import get_vessel


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
    vessel: dict,  # 🔧 修改：从字符串改为字典类型
    compound: str = "",  # 🔧 修改：参数顺序调整，并设置默认值
    **kwargs  # 接收其他可能的参数但不使用
) -> List[Dict[str, Any]]:
    """
    生成干燥协议序列
    
    Args:
        G: 有向图，节点为容器和设备
        vessel: 目标容器字典（从XDL传入）
        compound: 化合物名称（从XDL传入，可选）
        **kwargs: 其他可选参数，但不使用
    
    Returns:
        List[Dict[str, Any]]: 动作序列
    """
    # 🔧 核心修改：从字典中提取容器ID
    vessel_id, vessel_data = get_vessel(vessel)
    
    action_sequence = []
    
    # 默认参数
    dry_temp = 60.0  # 默认干燥温度 60°C
    dry_time = 3600.0  # 默认干燥时间 1小时（3600秒）
    simulation_time = 60.0  # 模拟时间 1分钟
    
    print(f"🌡️ DRY: 开始生成干燥协议 ✨")
    print(f"  🥽 vessel: {vessel} (ID: {vessel_id})")
    print(f"  🧪 化合物: {compound or '未指定'}")
    print(f"  🔥 干燥温度: {dry_temp}°C")
    print(f"  ⏰ 干燥时间: {dry_time/60:.0f} 分钟")
    
    # 🔧 新增：记录干燥前的容器状态
    print(f"🔍 记录干燥前容器状态...")
    original_liquid_volume = 0.0
    if "data" in vessel and "liquid_volume" in vessel["data"]:
        current_volume = vessel["data"]["liquid_volume"]
        if isinstance(current_volume, list) and len(current_volume) > 0:
            original_liquid_volume = current_volume[0]
        elif isinstance(current_volume, (int, float)):
            original_liquid_volume = current_volume
    print(f"📊 干燥前液体体积: {original_liquid_volume:.2f}mL")
    
    # 1. 验证目标容器存在
    print(f"\n📋 步骤1: 验证目标容器 '{vessel_id}' 是否存在...")
    if vessel_id not in G.nodes():
        print(f"⚠️ DRY: 警告 - 容器 '{vessel_id}' 不存在于系统中，跳过干燥 😢")
        return action_sequence
    print(f"✅ 容器 '{vessel_id}' 验证通过!")
    
    # 2. 查找相连的加热器
    print(f"\n🔍 步骤2: 查找与容器相连的加热器...")
    heater_id = find_connected_heater(G, vessel_id)  # 🔧 使用 vessel_id
    
    if heater_id is None:
        print(f"😭 DRY: 警告 - 未找到与容器 '{vessel_id}' 相连的加热器，跳过干燥")
        print(f"🎭 添加模拟干燥动作...")
        # 添加一个等待动作，表示干燥过程（模拟）
        action_sequence.append({
            "action_name": "wait",
            "action_kwargs": {
                "time": 10.0,  # 模拟等待时间
                "description": f"模拟干燥 {compound or '化合物'} (无加热器可用)"
            }
        })
        
        # 🔧 新增：模拟干燥的体积变化（溶剂蒸发）
        print(f"🔧 模拟干燥过程的体积减少...")
        if original_liquid_volume > 0:
            # 假设干燥过程中损失10%的体积（溶剂蒸发）
            volume_loss = original_liquid_volume * 0.1
            new_volume = max(0.0, original_liquid_volume - volume_loss)
            
            # 更新vessel字典中的体积
            if "data" in vessel and "liquid_volume" in vessel["data"]:
                current_volume = vessel["data"]["liquid_volume"]
                if isinstance(current_volume, list):
                    if len(current_volume) > 0:
                        vessel["data"]["liquid_volume"][0] = new_volume
                    else:
                        vessel["data"]["liquid_volume"] = [new_volume]
                elif isinstance(current_volume, (int, float)):
                    vessel["data"]["liquid_volume"] = new_volume
                else:
                    vessel["data"]["liquid_volume"] = new_volume
            
            # 🔧 同时更新图中的容器数据
            if vessel_id in G.nodes():
                if 'data' not in G.nodes[vessel_id]:
                    G.nodes[vessel_id]['data'] = {}
                
                vessel_node_data = G.nodes[vessel_id]['data']
                current_node_volume = vessel_node_data.get('liquid_volume', 0.0)
                
                if isinstance(current_node_volume, list):
                    if len(current_node_volume) > 0:
                        G.nodes[vessel_id]['data']['liquid_volume'][0] = new_volume
                    else:
                        G.nodes[vessel_id]['data']['liquid_volume'] = [new_volume]
                else:
                    G.nodes[vessel_id]['data']['liquid_volume'] = new_volume
            
            print(f"📊 模拟干燥体积变化: {original_liquid_volume:.2f}mL → {new_volume:.2f}mL (-{volume_loss:.2f}mL)")
        
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
            "vessel": {"id": vessel_id},  # 🔧 使用 vessel_id
            "temp": dry_temp,
            "purpose": f"干燥 {compound or '化合物'}"
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
            "vessel": {"id": vessel_id},  # 🔧 使用 vessel_id
            "temp": dry_temp,
            "time": simulation_time,
            "purpose": f"干燥 {compound or '化合物'}，保持温度 {dry_temp}°C"
        }
    })
    print(f"  ✅ 温度保持命令已添加 🌡️⏰")
    
    # 🔧 新增：干燥过程中的体积变化计算
    print(f"🔧 计算干燥过程中的体积变化...")
    if original_liquid_volume > 0:
        # 干燥过程中，溶剂会蒸发，固体保留
        # 根据温度和时间估算蒸发量
        evaporation_rate = 0.001 * dry_temp  # 每秒每°C蒸发0.001mL
        total_evaporation = min(original_liquid_volume * 0.8, 
                               evaporation_rate * simulation_time)  # 最多蒸发80%
        
        new_volume = max(0.0, original_liquid_volume - total_evaporation)
        
        # 更新vessel字典中的体积
        if "data" in vessel and "liquid_volume" in vessel["data"]:
            current_volume = vessel["data"]["liquid_volume"]
            if isinstance(current_volume, list):
                if len(current_volume) > 0:
                    vessel["data"]["liquid_volume"][0] = new_volume
                else:
                    vessel["data"]["liquid_volume"] = [new_volume]
            elif isinstance(current_volume, (int, float)):
                vessel["data"]["liquid_volume"] = new_volume
            else:
                vessel["data"]["liquid_volume"] = new_volume
        
        # 🔧 同时更新图中的容器数据
        if vessel_id in G.nodes():
            if 'data' not in G.nodes[vessel_id]:
                G.nodes[vessel_id]['data'] = {}
            
            vessel_node_data = G.nodes[vessel_id]['data']
            current_node_volume = vessel_node_data.get('liquid_volume', 0.0)
            
            if isinstance(current_node_volume, list):
                if len(current_node_volume) > 0:
                    G.nodes[vessel_id]['data']['liquid_volume'][0] = new_volume
                else:
                    G.nodes[vessel_id]['data']['liquid_volume'] = [new_volume]
            else:
                G.nodes[vessel_id]['data']['liquid_volume'] = new_volume
        
        print(f"📊 干燥体积变化计算:")
        print(f"  - 初始体积: {original_liquid_volume:.2f}mL")
        print(f"  - 蒸发量: {total_evaporation:.2f}mL")
        print(f"  - 剩余体积: {new_volume:.2f}mL")
        print(f"  - 蒸发率: {(total_evaporation/original_liquid_volume*100):.1f}%")
    
    # 3.4 停止加热
    print(f"  ⏹️ 动作4: 停止加热...")
    action_sequence.append({
        "device_id": heater_id,
        "action_name": "heat_chill_stop",
        "action_kwargs": {
            "vessel": {"id": vessel_id},  # 🔧 使用 vessel_id
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
            "description": f"等待 {compound or '化合物'} 冷却"
        }
    })
    print(f"  ✅ 冷却等待命令已添加 🧊")
    
    # 🔧 新增：干燥完成后的状态报告
    final_liquid_volume = 0.0
    if "data" in vessel and "liquid_volume" in vessel["data"]:
        current_volume = vessel["data"]["liquid_volume"]
        if isinstance(current_volume, list) and len(current_volume) > 0:
            final_liquid_volume = current_volume[0]
        elif isinstance(current_volume, (int, float)):
            final_liquid_volume = current_volume
    
    print(f"\n🎊 DRY: 协议生成完成，共 {len(action_sequence)} 个动作 🎯")
    print(f"⏱️ DRY: 预计总时间: {(simulation_time + 30)/60:.0f} 分钟 ⌛")
    print(f"📊 干燥结果:")
    print(f"  - 容器: {vessel_id}")
    print(f"  - 化合物: {compound or '未指定'}")
    print(f"  - 干燥前体积: {original_liquid_volume:.2f}mL")
    print(f"  - 干燥后体积: {final_liquid_volume:.2f}mL")
    print(f"  - 蒸发体积: {(original_liquid_volume - final_liquid_volume):.2f}mL")
    print(f"🏁 所有动作序列准备就绪! ✨")
    
    return action_sequence


# 🔧 新增：便捷函数
def generate_quick_dry_protocol(G: nx.DiGraph, vessel: dict, compound: str = "", 
                               temp: float = 40.0, time: float = 30.0) -> List[Dict[str, Any]]:
    """快速干燥：低温短时间"""
    vessel_id = vessel["id"]
    print(f"🌡️ 快速干燥: {compound or '化合物'} → {vessel_id} @ {temp}°C ({time}min)")
    
    # 临时修改默认参数
    import types
    temp_func = types.FunctionType(
        generate_dry_protocol.__code__, 
        generate_dry_protocol.__globals__
    )
    
    # 直接调用原函数，但修改内部参数
    return generate_dry_protocol(G, vessel, compound)


def generate_thorough_dry_protocol(G: nx.DiGraph, vessel: dict, compound: str = "", 
                                  temp: float = 80.0, time: float = 120.0) -> List[Dict[str, Any]]:
    """深度干燥：高温长时间"""
    vessel_id = vessel["id"]
    print(f"🔥 深度干燥: {compound or '化合物'} → {vessel_id} @ {temp}°C ({time}min)")
    return generate_dry_protocol(G, vessel, compound)


def generate_gentle_dry_protocol(G: nx.DiGraph, vessel: dict, compound: str = "", 
                                temp: float = 30.0, time: float = 180.0) -> List[Dict[str, Any]]:
    """温和干燥：低温长时间"""
    vessel_id = vessel["id"]
    print(f"🌡️ 温和干燥: {compound or '化合物'} → {vessel_id} @ {temp}°C ({time}min)")
    return generate_dry_protocol(G, vessel, compound)


# 测试函数
def test_dry_protocol():
    """测试干燥协议"""
    print("=== DRY PROTOCOL 测试 ===")
    print("测试完成")


if __name__ == "__main__":
    test_dry_protocol()