from typing import List, Dict, Any
import networkx as nx
from .pump_protocol import generate_pump_protocol


def find_solvent_vessel(G: nx.DiGraph, solvent: str) -> str:
    """
    查找溶剂容器
    """
    # 按照pump_protocol的命名规则查找溶剂瓶
    solvent_vessel_id = f"flask_{solvent}"
    
    if solvent_vessel_id in G.nodes():
        return solvent_vessel_id
    
    # 如果直接匹配失败，尝试模糊匹配
    for node in G.nodes():
        if node.startswith('flask_') and solvent.lower() in node.lower():
            return node
    
    # 如果还是找不到，列出所有可用的溶剂瓶
    available_flasks = [node for node in G.nodes() 
                       if node.startswith('flask_') 
                       and G.nodes[node].get('type') == 'container']
    
    raise ValueError(f"找不到溶剂 '{solvent}' 对应的溶剂瓶。可用溶剂瓶: {available_flasks}")


def find_connected_heatchill(G: nx.DiGraph, vessel: str) -> str:
    """
    查找与指定容器相连的加热搅拌器
    """
    # 查找所有加热搅拌器节点
    heatchill_nodes = [node for node in G.nodes() 
                      if G.nodes[node].get('class') == 'virtual_heatchill']
    
    # 检查哪个加热器与目标容器相连
    for heatchill in heatchill_nodes:
        if G.has_edge(heatchill, vessel) or G.has_edge(vessel, heatchill):
            return heatchill
    
    # 如果没有直接连接，返回第一个可用的加热器
    return heatchill_nodes[0] if heatchill_nodes else None


def generate_dissolve_protocol(
    G: nx.DiGraph,
    vessel: str,
    solvent: str,
    volume: float,
    amount: str = "",
    temp: float = 25.0,
    time: float = 0.0,
    stir_speed: float = 300.0
) -> List[Dict[str, Any]]:
    """
    生成溶解操作的协议序列，复用 pump_protocol 的成熟算法
    
    溶解流程：
    1. 溶剂转移：将溶剂从溶剂瓶转移到目标容器
    2. 启动加热搅拌：设置温度和搅拌
    3. 等待溶解：监控溶解过程
    4. 停止加热搅拌：完成溶解
    
    Args:
        G: 有向图，节点为设备和容器，边为流体管道
        vessel: 目标容器（要进行溶解的容器）
        solvent: 溶剂名称（用于查找对应的溶剂瓶）
        volume: 溶剂体积 (mL)
        amount: 要溶解的物质描述
        temp: 溶解温度 (°C)，默认25°C（室温）
        time: 溶解时间 (秒)，默认0（立即完成）
        stir_speed: 搅拌速度 (RPM)，默认300 RPM
    
    Returns:
        List[Dict[str, Any]]: 溶解操作的动作序列
    
    Raises:
        ValueError: 当找不到必要的设备或容器时
    
    Examples:
        dissolve_actions = generate_dissolve_protocol(G, "reaction_mixture", "DMF", 10.0, "NaCl 2g", 60.0, 600.0, 400.0)
    """
    action_sequence = []
    
    print(f"DISSOLVE: 开始生成溶解协议")
    print(f"  - 目标容器: {vessel}")
    print(f"  - 溶剂: {solvent}")
    print(f"  - 溶剂体积: {volume} mL")
    print(f"  - 要溶解的物质: {amount}")
    print(f"  - 溶解温度: {temp}°C")
    print(f"  - 溶解时间: {time}s ({time/60:.1f}分钟)" if time > 0 else "  - 溶解时间: 立即完成")
    print(f"  - 搅拌速度: {stir_speed} RPM")
    
    # 验证目标容器存在
    if vessel not in G.nodes():
        raise ValueError(f"目标容器 '{vessel}' 不存在于系统中")
    
    # 查找溶剂瓶
    try:
        solvent_vessel = find_solvent_vessel(G, solvent)
        print(f"DISSOLVE: 找到溶剂瓶: {solvent_vessel}")
    except ValueError as e:
        raise ValueError(f"无法找到溶剂 '{solvent}': {str(e)}")
    
    # 验证是否存在从溶剂瓶到目标容器的路径
    try:
        path = nx.shortest_path(G, source=solvent_vessel, target=vessel)
        print(f"DISSOLVE: 找到路径 {solvent_vessel} -> {vessel}: {path}")
    except nx.NetworkXNoPath:
        raise ValueError(f"从溶剂瓶 '{solvent_vessel}' 到目标容器 '{vessel}' 没有可用路径")
    
    # 查找加热搅拌器
    heatchill_id = None
    if temp > 25.0 or stir_speed > 0 or time > 0:
        try:
            heatchill_id = find_connected_heatchill(G, vessel)
            if heatchill_id:
                print(f"DISSOLVE: 找到加热搅拌器: {heatchill_id}")
            else:
                print(f"DISSOLVE: 警告 - 需要加热/搅拌但未找到与容器 {vessel} 相连的加热搅拌器")
        except Exception as e:
            print(f"DISSOLVE: 加热搅拌器配置出错: {str(e)}")
    
    # === 第一步：启动加热搅拌（在添加溶剂前） ===
    if heatchill_id and (temp > 25.0 or time > 0):
        print(f"DISSOLVE: 启动加热搅拌器，温度: {temp}°C")
        
        if time > 0:
            # 如果指定了时间，使用定时加热搅拌
            heatchill_action = {
                "device_id": heatchill_id,
                "action_name": "heat_chill",
                "action_kwargs": {
                    "vessel": vessel,
                    "temp": temp,
                    "time": time,
                    "stir": True,
                    "stir_speed": stir_speed,
                    "purpose": f"溶解 {amount} 在 {solvent} 中"
                }
            }
        else:
            # 如果没有指定时间，使用持续加热搅拌
            heatchill_action = {
                "device_id": heatchill_id,
                "action_name": "heat_chill_start",
                "action_kwargs": {
                    "vessel": vessel,
                    "temp": temp,
                    "purpose": f"溶解 {amount} 在 {solvent} 中"
                }
            }
        
        action_sequence.append(heatchill_action)
        
        # 等待温度稳定
        if temp > 25.0:
            wait_time = min(60, abs(temp - 25.0) * 1.5)  # 根据温差估算预热时间
            action_sequence.append({
                "action_name": "wait",
                "action_kwargs": {"time": wait_time}
            })
    
    # === 第二步：添加溶剂到目标容器 ===
    if volume > 0:
        print(f"DISSOLVE: 将 {volume} mL {solvent} 从 {solvent_vessel} 转移到 {vessel}")
        
        # 计算流速 - 溶解时通常用较慢的速度，避免飞溅
        transfer_flowrate = 1.0  # 较慢的转移速度
        flowrate = 0.5          # 较慢的注入速度
        
        try:
            # 使用成熟的 pump_protocol 算法进行液体转移
            pump_actions = generate_pump_protocol(
                G=G,
                from_vessel=solvent_vessel,
                to_vessel=vessel,
                volume=volume,
                flowrate=flowrate,           # 注入速度 - 较慢避免飞溅
                transfer_flowrate=transfer_flowrate  # 转移速度
            )
            
            action_sequence.extend(pump_actions)
            
        except Exception as e:
            raise ValueError(f"生成泵协议时出错: {str(e)}")
        
        # 溶剂添加后等待
        action_sequence.append({
            "action_name": "wait",
            "action_kwargs": {"time": 5}
        })
    
    # === 第三步：如果没有使用定时加热搅拌，但需要等待溶解 ===
    if time > 0 and heatchill_id and temp <= 25.0:
        # 只需要搅拌等待，不需要加热
        print(f"DISSOLVE: 室温搅拌 {time}s 等待溶解")
        
        stir_action = {
            "device_id": heatchill_id,
            "action_name": "heat_chill",
            "action_kwargs": {
                "vessel": vessel,
                "temp": 25.0,  # 室温
                "time": time,
                "stir": True,
                "stir_speed": stir_speed,
                "purpose": f"室温搅拌溶解 {amount}"
            }
        }
        action_sequence.append(stir_action)
    
    # === 第四步：如果使用了持续加热，需要手动停止 ===
    if heatchill_id and time == 0 and temp > 25.0:
        print(f"DISSOLVE: 停止加热搅拌器")
        
        stop_action = {
            "device_id": heatchill_id,
            "action_name": "heat_chill_stop",
            "action_kwargs": {
                "vessel": vessel
            }
        }
        action_sequence.append(stop_action)
    
    print(f"DISSOLVE: 生成了 {len(action_sequence)} 个动作")
    print(f"DISSOLVE: 溶解协议生成完成")
    
    return action_sequence


# 便捷函数：常用溶解方案
def generate_room_temp_dissolve_protocol(
    G: nx.DiGraph,
    vessel: str,
    solvent: str,
    volume: float,
    amount: str = "",
    stir_time: float = 300.0  # 5分钟
) -> List[Dict[str, Any]]:
    """室温溶解：快速搅拌，短时间"""
    return generate_dissolve_protocol(G, vessel, solvent, volume, amount, 25.0, stir_time, 400.0)


def generate_heated_dissolve_protocol(
    G: nx.DiGraph,
    vessel: str,
    solvent: str,
    volume: float,
    amount: str = "",
    temp: float = 60.0,
    dissolve_time: float = 900.0  # 15分钟
) -> List[Dict[str, Any]]:
    """加热溶解：中等温度，较长时间"""
    return generate_dissolve_protocol(G, vessel, solvent, volume, amount, temp, dissolve_time, 300.0)


def generate_gentle_dissolve_protocol(
    G: nx.DiGraph,
    vessel: str,
    solvent: str,
    volume: float,
    amount: str = "",
    temp: float = 40.0,
    dissolve_time: float = 1800.0  # 30分钟
) -> List[Dict[str, Any]]:
    """温和溶解：低温，长时间，慢搅拌"""
    return generate_dissolve_protocol(G, vessel, solvent, volume, amount, temp, dissolve_time, 200.0)


def generate_hot_dissolve_protocol(
    G: nx.DiGraph,
    vessel: str,
    solvent: str,
    volume: float,
    amount: str = "",
    temp: float = 80.0,
    dissolve_time: float = 600.0  # 10分钟
) -> List[Dict[str, Any]]:
    """高温溶解：高温，中等时间，快搅拌"""
    return generate_dissolve_protocol(G, vessel, solvent, volume, amount, temp, dissolve_time, 500.0)


def generate_sequential_dissolve_protocol(
    G: nx.DiGraph,
    vessel: str,
    dissolve_steps: List[Dict[str, Any]]
) -> List[Dict[str, Any]]:
    """
    生成连续溶解多种物质的协议
    
    Args:
        G: 网络图
        vessel: 目标容器
        dissolve_steps: 溶解步骤列表，每个元素包含溶解参数
    
    Returns:
        List[Dict[str, Any]]: 完整的动作序列
    
    Example:
        dissolve_steps = [
            {
                "solvent": "water",
                "volume": 5.0,
                "amount": "NaCl 1g",
                "temp": 25.0,
                "time": 300.0,
                "stir_speed": 300.0
            },
            {
                "solvent": "ethanol", 
                "volume": 2.0,
                "amount": "organic compound 0.5g",
                "temp": 40.0,
                "time": 600.0,
                "stir_speed": 400.0
            }
        ]
    """
    action_sequence = []
    
    for i, step in enumerate(dissolve_steps):
        print(f"DISSOLVE: 处理第 {i+1}/{len(dissolve_steps)} 个溶解步骤")
        
        # 生成单个溶解步骤的协议
        dissolve_actions = generate_dissolve_protocol(
            G=G,
            vessel=vessel,
            solvent=step.get('solvent'),
            volume=step.get('volume', 0.0),
            amount=step.get('amount', ''),
            temp=step.get('temp', 25.0),
            time=step.get('time', 0.0),
            stir_speed=step.get('stir_speed', 300.0)
        )
        
        action_sequence.extend(dissolve_actions)
        
        # 在步骤之间加入等待时间
        if i < len(dissolve_steps) - 1:  # 不是最后一个步骤
            action_sequence.append({
                "action_name": "wait",
                "action_kwargs": {"time": 10}
            })
    
    print(f"DISSOLVE: 连续溶解协议生成完成，共 {len(action_sequence)} 个动作")
    return action_sequence


# 测试函数
def test_dissolve_protocol():
    """测试溶解协议的示例"""
    print("=== DISSOLVE PROTOCOL 测试 ===")
    print("测试完成")


if __name__ == "__main__":
    test_dissolve_protocol()