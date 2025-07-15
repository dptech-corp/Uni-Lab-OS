import networkx as nx
from typing import List, Dict, Any, Optional


def parse_temperature(temp_str: str) -> float:
    """
    解析温度字符串，支持多种格式
    
    Args:
        temp_str: 温度字符串（如 "45 °C", "45°C", "45"）
    
    Returns:
        float: 温度值（摄氏度）
    """
    try:
        # 移除常见的温度单位和符号
        temp_clean = temp_str.replace("°C", "").replace("°", "").replace("C", "").strip()
        return float(temp_clean)
    except ValueError:
        print(f"HYDROGENATE: 无法解析温度 '{temp_str}'，使用默认温度 25°C")
        return 25.0


def parse_time(time_str: str) -> float:
    """
    解析时间字符串，支持多种格式
    
    Args:
        time_str: 时间字符串（如 "2 h", "120 min", "7200 s"）
    
    Returns:
        float: 时间值（秒）
    """
    try:
        time_clean = time_str.lower().strip()
        
        # 处理小时
        if "h" in time_clean:
            hours = float(time_clean.replace("h", "").strip())
            return hours * 3600.0
        
        # 处理分钟
        if "min" in time_clean:
            minutes = float(time_clean.replace("min", "").strip())
            return minutes * 60.0
        
        # 处理秒
        if "s" in time_clean:
            seconds = float(time_clean.replace("s", "").strip())
            return seconds
        
        # 默认按小时处理
        return float(time_clean) * 3600.0
    
    except ValueError:
        print(f"HYDROGENATE: 无法解析时间 '{time_str}'，使用默认时间 2小时")
        return 7200.0  # 2小时


def find_associated_solenoid_valve(G: nx.DiGraph, device_id: str) -> Optional[str]:
    """查找与指定设备相关联的电磁阀"""
    solenoid_valves = [
        node for node in G.nodes() 
        if ('solenoid' in (G.nodes[node].get('class') or '').lower()
            or 'solenoid_valve' in node)
    ]
    
    # 通过网络连接查找直接相连的电磁阀
    for solenoid in solenoid_valves:
        if G.has_edge(device_id, solenoid) or G.has_edge(solenoid, device_id):
            return solenoid
    
    # 通过命名规则查找关联的电磁阀
    device_type = ""
    if 'gas' in device_id.lower():
        device_type = "gas"
    elif 'h2' in device_id.lower() or 'hydrogen' in device_id.lower():
        device_type = "gas"
    
    if device_type:
        for solenoid in solenoid_valves:
            if device_type in solenoid.lower():
                return solenoid
    
    return None


def find_connected_device(G: nx.DiGraph, vessel: str, device_type: str) -> str:
    """
    查找与容器相连的指定类型设备
    
    Args:
        G: 网络图
        vessel: 容器名称
        device_type: 设备类型 ('heater', 'stirrer', 'gas_source')
    
    Returns:
        str: 设备ID，如果没有则返回None
    """
    print(f"HYDROGENATE: 正在查找与容器 '{vessel}' 相连的 {device_type}...")
    
    # 根据设备类型定义搜索关键词
    if device_type == 'heater':
        keywords = ['heater', 'heat', 'heatchill']
        device_class = 'virtual_heatchill'
    elif device_type == 'stirrer':
        keywords = ['stirrer', 'stir']
        device_class = 'virtual_stirrer'
    elif device_type == 'gas_source':
        keywords = ['gas', 'h2', 'hydrogen']
        device_class = 'virtual_gas_source'
    else:
        return None
    
    # 查找设备节点
    device_nodes = []
    for node in G.nodes():
        node_data = G.nodes[node]
        node_name = node.lower()
        node_class = node_data.get('class', '').lower()
        
        # 通过名称匹配
        if any(keyword in node_name for keyword in keywords):
            device_nodes.append(node)
        # 通过类型匹配
        elif device_class in node_class:
            device_nodes.append(node)
    
    print(f"HYDROGENATE: 找到的{device_type}节点: {device_nodes}")
    
    # 检查是否有设备与目标容器相连
    for device in device_nodes:
        if G.has_edge(device, vessel) or G.has_edge(vessel, device):
            print(f"HYDROGENATE: 找到与容器 '{vessel}' 相连的{device_type}: {device}")
            return device
    
    # 如果没有直接连接，查找距离最近的设备
    for device in device_nodes:
        try:
            path = nx.shortest_path(G, source=device, target=vessel)
            if len(path) <= 3:  # 最多2个中间节点
                print(f"HYDROGENATE: 找到距离较近的{device_type}: {device}")
                return device
        except nx.NetworkXNoPath:
            continue
    
    print(f"HYDROGENATE: 未找到与容器 '{vessel}' 相连的{device_type}")
    return None


def generate_hydrogenate_protocol(
    G: nx.DiGraph,
    temp: str,
    time: str,
    vessel: str,
    **kwargs  # 接收其他可能的参数但不使用
) -> List[Dict[str, Any]]:
    """
    生成氢化反应协议序列
    
    Args:
        G: 有向图，节点为容器和设备
        temp: 反应温度（如 "45 °C"）
        time: 反应时间（如 "2 h"）
        vessel: 反应容器
        **kwargs: 其他可选参数，但不使用
    
    Returns:
        List[Dict[str, Any]]: 动作序列
    """
    action_sequence = []
    
    # 解析参数
    temperature = parse_temperature(temp)
    reaction_time = parse_time(time)
    
    print(f"HYDROGENATE: 开始生成氢化反应协议")
    print(f"  - 反应温度: {temperature}°C")
    print(f"  - 反应时间: {reaction_time/3600:.1f} 小时")
    print(f"  - 反应容器: {vessel}")
    
    # 1. 验证目标容器存在
    if vessel not in G.nodes():
        print(f"HYDROGENATE: 警告 - 容器 '{vessel}' 不存在于系统中，跳过氢化反应")
        return action_sequence
    
    # 2. 查找相连的设备
    heater_id = find_connected_device(G, vessel, 'heater')
    stirrer_id = find_connected_device(G, vessel, 'stirrer')
    gas_source_id = find_connected_device(G, vessel, 'gas_source')
    
    # 3. 启动搅拌器
    if stirrer_id:
        print(f"HYDROGENATE: 启动搅拌器 {stirrer_id}")
        action_sequence.append({
            "device_id": stirrer_id,
            "action_name": "start_stir",
            "action_kwargs": {
                "vessel": vessel,
                "stir_speed": 300.0,
                "purpose": "氢化反应: 开始搅拌"
            }
        })
    else:
        print(f"HYDROGENATE: 警告 - 未找到搅拌器，继续执行")
    
    # 4. 启动气源（氢气）- 修复版本
    if gas_source_id:
        print(f"HYDROGENATE: 启动气源 {gas_source_id} (氢气)")
        action_sequence.append({
            "device_id": gas_source_id,
            "action_name": "set_status",  # 修改为 set_status
            "action_kwargs": {
                "string": "ON"  # 修改参数格式
            }
        })
        
        # 查找相关的电磁阀
        gas_solenoid = find_associated_solenoid_valve(G, gas_source_id)
        if gas_solenoid:
            print(f"HYDROGENATE: 开启气源电磁阀 {gas_solenoid}")
            action_sequence.append({
                "device_id": gas_solenoid,
                "action_name": "set_valve_position",
                "action_kwargs": {
                    "command": "OPEN"
                }
            })
    else:
        print(f"HYDROGENATE: 警告 - 未找到气源，继续执行")
    
    # 5. 等待气体稳定
    action_sequence.append({
        "action_name": "wait",
        "action_kwargs": {
            "time": 30.0,
            "description": "等待氢气环境稳定"
        }
    })
    
    # 6. 启动加热器
    if heater_id:
        print(f"HYDROGENATE: 启动加热器 {heater_id} 到 {temperature}°C")
        action_sequence.append({
            "device_id": heater_id,
            "action_name": "heat_chill_start",
            "action_kwargs": {
                "vessel": vessel,
                "temp": temperature,
                "purpose": f"氢化反应: 加热到 {temperature}°C"
            }
        })
        
        # 等待温度稳定
        action_sequence.append({
            "action_name": "wait",
            "action_kwargs": {
                "time": 20.0,
                "description": f"等待温度稳定到 {temperature}°C"
            }
        })
        
        # 🕐 模拟运行时间优化
        print("HYDROGENATE: 检查模拟运行时间限制...")
        original_reaction_time = reaction_time
        simulation_time_limit = 60.0  # 模拟运行时间限制：60秒
        
        if reaction_time > simulation_time_limit:
            reaction_time = simulation_time_limit
            print(f"HYDROGENATE: 模拟运行优化: {original_reaction_time}s → {reaction_time}s (限制为{simulation_time_limit}s)")
            print(f"HYDROGENATE: 时间缩短: {original_reaction_time/3600:.2f}小时 → {reaction_time/60:.1f}分钟")
        else:
            print(f"HYDROGENATE: 时间在限制内: {reaction_time}s ({reaction_time/60:.1f}分钟) 保持不变")
        
        # 保持反应温度
        action_sequence.append({
            "device_id": heater_id,
            "action_name": "heat_chill",
            "action_kwargs": {
                "vessel": vessel,
                "temp": temperature,
                "time": reaction_time,
                "purpose": f"氢化反应: 保持 {temperature}°C，反应 {reaction_time/60:.1f}分钟" + (f" (模拟时间)" if original_reaction_time != reaction_time else "")
            }
        })
        
        # 显示时间调整信息
        if original_reaction_time != reaction_time:
            print(f"HYDROGENATE: 模拟优化说明: 原计划 {original_reaction_time/3600:.2f}小时，实际模拟 {reaction_time/60:.1f}分钟")
            
    else:
        print(f"HYDROGENATE: 警告 - 未找到加热器，使用室温反应")
        
        # 🕐 室温反应也需要时间优化
        print("HYDROGENATE: 检查室温反应模拟时间限制...")
        original_reaction_time = reaction_time
        simulation_time_limit = 60.0  # 模拟运行时间限制：60秒
        
        if reaction_time > simulation_time_limit:
            reaction_time = simulation_time_limit
            print(f"HYDROGENATE: 室温反应时间优化: {original_reaction_time}s → {reaction_time}s")
            print(f"HYDROGENATE: 时间缩短: {original_reaction_time/3600:.2f}小时 → {reaction_time/60:.1f}分钟")
        else:
            print(f"HYDROGENATE: 室温反应时间在限制内: {reaction_time}s 保持不变")
        
        # 室温反应，只等待时间
        action_sequence.append({
            "action_name": "wait",
            "action_kwargs": {
                "time": reaction_time,
                "description": f"室温氢化反应 {reaction_time/60:.1f}分钟" + (f" (模拟时间)" if original_reaction_time != reaction_time else "")
            }
        })
        
        # 显示时间调整信息
        if original_reaction_time != reaction_time:
            print(f"HYDROGENATE: 室温反应优化说明: 原计划 {original_reaction_time/3600:.2f}小时，实际模拟 {reaction_time/60:.1f}分钟")
    
    # 7. 停止加热
    if heater_id:
        action_sequence.append({
            "device_id": heater_id,
            "action_name": "heat_chill_stop",
            "action_kwargs": {
                "vessel": vessel,
                "purpose": "氢化反应完成，停止加热"
            }
        })
    
    # 8. 等待冷却
    action_sequence.append({
        "action_name": "wait",
        "action_kwargs": {
            "time": 300.0,
            "description": "等待反应混合物冷却"
        }
    })
    
    # 9. 停止气源 - 修复版本
    if gas_source_id:
        # 先关闭电磁阀
        gas_solenoid = find_associated_solenoid_valve(G, gas_source_id)
        if gas_solenoid:
            print(f"HYDROGENATE: 关闭气源电磁阀 {gas_solenoid}")
            action_sequence.append({
                "device_id": gas_solenoid,
                "action_name": "set_valve_position",
                "action_kwargs": {
                    "command": "CLOSED"
                }
            })
        
        # 再关闭气源
        action_sequence.append({
            "device_id": gas_source_id,
            "action_name": "set_status",  # 修改为 set_status
            "action_kwargs": {
                "string": "OFF"  # 修改参数格式
            }
        })
    
    # 10. 停止搅拌
    if stirrer_id:
        action_sequence.append({
            "device_id": stirrer_id,
            "action_name": "stop_stir",
            "action_kwargs": {
                "vessel": vessel,
                "purpose": "氢化反应完成，停止搅拌"
            }
        })
    
    print(f"HYDROGENATE: 协议生成完成，共 {len(action_sequence)} 个动作")
    print(f"HYDROGENATE: 预计总时间: {(reaction_time + 450)/3600:.1f} 小时")
    
    return action_sequence


# 测试函数
def test_hydrogenate_protocol():
    """测试氢化反应协议"""
    print("=== HYDROGENATE PROTOCOL 测试 ===")
    
    # 测试温度解析
    test_temps = ["45 °C", "45°C", "45", "25 C", "invalid"]
    for temp in test_temps:
        parsed = parse_temperature(temp)
        print(f"温度 '{temp}' -> {parsed}°C")
    
    # 测试时间解析
    test_times = ["2 h", "120 min", "7200 s", "2", "invalid"]
    for time in test_times:
        parsed = parse_time(time)
        print(f"时间 '{time}' -> {parsed/3600:.1f} 小时")
    
    print("测试完成")


if __name__ == "__main__":
    test_hydrogenate_protocol()