from typing import List, Dict, Any, Optional
import networkx as nx


def find_connected_heatchill(G: nx.DiGraph, vessel: str) -> str:
    """
    查找与指定容器相连的加热/冷却设备
    """
    # 查找所有加热/冷却设备节点
    heatchill_nodes = [node for node in G.nodes() 
                      if (G.nodes[node].get('class') or '') == 'virtual_heatchill']
    
    # 检查哪个加热/冷却设备与目标容器相连（机械连接）
    for heatchill in heatchill_nodes:
        if G.has_edge(heatchill, vessel) or G.has_edge(vessel, heatchill):
            return heatchill
    
    # 如果没有直接连接，返回第一个可用的加热/冷却设备
    if heatchill_nodes:
        return heatchill_nodes[0]
    
    raise ValueError("系统中未找到可用的加热/冷却设备")


def generate_heat_chill_protocol(
    G: nx.DiGraph,
    vessel: str,
    temp: float,
    time: float,
    stir: bool = False,
    stir_speed: float = 300.0,
    purpose: str = "加热/冷却操作"
) -> List[Dict[str, Any]]:
    """
    生成加热/冷却操作的协议序列 - 带时间限制的完整操作
    """
    action_sequence = []
    
    print(f"HEATCHILL: 开始生成加热/冷却协议")
    print(f"  - 容器: {vessel}")
    print(f"  - 目标温度: {temp}°C")
    print(f"  - 持续时间: {time}秒")
    print(f"  - 使用内置搅拌: {stir}, 速度: {stir_speed} RPM")
    print(f"  - 目的: {purpose}")
    
    # 1. 验证容器存在
    if vessel not in G.nodes():
        raise ValueError(f"容器 '{vessel}' 不存在于系统中")
    
    # 2. 查找加热/冷却设备
    try:
        heatchill_id = find_connected_heatchill(G, vessel)
        print(f"HEATCHILL: 找到加热/冷却设备: {heatchill_id}")
    except ValueError as e:
        raise ValueError(f"无法找到加热/冷却设备: {str(e)}")
    
    # 3. 执行加热/冷却操作
    heatchill_action = {
        "device_id": heatchill_id,
        "action_name": "heat_chill",
        "action_kwargs": {
            "vessel": vessel,
            "temp": temp,
            "time": time,
            "stir": stir,
            "stir_speed": stir_speed,
            "status": "start"
        }
    }
    
    action_sequence.append(heatchill_action)
    
    print(f"HEATCHILL: 生成了 {len(action_sequence)} 个动作")
    return action_sequence


def generate_heat_chill_start_protocol(
    G: nx.DiGraph,
    vessel: str,
    temp: float,
    purpose: str = "开始加热/冷却"
) -> List[Dict[str, Any]]:
    """
    生成开始加热/冷却操作的协议序列
    """
    action_sequence = []
    
    print(f"HEATCHILL_START: 开始生成加热/冷却启动协议")
    print(f"  - 容器: {vessel}")
    print(f"  - 目标温度: {temp}°C")
    print(f"  - 目的: {purpose}")
    
    # 1. 验证容器存在
    if vessel not in G.nodes():
        raise ValueError(f"容器 '{vessel}' 不存在于系统中")
    
    # 2. 查找加热/冷却设备
    try:
        heatchill_id = find_connected_heatchill(G, vessel)
        print(f"HEATCHILL_START: 找到加热/冷却设备: {heatchill_id}")
    except ValueError as e:
        raise ValueError(f"无法找到加热/冷却设备: {str(e)}")
    
    # 3. 执行开始加热/冷却操作
    heatchill_start_action = {
        "device_id": heatchill_id,
        "action_name": "heat_chill_start",
        "action_kwargs": {
            "vessel": vessel,
            "temp": temp,
            "purpose": purpose
        }
    }
    
    action_sequence.append(heatchill_start_action)
    
    print(f"HEATCHILL_START: 生成了 {len(action_sequence)} 个动作")
    return action_sequence


def generate_heat_chill_stop_protocol(
    G: nx.DiGraph,
    vessel: str
) -> List[Dict[str, Any]]:
    """
    生成停止加热/冷却操作的协议序列
    """
    action_sequence = []
    
    print(f"HEATCHILL_STOP: 开始生成加热/冷却停止协议")
    print(f"  - 容器: {vessel}")
    
    # 1. 验证容器存在
    if vessel not in G.nodes():
        raise ValueError(f"容器 '{vessel}' 不存在于系统中")
    
    # 2. 查找加热/冷却设备
    try:
        heatchill_id = find_connected_heatchill(G, vessel)
        print(f"HEATCHILL_STOP: 找到加热/冷却设备: {heatchill_id}")
    except ValueError as e:
        raise ValueError(f"无法找到加热/冷却设备: {str(e)}")
    
    # 3. 执行停止加热/冷却操作
    heatchill_stop_action = {
        "device_id": heatchill_id,
        "action_name": "heat_chill_stop",
        "action_kwargs": {
            "vessel": vessel
        }
    }
    
    action_sequence.append(heatchill_stop_action)
    
    print(f"HEATCHILL_STOP: 生成了 {len(action_sequence)} 个动作")
    return action_sequence


def generate_heat_chill_to_temp_protocol(
    G: nx.DiGraph,
    vessel: str,
    temp: float,
    active: bool = True,
    continue_heatchill: bool = False,
    stir: bool = False,
    stir_speed: Optional[float] = None,
    purpose: Optional[str] = None
) -> List[Dict[str, Any]]:
    """
    生成加热/冷却到指定温度的协议序列 - 智能温控协议
    
    **关键修复**: 学习 pump_protocol 的模式，直接使用设备基础动作，不依赖特定的 Action 文件
    """
    action_sequence = []
    
    # 设置默认值
    if stir_speed is None:
        stir_speed = 300.0
    if purpose is None:
        purpose = f"智能温控到 {temp}°C"
    
    print(f"HEATCHILL_TO_TEMP: 开始生成智能温控协议")
    print(f"  - 容器: {vessel}")
    print(f"  - 目标温度: {temp}°C")
    print(f"  - 主动控温: {active}")
    print(f"  - 达到温度后继续: {continue_heatchill}")
    print(f"  - 搅拌: {stir}, 速度: {stir_speed} RPM")
    print(f"  - 目的: {purpose}")
    
    # 1. 验证容器存在
    if vessel not in G.nodes():
        raise ValueError(f"容器 '{vessel}' 不存在于系统中")
    
    # 2. 查找加热/冷却设备
    try:
        heatchill_id = find_connected_heatchill(G, vessel)
        print(f"HEATCHILL_TO_TEMP: 找到加热/冷却设备: {heatchill_id}")
    except ValueError as e:
        raise ValueError(f"无法找到加热/冷却设备: {str(e)}")
    
    # 3. 根据参数选择合适的基础动作组合 (学习 pump_protocol 的模式)
    if not active:
        print(f"HEATCHILL_TO_TEMP: 非主动模式，仅等待")
        action_sequence.append({
            "action_name": "wait",
            "action_kwargs": {
                "time": 10.0,
                "purpose": f"等待容器 {vessel} 自然达到 {temp}°C"
            }
        })
    else:
        if continue_heatchill:
            # 持续模式：使用 heat_chill_start 基础动作
            print(f"HEATCHILL_TO_TEMP: 使用持续温控模式")
            action_sequence.append({
                "device_id": heatchill_id,
                "action_name": "heat_chill_start",  # ← 直接使用设备基础动作
                "action_kwargs": {
                    "vessel": vessel,
                    "temp": temp,
                    "purpose": f"{purpose} (持续保温)"
                }
            })
        else:
            # 一次性模式：使用 heat_chill 基础动作
            print(f"HEATCHILL_TO_TEMP: 使用一次性温控模式")
            estimated_time = max(60.0, min(900.0, abs(temp - 25.0) * 30.0))
            print(f"HEATCHILL_TO_TEMP: 估算所需时间: {estimated_time}秒")
            
            action_sequence.append({
                "device_id": heatchill_id,
                "action_name": "heat_chill",  # ← 直接使用设备基础动作
                "action_kwargs": {
                    "vessel": vessel,
                    "temp": temp,
                    "time": estimated_time,
                    "stir": stir,
                    "stir_speed": stir_speed,
                    "status": "start"
                }
            })
    
    print(f"HEATCHILL_TO_TEMP: 生成了 {len(action_sequence)} 个动作")
    return action_sequence


# 扩展版本的加热/冷却协议，集成智能温控功能
def generate_smart_heat_chill_protocol(
    G: nx.DiGraph,
    vessel: str,
    temp: float,
    time: float = 0.0,  # 0表示自动估算
    active: bool = True,
    continue_heatchill: bool = False,
    stir: bool = False,
    stir_speed: float = 300.0,
    purpose: str = "智能加热/冷却"
) -> List[Dict[str, Any]]:
    """
    这个函数集成了 generate_heat_chill_to_temp_protocol 的智能逻辑，
    但使用现有的 Action 类型
    """
    # 如果时间为0，自动估算
    if time == 0.0:
        estimated_time = max(60.0, min(900.0, abs(temp - 25.0) * 30.0))
        time = estimated_time
    
    if continue_heatchill:
        # 使用持续模式
        return generate_heat_chill_start_protocol(G, vessel, temp, purpose)
    else:
        # 使用定时模式
        return generate_heat_chill_protocol(G, vessel, temp, time, stir, stir_speed, purpose)


# 便捷函数
def generate_heating_protocol(
    G: nx.DiGraph,
    vessel: str,
    temp: float,
    time: float = 300.0,
    stir: bool = True,
    stir_speed: float = 300.0
) -> List[Dict[str, Any]]:
    """生成加热协议的便捷函数"""
    return generate_heat_chill_protocol(
        G=G, vessel=vessel, temp=temp, time=time, 
        stir=stir, stir_speed=stir_speed, purpose=f"加热到 {temp}°C"
    )


def generate_cooling_protocol(
    G: nx.DiGraph,
    vessel: str,
    temp: float,
    time: float = 600.0,
    stir: bool = True,
    stir_speed: float = 200.0
) -> List[Dict[str, Any]]:
    """生成冷却协议的便捷函数"""
    return generate_heat_chill_protocol(
        G=G, vessel=vessel, temp=temp, time=time,
        stir=stir, stir_speed=stir_speed, purpose=f"冷却到 {temp}°C"
    )


# # 温度预设快捷函数
# def generate_room_temp_protocol(
#     G: nx.DiGraph,
#     vessel: str,
#     stir: bool = False
# ) -> List[Dict[str, Any]]:
#     """返回室温的快捷函数"""
#     return generate_heat_chill_to_temp_protocol(
#         G=G,
#         vessel=vessel,
#         temp=25.0,
#         active=True,
#         continue_heatchill=False,
#         stir=stir,
#         purpose="冷却到室温"
#     )


# def generate_reflux_heating_protocol(
#     G: nx.DiGraph,
#     vessel: str,
#     temp: float,
#     time: float = 3600.0  # 1小时回流
# ) -> List[Dict[str, Any]]:
#     """回流加热的快捷函数"""
#     return generate_heat_chill_protocol(
#         G=G,
#         vessel=vessel,
#         temp=temp,
#         time=time,
#         stir=True,
#         stir_speed=400.0,  # 回流时较快搅拌
#         purpose=f"回流加热到 {temp}°C"
#     )


# def generate_ice_bath_protocol(
#     G: nx.DiGraph,
#     vessel: str,
#     time: float = 600.0  # 10分钟冰浴
# ) -> List[Dict[str, Any]]:
#     """冰浴冷却的快捷函数"""
#     return generate_heat_chill_protocol(
#         G=G,
#         vessel=vessel,
#         temp=0.0,
#         time=time,
#         stir=True,
#         stir_speed=150.0,  # 冰浴时缓慢搅拌
#         purpose="冰浴冷却到 0°C"
#     )


# 测试函数
def test_heatchill_protocol():
    """测试加热/冷却协议的示例"""
    print("=== HEAT CHILL PROTOCOL 测试 ===")
    print("完整的四个协议函数：")
    print("1. generate_heat_chill_protocol - 带时间限制的完整操作")
    print("2. generate_heat_chill_start_protocol - 持续加热/冷却")
    print("3. generate_heat_chill_stop_protocol - 停止加热/冷却")
    print("4. generate_heat_chill_to_temp_protocol - 智能温控 (您的 HeatChillToTemp)")
    print("测试完成")


if __name__ == "__main__":
    test_heatchill_protocol()