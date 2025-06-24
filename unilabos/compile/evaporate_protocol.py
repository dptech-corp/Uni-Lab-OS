from typing import List, Dict, Any
import networkx as nx
from .pump_protocol import generate_pump_protocol


def get_vessel_liquid_volume(G: nx.DiGraph, vessel: str) -> float:
    """
    获取容器中的液体体积
    """
    if vessel not in G.nodes():
        return 0.0
    
    vessel_data = G.nodes[vessel].get('data', {})
    liquids = vessel_data.get('liquid', [])
    
    total_volume = 0.0
    for liquid in liquids:
        if isinstance(liquid, dict) and 'liquid_volume' in liquid:
            total_volume += liquid['liquid_volume']
    
    return total_volume


def find_rotavap_device(G: nx.DiGraph) -> str:
    """查找旋转蒸发仪设备"""
    rotavap_nodes = [node for node in G.nodes() 
                    if (G.nodes[node].get('class') or '') == 'virtual_rotavap']
    
    if rotavap_nodes:
        return rotavap_nodes[0]
    
    raise ValueError("系统中未找到旋转蒸发仪设备")


def find_solvent_recovery_vessel(G: nx.DiGraph) -> str:
    """查找溶剂回收容器"""
    possible_names = [
        "flask_distillate",
        "bottle_distillate", 
        "vessel_distillate",
        "distillate",
        "solvent_recovery",
        "flask_solvent_recovery",
        "collection_flask"
    ]
    
    for vessel_name in possible_names:
        if vessel_name in G.nodes():
            return vessel_name
    
    # 如果找不到专门的回收容器，使用废液容器
    waste_names = ["waste_workup", "flask_waste", "bottle_waste", "waste"]
    for vessel_name in waste_names:
        if vessel_name in G.nodes():
            return vessel_name
    
    raise ValueError(f"未找到溶剂回收容器。尝试了以下名称: {possible_names + waste_names}")


def generate_evaporate_protocol(
    G: nx.DiGraph,
    vessel: str,
    pressure: float = 0.1,
    temp: float = 60.0,
    time: float = 1800.0,
    stir_speed: float = 100.0
) -> List[Dict[str, Any]]:
    """
    生成蒸发操作的协议序列
    
    蒸发流程：
    1. 液体转移：将待蒸发溶液从源容器转移到旋转蒸发仪
    2. 蒸发操作：执行旋转蒸发
    3. (可选) 溶剂回收：将冷凝的溶剂转移到回收容器
    4. 残留物转移：将浓缩物从旋转蒸发仪转移回原容器或新容器
    
    Args:
        G: 有向图，节点为设备和容器，边为流体管道
        vessel: 包含待蒸发溶液的容器名称
        pressure: 蒸发时的真空度 (bar)，默认0.1 bar
        temp: 蒸发时的加热温度 (°C)，默认60°C  
        time: 蒸发时间 (秒)，默认1800秒(30分钟)
        stir_speed: 旋转速度 (RPM)，默认100 RPM
    
    Returns:
        List[Dict[str, Any]]: 蒸发操作的动作序列
    
    Raises:
        ValueError: 当找不到必要的设备时抛出异常
    
    Examples:
        evaporate_actions = generate_evaporate_protocol(G, "reaction_mixture", 0.05, 80.0, 3600.0)
    """
    action_sequence = []
    
    print(f"EVAPORATE: 开始生成蒸发协议")
    print(f"  - 源容器: {vessel}")
    print(f"  - 真空度: {pressure} bar")
    print(f"  - 温度: {temp}°C")
    print(f"  - 时间: {time}s ({time/60:.1f}分钟)")
    print(f"  - 旋转速度: {stir_speed} RPM")
    
    # 验证源容器存在
    if vessel not in G.nodes():
        raise ValueError(f"源容器 '{vessel}' 不存在于系统中")
    
    # 获取源容器中的液体体积
    source_volume = get_vessel_liquid_volume(G, vessel)
    print(f"EVAPORATE: 源容器 {vessel} 中有 {source_volume} mL 液体")
    
    # 查找旋转蒸发仪
    try:
        rotavap_id = find_rotavap_device(G)
        print(f"EVAPORATE: 找到旋转蒸发仪: {rotavap_id}")
    except ValueError as e:
        raise ValueError(f"无法找到旋转蒸发仪: {str(e)}")
    
    # 查找旋转蒸发仪样品容器
    rotavap_vessel = None
    possible_rotavap_vessels = ["rotavap_flask", "rotavap", "flask_rotavap", "evaporation_flask"]
    for rv in possible_rotavap_vessels:
        if rv in G.nodes():
            rotavap_vessel = rv
            break
    
    if not rotavap_vessel:
        raise ValueError(f"未找到旋转蒸发仪样品容器。尝试了: {possible_rotavap_vessels}")
    
    print(f"EVAPORATE: 找到旋转蒸发仪样品容器: {rotavap_vessel}")
    
    # 查找溶剂回收容器
    try:
        distillate_vessel = find_solvent_recovery_vessel(G)
        print(f"EVAPORATE: 找到溶剂回收容器: {distillate_vessel}")
    except ValueError as e:
        print(f"EVAPORATE: 警告 - {str(e)}")
        distillate_vessel = None
    
    # === 简化的体积计算策略 ===
    if source_volume > 0:
        # 如果能检测到液体体积，使用实际体积的大部分
        transfer_volume = min(source_volume * 0.9, 250.0)  # 90%或最多250mL
        print(f"EVAPORATE: 检测到液体体积，将转移 {transfer_volume} mL")
    else:
        # 如果检测不到液体体积，默认转移一整瓶 250mL
        transfer_volume = 250.0
        print(f"EVAPORATE: 未检测到液体体积，默认转移整瓶 {transfer_volume} mL")
    
    # === 第一步：将待蒸发溶液转移到旋转蒸发仪 ===
    print(f"EVAPORATE: 将 {transfer_volume} mL 溶液从 {vessel} 转移到 {rotavap_vessel}")
    try:
        transfer_to_rotavap_actions = generate_pump_protocol(
            G=G,
            from_vessel=vessel,
            to_vessel=rotavap_vessel,
            volume=transfer_volume,
            flowrate=2.0,
            transfer_flowrate=2.0
        )
        action_sequence.extend(transfer_to_rotavap_actions)
    except Exception as e:
        raise ValueError(f"无法将溶液转移到旋转蒸发仪: {str(e)}")
    
    # 转移后等待
    wait_action = {
        "action_name": "wait",
        "action_kwargs": {"time": 10}
    }
    action_sequence.append(wait_action)
    
    # === 第二步：执行旋转蒸发 ===
    print(f"EVAPORATE: 执行旋转蒸发操作")
    evaporate_action = {
        "device_id": rotavap_id,
        "action_name": "evaporate",
        "action_kwargs": {
            "vessel": rotavap_vessel,
            "pressure": pressure,
            "temp": temp,
            "time": time,
            "stir_speed": stir_speed
        }
    }
    action_sequence.append(evaporate_action)
    
    # 蒸发后等待系统稳定
    wait_action = {
        "action_name": "wait",
        "action_kwargs": {"time": 30}
    }
    action_sequence.append(wait_action)
    
    # === 第三步：溶剂回收（如果有回收容器）===
    if distillate_vessel:
        print(f"EVAPORATE: 回收冷凝溶剂到 {distillate_vessel}")
        try:
            condenser_vessel = "rotavap_condenser"
            if condenser_vessel in G.nodes():
                # 估算回收体积（约为转移体积的70% - 大部分溶剂被蒸发回收）
                recovery_volume = transfer_volume * 0.7
                print(f"EVAPORATE: 预计回收 {recovery_volume} mL 溶剂")
                
                recovery_actions = generate_pump_protocol(
                    G=G,
                    from_vessel=condenser_vessel,
                    to_vessel=distillate_vessel,
                    volume=recovery_volume,
                    flowrate=3.0,
                    transfer_flowrate=3.0
                )
                action_sequence.extend(recovery_actions)
            else:
                print("EVAPORATE: 未找到冷凝器容器，跳过溶剂回收")
        except Exception as e:
            print(f"EVAPORATE: 溶剂回收失败: {str(e)}")
    
    # === 第四步：将浓缩物转移回原容器 ===
    print(f"EVAPORATE: 将浓缩物从旋转蒸发仪转移回 {vessel}")
    try:
        # 估算浓缩物体积（约为转移体积的20% - 大部分溶剂已蒸发）
        concentrate_volume = transfer_volume * 0.2
        print(f"EVAPORATE: 预计浓缩物体积 {concentrate_volume} mL")
        
        transfer_back_actions = generate_pump_protocol(
            G=G,
            from_vessel=rotavap_vessel,
            to_vessel=vessel,
            volume=concentrate_volume,
            flowrate=1.0,  # 浓缩物可能粘稠，用较慢流速
            transfer_flowrate=1.0
        )
        action_sequence.extend(transfer_back_actions)
    except Exception as e:
        print(f"EVAPORATE: 将浓缩物转移回容器失败: {str(e)}")
    
    # === 第五步：清洗旋转蒸发仪 ===
    print(f"EVAPORATE: 清洗旋转蒸发仪")
    try:
        # 查找清洗溶剂
        cleaning_solvent = None
        for solvent in ["flask_ethanol", "flask_acetone", "flask_water"]:
            if solvent in G.nodes():
                cleaning_solvent = solvent
                break
        
        if cleaning_solvent and distillate_vessel:
            # 用固定量溶剂清洗（不依赖检测体积）
            cleaning_volume = 50.0  # 固定50mL清洗
            print(f"EVAPORATE: 用 {cleaning_volume} mL {cleaning_solvent} 清洗")
            
            # 清洗溶剂加入
            cleaning_actions = generate_pump_protocol(
                G=G,
                from_vessel=cleaning_solvent,
                to_vessel=rotavap_vessel,
                volume=cleaning_volume,
                flowrate=2.0,
                transfer_flowrate=2.0
            )
            action_sequence.extend(cleaning_actions)
            
            # 将清洗液转移到废液/回收容器
            waste_actions = generate_pump_protocol(
                G=G,
                from_vessel=rotavap_vessel,
                to_vessel=distillate_vessel,  # 使用回收容器作为废液
                volume=cleaning_volume,
                flowrate=2.0,
                transfer_flowrate=2.0
            )
            action_sequence.extend(waste_actions)
        
    except Exception as e:
        print(f"EVAPORATE: 清洗步骤失败: {str(e)}")
    
    print(f"EVAPORATE: 生成了 {len(action_sequence)} 个动作")
    print(f"EVAPORATE: 蒸发协议生成完成")
    print(f"EVAPORATE: 总处理体积: {transfer_volume} mL")
    
    return action_sequence


# 便捷函数：常用蒸发方案 - 都使用250mL标准瓶体积
def generate_quick_evaporate_protocol(
    G: nx.DiGraph,
    vessel: str,
    temp: float = 40.0,
    time: float = 900.0  # 15分钟
) -> List[Dict[str, Any]]:
    """快速蒸发：低温、短时间、整瓶处理"""
    return generate_evaporate_protocol(G, vessel, 0.2, temp, time, 80.0)


def generate_gentle_evaporate_protocol(
    G: nx.DiGraph,
    vessel: str,
    temp: float = 50.0,
    time: float = 2700.0  # 45分钟
) -> List[Dict[str, Any]]:
    """温和蒸发：中等条件、较长时间、整瓶处理"""
    return generate_evaporate_protocol(G, vessel, 0.1, temp, time, 60.0)


def generate_high_vacuum_evaporate_protocol(
    G: nx.DiGraph,
    vessel: str,
    temp: float = 35.0,
    time: float = 3600.0  # 1小时
) -> List[Dict[str, Any]]:
    """高真空蒸发：低温、高真空、长时间、整瓶处理"""
    return generate_evaporate_protocol(G, vessel, 0.01, temp, time, 120.0)


def generate_standard_evaporate_protocol(
    G: nx.DiGraph,
    vessel: str
) -> List[Dict[str, Any]]:
    """标准蒸发：常用参数、整瓶250mL处理"""
    return generate_evaporate_protocol(
        G=G,
        vessel=vessel, 
        pressure=0.1,      # 标准真空度
        temp=60.0,         # 适中温度
        time=1800.0,       # 30分钟
        stir_speed=100.0   # 适中旋转速度
    )
