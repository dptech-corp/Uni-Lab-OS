from typing import List, Dict, Any, Optional
import networkx as nx
import logging
from .pump_protocol import generate_pump_protocol

logger = logging.getLogger(__name__)

def debug_print(message):
    """调试输出"""
    print(f"[EVAPORATE] {message}", flush=True)
    logger.info(f"[EVAPORATE] {message}")

def get_vessel_liquid_volume(G: nx.DiGraph, vessel: str) -> float:
    """获取容器中的液体体积"""
    debug_print(f"检查容器 '{vessel}' 的液体体积...")
    
    if vessel not in G.nodes():
        debug_print(f"容器 '{vessel}' 不存在")
        return 0.0
    
    vessel_data = G.nodes[vessel].get('data', {})
    debug_print(f"容器数据: {vessel_data}")
    
    # 检查多种体积字段
    volume_keys = ['total_volume', 'volume', 'liquid_volume', 'current_volume']
    for key in volume_keys:
        if key in vessel_data:
            try:
                volume = float(vessel_data[key])
                debug_print(f"从 '{key}' 读取到体积: {volume}mL")
                return volume
            except (ValueError, TypeError):
                continue
    
    # 检查liquid数组
    liquids = vessel_data.get('liquid', [])
    if isinstance(liquids, list):
        total_volume = 0.0
        for liquid in liquids:
            if isinstance(liquid, dict):
                for vol_key in ['liquid_volume', 'volume', 'amount']:
                    if vol_key in liquid:
                        try:
                            vol = float(liquid[vol_key])
                            total_volume += vol
                            debug_print(f"从液体数据 '{vol_key}' 读取: {vol}mL")
                        except (ValueError, TypeError):
                            continue
        if total_volume > 0:
            return total_volume
    
    debug_print(f"未检测到液体体积，返回 0.0")
    return 0.0

def find_rotavap_device(G: nx.DiGraph) -> Optional[str]:
    """查找旋转蒸发仪设备"""
    debug_print("查找旋转蒸发仪设备...")
    
    # 查找各种可能的旋转蒸发仪设备
    possible_devices = []
    for node in G.nodes():
        node_data = G.nodes[node]
        node_class = node_data.get('class', '')
        
        if any(keyword in node_class.lower() for keyword in ['rotavap', 'evaporator']):
            possible_devices.append(node)
            debug_print(f"找到旋转蒸发仪设备: {node}")
    
    if possible_devices:
        return possible_devices[0]
    
    debug_print("未找到旋转蒸发仪设备")
    return None

def find_rotavap_vessel(G: nx.DiGraph) -> Optional[str]:
    """查找旋转蒸发仪样品容器"""
    debug_print("查找旋转蒸发仪样品容器...")
    
    possible_vessels = [
        "rotavap", "rotavap_flask", "flask_rotavap", 
        "evaporation_flask", "evaporator", "rotary_evaporator"
    ]
    
    for vessel in possible_vessels:
        if vessel in G.nodes():
            debug_print(f"找到旋转蒸发仪样品容器: {vessel}")
            return vessel
    
    debug_print("未找到旋转蒸发仪样品容器")
    return None

def find_recovery_vessel(G: nx.DiGraph) -> Optional[str]:
    """查找溶剂回收容器"""
    debug_print("查找溶剂回收容器...")
    
    possible_vessels = [
        "flask_distillate", "distillate", "solvent_recovery",
        "rotavap_condenser", "condenser", "waste_workup", "waste"
    ]
    
    for vessel in possible_vessels:
        if vessel in G.nodes():
            debug_print(f"找到回收容器: {vessel}")
            return vessel
    
    debug_print("未找到回收容器")
    return None

def generate_evaporate_protocol(
    G: nx.DiGraph,
    vessel: str,
    pressure: float = 0.1,
    temp: float = 60.0,
    time: float = 1800.0,
    stir_speed: float = 100.0,
    solvent: str = "",
    **kwargs  # 接受任意额外参数，增强兼容性
) -> List[Dict[str, Any]]:
    """
    生成蒸发操作的协议序列 - 增强兼容性版本
    
    Args:
        G: 设备图
        vessel: 蒸发容器名称（必需）
        pressure: 真空度 (bar)，默认0.1
        temp: 加热温度 (°C)，默认60
        time: 蒸发时间 (秒)，默认1800
        stir_speed: 旋转速度 (RPM)，默认100
        solvent: 溶剂名称（可选，用于参数优化）
        **kwargs: 其他参数（兼容性）
    
    Returns:
        List[Dict[str, Any]]: 动作序列
    """
    
    debug_print("=" * 50)
    debug_print("开始生成蒸发协议")
    debug_print(f"输入参数:")
    debug_print(f"  - vessel: {vessel}")
    debug_print(f"  - pressure: {pressure} bar")
    debug_print(f"  - temp: {temp}°C")
    debug_print(f"  - time: {time}s ({time/60:.1f}分钟)")
    debug_print(f"  - stir_speed: {stir_speed} RPM")
    debug_print(f"  - solvent: '{solvent}'")
    debug_print(f"  - 其他参数: {kwargs}")
    debug_print("=" * 50)
    
    action_sequence = []
    
    # === 参数验证和修正 ===
    debug_print("步骤1: 参数验证和修正...")
    
    # 验证必需参数
    if not vessel:
        raise ValueError("vessel 参数不能为空")
    
    if vessel not in G.nodes():
        raise ValueError(f"容器 '{vessel}' 不存在于系统中")
    
    # 修正参数范围
    if pressure <= 0 or pressure > 1.0:
        debug_print(f"真空度 {pressure} bar 超出范围，修正为 0.1 bar")
        pressure = 0.1
    
    if temp < 10.0 or temp > 200.0:
        debug_print(f"温度 {temp}°C 超出范围，修正为 60°C")
        temp = 60.0
    
    if time <= 0:
        debug_print(f"时间 {time}s 无效，修正为 1800s")
        time = 1800.0
    
    if stir_speed < 10.0 or stir_speed > 300.0:
        debug_print(f"旋转速度 {stir_speed} RPM 超出范围，修正为 100 RPM")
        stir_speed = 100.0
    
    # 根据溶剂优化参数
    if solvent:
        debug_print(f"根据溶剂 '{solvent}' 优化参数...")
        solvent_lower = solvent.lower()
        
        if any(s in solvent_lower for s in ['water', 'aqueous', 'h2o']):
            temp = max(temp, 80.0)
            pressure = max(pressure, 0.2)
            debug_print("水系溶剂：提高温度和真空度")
        elif any(s in solvent_lower for s in ['ethanol', 'methanol', 'acetone']):
            temp = min(temp, 50.0)
            pressure = min(pressure, 0.05)
            debug_print("易挥发溶剂：降低温度和真空度")
        elif any(s in solvent_lower for s in ['dmso', 'dmi', 'toluene']):
            temp = max(temp, 100.0)
            pressure = min(pressure, 0.01)
            debug_print("高沸点溶剂：提高温度，降低真空度")
    
    debug_print(f"最终参数: pressure={pressure}, temp={temp}, time={time}, stir_speed={stir_speed}")
    
    # === 查找设备 ===
    debug_print("步骤2: 查找设备...")
    
    # 查找旋转蒸发仪设备
    rotavap_device = find_rotavap_device(G)
    if not rotavap_device:
        debug_print("未找到旋转蒸发仪设备，使用通用设备")
        rotavap_device = "rotavap_1"  # 默认设备ID
    
    # 查找旋转蒸发仪样品容器
    rotavap_vessel = find_rotavap_vessel(G)
    if not rotavap_vessel:
        debug_print("未找到旋转蒸发仪样品容器，使用默认容器")
        rotavap_vessel = "rotavap"  # 默认容器
    
    # 查找回收容器
    recovery_vessel = find_recovery_vessel(G)
    
    debug_print(f"设备配置:")
    debug_print(f"  - 旋转蒸发仪设备: {rotavap_device}")
    debug_print(f"  - 样品容器: {rotavap_vessel}")
    debug_print(f"  - 回收容器: {recovery_vessel}")
    
    # === 体积计算 ===
    debug_print("步骤3: 体积计算...")
    
    source_volume = get_vessel_liquid_volume(G, vessel)
    
    if source_volume > 0:
        transfer_volume = min(source_volume * 0.9, 250.0)  # 90%或最多250mL
        debug_print(f"检测到液体体积 {source_volume}mL，转移 {transfer_volume}mL")
    else:
        transfer_volume = 50.0  # 默认小体积，更安全
        debug_print(f"未检测到液体体积，使用默认转移体积 {transfer_volume}mL")
    
    # === 生成动作序列 ===
    debug_print("步骤4: 生成动作序列...")
    
    # 动作1: 转移溶液到旋转蒸发仪
    if vessel != rotavap_vessel:
        debug_print(f"转移 {transfer_volume}mL 从 {vessel} 到 {rotavap_vessel}")
        try:
            transfer_actions = generate_pump_protocol(
                G=G,
                from_vessel=vessel,
                to_vessel=rotavap_vessel,
                volume=transfer_volume,
                flowrate=2.0,
                transfer_flowrate=2.0
            )
            action_sequence.extend(transfer_actions)
            debug_print(f"添加了 {len(transfer_actions)} 个转移动作")
        except Exception as e:
            debug_print(f"转移失败: {str(e)}")
            # 继续执行，不中断整个流程
    
    # 等待稳定
    action_sequence.append({
        "action_name": "wait",
        "action_kwargs": {"time": 10}
    })
    
    # 动作2: 执行蒸发
    debug_print(f"执行蒸发: {rotavap_device}")
    evaporate_action = {
        "device_id": rotavap_device,
        "action_name": "evaporate",
        "action_kwargs": {
            "vessel": rotavap_vessel,
            "pressure": pressure,
            "temp": temp,
            "time": time,
            "stir_speed": stir_speed,
            "solvent": solvent
        }
    }
    action_sequence.append(evaporate_action)
    
    # 蒸发后等待
    action_sequence.append({
        "action_name": "wait",
        "action_kwargs": {"time": 30}
    })
    
    # 动作3: 回收溶剂（如果有回收容器）
    if recovery_vessel:
        debug_print(f"回收溶剂到 {recovery_vessel}")
        try:
            recovery_volume = transfer_volume * 0.7  # 估算回收70%
            recovery_actions = generate_pump_protocol(
                G=G,
                from_vessel="rotavap_condenser",  # 假设的冷凝器
                to_vessel=recovery_vessel,
                volume=recovery_volume,
                flowrate=3.0,
                transfer_flowrate=3.0
            )
            action_sequence.extend(recovery_actions)
            debug_print(f"添加了 {len(recovery_actions)} 个回收动作")
        except Exception as e:
            debug_print(f"溶剂回收失败: {str(e)}")
    
    # 动作4: 转移浓缩物回原容器
    if vessel != rotavap_vessel:
        debug_print(f"转移浓缩物从 {rotavap_vessel} 到 {vessel}")
        try:
            concentrate_volume = transfer_volume * 0.2  # 估算浓缩物20%
            transfer_back_actions = generate_pump_protocol(
                G=G,
                from_vessel=rotavap_vessel,
                to_vessel=vessel,
                volume=concentrate_volume,
                flowrate=1.0,  # 浓缩物可能粘稠
                transfer_flowrate=1.0
            )
            action_sequence.extend(transfer_back_actions)
            debug_print(f"添加了 {len(transfer_back_actions)} 个转移回收动作")
        except Exception as e:
            debug_print(f"浓缩物转移失败: {str(e)}")
    
    # === 总结 ===
    debug_print("=" * 50)
    debug_print(f"蒸发协议生成完成")
    debug_print(f"总动作数: {len(action_sequence)}")
    debug_print(f"处理体积: {transfer_volume}mL")
    debug_print(f"蒸发参数: {pressure} bar, {temp}°C, {time}s, {stir_speed} RPM")
    debug_print("=" * 50)
    
    return action_sequence

# === 便捷函数 ===

def generate_quick_evaporate_protocol(
    G: nx.DiGraph,
    vessel: str,
    **kwargs
) -> List[Dict[str, Any]]:
    """快速蒸发：低温短时间"""
    return generate_evaporate_protocol(
        G, vessel, 
        pressure=0.2, 
        temp=40.0, 
        time=900.0, 
        stir_speed=80.0,
        **kwargs
    )

def generate_gentle_evaporate_protocol(
    G: nx.DiGraph,
    vessel: str,
    **kwargs
) -> List[Dict[str, Any]]:
    """温和蒸发：中等条件"""
    return generate_evaporate_protocol(
        G, vessel, 
        pressure=0.1, 
        temp=50.0, 
        time=2700.0, 
        stir_speed=60.0,
        **kwargs
    )

def generate_high_vacuum_evaporate_protocol(
    G: nx.DiGraph,
    vessel: str,
    **kwargs
) -> List[Dict[str, Any]]:
    """高真空蒸发：低温高真空"""
    return generate_evaporate_protocol(
        G, vessel, 
        pressure=0.01, 
        temp=35.0, 
        time=3600.0, 
        stir_speed=120.0,
        **kwargs
    )

def generate_standard_evaporate_protocol(
    G: nx.DiGraph,
    vessel: str,
    **kwargs
) -> List[Dict[str, Any]]:
    """标准蒸发：常用参数"""
    return generate_evaporate_protocol(
        G, vessel, 
        pressure=0.1, 
        temp=60.0, 
        time=1800.0, 
        stir_speed=100.0,
        **kwargs
    )
