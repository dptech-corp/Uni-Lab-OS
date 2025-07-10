from typing import List, Dict, Any, Optional
import networkx as nx
import logging

logger = logging.getLogger(__name__)

def debug_print(message):
    """调试输出"""
    print(f"[EVAPORATE] {message}", flush=True)
    logger.info(f"[EVAPORATE] {message}")

def find_rotavap_device(G: nx.DiGraph, vessel: str = None) -> Optional[str]:
    """
    在组态图中查找旋转蒸发仪设备
    
    Args:
        G: 设备图
        vessel: 指定的设备名称（可选）
    
    Returns:
        str: 找到的旋转蒸发仪设备ID，如果没找到返回None
    """
    debug_print("查找旋转蒸发仪设备...")
    
    # 如果指定了vessel，先检查是否存在且是旋转蒸发仪
    if vessel:
        if vessel in G.nodes():
            node_data = G.nodes[vessel]
            node_class = node_data.get('class', '')
            node_type = node_data.get('type', '')
            
            debug_print(f"检查指定设备 {vessel}: class={node_class}, type={node_type}")
            
            # 检查是否为旋转蒸发仪
            if any(keyword in str(node_class).lower() for keyword in ['rotavap', 'rotary', 'evaporat']):
                debug_print(f"✓ 找到指定的旋转蒸发仪: {vessel}")
                return vessel
            elif node_type == 'device':
                debug_print(f"✓ 指定设备存在，尝试直接使用: {vessel}")
                return vessel
        else:
            debug_print(f"✗ 指定的设备 {vessel} 不存在")
    
    # 在所有设备中查找旋转蒸发仪
    rotavap_candidates = []
    
    for node_id, node_data in G.nodes(data=True):
        node_class = node_data.get('class', '')
        node_type = node_data.get('type', '')
        
        # 跳过非设备节点
        if node_type != 'device':
            continue
            
        # 检查设备类型
        if any(keyword in str(node_class).lower() for keyword in ['rotavap', 'rotary', 'evaporat']):
            rotavap_candidates.append(node_id)
            debug_print(f"✓ 找到旋转蒸发仪候选: {node_id} (class: {node_class})")
        elif any(keyword in str(node_id).lower() for keyword in ['rotavap', 'rotary', 'evaporat']):
            rotavap_candidates.append(node_id)
            debug_print(f"✓ 找到旋转蒸发仪候选 (按名称): {node_id}")
    
    if rotavap_candidates:
        selected = rotavap_candidates[0]  # 选择第一个找到的
        debug_print(f"✓ 选择旋转蒸发仪: {selected}")
        return selected
    
    debug_print("✗ 未找到旋转蒸发仪设备")
    return None

def find_connected_vessel(G: nx.DiGraph, rotavap_device: str) -> Optional[str]:
    """
    查找与旋转蒸发仪连接的容器
    
    Args:
        G: 设备图
        rotavap_device: 旋转蒸发仪设备ID
    
    Returns:
        str: 连接的容器ID，如果没找到返回None
    """
    debug_print(f"查找与 {rotavap_device} 连接的容器...")
    
    # 查看旋转蒸发仪的子设备
    rotavap_data = G.nodes[rotavap_device]
    children = rotavap_data.get('children', [])
    
    for child_id in children:
        if child_id in G.nodes():
            child_data = G.nodes[child_id]
            child_type = child_data.get('type', '')
            
            if child_type == 'container':
                debug_print(f"✓ 找到连接的容器: {child_id}")
                return child_id
    
    # 查看邻接的容器
    for neighbor in G.neighbors(rotavap_device):
        neighbor_data = G.nodes[neighbor]
        neighbor_type = neighbor_data.get('type', '')
        
        if neighbor_type == 'container':
            debug_print(f"✓ 找到邻接的容器: {neighbor}")
            return neighbor
    
    debug_print("✗ 未找到连接的容器")
    return None

def generate_evaporate_protocol(
    G: nx.DiGraph,
    vessel: str,
    pressure: float = 0.1,
    temp: float = 60.0,
    time: float = 180.0,
    stir_speed: float = 100.0,
    solvent: str = "",
    **kwargs  # 接受任意额外参数，增强兼容性
) -> List[Dict[str, Any]]:
    """
    生成蒸发操作的协议序列
    
    Args:
        G: 设备图
        vessel: 容器名称或旋转蒸发仪名称
        pressure: 真空度 (bar)，默认0.1
        temp: 加热温度 (°C)，默认60
        time: 蒸发时间 (秒)，默认180
        stir_speed: 旋转速度 (RPM)，默认100
        solvent: 溶剂名称（用于参数优化）
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
    debug_print("=" * 50)
    
    # === 步骤1: 查找旋转蒸发仪设备 ===
    debug_print("步骤1: 查找旋转蒸发仪设备...")
    
    # 验证vessel参数
    if not vessel:
        raise ValueError("vessel 参数不能为空")
    
    # 查找旋转蒸发仪设备
    rotavap_device = find_rotavap_device(G, vessel)
    if not rotavap_device:
        raise ValueError(f"未找到旋转蒸发仪设备。请检查组态图中是否包含 class 包含 'rotavap'、'rotary' 或 'evaporat' 的设备")
    
    # === 步骤2: 确定目标容器 ===
    debug_print("步骤2: 确定目标容器...")
    
    target_vessel = vessel
    
    # 如果vessel就是旋转蒸发仪设备，查找连接的容器
    if vessel == rotavap_device:
        connected_vessel = find_connected_vessel(G, rotavap_device)
        if connected_vessel:
            target_vessel = connected_vessel
            debug_print(f"使用连接的容器: {target_vessel}")
        else:
            debug_print(f"未找到连接的容器，使用设备本身: {rotavap_device}")
            target_vessel = rotavap_device
    elif vessel in G.nodes() and G.nodes[vessel].get('type') == 'container':
        debug_print(f"使用指定的容器: {vessel}")
        target_vessel = vessel
    else:
        debug_print(f"容器 '{vessel}' 不存在或类型不正确，使用旋转蒸发仪设备: {rotavap_device}")
        target_vessel = rotavap_device
    
    # === 步骤3: 参数验证和修正 ===
    debug_print("步骤3: 参数验证和修正...")
    
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
    
    # === 步骤4: 生成动作序列 ===
    debug_print("步骤4: 生成动作序列...")
    
    action_sequence = []
    
    # 等待稳定
    action_sequence.append({
        "action_name": "wait",
        "action_kwargs": {"time": 10}
    })
    
    # 执行蒸发
    debug_print(f"执行蒸发: 设备={rotavap_device}, 容器={target_vessel}")
    evaporate_action = {
        "device_id": rotavap_device,
        "action_name": "evaporate",
        "action_kwargs": {
            "vessel": target_vessel,
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
    
    # === 总结 ===
    debug_print("=" * 50)
    debug_print(f"蒸发协议生成完成")
    debug_print(f"总动作数: {len(action_sequence)}")
    debug_print(f"旋转蒸发仪: {rotavap_device}")
    debug_print(f"目标容器: {target_vessel}")
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
