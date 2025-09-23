from typing import List, Dict, Any, Optional, Union
import networkx as nx
import logging
import re
from .utils.vessel_parser import get_vessel
from .utils.unit_parser import parse_time_input

logger = logging.getLogger(__name__)

def debug_print(message):
    """调试输出"""
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
    debug_print("🔍 开始查找旋转蒸发仪设备... 🌪️")
    
    # 如果指定了vessel，先检查是否存在且是旋转蒸发仪
    if vessel:
        debug_print(f"🎯 检查指定设备: {vessel} 🔧")
        if vessel in G.nodes():
            node_data = G.nodes[vessel]
            node_class = node_data.get('class', '')
            node_type = node_data.get('type', '')
            
            debug_print(f"📋 设备信息 {vessel}: class={node_class}, type={node_type}")
            
            # 检查是否为旋转蒸发仪
            if any(keyword in str(node_class).lower() for keyword in ['rotavap', 'rotary', 'evaporat']):
                debug_print(f"🎉 找到指定的旋转蒸发仪: {vessel} ✨")
                return vessel
            elif node_type == 'device':
                debug_print(f"✅ 指定设备存在，尝试直接使用: {vessel} 🔧")
                return vessel
        else:
            debug_print(f"❌ 指定的设备 {vessel} 不存在 😞")
    
    # 在所有设备中查找旋转蒸发仪
    debug_print("🔎 在所有设备中搜索旋转蒸发仪... 🕵️‍♀️")
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
            debug_print(f"🌟 找到旋转蒸发仪候选: {node_id} (class: {node_class}) 🌪️")
        elif any(keyword in str(node_id).lower() for keyword in ['rotavap', 'rotary', 'evaporat']):
            rotavap_candidates.append(node_id)
            debug_print(f"🌟 找到旋转蒸发仪候选 (按名称): {node_id} 🌪️")
    
    if rotavap_candidates:
        selected = rotavap_candidates[0]  # 选择第一个找到的
        debug_print(f"🎯 选择旋转蒸发仪: {selected} 🏆")
        return selected
    
    debug_print("😭 未找到旋转蒸发仪设备 💔")
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
    debug_print(f"🔗 查找与 {rotavap_device} 连接的容器... 🥽")
    
    # 查看旋转蒸发仪的子设备
    rotavap_data = G.nodes[rotavap_device]
    children = rotavap_data.get('children', [])
    
    debug_print(f"👶 检查子设备: {children}")
    for child_id in children:
        if child_id in G.nodes():
            child_data = G.nodes[child_id]
            child_type = child_data.get('type', '')
            
            if child_type == 'container':
                debug_print(f"🎉 找到连接的容器: {child_id} 🥽✨")
                return child_id
    
    # 查看邻接的容器
    debug_print("🤝 检查邻接设备...")
    for neighbor in G.neighbors(rotavap_device):
        neighbor_data = G.nodes[neighbor]
        neighbor_type = neighbor_data.get('type', '')
        
        if neighbor_type == 'container':
            debug_print(f"🎉 找到邻接的容器: {neighbor} 🥽✨")
            return neighbor
    
    debug_print("😞 未找到连接的容器 💔")
    return None

def generate_evaporate_protocol(
    G: nx.DiGraph,
    vessel: dict,  # 🔧 修改：从字符串改为字典类型
    pressure: float = 0.1,
    temp: float = 60.0,
    time: Union[str, float] = "180",     # 🔧 修改：支持字符串时间
    stir_speed: float = 100.0,
    solvent: str = "",
    **kwargs
) -> List[Dict[str, Any]]:
    """
    生成蒸发操作的协议序列 - 支持单位和体积运算
    
    Args:
        G: 设备图
        vessel: 容器字典（从XDL传入）
        pressure: 真空度 (bar)，默认0.1
        temp: 加热温度 (°C)，默认60
        time: 蒸发时间（支持 "3 min", "180", "0.5 h" 等）
        stir_speed: 旋转速度 (RPM)，默认100
        solvent: 溶剂名称（用于参数优化）
        **kwargs: 其他参数（兼容性）
    
    Returns:
        List[Dict[str, Any]]: 动作序列
    """
    
    # 🔧 核心修改：从字典中提取容器ID
    vessel_id, vessel_data = get_vessel(vessel)
    
    debug_print("🌟" * 20)
    debug_print("🌪️ 开始生成蒸发协议（支持单位和体积运算）✨")
    debug_print(f"📝 输入参数:")
    debug_print(f"  🥽 vessel: {vessel} (ID: {vessel_id})")
    debug_print(f"  💨 pressure: {pressure} bar")
    debug_print(f"  🌡️ temp: {temp}°C")
    debug_print(f"  ⏰ time: {time} (类型: {type(time)})")
    debug_print(f"  🌪️ stir_speed: {stir_speed} RPM")
    debug_print(f"  🧪 solvent: '{solvent}'")
    debug_print("🌟" * 20)
    
    # 🔧 新增：记录蒸发前的容器状态
    debug_print("🔍 记录蒸发前容器状态...")
    original_liquid_volume = 0.0
    if "data" in vessel and "liquid_volume" in vessel["data"]:
        current_volume = vessel["data"]["liquid_volume"]
        if isinstance(current_volume, list) and len(current_volume) > 0:
            original_liquid_volume = current_volume[0]
        elif isinstance(current_volume, (int, float)):
            original_liquid_volume = current_volume
    debug_print(f"📊 蒸发前液体体积: {original_liquid_volume:.2f}mL")
    
    # === 步骤1: 查找旋转蒸发仪设备 ===
    debug_print("📍 步骤1: 查找旋转蒸发仪设备... 🔍")
    
    # 验证vessel参数
    if not vessel_id:
        debug_print("❌ vessel 参数不能为空! 😱")
        raise ValueError("vessel 参数不能为空")
    
    # 查找旋转蒸发仪设备
    rotavap_device = find_rotavap_device(G, vessel_id)
    if not rotavap_device:
        debug_print("💥 未找到旋转蒸发仪设备! 😭")
        raise ValueError(f"未找到旋转蒸发仪设备。请检查组态图中是否包含 class 包含 'rotavap'、'rotary' 或 'evaporat' 的设备")
    
    debug_print(f"🎉 成功找到旋转蒸发仪: {rotavap_device} ✨")
    
    # === 步骤2: 确定目标容器 ===
    debug_print("📍 步骤2: 确定目标容器... 🥽")
    
    target_vessel = vessel_id
    
    # 如果vessel就是旋转蒸发仪设备，查找连接的容器
    if vessel_id == rotavap_device:
        debug_print("🔄 vessel就是旋转蒸发仪，查找连接的容器...")
        connected_vessel = find_connected_vessel(G, rotavap_device)
        if connected_vessel:
            target_vessel = connected_vessel
            debug_print(f"✅ 使用连接的容器: {target_vessel} 🥽✨")
        else:
            debug_print(f"⚠️ 未找到连接的容器，使用设备本身: {rotavap_device} 🔧")
            target_vessel = rotavap_device
    elif vessel_id in G.nodes() and G.nodes[vessel_id].get('type') == 'container':
        debug_print(f"✅ 使用指定的容器: {vessel_id} 🥽✨")
        target_vessel = vessel_id
    else:
        debug_print(f"⚠️ 容器 '{vessel_id}' 不存在或类型不正确，使用旋转蒸发仪设备: {rotavap_device} 🔧")
        target_vessel = rotavap_device
    
    # === 🔧 新增：步骤3：单位解析处理 ===
    debug_print("📍 步骤3: 单位解析处理... ⚡")
    
    # 解析时间
    final_time = parse_time_input(time)
    debug_print(f"🎯 时间解析完成: {time} → {final_time}s ({final_time/60:.1f}分钟) ⏰✨")
    
    # === 步骤4: 参数验证和修正 ===
    debug_print("📍 步骤4: 参数验证和修正... 🔧")
    
    # 修正参数范围
    if pressure <= 0 or pressure > 1.0:
        debug_print(f"⚠️ 真空度 {pressure} bar 超出范围，修正为 0.1 bar 💨")
        pressure = 0.1
    else:
        debug_print(f"✅ 真空度 {pressure} bar 在正常范围内 💨")
    
    if temp < 10.0 or temp > 200.0:
        debug_print(f"⚠️ 温度 {temp}°C 超出范围，修正为 60°C 🌡️")
        temp = 60.0
    else:
        debug_print(f"✅ 温度 {temp}°C 在正常范围内 🌡️")
    
    if final_time <= 0:
        debug_print(f"⚠️ 时间 {final_time}s 无效，修正为 180s (3分钟) ⏰")
        final_time = 180.0
    else:
        debug_print(f"✅ 时间 {final_time}s ({final_time/60:.1f}分钟) 有效 ⏰")
    
    if stir_speed < 10.0 or stir_speed > 300.0:
        debug_print(f"⚠️ 旋转速度 {stir_speed} RPM 超出范围，修正为 100 RPM 🌪️")
        stir_speed = 100.0
    else:
        debug_print(f"✅ 旋转速度 {stir_speed} RPM 在正常范围内 🌪️")
    
    # 根据溶剂优化参数
    if solvent:
        debug_print(f"🧪 根据溶剂 '{solvent}' 优化参数... 🔬")
        solvent_lower = solvent.lower()
        
        if any(s in solvent_lower for s in ['water', 'aqueous', 'h2o']):
            temp = max(temp, 80.0)
            pressure = max(pressure, 0.2)
            debug_print("💧 水系溶剂：提高温度和真空度 🌡️💨")
        elif any(s in solvent_lower for s in ['ethanol', 'methanol', 'acetone']):
            temp = min(temp, 50.0)
            pressure = min(pressure, 0.05)
            debug_print("🍺 易挥发溶剂：降低温度和真空度 🌡️💨")
        elif any(s in solvent_lower for s in ['dmso', 'dmi', 'toluene']):
            temp = max(temp, 100.0)
            pressure = min(pressure, 0.01)
            debug_print("🔥 高沸点溶剂：提高温度，降低真空度 🌡️💨")
        else:
            debug_print("🧪 通用溶剂，使用标准参数 ✨")
    else:
        debug_print("🤷‍♀️ 未指定溶剂，使用默认参数 ✨")
    
    debug_print(f"🎯 最终参数: pressure={pressure} bar 💨, temp={temp}°C 🌡️, time={final_time}s ⏰, stir_speed={stir_speed} RPM 🌪️")
    
    # === 🔧 新增：步骤5：蒸发体积计算 ===
    debug_print("📍 步骤5: 蒸发体积计算... 📊")
    
    # 根据温度、真空度、时间和溶剂类型估算蒸发量
    evaporation_volume = 0.0
    if original_liquid_volume > 0:
        # 基础蒸发速率（mL/min）
        base_evap_rate = 0.5  # 基础速率
        
        # 温度系数（高温蒸发更快）
        temp_factor = 1.0 + (temp - 25.0) / 100.0
        
        # 真空系数（真空度越高蒸发越快）
        vacuum_factor = 1.0 + (1.0 - pressure) * 2.0
        
        # 溶剂系数
        solvent_factor = 1.0
        if solvent:
            solvent_lower = solvent.lower()
            if any(s in solvent_lower for s in ['water', 'h2o']):
                solvent_factor = 0.8  # 水蒸发较慢
            elif any(s in solvent_lower for s in ['ethanol', 'methanol', 'acetone']):
                solvent_factor = 1.5  # 易挥发溶剂蒸发快
            elif any(s in solvent_lower for s in ['dmso', 'dmi']):
                solvent_factor = 0.3  # 高沸点溶剂蒸发慢
        
        # 计算总蒸发量
        total_evap_rate = base_evap_rate * temp_factor * vacuum_factor * solvent_factor
        evaporation_volume = min(
            original_liquid_volume * 0.95,  # 最多蒸发95%
            total_evap_rate * (final_time / 60.0)  # 时间相关的蒸发量
        )
        
        debug_print(f"📊 蒸发量计算:")
        debug_print(f"  - 基础蒸发速率: {base_evap_rate} mL/min")
        debug_print(f"  - 温度系数: {temp_factor:.2f} (基于 {temp}°C)")
        debug_print(f"  - 真空系数: {vacuum_factor:.2f} (基于 {pressure} bar)")
        debug_print(f"  - 溶剂系数: {solvent_factor:.2f} ({solvent or '通用'})")
        debug_print(f"  - 总蒸发速率: {total_evap_rate:.2f} mL/min")
        debug_print(f"  - 预计蒸发量: {evaporation_volume:.2f}mL ({evaporation_volume/original_liquid_volume*100:.1f}%)")
    
    # === 步骤6: 生成动作序列 ===
    debug_print("📍 步骤6: 生成动作序列... 🎬")
    
    action_sequence = []
    
    # 1. 等待稳定
    debug_print("  🔄 动作1: 添加初始等待稳定... ⏳")
    action_sequence.append({
        "action_name": "wait",
        "action_kwargs": {"time": 10}
    })
    debug_print("  ✅ 初始等待动作已添加 ⏳✨")
    
    # 2. 执行蒸发
    debug_print(f"  🌪️ 动作2: 执行蒸发操作...")
    debug_print(f"    🔧 设备: {rotavap_device}")
    debug_print(f"    🥽 容器: {target_vessel}")
    debug_print(f"    💨 真空度: {pressure} bar")
    debug_print(f"    🌡️ 温度: {temp}°C")
    debug_print(f"    ⏰ 时间: {final_time}s ({final_time/60:.1f}分钟)")
    debug_print(f"    🌪️ 旋转速度: {stir_speed} RPM")
    
    evaporate_action = {
        "device_id": rotavap_device,
        "action_name": "evaporate",
        "action_kwargs": {
            "vessel": {"id": target_vessel},
            "pressure": float(pressure),
            "temp": float(temp),
            "time": float(final_time),  # 🔧 强制转换为float类型
            "stir_speed": float(stir_speed),
            "solvent": str(solvent)
        }
    }
    action_sequence.append(evaporate_action)
    debug_print("  ✅ 蒸发动作已添加 🌪️✨")
    
    # 🔧 新增：蒸发过程中的体积变化
    debug_print("  🔧 更新容器体积 - 蒸发过程...")
    if evaporation_volume > 0:
        new_volume = max(0.0, original_liquid_volume - evaporation_volume)
        
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
        
        debug_print(f"  📊 蒸发体积变化: {original_liquid_volume:.2f}mL → {new_volume:.2f}mL (-{evaporation_volume:.2f}mL)")
    
    # 3. 蒸发后等待
    debug_print("  🔄 动作3: 添加蒸发后等待... ⏳")
    action_sequence.append({
        "action_name": "wait",
        "action_kwargs": {"time": 10}
    })
    debug_print("  ✅ 蒸发后等待动作已添加 ⏳✨")
    
    # 🔧 新增：蒸发完成后的状态报告
    final_liquid_volume = 0.0
    if "data" in vessel and "liquid_volume" in vessel["data"]:
        current_volume = vessel["data"]["liquid_volume"]
        if isinstance(current_volume, list) and len(current_volume) > 0:
            final_liquid_volume = current_volume[0]
        elif isinstance(current_volume, (int, float)):
            final_liquid_volume = current_volume
    
    # === 总结 ===
    debug_print("🎊" * 20)
    debug_print(f"🎉 蒸发协议生成完成! ✨")
    debug_print(f"📊 总动作数: {len(action_sequence)} 个 📝")
    debug_print(f"🌪️ 旋转蒸发仪: {rotavap_device} 🔧")
    debug_print(f"🥽 目标容器: {target_vessel} 🧪")
    debug_print(f"⚙️ 蒸发参数: {pressure} bar 💨, {temp}°C 🌡️, {final_time}s ⏰, {stir_speed} RPM 🌪️")
    debug_print(f"⏱️ 预计总时间: {(final_time + 20)/60:.1f} 分钟 ⌛")
    debug_print(f"📊 体积变化:")
    debug_print(f"  - 蒸发前: {original_liquid_volume:.2f}mL")
    debug_print(f"  - 蒸发后: {final_liquid_volume:.2f}mL") 
    debug_print(f"  - 蒸发量: {evaporation_volume:.2f}mL ({evaporation_volume/max(original_liquid_volume, 0.01)*100:.1f}%)")
    debug_print("🎊" * 20)
    
    return action_sequence
