from typing import List, Dict, Any, Union
import networkx as nx
import logging
import re

logger = logging.getLogger(__name__)

def debug_print(message):
    """调试输出"""
    print(f"🧼 [WASH_SOLID] {message}", flush=True)
    logger.info(f"[WASH_SOLID] {message}")

def parse_time_input(time_input: Union[str, float, int]) -> float:
    """统一时间解析函数（精简版）"""
    if not time_input:
        return 0.0
    
    # 🔢 处理数值输入
    if isinstance(time_input, (int, float)):
        result = float(time_input)
        debug_print(f"⏰ 数值时间: {time_input} → {result}s")
        return result
    
    # 📝 处理字符串输入
    time_str = str(time_input).lower().strip()
    
    # ❓ 特殊值快速处理
    special_times = {
        '?': 60.0, 'unknown': 60.0, 'briefly': 30.0, 
        'quickly': 45.0, 'slowly': 120.0
    }
    
    if time_str in special_times:
        result = special_times[time_str]
        debug_print(f"🎯 特殊时间: '{time_str}' → {result}s")
        return result
    
    # 🔢 数字提取（简化正则）
    try:
        # 提取数字
        numbers = re.findall(r'\d+\.?\d*', time_str)
        if numbers:
            value = float(numbers[0])
            
            # 简化单位判断
            if any(unit in time_str for unit in ['min', 'm']):
                result = value * 60.0
            elif any(unit in time_str for unit in ['h', 'hour']):
                result = value * 3600.0
            else:
                result = value  # 默认秒
            
            debug_print(f"✅ 时间解析: '{time_str}' → {result}s")
            return result
    except:
        pass
    
    debug_print(f"⚠️ 时间解析失败: '{time_str}'，使用默认60s")
    return 60.0

def parse_volume_input(volume: Union[float, str], volume_spec: str = "", mass: str = "") -> float:
    """统一体积解析函数（精简版）"""
    debug_print(f"💧 解析体积: volume={volume}, spec='{volume_spec}', mass='{mass}'")
    
    # 🎯 优先级1：volume_spec（快速映射）
    if volume_spec:
        spec_map = {
            'small': 20.0, 'medium': 50.0, 'large': 100.0,
            'minimal': 10.0, 'normal': 50.0, 'generous': 150.0
        }
        for key, val in spec_map.items():
            if key in volume_spec.lower():
                debug_print(f"🎯 规格匹配: '{volume_spec}' → {val}mL")
                return val
    
    # 🧮 优先级2：mass转体积（简化：1g=1mL）
    if mass:
        try:
            numbers = re.findall(r'\d+\.?\d*', mass)
            if numbers:
                value = float(numbers[0])
                if 'mg' in mass.lower():
                    result = value / 1000.0
                elif 'kg' in mass.lower():
                    result = value * 1000.0
                else:
                    result = value  # 默认g
                debug_print(f"⚖️ 质量转换: {mass} → {result}mL")
                return result
        except:
            pass
    
    # 📦 优先级3：volume
    if volume:
        if isinstance(volume, (int, float)):
            result = float(volume)
            debug_print(f"💧 数值体积: {volume} → {result}mL")
            return result
        elif isinstance(volume, str):
            try:
                # 提取数字
                numbers = re.findall(r'\d+\.?\d*', volume)
                if numbers:
                    value = float(numbers[0])
                    # 简化单位判断
                    if 'l' in volume.lower() and 'ml' not in volume.lower():
                        result = value * 1000.0  # L转mL
                    else:
                        result = value  # 默认mL
                    debug_print(f"💧 字符串体积: '{volume}' → {result}mL")
                    return result
            except:
                pass
    
    # 默认值
    debug_print(f"⚠️ 体积解析失败，使用默认50mL")
    return 50.0

def find_solvent_source(G: nx.DiGraph, solvent: str) -> str:
    """查找溶剂源（精简版）"""
    debug_print(f"🔍 查找溶剂源: {solvent}")
    
    # 简化搜索列表
    search_patterns = [
        f"flask_{solvent}", f"bottle_{solvent}", f"reagent_{solvent}",
        "liquid_reagent_bottle_1", "flask_1", "solvent_bottle"
    ]
    
    for pattern in search_patterns:
        if pattern in G.nodes():
            debug_print(f"🎉 找到溶剂源: {pattern}")
            return pattern
    
    debug_print(f"⚠️ 使用默认溶剂源: flask_{solvent}")
    return f"flask_{solvent}"

def find_filtrate_vessel(G: nx.DiGraph, filtrate_vessel: str = "") -> str:
    """查找滤液容器（精简版）"""
    debug_print(f"🔍 查找滤液容器: {filtrate_vessel}")
    
    # 如果指定了且存在，直接使用
    if filtrate_vessel and filtrate_vessel in G.nodes():
        debug_print(f"✅ 使用指定容器: {filtrate_vessel}")
        return filtrate_vessel
    
    # 简化搜索列表
    default_vessels = ["waste_workup", "filtrate_vessel", "flask_1", "collection_bottle_1"]
    
    for vessel in default_vessels:
        if vessel in G.nodes():
            debug_print(f"🎉 找到滤液容器: {vessel}")
            return vessel
    
    debug_print(f"⚠️ 使用默认滤液容器: waste_workup")
    return "waste_workup"

def extract_vessel_id(vessel: Union[str, dict]) -> str:
    """
    从vessel参数中提取vessel_id
    
    Args:
        vessel: vessel字典或vessel_id字符串
        
    Returns:
        str: vessel_id
    """
    if isinstance(vessel, dict):
        vessel_id = list(vessel.values())[0].get("id", "")
        debug_print(f"🔧 从vessel字典提取ID: {vessel_id}")
        return vessel_id
    elif isinstance(vessel, str):
        debug_print(f"🔧 vessel参数为字符串: {vessel}")
        return vessel
    else:
        debug_print(f"⚠️ 无效的vessel参数类型: {type(vessel)}")
        return ""

def get_vessel_display_info(vessel: Union[str, dict]) -> str:
    """
    获取容器的显示信息（用于日志）
    
    Args:
        vessel: vessel字典或vessel_id字符串
        
    Returns:
        str: 显示信息
    """
    if isinstance(vessel, dict):
        vessel_id = vessel.get("id", "unknown")
        vessel_name = vessel.get("name", "")
        if vessel_name:
            return f"{vessel_id} ({vessel_name})"
        else:
            return vessel_id
    else:
        return str(vessel)

def get_vessel_liquid_volume(vessel: dict) -> float:
    """
    获取容器中的液体体积 - 支持vessel字典
    
    Args:
        vessel: 容器字典
        
    Returns:
        float: 液体体积（mL）
    """
    if not vessel or "data" not in vessel:
        debug_print(f"⚠️ 容器数据为空，返回 0.0mL")
        return 0.0
    
    vessel_data = vessel["data"]
    vessel_id = vessel.get("id", "unknown")
    
    debug_print(f"🔍 读取容器 '{vessel_id}' 体积数据: {vessel_data}")
    
    # 检查liquid_volume字段
    if "liquid_volume" in vessel_data:
        liquid_volume = vessel_data["liquid_volume"]
        
        # 处理列表格式
        if isinstance(liquid_volume, list):
            if len(liquid_volume) > 0:
                volume = liquid_volume[0]
                if isinstance(volume, (int, float)):
                    debug_print(f"✅ 容器 '{vessel_id}' 体积: {volume}mL (列表格式)")
                    return float(volume)
        
        # 处理直接数值格式
        elif isinstance(liquid_volume, (int, float)):
            debug_print(f"✅ 容器 '{vessel_id}' 体积: {liquid_volume}mL (数值格式)")
            return float(liquid_volume)
    
    # 检查其他可能的体积字段
    volume_keys = ['current_volume', 'total_volume', 'volume']
    for key in volume_keys:
        if key in vessel_data:
            try:
                volume = float(vessel_data[key])
                if volume > 0:
                    debug_print(f"✅ 容器 '{vessel_id}' 体积: {volume}mL (字段: {key})")
                    return volume
            except (ValueError, TypeError):
                continue
    
    debug_print(f"⚠️ 无法获取容器 '{vessel_id}' 的体积，返回默认值 0.0mL")
    return 0.0

def update_vessel_volume(vessel: dict, G: nx.DiGraph, new_volume: float, description: str = "") -> None:
    """
    更新容器体积（同时更新vessel字典和图节点）
    
    Args:
        vessel: 容器字典
        G: 网络图
        new_volume: 新体积
        description: 更新描述
    """
    vessel_id = vessel.get("id", "unknown")
    
    if description:
        debug_print(f"🔧 更新容器体积 - {description}")
    
    # 更新vessel字典中的体积
    if "data" in vessel:
        if "liquid_volume" in vessel["data"]:
            current_volume = vessel["data"]["liquid_volume"]
            if isinstance(current_volume, list):
                if len(current_volume) > 0:
                    vessel["data"]["liquid_volume"][0] = new_volume
                else:
                    vessel["data"]["liquid_volume"] = [new_volume]
            else:
                vessel["data"]["liquid_volume"] = new_volume
        else:
            vessel["data"]["liquid_volume"] = new_volume
    else:
        vessel["data"] = {"liquid_volume": new_volume}
    
    # 同时更新图中的容器数据
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
    
    debug_print(f"📊 容器 '{vessel_id}' 体积已更新为: {new_volume:.2f}mL")

def generate_wash_solid_protocol(
    G: nx.DiGraph,
    vessel: Union[str, dict],  # 🔧 修改：支持vessel字典
    solvent: str,
    volume: Union[float, str] = "50",
    filtrate_vessel: Union[str, dict] = "",  # 🔧 修改：支持vessel字典
    temp: float = 25.0,
    stir: bool = False,
    stir_speed: float = 0.0,
    time: Union[str, float] = "0",
    repeats: int = 1,
    volume_spec: str = "",
    repeats_spec: str = "",
    mass: str = "",
    event: str = "",
    **kwargs
) -> List[Dict[str, Any]]:
    """
    生成固体清洗协议 - 支持vessel字典和体积运算
    
    Args:
        G: 有向图，节点为设备和容器，边为流体管道
        vessel: 清洗容器字典（从XDL传入）或容器ID字符串
        solvent: 清洗溶剂名称
        volume: 溶剂体积（每次清洗）
        filtrate_vessel: 滤液收集容器字典或容器ID字符串
        temp: 清洗温度（°C）
        stir: 是否搅拌
        stir_speed: 搅拌速度（RPM）
        time: 搅拌时间
        repeats: 清洗重复次数
        volume_spec: 体积规格（small/medium/large）
        repeats_spec: 重复次数规格（few/several/many）
        mass: 固体质量（用于计算溶剂用量）
        event: 事件描述
        **kwargs: 其他可选参数
    
    Returns:
        List[Dict[str, Any]]: 固体清洗操作的动作序列
    """
    
    # 🔧 核心修改：从vessel参数中提取vessel_id
    vessel_id = extract_vessel_id(vessel)
    vessel_display = get_vessel_display_info(vessel)
    
    # 🔧 处理filtrate_vessel参数
    filtrate_vessel_id = extract_vessel_id(filtrate_vessel) if filtrate_vessel else ""
    
    debug_print("🧼" * 20)
    debug_print("🚀 开始生成固体清洗协议（支持vessel字典和体积运算）✨")
    debug_print(f"📝 输入参数:")
    debug_print(f"  🥽 vessel: {vessel_display} (ID: {vessel_id})")
    debug_print(f"  🧪 solvent: {solvent}")
    debug_print(f"  💧 volume: {volume}")
    debug_print(f"  🗑️ filtrate_vessel: {filtrate_vessel_id}")
    debug_print(f"  ⏰ time: {time}")
    debug_print(f"  🔄 repeats: {repeats}")
    debug_print("🧼" * 20)
    
    # 🔧 新增：记录清洗前的容器状态
    debug_print("🔍 记录清洗前容器状态...")
    if isinstance(vessel, dict):
        original_volume = get_vessel_liquid_volume(vessel)
        debug_print(f"📊 清洗前液体体积: {original_volume:.2f}mL")
    else:
        original_volume = 0.0
        debug_print(f"📊 vessel为字符串格式，无法获取体积信息")
    
    # 📋 快速验证
    if not vessel_id or vessel_id not in G.nodes():  # 🔧 使用 vessel_id
        debug_print("❌ 容器验证失败! 😱")
        raise ValueError("vessel 参数无效")
    
    if not solvent:
        debug_print("❌ 溶剂不能为空! 😱")
        raise ValueError("solvent 参数不能为空")
    
    debug_print("✅ 基础验证通过 🎯")
    
    # 🔄 参数解析
    debug_print("📍 步骤1: 参数解析... ⚡")
    final_volume = parse_volume_input(volume, volume_spec, mass)
    final_time = parse_time_input(time)
    
    # 重复次数处理（简化）
    if repeats_spec:
        spec_map = {'few': 2, 'several': 3, 'many': 4, 'thorough': 5}
        final_repeats = next((v for k, v in spec_map.items() if k in repeats_spec.lower()), repeats)
    else:
        final_repeats = max(1, min(repeats, 5))  # 限制1-5次
    
    # 🕐 模拟时间优化
    debug_print("  ⏱️ 模拟时间优化...")
    original_time = final_time
    if final_time > 60.0:
        final_time = 60.0  # 限制最长60秒
        debug_print(f"  🎮 时间优化: {original_time}s → {final_time}s ⚡")
    
    # 参数修正
    temp = max(25.0, min(temp, 80.0))  # 温度范围25-80°C
    stir_speed = max(0.0, min(stir_speed, 300.0)) if stir else 0.0  # 速度范围0-300
    
    debug_print(f"🎯 最终参数: 体积={final_volume}mL, 时间={final_time}s, 重复={final_repeats}次")
    
    # 🔍 查找设备
    debug_print("📍 步骤2: 查找设备... 🔍")
    try:
        solvent_source = find_solvent_source(G, solvent)
        actual_filtrate_vessel = find_filtrate_vessel(G, filtrate_vessel_id)
        debug_print(f"🎉 设备配置完成 ✨")
        debug_print(f"  🧪 溶剂源: {solvent_source}")
        debug_print(f"  🗑️ 滤液容器: {actual_filtrate_vessel}")
    except Exception as e:
        debug_print(f"❌ 设备查找失败: {str(e)} 😭")
        raise ValueError(f"设备查找失败: {str(e)}")
    
    # 🚀 生成动作序列
    debug_print("📍 步骤3: 生成清洗动作... 🧼")
    action_sequence = []
    
    # 🔧 新增：体积变化跟踪变量
    current_volume = original_volume
    total_solvent_used = 0.0
    
    for cycle in range(final_repeats):
        debug_print(f"  🔄 第{cycle+1}/{final_repeats}次清洗...")
        
        # 1. 转移溶剂
        try:
            from .pump_protocol import generate_pump_protocol_with_rinsing
            
            debug_print(f"    💧 添加溶剂: {final_volume}mL {solvent}")
            transfer_actions = generate_pump_protocol_with_rinsing(
                G=G,
                from_vessel=solvent_source,
                to_vessel=vessel_id,  # 🔧 使用 vessel_id
                volume=final_volume,
                amount="",
                time=0.0,
                viscous=False,
                rinsing_solvent="",
                rinsing_volume=0.0,
                rinsing_repeats=0,
                solid=False,
                flowrate=2.5,
                transfer_flowrate=0.5
            )
            
            if transfer_actions:
                action_sequence.extend(transfer_actions)
                debug_print(f"    ✅ 转移动作: {len(transfer_actions)}个 🚚")
                
                # 🔧 新增：更新体积 - 添加溶剂后
                current_volume += final_volume
                total_solvent_used += final_volume
                
                if isinstance(vessel, dict):
                    update_vessel_volume(vessel, G, current_volume, 
                                       f"第{cycle+1}次清洗添加{final_volume}mL溶剂后")
            
        except Exception as e:
            debug_print(f"    ❌ 转移失败: {str(e)} 😞")
        
        # 2. 搅拌（如果需要）
        if stir and final_time > 0:
            debug_print(f"    🌪️ 搅拌: {final_time}s @ {stir_speed}RPM")
            stir_action = {
                "device_id": "stirrer_1",
                "action_name": "stir",
                "action_kwargs": {
                    "vessel": vessel_id,  # 🔧 使用 vessel_id
                    "time": str(time),
                    "stir_time": final_time,
                    "stir_speed": stir_speed,
                    "settling_time": 10.0  # 🕐 缩短沉降时间
                }
            }
            action_sequence.append(stir_action)
            debug_print(f"    ✅ 搅拌动作: {final_time}s, {stir_speed}RPM 🌪️")
        
        # 3. 过滤
        debug_print(f"    🌊 过滤到: {actual_filtrate_vessel}")
        filter_action = {
            "device_id": "filter_1",
            "action_name": "filter",
            "action_kwargs": {
                "vessel": vessel_id,  # 🔧 使用 vessel_id
                "filtrate_vessel": actual_filtrate_vessel,
                "temp": temp,
                "volume": final_volume
            }
        }
        action_sequence.append(filter_action)
        debug_print(f"    ✅ 过滤动作: → {actual_filtrate_vessel} 🌊")
        
        # 🔧 新增：更新体积 - 过滤后（液体被滤除）
        # 假设滤液完全被移除，固体残留在容器中
        filtered_volume = current_volume * 0.9  # 假设90%的液体被过滤掉
        current_volume = current_volume - filtered_volume
        
        if isinstance(vessel, dict):
            update_vessel_volume(vessel, G, current_volume, 
                               f"第{cycle+1}次清洗过滤后")
        
        # 4. 等待（缩短时间）
        wait_time = 5.0  # 🕐 缩短等待时间：10s → 5s
        action_sequence.append({
            "action_name": "wait",
            "action_kwargs": {"time": wait_time}
        })
        debug_print(f"    ✅ 等待: {wait_time}s ⏰")
    
    # 🔧 新增：清洗完成后的最终状态报告
    if isinstance(vessel, dict):
        final_volume_vessel = get_vessel_liquid_volume(vessel)
    else:
        final_volume_vessel = current_volume
    
    # 🎊 总结
    debug_print("🧼" * 20)
    debug_print(f"🎉 固体清洗协议生成完成! ✨")
    debug_print(f"📊 协议统计:")
    debug_print(f"  📋 总动作数: {len(action_sequence)} 个")
    debug_print(f"  🥽 清洗容器: {vessel_display}")
    debug_print(f"  🧪 使用溶剂: {solvent}")
    debug_print(f"  💧 单次体积: {final_volume}mL")
    debug_print(f"  🔄 清洗次数: {final_repeats}次")
    debug_print(f"  💧 总溶剂用量: {total_solvent_used:.2f}mL")
    debug_print(f"📊 体积变化统计:")
    debug_print(f"  - 清洗前体积: {original_volume:.2f}mL")
    debug_print(f"  - 清洗后体积: {final_volume_vessel:.2f}mL")
    debug_print(f"  - 溶剂总用量: {total_solvent_used:.2f}mL")
    debug_print(f"⏱️ 预计总时间: {(final_time + 5) * final_repeats / 60:.1f} 分钟")
    debug_print("🧼" * 20)
    
    return action_sequence

# 🔧 新增：便捷函数
def wash_with_water(G: nx.DiGraph, vessel: Union[str, dict], 
                   volume: Union[float, str] = "50", 
                   repeats: int = 2) -> List[Dict[str, Any]]:
    """用水清洗固体"""
    vessel_display = get_vessel_display_info(vessel)
    debug_print(f"💧 水洗固体: {vessel_display} ({repeats} 次)")
    return generate_wash_solid_protocol(G, vessel, "water", volume=volume, repeats=repeats)

def wash_with_ethanol(G: nx.DiGraph, vessel: Union[str, dict], 
                     volume: Union[float, str] = "30", 
                     repeats: int = 1) -> List[Dict[str, Any]]:
    """用乙醇清洗固体"""
    vessel_display = get_vessel_display_info(vessel)
    debug_print(f"🍺 乙醇洗固体: {vessel_display} ({repeats} 次)")
    return generate_wash_solid_protocol(G, vessel, "ethanol", volume=volume, repeats=repeats)

def wash_with_acetone(G: nx.DiGraph, vessel: Union[str, dict], 
                     volume: Union[float, str] = "25", 
                     repeats: int = 1) -> List[Dict[str, Any]]:
    """用丙酮清洗固体"""
    vessel_display = get_vessel_display_info(vessel)
    debug_print(f"💨 丙酮洗固体: {vessel_display} ({repeats} 次)")
    return generate_wash_solid_protocol(G, vessel, "acetone", volume=volume, repeats=repeats)

def wash_with_ether(G: nx.DiGraph, vessel: Union[str, dict], 
                   volume: Union[float, str] = "40", 
                   repeats: int = 2) -> List[Dict[str, Any]]:
    """用乙醚清洗固体"""
    vessel_display = get_vessel_display_info(vessel)
    debug_print(f"🌬️ 乙醚洗固体: {vessel_display} ({repeats} 次)")
    return generate_wash_solid_protocol(G, vessel, "diethyl_ether", volume=volume, repeats=repeats)

def wash_with_cold_solvent(G: nx.DiGraph, vessel: Union[str, dict], 
                          solvent: str, volume: Union[float, str] = "30", 
                          repeats: int = 1) -> List[Dict[str, Any]]:
    """用冷溶剂清洗固体"""
    vessel_display = get_vessel_display_info(vessel)
    debug_print(f"❄️ 冷{solvent}洗固体: {vessel_display} ({repeats} 次)")
    return generate_wash_solid_protocol(G, vessel, solvent, volume=volume, 
                                      temp=5.0, repeats=repeats)

def wash_with_hot_solvent(G: nx.DiGraph, vessel: Union[str, dict], 
                         solvent: str, volume: Union[float, str] = "50", 
                         repeats: int = 1) -> List[Dict[str, Any]]:
    """用热溶剂清洗固体"""
    vessel_display = get_vessel_display_info(vessel)
    debug_print(f"🔥 热{solvent}洗固体: {vessel_display} ({repeats} 次)")
    return generate_wash_solid_protocol(G, vessel, solvent, volume=volume, 
                                      temp=60.0, repeats=repeats)

def wash_with_stirring(G: nx.DiGraph, vessel: Union[str, dict], 
                      solvent: str, volume: Union[float, str] = "50", 
                      stir_time: Union[str, float] = "5 min", 
                      repeats: int = 1) -> List[Dict[str, Any]]:
    """带搅拌的溶剂清洗"""
    vessel_display = get_vessel_display_info(vessel)
    debug_print(f"🌪️ 搅拌清洗: {vessel_display} with {solvent} ({repeats} 次)")
    return generate_wash_solid_protocol(G, vessel, solvent, volume=volume, 
                                      stir=True, stir_speed=200.0, 
                                      time=stir_time, repeats=repeats)

def thorough_wash(G: nx.DiGraph, vessel: Union[str, dict], 
                 solvent: str, volume: Union[float, str] = "50") -> List[Dict[str, Any]]:
    """彻底清洗（多次重复）"""
    vessel_display = get_vessel_display_info(vessel)
    debug_print(f"🔄 彻底清洗: {vessel_display} with {solvent} (5 次)")
    return generate_wash_solid_protocol(G, vessel, solvent, volume=volume, repeats=5)

def quick_rinse(G: nx.DiGraph, vessel: Union[str, dict], 
               solvent: str, volume: Union[float, str] = "20") -> List[Dict[str, Any]]:
    """快速冲洗（单次，小体积）"""
    vessel_display = get_vessel_display_info(vessel)
    debug_print(f"⚡ 快速冲洗: {vessel_display} with {solvent}")
    return generate_wash_solid_protocol(G, vessel, solvent, volume=volume, repeats=1)

def sequential_wash(G: nx.DiGraph, vessel: Union[str, dict], 
                   solvents: list, volume: Union[float, str] = "40") -> List[Dict[str, Any]]:
    """连续多溶剂清洗"""
    vessel_display = get_vessel_display_info(vessel)
    debug_print(f"📝 连续清洗: {vessel_display} with {' → '.join(solvents)}")
    
    action_sequence = []
    for solvent in solvents:
        wash_actions = generate_wash_solid_protocol(G, vessel, solvent, 
                                                   volume=volume, repeats=1)
        action_sequence.extend(wash_actions)
    
    return action_sequence

# 测试函数
def test_wash_solid_protocol():
    """测试固体清洗协议"""
    debug_print("🧪 === WASH SOLID PROTOCOL 测试 === ✨")
    
    # 测试vessel参数处理
    debug_print("🔧 测试vessel参数处理...")
    
    # 测试字典格式
    vessel_dict = {"id": "filter_flask_1", "name": "过滤瓶1", 
                  "data": {"liquid_volume": 25.0}}
    vessel_id = extract_vessel_id(vessel_dict)
    vessel_display = get_vessel_display_info(vessel_dict)
    volume = get_vessel_liquid_volume(vessel_dict)
    debug_print(f"  字典格式: {vessel_dict}")
    debug_print(f"    → ID: {vessel_id}, 显示: {vessel_display}, 体积: {volume}mL")
    
    # 测试字符串格式
    vessel_str = "filter_flask_2"
    vessel_id = extract_vessel_id(vessel_str)
    vessel_display = get_vessel_display_info(vessel_str)
    debug_print(f"  字符串格式: {vessel_str}")
    debug_print(f"    → ID: {vessel_id}, 显示: {vessel_display}")
    
    debug_print("✅ 测试完成 🎉")

if __name__ == "__main__":
    test_wash_solid_protocol()