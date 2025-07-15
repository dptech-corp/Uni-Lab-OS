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

def generate_wash_solid_protocol(
    G: nx.DiGraph,
    vessel: str,
    solvent: str,
    volume: Union[float, str] = "50",
    filtrate_vessel: str = "",
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
    生成固体清洗协议（精简版）
    """
    
    debug_print("🧼" * 20)
    debug_print("🚀 开始生成固体清洗协议 ✨")
    debug_print(f"📝 输入参数:")
    debug_print(f"  🥽 vessel: {vessel}")
    debug_print(f"  🧪 solvent: {solvent}")
    debug_print(f"  💧 volume: {volume}")
    debug_print(f"  ⏰ time: {time}")
    debug_print(f"  🔄 repeats: {repeats}")
    debug_print("🧼" * 20)
    
    # 📋 快速验证
    if not vessel or vessel not in G.nodes():
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
        actual_filtrate_vessel = find_filtrate_vessel(G, filtrate_vessel)
        debug_print(f"🎉 设备配置完成 ✨")
    except Exception as e:
        debug_print(f"❌ 设备查找失败: {str(e)} 😭")
        raise ValueError(f"设备查找失败: {str(e)}")
    
    # 🚀 生成动作序列
    debug_print("📍 步骤3: 生成清洗动作... 🧼")
    action_sequence = []
    
    for cycle in range(final_repeats):
        debug_print(f"  🔄 第{cycle+1}/{final_repeats}次清洗...")
        
        # 1. 转移溶剂
        try:
            from .pump_protocol import generate_pump_protocol_with_rinsing
            
            transfer_actions = generate_pump_protocol_with_rinsing(
                G=G,
                from_vessel=solvent_source,
                to_vessel=vessel,
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
            
        except Exception as e:
            debug_print(f"    ❌ 转移失败: {str(e)} 😞")
        
        # 2. 搅拌（如果需要）
        if stir and final_time > 0:
            stir_action = {
                "device_id": "stirrer_1",
                "action_name": "stir",
                "action_kwargs": {
                    "vessel": vessel,
                    "time": str(time),
                    "stir_time": final_time,
                    "stir_speed": stir_speed,
                    "settling_time": 10.0  # 🕐 缩短沉降时间
                }
            }
            action_sequence.append(stir_action)
            debug_print(f"    ✅ 搅拌动作: {final_time}s, {stir_speed}RPM 🌪️")
        
        # 3. 过滤
        filter_action = {
            "device_id": "filter_1",
            "action_name": "filter",
            "action_kwargs": {
                "vessel": vessel,
                "filtrate_vessel": actual_filtrate_vessel,
                "temp": temp,
                "volume": final_volume
            }
        }
        action_sequence.append(filter_action)
        debug_print(f"    ✅ 过滤动作: → {actual_filtrate_vessel} 🌊")
        
        # 4. 等待（缩短时间）
        wait_time = 5.0  # 🕐 缩短等待时间：10s → 5s
        action_sequence.append({
            "action_name": "wait",
            "action_kwargs": {"time": wait_time}
        })
        debug_print(f"    ✅ 等待: {wait_time}s ⏰")
    
    # 🎊 总结
    debug_print("🧼" * 20)
    debug_print(f"🎉 固体清洗协议生成完成! ✨")
    debug_print(f"📊 总动作数: {len(action_sequence)} 个")
    debug_print(f"🥽 清洗容器: {vessel}")
    debug_print(f"🧪 使用溶剂: {solvent}")
    debug_print(f"💧 清洗体积: {final_volume}mL × {final_repeats}次")
    debug_print(f"⏱️ 预计总时间: {(final_time + 5) * final_repeats / 60:.1f} 分钟")
    debug_print("🧼" * 20)
    
    return action_sequence