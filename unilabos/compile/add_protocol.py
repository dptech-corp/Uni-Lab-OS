from functools import partial

import networkx as nx
import re
import logging
from typing import List, Dict, Any, Union

from .utils.unit_parser import parse_volume_input, parse_mass_input, parse_time_input
from .utils.vessel_parser import get_vessel, find_solid_dispenser, find_connected_stirrer, find_reagent_vessel
from .utils.logger_util import action_log
from .pump_protocol import generate_pump_protocol_with_rinsing

logger = logging.getLogger(__name__)

def debug_print(message):
    """调试输出"""
    logger.info(f"[ADD] {message}")


# 🆕 创建进度日志动作
create_action_log = partial(action_log, prefix="[ADD]")

def generate_add_protocol(
    G: nx.DiGraph,
    vessel: dict,  # 🔧 修改：现在接收字典类型的 vessel
    reagent: str,
    # 🔧 修复：所有参数都用 Union 类型，支持字符串和数值
    volume: Union[str, float] = 0.0,
    mass: Union[str, float] = 0.0,
    amount: str = "",
    time: Union[str, float] = 0.0,
    stir: bool = False,
    stir_speed: float = 300.0,
    viscous: bool = False,
    purpose: str = "添加试剂",
    # XDL扩展参数
    mol: str = "",
    event: str = "",
    rate_spec: str = "",
    equiv: str = "",
    ratio: str = "",
    **kwargs
) -> List[Dict[str, Any]]:
    """
    生成添加试剂协议 - 修复版
    
    支持所有XDL参数和单位：
    - vessel: Resource类型字典，包含id字段
    - volume: "2.7 mL", "2.67 mL", "?" 或数值
    - mass: "19.3 g", "4.5 g" 或数值
    - time: "1 h", "20 min" 或数值（秒）
    - mol: "0.28 mol", "16.2 mmol", "25.2 mmol"
    - rate_spec: "portionwise", "dropwise"
    - event: "A", "B"
    - equiv: "1.1"
    - ratio: "?", "1:1"
    """

    # 🔧 核心修改：从字典中提取容器ID
    vessel_id, vessel_data = get_vessel(vessel)
    
    # 🔧 修改：更新容器的液体体积（假设有 liquid_volume 字段）
    if "data" in vessel and "liquid_volume" in vessel["data"]:
        if isinstance(vessel["data"]["liquid_volume"], list) and len(vessel["data"]["liquid_volume"]) > 0:
            vessel["data"]["liquid_volume"][0] -= parse_volume_input(volume)
    
    debug_print("=" * 60)
    debug_print("🚀 开始生成添加试剂协议")
    debug_print(f"📋 原始参数:")
    debug_print(f"  🥼 vessel: {vessel} (ID: {vessel_id})")
    debug_print(f"  🧪 reagent: '{reagent}'")
    debug_print(f"  📏 volume: {volume} (类型: {type(volume)})")
    debug_print(f"  ⚖️ mass: {mass} (类型: {type(mass)})")
    debug_print(f"  ⏱️ time: {time} (类型: {type(time)})")
    debug_print(f"  🧬 mol: '{mol}'")
    debug_print(f"  🎯 event: '{event}'")
    debug_print(f"  ⚡ rate_spec: '{rate_spec}'")
    debug_print(f"  🌪️ stir: {stir}")
    debug_print(f"  🔄 stir_speed: {stir_speed} rpm")
    debug_print("=" * 60)
    
    action_sequence = []
    
    # === 参数验证 ===
    debug_print("🔍 步骤1: 参数验证...")
    action_sequence.append(create_action_log(f"开始添加试剂 '{reagent}' 到容器 '{vessel_id}'", "🎬"))
    
    if not vessel or not vessel_id:
        debug_print("❌ vessel 参数不能为空")
        raise ValueError("vessel 参数不能为空")
    if not reagent:
        debug_print("❌ reagent 参数不能为空")
        raise ValueError("reagent 参数不能为空")
    
    if vessel_id not in G.nodes():
        debug_print(f"❌ 容器 '{vessel_id}' 不存在于系统中")
        raise ValueError(f"容器 '{vessel_id}' 不存在于系统中")
    
    debug_print("✅ 基本参数验证通过")
    
    # === 🔧 关键修复：参数解析 ===
    debug_print("🔍 步骤2: 参数解析...")
    action_sequence.append(create_action_log("正在解析添加参数...", "🔍"))
    
    # 解析各种参数为数值
    final_volume = parse_volume_input(volume)
    final_mass = parse_mass_input(mass)
    final_time = parse_time_input(time)
    
    debug_print(f"📊 解析结果:")
    debug_print(f"  体积: {final_volume}mL, 质量: {final_mass}g, 时间: {final_time}s, 摩尔: '{mol}', 事件: '{event}', 速率: '{rate_spec}'")
    
    # === 判断添加类型 ===
    debug_print("🔍 步骤3: 判断添加类型...")
    
    # 🔧 修复：现在使用解析后的数值进行比较
    is_solid = (final_mass > 0 or (mol and mol.strip() != ""))
    is_liquid = (final_volume > 0)
    
    if not is_solid and not is_liquid:
        # 默认为液体，10mL
        is_liquid = True
        final_volume = 10.0
        debug_print("⚠️ 未指定体积或质量，默认为10mL液体")
    
    add_type = "固体" if is_solid else "液体"
    add_emoji = "🧂" if is_solid else "💧"
    debug_print(f"📋 添加类型: {add_type} {add_emoji}")
    
    action_sequence.append(create_action_log(f"确定添加类型: {add_type} {add_emoji}", "📋"))
    
    # === 执行添加流程 ===
    debug_print("🔍 步骤4: 执行添加流程...")
    
    try:
        if is_solid:
            # === 固体添加路径 ===
            debug_print(f"🧂 使用固体添加路径")
            action_sequence.append(create_action_log("开始固体试剂添加流程", "🧂"))
            
            solid_dispenser = find_solid_dispenser(G)
            if solid_dispenser:
                action_sequence.append(create_action_log(f"找到固体加样器: {solid_dispenser}", "🥄"))
                
                # 启动搅拌
                if stir:
                    debug_print("🌪️ 准备启动搅拌...")
                    action_sequence.append(create_action_log("准备启动搅拌器", "🌪️"))
                    
                    stirrer_id = find_connected_stirrer(G, vessel_id)  # 🔧 使用 vessel_id
                    if stirrer_id:
                        action_sequence.append(create_action_log(f"启动搅拌器 {stirrer_id} (速度: {stir_speed} rpm)", "🔄"))
                        
                        action_sequence.append({
                            "device_id": stirrer_id,
                            "action_name": "start_stir",
                            "action_kwargs": {
                                "vessel": {"id": vessel_id},  # 🔧 使用 vessel_id
                                "stir_speed": stir_speed,
                                "purpose": f"准备添加固体 {reagent}"
                            }
                        })
                        # 等待搅拌稳定
                        action_sequence.append(create_action_log("等待搅拌稳定...", "⏳"))
                        action_sequence.append({
                            "action_name": "wait",
                            "action_kwargs": {"time": 3}
                        })
                
                # 固体加样
                add_kwargs = {
                    "vessel": {"id": vessel_id},  # 🔧 使用 vessel_id
                    "reagent": reagent,
                    "purpose": purpose,
                    "event": event,
                    "rate_spec": rate_spec
                }
                
                if final_mass > 0:
                    add_kwargs["mass"] = str(final_mass)
                    action_sequence.append(create_action_log(f"准备添加固体: {final_mass}g", "⚖️"))
                if mol and mol.strip():
                    add_kwargs["mol"] = mol
                    action_sequence.append(create_action_log(f"按摩尔数添加: {mol}", "🧬"))
                if equiv and equiv.strip():
                    add_kwargs["equiv"] = equiv
                    action_sequence.append(create_action_log(f"当量: {equiv}", "🔢"))
                
                action_sequence.append(create_action_log("开始固体加样操作", "🥄"))
                action_sequence.append({
                    "device_id": solid_dispenser,
                    "action_name": "add_solid",
                    "action_kwargs": add_kwargs
                })
                
                action_sequence.append(create_action_log("固体加样完成", "✅"))
                
                # 添加后等待
                if final_time > 0:
                    wait_minutes = final_time / 60
                    action_sequence.append(create_action_log(f"等待反应进行 ({wait_minutes:.1f}分钟)", "⏰"))
                    action_sequence.append({
                        "action_name": "wait",
                        "action_kwargs": {"time": final_time}
                    })
                    
                debug_print(f"✅ 固体添加完成")
            else:
                debug_print("❌ 未找到固体加样器，跳过固体添加")
                action_sequence.append(create_action_log("未找到固体加样器，无法添加固体", "❌"))
        
        else:
            # === 液体添加路径 ===
            debug_print(f"💧 使用液体添加路径")
            action_sequence.append(create_action_log("开始液体试剂添加流程", "💧"))
            
            # 查找试剂容器
            action_sequence.append(create_action_log("正在查找试剂容器...", "🔍"))
            reagent_vessel = find_reagent_vessel(G, reagent)
            action_sequence.append(create_action_log(f"找到试剂容器: {reagent_vessel}", "🧪"))
            
            # 启动搅拌
            if stir:
                debug_print("🌪️ 准备启动搅拌...")
                action_sequence.append(create_action_log("准备启动搅拌器", "🌪️"))
                
                stirrer_id = find_connected_stirrer(G, vessel_id)  # 🔧 使用 vessel_id
                if stirrer_id:
                    action_sequence.append(create_action_log(f"启动搅拌器 {stirrer_id} (速度: {stir_speed} rpm)", "🔄"))
                    
                    action_sequence.append({
                        "device_id": stirrer_id,
                        "action_name": "start_stir",
                        "action_kwargs": {
                            "vessel": {"id": vessel_id},  # 🔧 使用 vessel_id
                            "stir_speed": stir_speed,
                            "purpose": f"准备添加液体 {reagent}"
                        }
                    })
                    # 等待搅拌稳定
                    action_sequence.append(create_action_log("等待搅拌稳定...", "⏳"))
                    action_sequence.append({
                        "action_name": "wait",
                        "action_kwargs": {"time": 5}
                    })
            
            # 计算流速
            if final_time > 0:
                flowrate = final_volume / final_time * 60  # mL/min
                transfer_flowrate = flowrate
                debug_print(f"⚡ 根据时间计算流速: {flowrate:.2f} mL/min")
            else:
                if rate_spec == "dropwise":
                    flowrate = 0.5  # 滴加，很慢
                    transfer_flowrate = 0.2
                    debug_print(f"💧 滴加模式，流速: {flowrate} mL/min")
                elif viscous:
                    flowrate = 1.0  # 粘性液体
                    transfer_flowrate = 0.3
                    debug_print(f"🍯 粘性液体，流速: {flowrate} mL/min")
                else:
                    flowrate = 2.5  # 正常流速
                    transfer_flowrate = 0.5
                    debug_print(f"⚡ 正常流速: {flowrate} mL/min")
            
            action_sequence.append(create_action_log(f"设置流速: {flowrate:.2f} mL/min", "⚡"))
            action_sequence.append(create_action_log(f"开始转移 {final_volume}mL 液体", "🚰"))
            
            # 调用pump protocol
            pump_actions = generate_pump_protocol_with_rinsing(
                G=G,
                from_vessel=reagent_vessel,
                to_vessel=vessel_id,  # 🔧 使用 vessel_id
                volume=final_volume,
                amount=amount,
                time=final_time,
                viscous=viscous,
                rinsing_solvent="",
                rinsing_volume=0.0,
                rinsing_repeats=0,
                solid=False,
                flowrate=flowrate,
                transfer_flowrate=transfer_flowrate,
                rate_spec=rate_spec,
                event=event,
                through="",
                **kwargs
            )
            action_sequence.extend(pump_actions)
            debug_print(f"✅ 液体转移完成，添加了 {len(pump_actions)} 个动作")
            action_sequence.append(create_action_log(f"液体转移完成 ({len(pump_actions)} 个操作)", "✅"))
            
    except Exception as e:
        debug_print(f"❌ 试剂添加失败: {str(e)}")
        action_sequence.append(create_action_log(f"试剂添加失败: {str(e)}", "❌"))
        # 添加错误日志
        action_sequence.append({
            "device_id": "system",
            "action_name": "log_message",
            "action_kwargs": {
                "message": f"试剂 '{reagent}' 添加失败: {str(e)}"
            }
        })
    
    # === 最终结果 ===
    debug_print("=" * 60)
    debug_print(f"🎉 添加试剂协议生成完成")
    debug_print(f"📊 总动作数: {len(action_sequence)}")
    debug_print(f"📋 处理总结:")
    debug_print(f"  🧪 试剂: {reagent}")
    debug_print(f"  {add_emoji} 添加类型: {add_type}")
    debug_print(f"  🥼 目标容器: {vessel_id}")
    if is_liquid:
        debug_print(f"  📏 体积: {final_volume}mL")
    if is_solid:
        debug_print(f"  ⚖️ 质量: {final_mass}g")
        debug_print(f"  🧬 摩尔: {mol}")
    debug_print("=" * 60)
    
    # 添加完成日志
    summary_msg = f"试剂添加协议完成: {reagent} → {vessel_id}"
    if is_liquid:
        summary_msg += f" ({final_volume}mL)"
    if is_solid:
        summary_msg += f" ({final_mass}g)"
    
    action_sequence.append(create_action_log(summary_msg, "🎉"))
    
    return action_sequence

# === 便捷函数 ===
# 🔧 修改便捷函数的参数类型

def add_liquid_volume(G: nx.DiGraph, vessel: dict, reagent: str, volume: Union[str, float], 
                     time: Union[str, float] = 0.0, rate_spec: str = "") -> List[Dict[str, Any]]:
    """添加指定体积的液体试剂"""
    vessel_id = vessel["id"]
    debug_print(f"💧 快速添加液体: {reagent} ({volume}) → {vessel_id}")
    return generate_add_protocol(
        G, vessel, reagent, 
        volume=volume, 
        time=time, 
        rate_spec=rate_spec
    )

def add_solid_mass(G: nx.DiGraph, vessel: dict, reagent: str, mass: Union[str, float], 
                   event: str = "") -> List[Dict[str, Any]]:
    """添加指定质量的固体试剂"""
    vessel_id = vessel["id"]
    debug_print(f"🧂 快速添加固体: {reagent} ({mass}) → {vessel_id}")
    return generate_add_protocol(
        G, vessel, reagent, 
        mass=mass, 
        event=event
    )

def add_solid_moles(G: nx.DiGraph, vessel: dict, reagent: str, mol: str, 
                    event: str = "") -> List[Dict[str, Any]]:
    """按摩尔数添加固体试剂"""
    vessel_id = vessel["id"]
    debug_print(f"🧬 按摩尔数添加固体: {reagent} ({mol}) → {vessel_id}")
    return generate_add_protocol(
        G, vessel, reagent, 
        mol=mol, 
        event=event
    )

def add_dropwise_liquid(G: nx.DiGraph, vessel: dict, reagent: str, volume: Union[str, float], 
                        time: Union[str, float] = "20 min", event: str = "") -> List[Dict[str, Any]]:
    """滴加液体试剂"""
    vessel_id = vessel["id"]
    debug_print(f"💧 滴加液体: {reagent} ({volume}) → {vessel_id} (用时: {time})")
    return generate_add_protocol(
        G, vessel, reagent, 
        volume=volume, 
        time=time, 
        rate_spec="dropwise", 
        event=event
    )

def add_portionwise_solid(G: nx.DiGraph, vessel: dict, reagent: str, mass: Union[str, float], 
                          time: Union[str, float] = "1 h", event: str = "") -> List[Dict[str, Any]]:
    """分批添加固体试剂"""
    vessel_id = vessel["id"]
    debug_print(f"🧂 分批添加固体: {reagent} ({mass}) → {vessel_id} (用时: {time})")
    return generate_add_protocol(
        G, vessel, reagent, 
        mass=mass, 
        time=time, 
        rate_spec="portionwise", 
        event=event
    )

# 测试函数
def test_add_protocol():
    """测试添加协议的各种参数解析"""
    print("=== ADD PROTOCOL 增强版测试 ===")
    
    # 测试体积解析
    debug_print("🧪 测试体积解析...")
    volumes = ["2.7 mL", "2.67 mL", "?", 10.0, "1 L", "500 μL"]
    for vol in volumes:
        result = parse_volume_input(vol)
        print(f"📏 体积解析: {vol} → {result}mL")
    
    # 测试质量解析
    debug_print("⚖️ 测试质量解析...")
    masses = ["19.3 g", "4.5 g", 2.5, "500 mg", "1 kg"]
    for mass in masses:
        result = parse_mass_input(mass)
        print(f"⚖️ 质量解析: {mass} → {result}g")
    
    # 测试时间解析
    debug_print("⏱️ 测试时间解析...")
    times = ["1 h", "20 min", "30 s", 60.0, "?"]
    for time in times:
        result = parse_time_input(time)
        print(f"⏱️ 时间解析: {time} → {result}s")
    
    print("✅ 测试完成")

if __name__ == "__main__":
    test_add_protocol()