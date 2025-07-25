import networkx as nx
import re
import logging
from typing import List, Dict, Any, Tuple, Union
from .pump_protocol import generate_pump_protocol_with_rinsing

logger = logging.getLogger(__name__)

def debug_print(message):
    """调试输出"""
    print(f"💎 [RECRYSTALLIZE] {message}", flush=True)
    logger.info(f"[RECRYSTALLIZE] {message}")


def parse_volume_with_units(volume_input: Union[str, float, int], default_unit: str = "mL") -> float:
    """
    解析带单位的体积输入
    
    Args:
        volume_input: 体积输入（如 "100 mL", "2.5 L", "500", "?", 100.0）
        default_unit: 默认单位（默认为毫升）
    
    Returns:
        float: 体积（毫升）
    """
    if not volume_input:
        debug_print("⚠️ 体积输入为空，返回 0.0mL 📦")
        return 0.0
    
    # 处理数值输入
    if isinstance(volume_input, (int, float)):
        result = float(volume_input)
        debug_print(f"🔢 数值体积输入: {volume_input} → {result}mL（默认单位）💧")
        return result
    
    # 处理字符串输入
    volume_str = str(volume_input).lower().strip()
    debug_print(f"🔍 解析体积字符串: '{volume_str}' 📝")
    
    # 处理特殊值
    if volume_str in ['?', 'unknown', 'tbd', 'to be determined']:
        default_volume = 50.0  # 50mL默认值
        debug_print(f"❓ 检测到未知体积，使用默认值: {default_volume}mL 🎯")
        return default_volume
    
    # 如果是纯数字，使用默认单位
    try:
        value = float(volume_str)
        if default_unit.lower() in ["ml", "milliliter"]:
            result = value
        elif default_unit.lower() in ["l", "liter"]:
            result = value * 1000.0
        elif default_unit.lower() in ["μl", "ul", "microliter"]:
            result = value / 1000.0
        else:
            result = value  # 默认mL
        debug_print(f"🔢 纯数字输入: {volume_str} → {result}mL（单位: {default_unit}）📏")
        return result
    except ValueError:
        pass
    
    # 移除空格并提取数字和单位
    volume_clean = re.sub(r'\s+', '', volume_str)
    
    # 匹配数字和单位的正则表达式
    match = re.match(r'([0-9]*\.?[0-9]+)\s*(ml|l|μl|ul|microliter|milliliter|liter)?', volume_clean)
    
    if not match:
        debug_print(f"⚠️ 无法解析体积: '{volume_str}'，使用默认值: 50mL 🎯")
        return 50.0
    
    value = float(match.group(1))
    unit = match.group(2) or default_unit.lower()
    
    # 转换为毫升
    if unit in ['l', 'liter']:
        volume = value * 1000.0  # L -> mL
        debug_print(f"📏 升转毫升: {value}L → {volume}mL 💧")
    elif unit in ['μl', 'ul', 'microliter']:
        volume = value / 1000.0  # μL -> mL
        debug_print(f"📏 微升转毫升: {value}μL → {volume}mL 💧")
    else:  # ml, milliliter 或默认
        volume = value  # 已经是mL
        debug_print(f"📏 毫升单位: {value}mL → {volume}mL 💧")
    
    debug_print(f"✅ 体积解析完成: '{volume_str}' → {volume}mL ✨")
    return volume


def parse_ratio(ratio_str: str) -> Tuple[float, float]:
    """
    解析比例字符串，支持多种格式
    
    Args:
        ratio_str: 比例字符串（如 "1:1", "3:7", "50:50"）
    
    Returns:
        Tuple[float, float]: 比例元组 (ratio1, ratio2)
    """
    debug_print(f"⚖️ 开始解析比例: '{ratio_str}' 📊")
    
    try:
        # 处理 "1:1", "3:7", "50:50" 等格式
        if ":" in ratio_str:
            parts = ratio_str.split(":")
            if len(parts) == 2:
                ratio1 = float(parts[0])
                ratio2 = float(parts[1])
                debug_print(f"✅ 冒号格式解析成功: {ratio1}:{ratio2} 🎯")
                return ratio1, ratio2
        
        # 处理 "1-1", "3-7" 等格式
        if "-" in ratio_str:
            parts = ratio_str.split("-")
            if len(parts) == 2:
                ratio1 = float(parts[0])
                ratio2 = float(parts[1])
                debug_print(f"✅ 横线格式解析成功: {ratio1}:{ratio2} 🎯")
                return ratio1, ratio2
        
        # 处理 "1,1", "3,7" 等格式
        if "," in ratio_str:
            parts = ratio_str.split(",")
            if len(parts) == 2:
                ratio1 = float(parts[0])
                ratio2 = float(parts[1])
                debug_print(f"✅ 逗号格式解析成功: {ratio1}:{ratio2} 🎯")
                return ratio1, ratio2
        
        # 默认 1:1
        debug_print(f"⚠️ 无法解析比例 '{ratio_str}'，使用默认比例 1:1 🎭")
        return 1.0, 1.0
    
    except ValueError:
        debug_print(f"❌ 比例解析错误 '{ratio_str}'，使用默认比例 1:1 🎭")
        return 1.0, 1.0


def find_solvent_vessel(G: nx.DiGraph, solvent: str) -> str:
    """
    查找溶剂容器
    
    Args:
        G: 网络图
        solvent: 溶剂名称
    
    Returns:
        str: 溶剂容器ID
    """
    debug_print(f"🔍 正在查找溶剂 '{solvent}' 的容器... 🧪")
    
    # 构建可能的容器名称
    possible_names = [
        f"flask_{solvent}",
        f"bottle_{solvent}",
        f"reagent_{solvent}",
        f"reagent_bottle_{solvent}",
        f"{solvent}_flask",
        f"{solvent}_bottle",
        f"{solvent}",
        f"vessel_{solvent}",
    ]
    
    debug_print(f"📋 候选容器名称: {possible_names[:3]}... (共{len(possible_names)}个) 📝")
    
    # 第一步：通过容器名称匹配
    debug_print("  🎯 步骤1: 精确名称匹配...")
    for vessel_name in possible_names:
        if vessel_name in G.nodes():
            debug_print(f"  🎉 通过名称匹配找到容器: {vessel_name} ✨")
            return vessel_name
    
    # 第二步：通过模糊匹配（节点ID和名称）
    debug_print("  🔍 步骤2: 模糊名称匹配...")
    for node_id in G.nodes():
        if G.nodes[node_id].get('type') == 'container':
            node_name = G.nodes[node_id].get('name', '').lower()
            
            if solvent.lower() in node_id.lower() or solvent.lower() in node_name:
                debug_print(f"  🎉 通过模糊匹配找到容器: {node_id} (名称: {node_name}) ✨")
                return node_id
    
    # 第三步：通过配置中的试剂信息匹配
    debug_print("  🧪 步骤3: 配置试剂信息匹配...")
    for node_id in G.nodes():
        if G.nodes[node_id].get('type') == 'container':
            # 检查 config 中的 reagent 字段
            node_config = G.nodes[node_id].get('config', {})
            config_reagent = node_config.get('reagent', '').lower()
            
            if config_reagent and solvent.lower() == config_reagent:
                debug_print(f"  🎉 通过config.reagent匹配找到容器: {node_id} (试剂: {config_reagent}) ✨")
                return node_id
    
    # 第四步：通过数据中的试剂信息匹配
    debug_print("  🧪 步骤4: 数据试剂信息匹配...")
    for node_id in G.nodes():
        if G.nodes[node_id].get('type') == 'container':
            vessel_data = G.nodes[node_id].get('data', {})
            
            # 检查 data 中的 reagent_name 字段
            reagent_name = vessel_data.get('reagent_name', '').lower()
            if reagent_name and solvent.lower() == reagent_name:
                debug_print(f"  🎉 通过data.reagent_name匹配找到容器: {node_id} (试剂: {reagent_name}) ✨")
                return node_id
            
            # 检查 data 中的液体信息
            liquids = vessel_data.get('liquid', [])
            for liquid in liquids:
                if isinstance(liquid, dict):
                    liquid_type = (liquid.get('liquid_type') or liquid.get('name', '')).lower()
                    
                    if solvent.lower() in liquid_type:
                        debug_print(f"  🎉 通过液体类型匹配找到容器: {node_id} (液体类型: {liquid_type}) ✨")
                        return node_id
    
    # 第五步：部分匹配（如果前面都没找到）
    debug_print("  🔍 步骤5: 部分匹配...")
    for node_id in G.nodes():
        if G.nodes[node_id].get('type') == 'container':
            node_config = G.nodes[node_id].get('config', {})
            node_data = G.nodes[node_id].get('data', {})
            node_name = G.nodes[node_id].get('name', '').lower()
            
            config_reagent = node_config.get('reagent', '').lower()
            data_reagent = node_data.get('reagent_name', '').lower()
            
            # 检查是否包含溶剂名称
            if (solvent.lower() in config_reagent or 
                solvent.lower() in data_reagent or 
                solvent.lower() in node_name or
                solvent.lower() in node_id.lower()):
                debug_print(f"  🎉 通过部分匹配找到容器: {node_id} ✨")
                debug_print(f"    - 节点名称: {node_name}")
                debug_print(f"    - 配置试剂: {config_reagent}")
                debug_print(f"    - 数据试剂: {data_reagent}")
                return node_id
    
    # 调试信息：列出所有容器
    debug_print("  🔎 调试信息：列出所有容器...")
    container_list = []
    for node_id in G.nodes():
        if G.nodes[node_id].get('type') == 'container':
            node_config = G.nodes[node_id].get('config', {})
            node_data = G.nodes[node_id].get('data', {})
            node_name = G.nodes[node_id].get('name', '')
            
            container_info = {
                'id': node_id,
                'name': node_name,
                'config_reagent': node_config.get('reagent', ''),
                'data_reagent': node_data.get('reagent_name', '')
            }
            container_list.append(container_info)
            debug_print(f"    - 容器: {node_id}, 名称: {node_name}, config试剂: {node_config.get('reagent', '')}, data试剂: {node_data.get('reagent_name', '')}")
    
    debug_print(f"❌ 找不到溶剂 '{solvent}' 对应的容器 😭")
    debug_print(f"🔍 查找的溶剂: '{solvent}' (小写: '{solvent.lower()}')")
    debug_print(f"📊 总共发现 {len(container_list)} 个容器")
    
    raise ValueError(f"找不到溶剂 '{solvent}' 对应的容器")


def generate_recrystallize_protocol(
    G: nx.DiGraph,
    vessel: dict,  # 🔧 修改：从字符串改为字典类型
    ratio: str,
    solvent1: str,
    solvent2: str,
    volume: Union[str, float],  # 支持字符串和数值
    **kwargs
) -> List[Dict[str, Any]]:
    """
    生成重结晶协议序列 - 支持vessel字典和体积运算
    
    Args:
        G: 有向图，节点为容器和设备
        vessel: 目标容器字典（从XDL传入）
        ratio: 溶剂比例（如 "1:1", "3:7"）
        solvent1: 第一种溶剂名称
        solvent2: 第二种溶剂名称
        volume: 总体积（支持 "100 mL", "50", "2.5 L" 等）
        **kwargs: 其他可选参数
    
    Returns:
        List[Dict[str, Any]]: 动作序列
    """
    
    # 🔧 核心修改：从字典中提取容器ID
    # 统一处理vessel参数
    if isinstance(vessel, dict):
        if "id" not in vessel:
            vessel_id = list(vessel.values())[0].get("id", "")
        else:
            vessel_id = vessel.get("id", "")
        vessel_data = vessel.get("data", {})
    else:
        vessel_id = str(vessel)
        vessel_data = G.nodes[vessel_id].get("data", {}) if vessel_id in G.nodes() else {}
    
    action_sequence = []
    
    debug_print("💎" * 20)
    debug_print("🚀 开始生成重结晶协议（支持vessel字典和体积运算）✨")
    debug_print(f"📝 输入参数:")
    debug_print(f"  🥽 vessel: {vessel} (ID: {vessel_id})")
    debug_print(f"  ⚖️ 比例: {ratio}")
    debug_print(f"  🧪 溶剂1: {solvent1}")
    debug_print(f"  🧪 溶剂2: {solvent2}")
    debug_print(f"  💧 总体积: {volume} (类型: {type(volume)})")
    debug_print("💎" * 20)
    
    # 🔧 新增：记录重结晶前的容器状态
    debug_print("🔍 记录重结晶前容器状态...")
    original_liquid_volume = 0.0
    if "data" in vessel and "liquid_volume" in vessel["data"]:
        current_volume = vessel["data"]["liquid_volume"]
        if isinstance(current_volume, list) and len(current_volume) > 0:
            original_liquid_volume = current_volume[0]
        elif isinstance(current_volume, (int, float)):
            original_liquid_volume = current_volume
    debug_print(f"📊 重结晶前液体体积: {original_liquid_volume:.2f}mL")
    
    # 1. 验证目标容器存在
    debug_print("📍 步骤1: 验证目标容器... 🔧")
    if vessel_id not in G.nodes():  # 🔧 使用 vessel_id
        debug_print(f"❌ 目标容器 '{vessel_id}' 不存在于系统中! 😱")
        raise ValueError(f"目标容器 '{vessel_id}' 不存在于系统中")
    debug_print(f"✅ 目标容器 '{vessel_id}' 验证通过 🎯")
    
    # 2. 解析体积（支持单位）
    debug_print("📍 步骤2: 解析体积（支持单位）... 💧")
    final_volume = parse_volume_with_units(volume, "mL")
    debug_print(f"🎯 体积解析完成: {volume} → {final_volume}mL ✨")
    
    # 3. 解析比例
    debug_print("📍 步骤3: 解析比例... ⚖️")
    ratio1, ratio2 = parse_ratio(ratio)
    total_ratio = ratio1 + ratio2
    debug_print(f"🎯 比例解析完成: {ratio1}:{ratio2} (总比例: {total_ratio}) ✨")
    
    # 4. 计算各溶剂体积
    debug_print("📍 步骤4: 计算各溶剂体积... 🧮")
    volume1 = final_volume * (ratio1 / total_ratio)
    volume2 = final_volume * (ratio2 / total_ratio)
    
    debug_print(f"🧪 {solvent1} 体积: {volume1:.2f} mL ({ratio1}/{total_ratio} × {final_volume})")
    debug_print(f"🧪 {solvent2} 体积: {volume2:.2f} mL ({ratio2}/{total_ratio} × {final_volume})")
    debug_print(f"✅ 体积计算完成: 总计 {volume1 + volume2:.2f} mL 🎯")
    
    # 5. 查找溶剂容器
    debug_print("📍 步骤5: 查找溶剂容器... 🔍")
    try:
        debug_print(f"  🔍 查找溶剂1容器...")
        solvent1_vessel = find_solvent_vessel(G, solvent1)
        debug_print(f"  🎉 找到溶剂1容器: {solvent1_vessel} ✨")
    except ValueError as e:
        debug_print(f"  ❌ 溶剂1容器查找失败: {str(e)} 😭")
        raise ValueError(f"无法找到溶剂1 '{solvent1}': {str(e)}")
    
    try:
        debug_print(f"  🔍 查找溶剂2容器...")
        solvent2_vessel = find_solvent_vessel(G, solvent2)
        debug_print(f"  🎉 找到溶剂2容器: {solvent2_vessel} ✨")
    except ValueError as e:
        debug_print(f"  ❌ 溶剂2容器查找失败: {str(e)} 😭")
        raise ValueError(f"无法找到溶剂2 '{solvent2}': {str(e)}")
    
    # 6. 验证路径存在
    debug_print("📍 步骤6: 验证传输路径... 🛤️")
    try:
        path1 = nx.shortest_path(G, source=solvent1_vessel, target=vessel_id)  # 🔧 使用 vessel_id
        debug_print(f"  🛤️ 溶剂1路径: {' → '.join(path1)} ✅")
    except nx.NetworkXNoPath:
        debug_print(f"  ❌ 溶剂1路径不可达: {solvent1_vessel} → {vessel_id} 😞")
        raise ValueError(f"从溶剂1容器 '{solvent1_vessel}' 到目标容器 '{vessel_id}' 没有可用路径")
    
    try:
        path2 = nx.shortest_path(G, source=solvent2_vessel, target=vessel_id)  # 🔧 使用 vessel_id
        debug_print(f"  🛤️ 溶剂2路径: {' → '.join(path2)} ✅")
    except nx.NetworkXNoPath:
        debug_print(f"  ❌ 溶剂2路径不可达: {solvent2_vessel} → {vessel_id} 😞")
        raise ValueError(f"从溶剂2容器 '{solvent2_vessel}' 到目标容器 '{vessel_id}' 没有可用路径")
    
    # 7. 添加第一种溶剂
    debug_print("📍 步骤7: 添加第一种溶剂... 🧪")
    debug_print(f"  🚰 开始添加溶剂1: {solvent1} ({volume1:.2f} mL)")
    
    try:
        pump_actions1 = generate_pump_protocol_with_rinsing(
            G=G,
            from_vessel=solvent1_vessel,
            to_vessel=vessel_id,  # 🔧 使用 vessel_id
            volume=volume1,             # 使用解析后的体积
            amount="",
            time=0.0,
            viscous=False,
            rinsing_solvent="",  # 重结晶不需要清洗
            rinsing_volume=0.0,
            rinsing_repeats=0,
            solid=False,
            flowrate=2.0,  # 正常流速
            transfer_flowrate=0.5
        )
        
        action_sequence.extend(pump_actions1)
        debug_print(f"  ✅ 溶剂1泵送动作已添加: {len(pump_actions1)} 个动作 🚰✨")
        
    except Exception as e:
        debug_print(f"  ❌ 溶剂1泵协议生成失败: {str(e)} 😭")
        raise ValueError(f"生成溶剂1泵协议时出错: {str(e)}")
    
    # 🔧 新增：更新容器体积 - 添加溶剂1后
    debug_print("  🔧 更新容器体积 - 添加溶剂1后...")
    new_volume_after_solvent1 = original_liquid_volume + volume1
    
    # 更新vessel字典中的体积
    if "data" in vessel and "liquid_volume" in vessel["data"]:
        current_volume = vessel["data"]["liquid_volume"]
        if isinstance(current_volume, list):
            if len(current_volume) > 0:
                vessel["data"]["liquid_volume"][0] = new_volume_after_solvent1
            else:
                vessel["data"]["liquid_volume"] = [new_volume_after_solvent1]
        else:
            vessel["data"]["liquid_volume"] = new_volume_after_solvent1
    
    # 同时更新图中的容器数据
    if vessel_id in G.nodes():
        if 'data' not in G.nodes[vessel_id]:
            G.nodes[vessel_id]['data'] = {}
        
        vessel_node_data = G.nodes[vessel_id]['data']
        current_node_volume = vessel_node_data.get('liquid_volume', 0.0)
        
        if isinstance(current_node_volume, list):
            if len(current_node_volume) > 0:
                G.nodes[vessel_id]['data']['liquid_volume'][0] = new_volume_after_solvent1
            else:
                G.nodes[vessel_id]['data']['liquid_volume'] = [new_volume_after_solvent1]
        else:
            G.nodes[vessel_id]['data']['liquid_volume'] = new_volume_after_solvent1
    
    debug_print(f"  📊 体积更新: {original_liquid_volume:.2f}mL + {volume1:.2f}mL = {new_volume_after_solvent1:.2f}mL")
    
    # 8. 等待溶剂1稳定
    debug_print("  ⏳ 添加溶剂1稳定等待...")
    action_sequence.append({
        "action_name": "wait",
        "action_kwargs": {
            "time": 5.0,  # 缩短等待时间
            "description": f"等待溶剂1 {solvent1} 稳定"
        }
    })
    debug_print("  ✅ 溶剂1稳定等待已添加 ⏰✨")
    
    # 9. 添加第二种溶剂
    debug_print("📍 步骤8: 添加第二种溶剂... 🧪")
    debug_print(f"  🚰 开始添加溶剂2: {solvent2} ({volume2:.2f} mL)")
    
    try:
        pump_actions2 = generate_pump_protocol_with_rinsing(
            G=G,
            from_vessel=solvent2_vessel,
            to_vessel=vessel_id,  # 🔧 使用 vessel_id
            volume=volume2,             # 使用解析后的体积
            amount="",
            time=0.0,
            viscous=False,
            rinsing_solvent="",  # 重结晶不需要清洗
            rinsing_volume=0.0,
            rinsing_repeats=0,
            solid=False,
            flowrate=2.0,  # 正常流速
            transfer_flowrate=0.5
        )
        
        action_sequence.extend(pump_actions2)
        debug_print(f"  ✅ 溶剂2泵送动作已添加: {len(pump_actions2)} 个动作 🚰✨")
        
    except Exception as e:
        debug_print(f"  ❌ 溶剂2泵协议生成失败: {str(e)} 😭")
        raise ValueError(f"生成溶剂2泵协议时出错: {str(e)}")
    
    # 🔧 新增：更新容器体积 - 添加溶剂2后
    debug_print("  🔧 更新容器体积 - 添加溶剂2后...")
    final_liquid_volume = new_volume_after_solvent1 + volume2
    
    # 更新vessel字典中的体积
    if "data" in vessel and "liquid_volume" in vessel["data"]:
        current_volume = vessel["data"]["liquid_volume"]
        if isinstance(current_volume, list):
            if len(current_volume) > 0:
                vessel["data"]["liquid_volume"][0] = final_liquid_volume
            else:
                vessel["data"]["liquid_volume"] = [final_liquid_volume]
        else:
            vessel["data"]["liquid_volume"] = final_liquid_volume
    
    # 同时更新图中的容器数据
    if vessel_id in G.nodes():
        if 'data' not in G.nodes[vessel_id]:
            G.nodes[vessel_id]['data'] = {}
        
        vessel_node_data = G.nodes[vessel_id]['data']
        current_node_volume = vessel_node_data.get('liquid_volume', 0.0)
        
        if isinstance(current_node_volume, list):
            if len(current_node_volume) > 0:
                G.nodes[vessel_id]['data']['liquid_volume'][0] = final_liquid_volume
            else:
                G.nodes[vessel_id]['data']['liquid_volume'] = [final_liquid_volume]
        else:
            G.nodes[vessel_id]['data']['liquid_volume'] = final_liquid_volume
    
    debug_print(f"  📊 最终体积: {new_volume_after_solvent1:.2f}mL + {volume2:.2f}mL = {final_liquid_volume:.2f}mL")
    
    # 10. 等待溶剂2稳定
    debug_print("  ⏳ 添加溶剂2稳定等待...")
    action_sequence.append({
        "action_name": "wait",
        "action_kwargs": {
            "time": 5.0,  # 缩短等待时间
            "description": f"等待溶剂2 {solvent2} 稳定"
        }
    })
    debug_print("  ✅ 溶剂2稳定等待已添加 ⏰✨")
    
    # 11. 等待重结晶完成
    debug_print("📍 步骤9: 等待重结晶完成... 💎")
    
    # 模拟运行时间优化
    debug_print("  ⏱️ 检查模拟运行时间限制...")
    original_crystallize_time = 600.0  # 原始重结晶时间
    simulation_time_limit = 60.0  # 模拟运行时间限制：60秒

    final_crystallize_time = min(original_crystallize_time, simulation_time_limit)
    
    if original_crystallize_time > simulation_time_limit:
        debug_print(f"  🎮 模拟运行优化: {original_crystallize_time}s → {final_crystallize_time}s ⚡")
        debug_print(f"  📊 时间缩短: {original_crystallize_time/60:.1f}分钟 → {final_crystallize_time/60:.1f}分钟 🚀")
    else:
        debug_print(f"  ✅ 时间在限制内: {final_crystallize_time}s 保持不变 🎯")
    
    action_sequence.append({
        "action_name": "wait",
        "action_kwargs": {
            "time": final_crystallize_time,
            "description": f"等待重结晶完成（{solvent1}:{solvent2} = {ratio}，总体积 {final_volume}mL）" + (f" (模拟时间)" if original_crystallize_time != final_crystallize_time else "")
        }
    })
    debug_print(f"  ✅ 重结晶等待已添加: {final_crystallize_time}s 💎✨")
    
    # 显示时间调整信息
    if original_crystallize_time != final_crystallize_time:
        debug_print(f"  🎭 模拟优化说明: 原计划 {original_crystallize_time/60:.1f}分钟，实际模拟 {final_crystallize_time/60:.1f}分钟 ⚡")
    
    # 总结
    debug_print("💎" * 20)
    debug_print(f"🎉 重结晶协议生成完成! ✨")
    debug_print(f"📊 总动作数: {len(action_sequence)} 个")
    debug_print(f"🥽 目标容器: {vessel_id}")
    debug_print(f"💧 总体积变化:")
    debug_print(f"  - 原始体积: {original_liquid_volume:.2f}mL")
    debug_print(f"  - 添加溶剂: {final_volume:.2f}mL")
    debug_print(f"  - 最终体积: {final_liquid_volume:.2f}mL")
    debug_print(f"⚖️ 溶剂比例: {solvent1}:{solvent2} = {ratio1}:{ratio2}")
    debug_print(f"🧪 溶剂1: {solvent1} ({volume1:.2f}mL)")
    debug_print(f"🧪 溶剂2: {solvent2} ({volume2:.2f}mL)")
    debug_print(f"⏱️ 预计总时间: {(final_crystallize_time + 10)/60:.1f} 分钟 ⌛")
    debug_print("💎" * 20)
    
    return action_sequence


# 测试函数
def test_recrystallize_protocol():
    """测试重结晶协议"""
    debug_print("🧪 === RECRYSTALLIZE PROTOCOL 测试 === ✨")
    
    # 测试体积解析
    debug_print("💧 测试体积解析...")
    test_volumes = ["100 mL", "2.5 L", "500", "50.5", "?", "invalid"]
    for vol in test_volumes:
        parsed = parse_volume_with_units(vol)
        debug_print(f"  📊 体积 '{vol}' -> {parsed}mL")
    
    # 测试比例解析
    debug_print("⚖️ 测试比例解析...")
    test_ratios = ["1:1", "3:7", "50:50", "1-1", "2,8", "invalid"]
    for ratio in test_ratios:
        r1, r2 = parse_ratio(ratio)
        debug_print(f"  📊 比例 '{ratio}' -> {r1}:{r2}")
    
    debug_print("✅ 测试完成 🎉")

if __name__ == "__main__":
    test_recrystallize_protocol()