import networkx as nx
import logging
from typing import List, Dict, Any
from .pump_protocol import generate_pump_protocol_with_rinsing

logger = logging.getLogger(__name__)

def debug_print(message):
    """调试输出"""
    print(f"[ADJUST_PH] {message}", flush=True)
    logger.info(f"[ADJUST_PH] {message}")

# 🆕 创建进度日志动作
def create_action_log(message: str, emoji: str = "📝") -> Dict[str, Any]:
    """创建一个动作日志"""
    full_message = f"{emoji} {message}"
    debug_print(full_message)
    logger.info(full_message)
    print(f"[ACTION] {full_message}", flush=True)
    
    return {
        "action_name": "wait",
        "action_kwargs": {
            "time": 0.1,
            "log_message": full_message
        }
    }

def find_acid_base_vessel(G: nx.DiGraph, reagent: str) -> str:
    """
    查找酸碱试剂容器，支持多种匹配模式
    
    Args:
        G: 网络图
        reagent: 试剂名称（如 "hydrochloric acid", "sodium hydroxide"）
    
    Returns:
        str: 试剂容器ID
    """
    debug_print(f"🔍 正在查找试剂 '{reagent}' 的容器...")
    
    # 常见酸碱试剂的别名映射
    reagent_aliases = {
        "hydrochloric acid": ["HCl", "hydrochloric_acid", "hcl", "muriatic_acid"],
        "sodium hydroxide": ["NaOH", "sodium_hydroxide", "naoh", "caustic_soda"],
        "sulfuric acid": ["H2SO4", "sulfuric_acid", "h2so4"],
        "nitric acid": ["HNO3", "nitric_acid", "hno3"],
        "acetic acid": ["CH3COOH", "acetic_acid", "glacial_acetic_acid"],
        "ammonia": ["NH3", "ammonium_hydroxide", "nh3"],
        "potassium hydroxide": ["KOH", "potassium_hydroxide", "koh"]
    }
    
    # 构建搜索名称列表
    search_names = [reagent.lower()]
    debug_print(f"📋 基础搜索名称: {reagent.lower()}")
    
    # 添加别名
    for base_name, aliases in reagent_aliases.items():
        if reagent.lower() in base_name.lower() or base_name.lower() in reagent.lower():
            search_names.extend([alias.lower() for alias in aliases])
            debug_print(f"🔗 添加别名: {aliases}")
            break
    
    debug_print(f"📝 完整搜索列表: {search_names}")
    
    # 构建可能的容器名称
    possible_names = []
    for name in search_names:
        name_clean = name.replace(" ", "_").replace("-", "_")
        possible_names.extend([
            f"flask_{name_clean}",
            f"bottle_{name_clean}",
            f"reagent_{name_clean}",
            f"acid_{name_clean}" if "acid" in name else f"base_{name_clean}",
            f"{name_clean}_bottle",
            f"{name_clean}_flask",
            name_clean
        ])
    
    debug_print(f"🎯 可能的容器名称 (前5个): {possible_names[:5]}... (共{len(possible_names)}个)")
    
    # 第一步：通过容器名称匹配
    debug_print(f"📋 方法1: 精确名称匹配...")
    for vessel_name in possible_names:
        if vessel_name in G.nodes():
            debug_print(f"✅ 通过名称匹配找到容器: {vessel_name} 🎯")
            return vessel_name
    
    # 第二步：通过模糊匹配
    debug_print(f"📋 方法2: 模糊名称匹配...")
    for node_id in G.nodes():
        if G.nodes[node_id].get('type') == 'container':
            node_name = G.nodes[node_id].get('name', '').lower()
            
            # 检查是否包含任何搜索名称
            for search_name in search_names:
                if search_name in node_id.lower() or search_name in node_name:
                    debug_print(f"✅ 通过模糊匹配找到容器: {node_id} 🔍")
                    return node_id
    
    # 第三步：通过液体类型匹配
    debug_print(f"📋 方法3: 液体类型匹配...")
    for node_id in G.nodes():
        if G.nodes[node_id].get('type') == 'container':
            vessel_data = G.nodes[node_id].get('data', {})
            liquids = vessel_data.get('liquid', [])
            
            for liquid in liquids:
                if isinstance(liquid, dict):
                    liquid_type = (liquid.get('liquid_type') or liquid.get('name', '')).lower()
                    reagent_name = vessel_data.get('reagent_name', '').lower()
                    
                    for search_name in search_names:
                        if search_name in liquid_type or search_name in reagent_name:
                            debug_print(f"✅ 通过液体类型匹配找到容器: {node_id} 💧")
                            return node_id
    
    # 列出可用容器帮助调试
    debug_print(f"📊 列出可用容器帮助调试...")
    available_containers = []
    for node_id in G.nodes():
        if G.nodes[node_id].get('type') == 'container':
            vessel_data = G.nodes[node_id].get('data', {})
            liquids = vessel_data.get('liquid', [])
            liquid_types = [liquid.get('liquid_type', '') or liquid.get('name', '') 
                           for liquid in liquids if isinstance(liquid, dict)]
            
            available_containers.append({
                'id': node_id,
                'name': G.nodes[node_id].get('name', ''),
                'liquids': liquid_types,
                'reagent_name': vessel_data.get('reagent_name', '')
            })
    
    debug_print(f"📋 可用容器列表:")
    for container in available_containers:
        debug_print(f"  - 🧪 {container['id']}: {container['name']}")
        debug_print(f"    💧 液体: {container['liquids']}")
        debug_print(f"    🏷️ 试剂: {container['reagent_name']}")
    
    debug_print(f"❌ 所有匹配方法都失败了")
    raise ValueError(f"找不到试剂 '{reagent}' 对应的容器。尝试了: {possible_names[:10]}...")

def find_connected_stirrer(G: nx.DiGraph, vessel: str) -> str:
    """查找与容器相连的搅拌器"""
    debug_print(f"🔍 查找连接到容器 '{vessel}' 的搅拌器...")
    
    stirrer_nodes = [node for node in G.nodes() 
                    if (G.nodes[node].get('class') or '') == 'virtual_stirrer']
    
    debug_print(f"📊 发现 {len(stirrer_nodes)} 个搅拌器: {stirrer_nodes}")
    
    for stirrer in stirrer_nodes:
        if G.has_edge(stirrer, vessel) or G.has_edge(vessel, stirrer):
            debug_print(f"✅ 找到连接的搅拌器: {stirrer} 🔗")
            return stirrer
    
    if stirrer_nodes:
        debug_print(f"⚠️ 未找到直接连接的搅拌器，使用第一个: {stirrer_nodes[0]} 🔄")
        return stirrer_nodes[0]
    
    debug_print(f"❌ 未找到任何搅拌器")
    return None

def calculate_reagent_volume(target_ph_value: float, reagent: str, vessel_volume: float = 100.0) -> float:
    """
    估算需要的试剂体积来调节pH
    
    Args:
        target_ph_value: 目标pH值
        reagent: 试剂名称
        vessel_volume: 容器体积 (mL)
    
    Returns:
        float: 估算的试剂体积 (mL)
    """
    debug_print(f"🧮 计算试剂体积...")
    debug_print(f"  📍 目标pH: {target_ph_value}")
    debug_print(f"  🧪 试剂: {reagent}")
    debug_print(f"  📏 容器体积: {vessel_volume}mL")
    
    # 简化的pH调节体积估算（实际应用中需要更精确的计算）
    if "acid" in reagent.lower() or "hcl" in reagent.lower():
        debug_print(f"🍋 检测到酸性试剂")
        # 酸性试剂：pH越低需要的体积越大
        if target_ph_value < 3:
            volume = vessel_volume * 0.05  # 5%
            debug_print(f"  💪 强酸性 (pH<3): 使用 5% 体积")
        elif target_ph_value < 5:
            volume = vessel_volume * 0.02  # 2%
            debug_print(f"  🔸 中酸性 (pH<5): 使用 2% 体积")
        else:
            volume = vessel_volume * 0.01  # 1%
            debug_print(f"  🔹 弱酸性 (pH≥5): 使用 1% 体积")
    
    elif "hydroxide" in reagent.lower() or "naoh" in reagent.lower():
        debug_print(f"🧂 检测到碱性试剂")
        # 碱性试剂：pH越高需要的体积越大
        if target_ph_value > 11:
            volume = vessel_volume * 0.05  # 5%
            debug_print(f"  💪 强碱性 (pH>11): 使用 5% 体积")
        elif target_ph_value > 9:
            volume = vessel_volume * 0.02  # 2%
            debug_print(f"  🔸 中碱性 (pH>9): 使用 2% 体积")
        else:
            volume = vessel_volume * 0.01  # 1%
            debug_print(f"  🔹 弱碱性 (pH≤9): 使用 1% 体积")
    
    else:
        # 未知试剂，使用默认值
        volume = vessel_volume * 0.01
        debug_print(f"❓ 未知试剂类型，使用默认 1% 体积")
    
    debug_print(f"📊 计算结果: {volume:.2f}mL")
    return volume

def generate_adjust_ph_protocol(
    G: nx.DiGraph,
    vessel: str,
    ph_value: float,
    reagent: str,
    **kwargs
) -> List[Dict[str, Any]]:
    """
    生成调节pH的协议序列
    
    Args:
        G: 有向图，节点为容器和设备
        vessel: 目标容器（需要调节pH的容器）
        ph_value: 目标pH值（从XDL传入）
        reagent: 酸碱试剂名称（从XDL传入）
        **kwargs: 其他可选参数，使用默认值
    
    Returns:
        List[Dict[str, Any]]: 动作序列
    """
    
    debug_print("=" * 60)
    debug_print("🧪 开始生成pH调节协议")
    debug_print(f"📋 原始参数:")
    debug_print(f"  🥼 vessel: '{vessel}'")
    debug_print(f"  📊 ph_value: {ph_value}")
    debug_print(f"  🧪 reagent: '{reagent}'")
    debug_print(f"  📦 kwargs: {kwargs}")
    debug_print("=" * 60)
    
    action_sequence = []
    
    # 从kwargs中获取可选参数，如果没有则使用默认值
    volume = kwargs.get('volume', 0.0)           # 自动估算体积
    stir = kwargs.get('stir', True)              # 默认搅拌
    stir_speed = kwargs.get('stir_speed', 300.0) # 默认搅拌速度
    stir_time = kwargs.get('stir_time', 60.0)    # 默认搅拌时间
    settling_time = kwargs.get('settling_time', 30.0)  # 默认平衡时间
    
    debug_print(f"🔧 处理后的参数:")
    debug_print(f"  📏 volume: {volume}mL (0.0表示自动估算)")
    debug_print(f"  🌪️ stir: {stir}")
    debug_print(f"  🔄 stir_speed: {stir_speed}rpm")
    debug_print(f"  ⏱️ stir_time: {stir_time}s")
    debug_print(f"  ⏳ settling_time: {settling_time}s")
    
    # 开始处理
    action_sequence.append(create_action_log(f"开始调节pH至 {ph_value}", "🧪"))
    action_sequence.append(create_action_log(f"目标容器: {vessel}", "🥼"))
    action_sequence.append(create_action_log(f"使用试剂: {reagent}", "⚗️"))
    
    # 1. 验证目标容器存在
    debug_print(f"🔍 步骤1: 验证目标容器...")
    if vessel not in G.nodes():
        debug_print(f"❌ 目标容器 '{vessel}' 不存在于系统中")
        raise ValueError(f"目标容器 '{vessel}' 不存在于系统中")
    
    debug_print(f"✅ 目标容器验证通过")
    action_sequence.append(create_action_log("目标容器验证通过", "✅"))
    
    # 2. 查找酸碱试剂容器
    debug_print(f"🔍 步骤2: 查找试剂容器...")
    action_sequence.append(create_action_log("正在查找试剂容器...", "🔍"))
    
    try:
        reagent_vessel = find_acid_base_vessel(G, reagent)
        debug_print(f"✅ 找到试剂容器: {reagent_vessel}")
        action_sequence.append(create_action_log(f"找到试剂容器: {reagent_vessel}", "🧪"))
    except ValueError as e:
        debug_print(f"❌ 无法找到试剂容器: {str(e)}")
        action_sequence.append(create_action_log(f"试剂容器查找失败: {str(e)}", "❌"))
        raise ValueError(f"无法找到试剂 '{reagent}': {str(e)}")
    
    # 3. 体积估算
    debug_print(f"🔍 步骤3: 体积处理...")
    if volume <= 0:
        action_sequence.append(create_action_log("开始自动估算试剂体积", "🧮"))
        
        # 获取目标容器的体积信息
        vessel_data = G.nodes[vessel].get('data', {})
        vessel_volume = vessel_data.get('max_volume', 100.0)  # 默认100mL
        debug_print(f"📏 容器最大体积: {vessel_volume}mL")
        
        estimated_volume = calculate_reagent_volume(ph_value, reagent, vessel_volume)
        volume = estimated_volume
        debug_print(f"✅ 自动估算试剂体积: {volume:.2f} mL")
        action_sequence.append(create_action_log(f"估算试剂体积: {volume:.2f}mL", "📊"))
    else:
        debug_print(f"📏 使用指定体积: {volume}mL")
        action_sequence.append(create_action_log(f"使用指定体积: {volume}mL", "📏"))
    
    # 4. 验证路径存在
    debug_print(f"🔍 步骤4: 路径验证...")
    action_sequence.append(create_action_log("验证转移路径...", "🛤️"))
    
    try:
        path = nx.shortest_path(G, source=reagent_vessel, target=vessel)
        debug_print(f"✅ 找到路径: {' → '.join(path)}")
        action_sequence.append(create_action_log(f"找到转移路径: {' → '.join(path)}", "🛤️"))
    except nx.NetworkXNoPath:
        debug_print(f"❌ 无法找到转移路径")
        action_sequence.append(create_action_log("转移路径不存在", "❌"))
        raise ValueError(f"从试剂容器 '{reagent_vessel}' 到目标容器 '{vessel}' 没有可用路径")
    
    # 5. 搅拌器设置
    debug_print(f"🔍 步骤5: 搅拌器设置...")
    stirrer_id = None
    if stir:
        action_sequence.append(create_action_log("准备启动搅拌器", "🌪️"))
        
        try:
            stirrer_id = find_connected_stirrer(G, vessel)
            
            if stirrer_id:
                debug_print(f"✅ 找到搅拌器 {stirrer_id}，启动搅拌")
                action_sequence.append(create_action_log(f"启动搅拌器 {stirrer_id} (速度: {stir_speed}rpm)", "🔄"))
                
                action_sequence.append({
                    "device_id": stirrer_id,
                    "action_name": "start_stir",
                    "action_kwargs": {
                        "vessel": vessel,
                        "stir_speed": stir_speed,
                        "purpose": f"pH调节: 启动搅拌，准备添加 {reagent}"
                    }
                })
                
                # 等待搅拌稳定
                action_sequence.append(create_action_log("等待搅拌稳定...", "⏳"))
                action_sequence.append({
                    "action_name": "wait",
                    "action_kwargs": {"time": 5}
                })
            else:
                debug_print(f"⚠️ 未找到搅拌器，继续执行")
                action_sequence.append(create_action_log("未找到搅拌器，跳过搅拌", "⚠️"))
        
        except Exception as e:
            debug_print(f"❌ 搅拌器配置出错: {str(e)}")
            action_sequence.append(create_action_log(f"搅拌器配置失败: {str(e)}", "❌"))
    else:
        debug_print(f"📋 跳过搅拌设置")
        action_sequence.append(create_action_log("跳过搅拌设置", "⏭️"))
    
    # 6. 试剂添加
    debug_print(f"🔍 步骤6: 试剂添加...")
    action_sequence.append(create_action_log(f"开始添加试剂 {volume:.2f}mL", "🚰"))
    
    # 计算添加时间（pH调节需要缓慢添加）
    addition_time = max(30.0, volume * 2.0)  # 至少30秒，每mL需要2秒
    debug_print(f"⏱️ 计算添加时间: {addition_time}s (缓慢注入)")
    action_sequence.append(create_action_log(f"设置添加时间: {addition_time:.0f}s (缓慢注入)", "⏱️"))
    
    try:
        action_sequence.append(create_action_log("调用泵协议进行试剂转移", "🔄"))
        
        pump_actions = generate_pump_protocol_with_rinsing(
            G=G,
            from_vessel=reagent_vessel,
            to_vessel=vessel,
            volume=volume,
            amount="",
            time=addition_time,
            viscous=False,
            rinsing_solvent="",  # pH调节不需要清洗
            rinsing_volume=0.0,
            rinsing_repeats=0,
            solid=False,
            flowrate=0.5,  # 缓慢注入
            transfer_flowrate=0.3
        )
        
        action_sequence.extend(pump_actions)
        debug_print(f"✅ 泵协议生成完成，添加了 {len(pump_actions)} 个动作")
        action_sequence.append(create_action_log(f"试剂转移完成 ({len(pump_actions)} 个操作)", "✅"))
        
    except Exception as e:
        debug_print(f"❌ 生成泵协议时出错: {str(e)}")
        action_sequence.append(create_action_log(f"泵协议生成失败: {str(e)}", "❌"))
        raise ValueError(f"生成泵协议时出错: {str(e)}")
    
    # 7. 混合搅拌
    if stir and stirrer_id:
        debug_print(f"🔍 步骤7: 混合搅拌...")
        action_sequence.append(create_action_log(f"开始混合搅拌 {stir_time:.0f}s", "🌀"))
        
        action_sequence.append({
            "device_id": stirrer_id,
            "action_name": "stir",
            "action_kwargs": {
                "stir_time": stir_time,
                "stir_speed": stir_speed,
                "settling_time": settling_time,
                "purpose": f"pH调节: 混合试剂，目标pH={ph_value}"
            }
        })
        
        debug_print(f"✅ 混合搅拌设置完成")
    else:
        debug_print(f"⏭️ 跳过混合搅拌")
        action_sequence.append(create_action_log("跳过混合搅拌", "⏭️"))
    
    # 8. 等待平衡
    debug_print(f"🔍 步骤8: 反应平衡...")
    action_sequence.append(create_action_log(f"等待pH平衡 {settling_time:.0f}s", "⚖️"))
    
    action_sequence.append({
        "action_name": "wait",
        "action_kwargs": {
            "time": settling_time,
            "description": f"等待pH平衡到目标值 {ph_value}"
        }
    })
    
    # 9. 完成总结
    total_time = addition_time + stir_time + settling_time
    
    debug_print("=" * 60)
    debug_print(f"🎉 pH调节协议生成完成")
    debug_print(f"📊 协议统计:")
    debug_print(f"  📋 总动作数: {len(action_sequence)}")
    debug_print(f"  ⏱️ 预计总时间: {total_time:.0f}s ({total_time/60:.1f}分钟)")
    debug_print(f"  🧪 试剂: {reagent}")
    debug_print(f"  📏 体积: {volume:.2f}mL")
    debug_print(f"  📊 目标pH: {ph_value}")
    debug_print(f"  🥼 目标容器: {vessel}")
    debug_print("=" * 60)
    
    # 添加完成日志
    summary_msg = f"pH调节协议完成: {vessel} → pH {ph_value} (使用 {volume:.2f}mL {reagent})"
    action_sequence.append(create_action_log(summary_msg, "🎉"))
    
    return action_sequence

def generate_adjust_ph_protocol_stepwise(
    G: nx.DiGraph,
    vessel: str,
    ph_value: float,
    reagent: str,
    max_volume: float = 10.0,
    steps: int = 3
) -> List[Dict[str, Any]]:
    """
    分步调节pH的协议（更安全，避免过度调节）
    
    Args:
        G: 网络图
        vessel: 目标容器
        ph_value: 目标pH值
        reagent: 酸碱试剂
        max_volume: 最大试剂体积
        steps: 分步数量
    
    Returns:
        List[Dict[str, Any]]: 动作序列
    """
    debug_print("=" * 60)
    debug_print(f"🔄 开始分步pH调节")
    debug_print(f"📋 分步参数:")
    debug_print(f"  🥼 vessel: {vessel}")
    debug_print(f"  📊 ph_value: {ph_value}")
    debug_print(f"  🧪 reagent: {reagent}")
    debug_print(f"  📏 max_volume: {max_volume}mL")
    debug_print(f"  🔢 steps: {steps}")
    debug_print("=" * 60)
    
    action_sequence = []
    
    # 每步添加的体积
    step_volume = max_volume / steps
    debug_print(f"📊 每步体积: {step_volume:.2f}mL")
    
    action_sequence.append(create_action_log(f"开始分步pH调节 ({steps}步)", "🔄"))
    action_sequence.append(create_action_log(f"每步添加: {step_volume:.2f}mL", "📏"))
    
    for i in range(steps):
        debug_print(f"🔄 执行第 {i+1}/{steps} 步，添加 {step_volume:.2f}mL")
        action_sequence.append(create_action_log(f"第 {i+1}/{steps} 步开始", "🚀"))
        
        # 生成单步协议
        step_actions = generate_adjust_ph_protocol(
            G=G,
            vessel=vessel,
            ph_value=ph_value,
            reagent=reagent,
            volume=step_volume,
            stir=True,
            stir_speed=300.0,
            stir_time=30.0,
            settling_time=20.0
        )
        
        action_sequence.extend(step_actions)
        debug_print(f"✅ 第 {i+1}/{steps} 步完成，添加了 {len(step_actions)} 个动作")
        action_sequence.append(create_action_log(f"第 {i+1}/{steps} 步完成", "✅"))
        
        # 步骤间等待
        if i < steps - 1:
            debug_print(f"⏳ 步骤间等待30s")
            action_sequence.append(create_action_log("步骤间等待...", "⏳"))
            action_sequence.append({
                "action_name": "wait",
                "action_kwargs": {
                    "time": 30,
                    "description": f"pH调节第{i+1}步完成，等待下一步"
                }
            })
    
    debug_print(f"🎉 分步pH调节完成，共 {len(action_sequence)} 个动作")
    action_sequence.append(create_action_log("分步pH调节全部完成", "🎉"))
    
    return action_sequence

# 便捷函数：常用pH调节
def generate_acidify_protocol(
    G: nx.DiGraph,
    vessel: str,
    target_ph: float = 2.0,
    acid: str = "hydrochloric acid"
) -> List[Dict[str, Any]]:
    """酸化协议"""
    debug_print(f"🍋 生成酸化协议: {vessel} → pH {target_ph} (使用 {acid})")
    return generate_adjust_ph_protocol(
        G, vessel, target_ph, acid
    )

def generate_basify_protocol(
    G: nx.DiGraph,
    vessel: str,
    target_ph: float = 12.0,
    base: str = "sodium hydroxide"
) -> List[Dict[str, Any]]:
    """碱化协议"""
    debug_print(f"🧂 生成碱化协议: {vessel} → pH {target_ph} (使用 {base})")
    return generate_adjust_ph_protocol(
        G, vessel, target_ph, base
    )

def generate_neutralize_protocol(
    G: nx.DiGraph,
    vessel: str,
    reagent: str = "sodium hydroxide"
) -> List[Dict[str, Any]]:
    """中和协议（pH=7）"""
    debug_print(f"⚖️ 生成中和协议: {vessel} → pH 7.0 (使用 {reagent})")
    return generate_adjust_ph_protocol(
        G, vessel, 7.0, reagent
    )

# 测试函数
def test_adjust_ph_protocol():
    """测试pH调节协议"""
    debug_print("=== ADJUST PH PROTOCOL 增强版测试 ===")
    
    # 测试体积计算
    debug_print("🧮 测试体积计算...")
    test_cases = [
        (2.0, "hydrochloric acid", 100.0),
        (4.0, "hydrochloric acid", 100.0),
        (12.0, "sodium hydroxide", 100.0),
        (10.0, "sodium hydroxide", 100.0),
        (7.0, "unknown reagent", 100.0)
    ]
    
    for ph, reagent, volume in test_cases:
        result = calculate_reagent_volume(ph, reagent, volume)
        debug_print(f"📊 {reagent} → pH {ph}: {result:.2f}mL")
    
    debug_print("✅ 测试完成")

if __name__ == "__main__":
    test_adjust_ph_protocol()