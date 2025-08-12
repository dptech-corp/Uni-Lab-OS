from typing import List, Dict, Any, Optional
import networkx as nx
import logging
from .utils.vessel_parser import get_vessel
from .pump_protocol import generate_pump_protocol_with_rinsing

logger = logging.getLogger(__name__)

def debug_print(message):
    """调试输出"""
    logger.info(f"[FILTER] {message}")

def find_filter_device(G: nx.DiGraph) -> str:
    """查找过滤器设备"""
    debug_print("🔍 查找过滤器设备... 🌊")
    
    # 查找过滤器设备
    for node in G.nodes():
        node_data = G.nodes[node]
        node_class = node_data.get('class', '') or ''
        
        if 'filter' in node_class.lower() or 'filter' in node.lower():
            debug_print(f"🎉 找到过滤器设备: {node} ✨")
            return node
    
    # 如果没找到，寻找可能的过滤器名称
    debug_print("🔎 在预定义名称中搜索过滤器... 📋")
    possible_names = ["filter", "filter_1", "virtual_filter", "filtration_unit"]
    for name in possible_names:
        if name in G.nodes():
            debug_print(f"🎉 找到过滤器设备: {name} ✨")
            return name
    
    debug_print("😭 未找到过滤器设备 💔")
    raise ValueError("未找到过滤器设备")

def validate_vessel(G: nx.DiGraph, vessel: str, vessel_type: str = "容器") -> None:
    """验证容器是否存在"""
    debug_print(f"🔍 验证{vessel_type}: '{vessel}' 🧪")
    
    if not vessel:
        debug_print(f"❌ {vessel_type}不能为空! 😱")
        raise ValueError(f"{vessel_type}不能为空")
    
    if vessel not in G.nodes():
        debug_print(f"❌ {vessel_type} '{vessel}' 不存在于系统中! 😞")
        raise ValueError(f"{vessel_type} '{vessel}' 不存在于系统中")
    
    debug_print(f"✅ {vessel_type} '{vessel}' 验证通过 🎯")

def generate_filter_protocol(
    G: nx.DiGraph,
    vessel: dict,  # 🔧 修改：从字符串改为字典类型
    filtrate_vessel: dict = {"id": "waste"},
    **kwargs
) -> List[Dict[str, Any]]:
    """
    生成过滤操作的协议序列 - 支持体积运算
    
    Args:
        G: 设备图
        vessel: 过滤容器字典（必需）- 包含需要过滤的混合物
        filtrate_vessel: 滤液容器名称（可选）- 如果提供则收集滤液
        **kwargs: 其他参数（兼容性）
    
    Returns:
        List[Dict[str, Any]]: 过滤操作的动作序列
    """
    
    # 🔧 核心修改：从字典中提取容器ID
    vessel_id, vessel_data = get_vessel(vessel)
    filtrate_vessel_id, filtrate_vessel_data = get_vessel(filtrate_vessel)
    
    debug_print("🌊" * 20)
    debug_print("🚀 开始生成过滤协议（支持体积运算）✨")
    debug_print(f"📝 输入参数:")
    debug_print(f"  🥽 vessel: {vessel} (ID: {vessel_id})")
    debug_print(f"  🧪 filtrate_vessel: {filtrate_vessel}")
    debug_print(f"  ⚙️ 其他参数: {kwargs}")
    debug_print("🌊" * 20)
    
    action_sequence = []
    
    # 🔧 新增：记录过滤前的容器状态
    debug_print("🔍 记录过滤前容器状态...")
    original_liquid_volume = 0.0
    if "data" in vessel and "liquid_volume" in vessel["data"]:
        current_volume = vessel["data"]["liquid_volume"]
        if isinstance(current_volume, list) and len(current_volume) > 0:
            original_liquid_volume = current_volume[0]
        elif isinstance(current_volume, (int, float)):
            original_liquid_volume = current_volume
    debug_print(f"📊 过滤前液体体积: {original_liquid_volume:.2f}mL")
    
    # === 参数验证 ===
    debug_print("📍 步骤1: 参数验证... 🔧")
    
    # 验证必需参数
    debug_print("  🔍 验证必需参数...")
    validate_vessel(G, vessel_id, "过滤容器")  # 🔧 使用 vessel_id
    debug_print("  ✅ 必需参数验证完成 🎯")
    
    # 验证可选参数
    debug_print("  🔍 验证可选参数...")
    if filtrate_vessel:
        validate_vessel(G, filtrate_vessel_id, "滤液容器")
        debug_print("  🌊 模式: 过滤并收集滤液 💧")
    else:
        debug_print("  🧱 模式: 过滤并收集固体 🔬")
    debug_print("  ✅ 可选参数验证完成 🎯")
    
    # === 查找设备 ===
    debug_print("📍 步骤2: 查找设备... 🔍")
    
    try:
        debug_print("  🔎 搜索过滤器设备...")
        filter_device = find_filter_device(G)
        debug_print(f"  🎉 使用过滤器设备: {filter_device} 🌊✨")
        
    except Exception as e:
        debug_print(f"  ❌ 设备查找失败: {str(e)} 😭")
        raise ValueError(f"设备查找失败: {str(e)}")
    
    # 🔧 新增：过滤效率和体积分配估算
    debug_print("📍 步骤2.5: 过滤体积分配估算... 📊")
    
    # 估算过滤分离比例（基于经验数据）
    solid_ratio = 0.1  # 假设10%是固体（保留在过滤器上）
    liquid_ratio = 0.9  # 假设90%是液体（通过过滤器）
    volume_loss_ratio = 0.05  # 假设5%体积损失（残留在过滤器等）
    
    # 从kwargs中获取过滤参数进行优化
    if "solid_content" in kwargs:
        try:
            solid_ratio = float(kwargs["solid_content"])
            liquid_ratio = 1.0 - solid_ratio
            debug_print(f"📋 使用指定的固体含量: {solid_ratio*100:.1f}%")
        except:
            debug_print("⚠️ 固体含量参数无效，使用默认值")
    
    if original_liquid_volume > 0:
        expected_filtrate_volume = original_liquid_volume * liquid_ratio * (1.0 - volume_loss_ratio)
        expected_solid_volume = original_liquid_volume * solid_ratio
        volume_loss = original_liquid_volume * volume_loss_ratio
        
        debug_print(f"📊 过滤体积分配估算:")
        debug_print(f"  - 原始体积: {original_liquid_volume:.2f}mL")
        debug_print(f"  - 预计滤液体积: {expected_filtrate_volume:.2f}mL ({liquid_ratio*100:.1f}%)")
        debug_print(f"  - 预计固体体积: {expected_solid_volume:.2f}mL ({solid_ratio*100:.1f}%)")
        debug_print(f"  - 预计损失体积: {volume_loss:.2f}mL ({volume_loss_ratio*100:.1f}%)")
    
    # === 转移到过滤器（如果需要）===
    debug_print("📍 步骤3: 转移到过滤器... 🚚")
    
    if vessel_id != filter_device:  # 🔧 使用 vessel_id
        debug_print(f"  🚛 需要转移: {vessel_id} → {filter_device} 📦")
        
        try:
            debug_print("  🔄 开始执行转移操作...")
            # 使用pump protocol转移液体到过滤器
            transfer_actions = generate_pump_protocol_with_rinsing(
                G=G,
                from_vessel=vessel_id,  # 🔧 使用 vessel_id
                to_vessel=filter_device,
                volume=0.0,  # 转移所有液体
                amount="",
                time=0.0,
                viscous=False,
                rinsing_solvent="",
                rinsing_volume=0.0,
                rinsing_repeats=0,
                solid=False,
                flowrate=2.0,
                transfer_flowrate=2.0
            )
            
            if transfer_actions:
                action_sequence.extend(transfer_actions)
                debug_print(f"  ✅ 添加了 {len(transfer_actions)} 个转移动作 🚚✨")
                
                # 🔧 新增：转移后更新容器体积
                debug_print("  🔧 更新转移后的容器体积...")
                
                # 原容器体积变为0（所有液体已转移）
                if "data" in vessel and "liquid_volume" in vessel["data"]:
                    current_volume = vessel["data"]["liquid_volume"]
                    if isinstance(current_volume, list):
                        vessel["data"]["liquid_volume"] = [0.0] if len(current_volume) > 0 else [0.0]
                    else:
                        vessel["data"]["liquid_volume"] = 0.0
                
                # 同时更新图中的容器数据
                if vessel_id in G.nodes():
                    if 'data' not in G.nodes[vessel_id]:
                        G.nodes[vessel_id]['data'] = {}
                    G.nodes[vessel_id]['data']['liquid_volume'] = 0.0
                
                debug_print(f"  📊 转移完成，{vessel_id} 体积更新为 0.0mL")
                
            else:
                debug_print("  ⚠️ 转移协议返回空序列 🤔")
                
        except Exception as e:
            debug_print(f"  ❌ 转移失败: {str(e)} 😞")
            debug_print("  🔄 继续执行，可能是直接连接的过滤器 🤞")
    else:
        debug_print("  ✅ 过滤容器就是过滤器，无需转移 🎯")
    
    # === 执行过滤操作 ===
    debug_print("📍 步骤4: 执行过滤操作... 🌊")
    
    # 构建过滤动作参数
    debug_print("  ⚙️ 构建过滤参数...")
    filter_kwargs = {
        "vessel": filter_device,  # 过滤器设备
        "filtrate_vessel": filtrate_vessel_id,  # 滤液容器（可能为空）
        "stir": kwargs.get("stir", False),
        "stir_speed": kwargs.get("stir_speed", 0.0),
        "temp": kwargs.get("temp", 25.0),
        "continue_heatchill": kwargs.get("continue_heatchill", False),
        "volume": kwargs.get("volume", 0.0)  # 0表示过滤所有
    }
    
    debug_print(f"  📋 过滤参数: {filter_kwargs}")
    debug_print("  🌊 开始过滤操作...")
    
    # 过滤动作
    filter_action = {
        "device_id": filter_device,
        "action_name": "filter",
        "action_kwargs": filter_kwargs
    }
    action_sequence.append(filter_action)
    debug_print("  ✅ 过滤动作已添加 🌊✨")
    
    # 过滤后等待
    debug_print("  ⏳ 添加过滤后等待...")
    action_sequence.append({
        "action_name": "wait",
        "action_kwargs": {"time": 10.0}
    })
    debug_print("  ✅ 过滤后等待动作已添加 ⏰✨")
    
    # === 收集滤液（如果需要）===
    debug_print("📍 步骤5: 收集滤液... 💧")
    
    if filtrate_vessel_id and filtrate_vessel_id not in G.neighbors(filter_device):
        debug_print(f"  🧪 收集滤液: {filter_device} → {filtrate_vessel_id} 💧")
        
        try:
            debug_print("  🔄 开始执行收集操作...")
            # 使用pump protocol收集滤液
            collect_actions = generate_pump_protocol_with_rinsing(
                G=G,
                from_vessel=filter_device,
                to_vessel=filtrate_vessel,
                volume=0.0,  # 收集所有滤液
                amount="",
                time=0.0,
                viscous=False,
                rinsing_solvent="",
                rinsing_volume=0.0,
                rinsing_repeats=0,
                solid=False,
                flowrate=2.0,
                transfer_flowrate=2.0
            )
            
            if collect_actions:
                action_sequence.extend(collect_actions)
                debug_print(f"  ✅ 添加了 {len(collect_actions)} 个收集动作 🧪✨")
                
                # 🔧 新增：收集滤液后的体积更新
                debug_print("  🔧 更新滤液容器体积...")
                
                # 更新filtrate_vessel在图中的体积（如果它是节点）
                if filtrate_vessel_id in G.nodes():
                    if 'data' not in G.nodes[filtrate_vessel_id]:
                        G.nodes[filtrate_vessel_id]['data'] = {}
                    
                    current_filtrate_volume = G.nodes[filtrate_vessel_id]['data'].get('liquid_volume', 0.0)
                    if isinstance(current_filtrate_volume, list):
                        if len(current_filtrate_volume) > 0:
                            G.nodes[filtrate_vessel_id]['data']['liquid_volume'][0] += expected_filtrate_volume
                        else:
                            G.nodes[filtrate_vessel_id]['data']['liquid_volume'] = [expected_filtrate_volume]
                    else:
                        G.nodes[filtrate_vessel_id]['data']['liquid_volume'] = current_filtrate_volume + expected_filtrate_volume
                    
                    debug_print(f"  📊 滤液容器 {filtrate_vessel_id} 体积增加 {expected_filtrate_volume:.2f}mL")
                
            else:
                debug_print("  ⚠️ 收集协议返回空序列 🤔")
                
        except Exception as e:
            debug_print(f"  ❌ 收集滤液失败: {str(e)} 😞")
            debug_print("  🔄 继续执行，可能滤液直接流入指定容器 🤞")
    else:
        debug_print("  🧱 未指定滤液容器，固体保留在过滤器中 🔬")
    
    # 🔧 新增：过滤完成后的容器状态更新
    debug_print("📍 步骤5.5: 过滤完成后状态更新... 📊")
    
    if vessel_id == filter_device:
        # 如果过滤容器就是过滤器，需要更新其体积状态
        if original_liquid_volume > 0:
            if filtrate_vessel:
                # 收集滤液模式：过滤器中主要保留固体
                remaining_volume = expected_solid_volume
                debug_print(f"  🧱 过滤器中保留固体: {remaining_volume:.2f}mL")
            else:
                # 保留固体模式：过滤器中保留所有物质
                remaining_volume = original_liquid_volume * (1.0 - volume_loss_ratio)
                debug_print(f"  🔬 过滤器中保留所有物质: {remaining_volume:.2f}mL")
            
            # 更新vessel字典中的体积
            if "data" in vessel and "liquid_volume" in vessel["data"]:
                current_volume = vessel["data"]["liquid_volume"]
                if isinstance(current_volume, list):
                    vessel["data"]["liquid_volume"] = [remaining_volume] if len(current_volume) > 0 else [remaining_volume]
                else:
                    vessel["data"]["liquid_volume"] = remaining_volume
            
            # 同时更新图中的容器数据
            if vessel_id in G.nodes():
                if 'data' not in G.nodes[vessel_id]:
                    G.nodes[vessel_id]['data'] = {}
                G.nodes[vessel_id]['data']['liquid_volume'] = remaining_volume
            
            debug_print(f"  📊 过滤器 {vessel_id} 体积更新为: {remaining_volume:.2f}mL")
    
    # === 最终等待 ===
    debug_print("📍 步骤6: 最终等待... ⏰")
    action_sequence.append({
        "action_name": "wait",
        "action_kwargs": {"time": 5.0}
    })
    debug_print("  ✅ 最终等待动作已添加 ⏰✨")
    
    # 🔧 新增：过滤完成后的状态报告
    final_vessel_volume = 0.0
    if "data" in vessel and "liquid_volume" in vessel["data"]:
        current_volume = vessel["data"]["liquid_volume"]
        if isinstance(current_volume, list) and len(current_volume) > 0:
            final_vessel_volume = current_volume[0]
        elif isinstance(current_volume, (int, float)):
            final_vessel_volume = current_volume
    
    # === 总结 ===
    debug_print("🎊" * 20)
    debug_print(f"🎉 过滤协议生成完成! ✨")
    debug_print(f"📊 总动作数: {len(action_sequence)} 个 📝")
    debug_print(f"🥽 过滤容器: {vessel_id} 🧪")
    debug_print(f"🌊 过滤器设备: {filter_device} 🔧")
    debug_print(f"💧 滤液容器: {filtrate_vessel_id or '无（保留固体）'} 🧱")
    debug_print(f"⏱️ 预计总时间: {(len(action_sequence) * 5):.0f} 秒 ⌛")
    if original_liquid_volume > 0:
        debug_print(f"📊 体积变化统计:")
        debug_print(f"  - 过滤前体积: {original_liquid_volume:.2f}mL")
        debug_print(f"  - 过滤后容器体积: {final_vessel_volume:.2f}mL")
        if filtrate_vessel:
            debug_print(f"  - 预计滤液体积: {expected_filtrate_volume:.2f}mL")
        debug_print(f"  - 预计损失体积: {volume_loss:.2f}mL")
    debug_print("🎊" * 20)
    
    return action_sequence
