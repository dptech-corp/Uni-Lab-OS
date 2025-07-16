import networkx as nx
import logging
import sys
from typing import List, Dict, Any, Optional
from .pump_protocol import generate_pump_protocol_with_rinsing

# 设置日志
logger = logging.getLogger(__name__)

# 确保输出编码为UTF-8
if hasattr(sys.stdout, 'reconfigure'):
    try:
        sys.stdout.reconfigure(encoding='utf-8')
        sys.stderr.reconfigure(encoding='utf-8')
    except:
        pass

def debug_print(message):
    """调试输出函数 - 支持中文"""
    try:
        # 确保消息是字符串格式
        safe_message = str(message)
        print(f"[重置处理] {safe_message}", flush=True)
        logger.info(f"[重置处理] {safe_message}")
    except UnicodeEncodeError:
        # 如果编码失败，尝试替换不支持的字符
        safe_message = str(message).encode('utf-8', errors='replace').decode('utf-8')
        print(f"[重置处理] {safe_message}", flush=True)
        logger.info(f"[重置处理] {safe_message}")
    except Exception as e:
        # 最后的安全措施
        fallback_message = f"日志输出错误: {repr(message)}"
        print(f"[重置处理] {fallback_message}", flush=True)
        logger.info(f"[重置处理] {fallback_message}")

def create_action_log(message: str, emoji: str = "📝") -> Dict[str, Any]:
    """创建一个动作日志 - 支持中文和emoji"""
    try:
        full_message = f"{emoji} {message}"
        debug_print(full_message)
        logger.info(full_message)
        
        return {
            "action_name": "wait",
            "action_kwargs": {
                "time": 0.1,
                "log_message": full_message,
                "progress_message": full_message
            }
        }
    except Exception as e:
        # 如果emoji有问题，使用纯文本
        safe_message = f"[日志] {message}"
        debug_print(safe_message)
        logger.info(safe_message)
        
        return {
            "action_name": "wait", 
            "action_kwargs": {
                "time": 0.1,
                "log_message": safe_message,
                "progress_message": safe_message
            }
        }

def find_solvent_vessel(G: nx.DiGraph, solvent: str) -> str:
    """
    查找溶剂容器，支持多种匹配模式
    
    Args:
        G: 网络图
        solvent: 溶剂名称（如 "methanol", "ethanol", "water"）
    
    Returns:
        str: 溶剂容器ID
    """
    debug_print(f"🔍 正在查找溶剂 '{solvent}' 的容器...")
    
    # 构建可能的容器名称
    possible_names = [
        f"flask_{solvent}",           # flask_methanol
        f"bottle_{solvent}",          # bottle_methanol
        f"reagent_{solvent}",         # reagent_methanol
        f"reagent_bottle_{solvent}",  # reagent_bottle_methanol
        f"{solvent}_flask",           # methanol_flask
        f"{solvent}_bottle",          # methanol_bottle
        f"{solvent}",                 # methanol
        f"vessel_{solvent}",          # vessel_methanol
    ]
    
    debug_print(f"🎯 候选容器名称: {possible_names[:3]}... (共{len(possible_names)}个)")
    
    # 第一步：通过容器名称匹配
    debug_print("📋 方法1: 精确名称匹配...")
    for vessel_name in possible_names:
        if vessel_name in G.nodes():
            debug_print(f"✅ 通过名称匹配找到容器: {vessel_name}")
            return vessel_name
    debug_print("⚠️ 精确名称匹配失败，尝试模糊匹配...")
    
    # 第二步：通过模糊匹配
    debug_print("📋 方法2: 模糊名称匹配...")
    for node_id in G.nodes():
        if G.nodes[node_id].get('type') == 'container':
            node_name = G.nodes[node_id].get('name', '').lower()
            
            # 检查是否包含溶剂名称
            if solvent.lower() in node_id.lower() or solvent.lower() in node_name:
                debug_print(f"✅ 通过模糊匹配找到容器: {node_id}")
                return node_id
    debug_print("⚠️ 模糊匹配失败，尝试液体类型匹配...")
    
    # 第三步：通过液体类型匹配
    debug_print("📋 方法3: 液体类型匹配...")
    for node_id in G.nodes():
        if G.nodes[node_id].get('type') == 'container':
            vessel_data = G.nodes[node_id].get('data', {})
            liquids = vessel_data.get('liquid', [])
            
            for liquid in liquids:
                if isinstance(liquid, dict):
                    liquid_type = (liquid.get('liquid_type') or liquid.get('name', '')).lower()
                    reagent_name = vessel_data.get('reagent_name', '').lower()
                    
                    if solvent.lower() in liquid_type or solvent.lower() in reagent_name:
                        debug_print(f"✅ 通过液体类型匹配找到容器: {node_id}")
                        return node_id
    
    # 列出可用容器帮助调试
    debug_print("📊 显示可用容器信息...")
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
    
    debug_print(f"📋 可用容器列表 (共{len(available_containers)}个):")
    for i, container in enumerate(available_containers[:5]):  # 只显示前5个
        debug_print(f"  {i+1}. 🥽 {container['id']}: {container['name']}")
        debug_print(f"     💧 液体: {container['liquids']}")
        debug_print(f"     🧪 试剂: {container['reagent_name']}")
    
    if len(available_containers) > 5:
        debug_print(f"    ... 还有 {len(available_containers)-5} 个容器")
    
    debug_print(f"❌ 找不到溶剂 '{solvent}' 对应的容器")
    raise ValueError(f"找不到溶剂 '{solvent}' 对应的容器。尝试了: {possible_names[:3]}...")

def generate_reset_handling_protocol(
    G: nx.DiGraph,
    solvent: str,
    vessel: Optional[str] = None,  # 🆕 新增可选vessel参数
    **kwargs  # 接收其他可能的参数但不使用
) -> List[Dict[str, Any]]:
    """
    生成重置处理协议序列 - 支持自定义容器
    
    Args:
        G: 有向图，节点为容器和设备
        solvent: 溶剂名称（从XDL传入）
        vessel: 目标容器名称（可选，默认为 "main_reactor"）
        **kwargs: 其他可选参数，但不使用
    
    Returns:
        List[Dict[str, Any]]: 动作序列
    """
    action_sequence = []
    
    # 🔧 修改：支持自定义vessel参数
    target_vessel = vessel if vessel is not None else "main_reactor"  # 默认目标容器
    volume = 50.0  # 默认体积 50 mL

    debug_print("=" * 60)
    debug_print("🚀 开始生成重置处理协议")
    debug_print(f"📋 输入参数:")
    debug_print(f"  🧪 溶剂: {solvent}")
    debug_print(f"  🥽 目标容器: {target_vessel} {'(默认)' if vessel is None else '(指定)'}")
    debug_print(f"  💧 体积: {volume} mL")
    debug_print(f"  ⚙️ 其他参数: {kwargs}")
    debug_print("=" * 60)
    
    # 添加初始日志
    action_sequence.append(create_action_log(f"开始重置处理操作 - 容器: {target_vessel}", "🎬"))
    action_sequence.append(create_action_log(f"使用溶剂: {solvent}", "🧪"))
    action_sequence.append(create_action_log(f"重置体积: {volume}mL", "💧"))
    
    if vessel is None:
        action_sequence.append(create_action_log("使用默认目标容器: main_reactor", "⚙️"))
    else:
        action_sequence.append(create_action_log(f"使用指定目标容器: {vessel}", "🎯"))
    
    # 1. 验证目标容器存在
    debug_print("🔍 步骤1: 验证目标容器...")
    action_sequence.append(create_action_log("正在验证目标容器...", "🔍"))
    
    if target_vessel not in G.nodes():
        debug_print(f"❌ 目标容器 '{target_vessel}' 不存在于系统中!")
        action_sequence.append(create_action_log(f"目标容器 '{target_vessel}' 不存在", "❌"))
        raise ValueError(f"目标容器 '{target_vessel}' 不存在于系统中")
    
    debug_print(f"✅ 目标容器 '{target_vessel}' 验证通过")
    action_sequence.append(create_action_log(f"目标容器验证通过: {target_vessel}", "✅"))
    
    # 2. 查找溶剂容器
    debug_print("🔍 步骤2: 查找溶剂容器...")
    action_sequence.append(create_action_log("正在查找溶剂容器...", "🔍"))
    
    try:
        solvent_vessel = find_solvent_vessel(G, solvent)
        debug_print(f"✅ 找到溶剂容器: {solvent_vessel}")
        action_sequence.append(create_action_log(f"找到溶剂容器: {solvent_vessel}", "✅"))
    except ValueError as e:
        debug_print(f"❌ 溶剂容器查找失败: {str(e)}")
        action_sequence.append(create_action_log(f"溶剂容器查找失败: {str(e)}", "❌"))
        raise ValueError(f"无法找到溶剂 '{solvent}': {str(e)}")
    
    # 3. 验证路径存在
    debug_print("🔍 步骤3: 验证传输路径...")
    action_sequence.append(create_action_log("正在验证传输路径...", "🛤️"))
    
    try:
        path = nx.shortest_path(G, source=solvent_vessel, target=target_vessel)
        debug_print(f"✅ 找到路径: {' → '.join(path)}")
        action_sequence.append(create_action_log(f"传输路径: {' → '.join(path)}", "🛤️"))
    except nx.NetworkXNoPath:
        debug_print(f"❌ 路径不可达: {solvent_vessel} → {target_vessel}")
        action_sequence.append(create_action_log(f"路径不可达: {solvent_vessel} → {target_vessel}", "❌"))
        raise ValueError(f"从溶剂容器 '{solvent_vessel}' 到目标容器 '{target_vessel}' 没有可用路径")
    
    # 4. 使用pump_protocol转移溶剂
    debug_print("🔍 步骤4: 转移溶剂...")
    action_sequence.append(create_action_log("开始溶剂转移操作...", "🚰"))
    
    debug_print(f"🚛 开始转移: {solvent_vessel} → {target_vessel}")
    debug_print(f"💧 转移体积: {volume} mL")
    action_sequence.append(create_action_log(f"转移: {solvent_vessel} → {target_vessel} ({volume}mL)", "🚛"))
    
    try:
        debug_print("🔄 生成泵送协议...")
        action_sequence.append(create_action_log("正在生成泵送协议...", "🔄"))
        
        pump_actions = generate_pump_protocol_with_rinsing(
            G=G,
            from_vessel=solvent_vessel,
            to_vessel=target_vessel,
            volume=volume,
            amount="",
            time=0.0,
            viscous=False,
            rinsing_solvent="",  # 重置处理不需要清洗
            rinsing_volume=0.0,
            rinsing_repeats=0,
            solid=False,
            flowrate=2.5,  # 正常流速
            transfer_flowrate=0.5  # 正常转移流速
        )
        
        action_sequence.extend(pump_actions)
        debug_print(f"✅ 泵送协议已添加: {len(pump_actions)} 个动作")
        action_sequence.append(create_action_log(f"泵送协议完成 ({len(pump_actions)} 个操作)", "✅"))
        
    except Exception as e:
        debug_print(f"❌ 泵送协议生成失败: {str(e)}")
        action_sequence.append(create_action_log(f"泵送协议生成失败: {str(e)}", "❌"))
        raise ValueError(f"生成泵协议时出错: {str(e)}")
    
    # 5. 等待溶剂稳定
    debug_print("🔍 步骤5: 等待溶剂稳定...")
    action_sequence.append(create_action_log("等待溶剂稳定...", "⏳"))
    
    # 模拟运行时间优化
    debug_print("⏱️ 检查模拟运行时间限制...")
    original_wait_time = 10.0  # 原始等待时间
    simulation_time_limit = 5.0  # 模拟运行时间限制：5秒
    
    final_wait_time = min(original_wait_time, simulation_time_limit)
    
    if original_wait_time > simulation_time_limit:
        debug_print(f"🎮 模拟运行优化: {original_wait_time}s → {final_wait_time}s")
        action_sequence.append(create_action_log(f"时间优化: {original_wait_time}s → {final_wait_time}s", "⚡"))
    else:
        debug_print(f"✅ 时间在限制内: {final_wait_time}s 保持不变")
        action_sequence.append(create_action_log(f"等待时间: {final_wait_time}s", "⏰"))
    
    action_sequence.append({
        "action_name": "wait",
        "action_kwargs": {
            "time": final_wait_time,
            "description": f"等待溶剂 {solvent} 在容器 {target_vessel} 中稳定" + (f" (模拟时间)" if original_wait_time != final_wait_time else "")
        }
    })
    debug_print(f"✅ 稳定等待已添加: {final_wait_time}s")
    
    # 显示时间调整信息
    if original_wait_time != final_wait_time:
        debug_print(f"🎭 模拟优化说明: 原计划 {original_wait_time}s，实际模拟 {final_wait_time}s")
        action_sequence.append(create_action_log("应用模拟时间优化", "🎭"))
    
    # 总结
    debug_print("=" * 60)
    debug_print(f"🎉 重置处理协议生成完成!")
    debug_print(f"📊 总结信息:")
    debug_print(f"  📋 总动作数: {len(action_sequence)} 个")
    debug_print(f"  🧪 溶剂: {solvent}")
    debug_print(f"  🥽 源容器: {solvent_vessel}")
    debug_print(f"  🥽 目标容器: {target_vessel} {'(默认)' if vessel is None else '(指定)'}")
    debug_print(f"  💧 转移体积: {volume} mL")
    debug_print(f"  ⏱️ 预计总时间: {(final_wait_time + 5):.0f} 秒")
    debug_print(f"  🎯 操作结果: 已添加 {volume} mL {solvent} 到 {target_vessel}")
    debug_print("=" * 60)
    
    # 添加完成日志
    summary_msg = f"重置处理完成: {target_vessel} (使用 {volume}mL {solvent})"
    if vessel is None:
        summary_msg += " [默认容器]"
    else:
        summary_msg += " [指定容器]"
    
    action_sequence.append(create_action_log(summary_msg, "🎉"))
    
    return action_sequence

# === 便捷函数 ===

def reset_main_reactor(G: nx.DiGraph, solvent: str = "methanol", **kwargs) -> List[Dict[str, Any]]:
    """重置主反应器 (默认行为)"""
    debug_print(f"🔄 重置主反应器，使用溶剂: {solvent}")
    return generate_reset_handling_protocol(G, solvent=solvent, vessel=None, **kwargs)

def reset_custom_vessel(G: nx.DiGraph, vessel: str, solvent: str = "methanol", **kwargs) -> List[Dict[str, Any]]:
    """重置指定容器"""
    debug_print(f"🔄 重置指定容器: {vessel}，使用溶剂: {solvent}")
    return generate_reset_handling_protocol(G, solvent=solvent, vessel=vessel, **kwargs)

def reset_with_water(G: nx.DiGraph, vessel: Optional[str] = None, **kwargs) -> List[Dict[str, Any]]:
    """使用水重置容器"""
    target = vessel or "main_reactor"
    debug_print(f"💧 使用水重置容器: {target}")
    return generate_reset_handling_protocol(G, solvent="water", vessel=vessel, **kwargs)

def reset_with_methanol(G: nx.DiGraph, vessel: Optional[str] = None, **kwargs) -> List[Dict[str, Any]]:
    """使用甲醇重置容器"""
    target = vessel or "main_reactor"
    debug_print(f"🧪 使用甲醇重置容器: {target}")
    return generate_reset_handling_protocol(G, solvent="methanol", vessel=vessel, **kwargs)

def reset_with_ethanol(G: nx.DiGraph, vessel: Optional[str] = None, **kwargs) -> List[Dict[str, Any]]:
    """使用乙醇重置容器"""
    target = vessel or "main_reactor"
    debug_print(f"🧪 使用乙醇重置容器: {target}")
    return generate_reset_handling_protocol(G, solvent="ethanol", vessel=vessel, **kwargs)

# 测试函数
def test_reset_handling_protocol():
    """测试重置处理协议"""
    debug_print("=== 重置处理协议增强中文版测试 ===")
    
    # 测试溶剂名称
    debug_print("🧪 测试常用溶剂名称...")
    test_solvents = ["methanol", "ethanol", "water", "acetone", "dmso"]
    for solvent in test_solvents:
        debug_print(f"  🔍 测试溶剂: {solvent}")
    
    # 测试容器参数
    debug_print("🥽 测试容器参数...")
    test_cases = [
        {"solvent": "methanol", "vessel": None, "desc": "默认容器"},
        {"solvent": "ethanol", "vessel": "reactor_2", "desc": "指定容器"},
        {"solvent": "water", "vessel": "flask_1", "desc": "自定义容器"}
    ]
    
    for case in test_cases:
        debug_print(f"  🧪 测试案例: {case['desc']} - {case['solvent']} -> {case['vessel'] or 'main_reactor'}")
    
    debug_print("✅ 测试完成")

if __name__ == "__main__":
    test_reset_handling_protocol()