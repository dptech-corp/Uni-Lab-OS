import numpy as np
import networkx as nx
import asyncio
import time as time_module  # 🔧 重命名time模块
from typing import List, Dict, Any
import logging
import sys

logger = logging.getLogger(__name__)

def debug_print(message):
    """强制输出调试信息"""
    timestamp = time_module.strftime("%H:%M:%S")
    output = f"[{timestamp}] {message}"
    print(output, flush=True)
    sys.stdout.flush()
    # 同时写入日志
    logger.info(output)

def get_vessel_liquid_volume(G: nx.DiGraph, vessel: str) -> float:
    """
    从容器节点的数据中获取液体体积
    """
    debug_print(f"🔍 开始读取容器 '{vessel}' 的液体体积...")
    
    if vessel not in G.nodes():
        logger.error(f"❌ 容器 '{vessel}' 不存在于系统图中")
        debug_print(f"  - 系统中的容器: {list(G.nodes())}")
        return 0.0
    
    vessel_data = G.nodes[vessel].get('data', {})
    debug_print(f"📋 容器 '{vessel}' 的数据结构: {vessel_data}")
    
    total_volume = 0.0
    
    # 方法1：检查 'liquid' 字段（列表格式）
    debug_print("🔍 方法1: 检查 'liquid' 字段...")
    if 'liquid' in vessel_data:
        liquids = vessel_data['liquid']
        debug_print(f"  - liquid 字段类型: {type(liquids)}")
        debug_print(f"  - liquid 字段内容: {liquids}")
        
        if isinstance(liquids, list):
            debug_print(f"  - liquid 是列表，包含 {len(liquids)} 个元素")
            for i, liquid in enumerate(liquids):
                debug_print(f"    液体 {i+1}: {liquid}")
                if isinstance(liquid, dict):
                    volume_keys = ['liquid_volume', 'volume', 'amount', 'quantity']
                    for key in volume_keys:
                        if key in liquid:
                            try:
                                vol = float(liquid[key])
                                total_volume += vol
                                debug_print(f"    ✅ 从 '{key}' 读取体积: {vol}mL")
                                break
                            except (ValueError, TypeError) as e:
                                logger.warning(f"    ⚠️ 无法转换 '{key}': {liquid[key]} -> {str(e)}")
                                continue
        else:
            debug_print(f"  - liquid 不是列表: {type(liquids)}")
    else:
        debug_print("  - 没有 'liquid' 字段")
    
    # 方法2：检查直接的体积字段
    debug_print("🔍 方法2: 检查直接体积字段...")
    volume_keys = ['total_volume', 'volume', 'liquid_volume', 'amount', 'current_volume']
    for key in volume_keys:
        if key in vessel_data:
            try:
                vol = float(vessel_data[key])
                total_volume = max(total_volume, vol)  # 取最大值
                debug_print(f"  ✅ 从容器数据 '{key}' 读取体积: {vol}mL")
                break
            except (ValueError, TypeError) as e:
                logger.warning(f"  ⚠️ 无法转换 '{key}': {vessel_data[key]} -> {str(e)}")
                continue
    
    # 方法3：检查 'state' 或 'status' 字段
    debug_print("🔍 方法3: 检查 'state' 字段...")
    if 'state' in vessel_data and isinstance(vessel_data['state'], dict):
        state = vessel_data['state']
        debug_print(f"  - state 字段内容: {state}")
        if 'volume' in state:
            try:
                vol = float(state['volume'])
                total_volume = max(total_volume, vol)
                debug_print(f"  ✅ 从容器状态读取体积: {vol}mL")
            except (ValueError, TypeError) as e:
                logger.warning(f"  ⚠️ 无法转换 state.volume: {state['volume']} -> {str(e)}")
    else:
        debug_print("  - 没有 'state' 字段或不是字典")
    
    debug_print(f"📊 容器 '{vessel}' 最终检测体积: {total_volume}mL")
    return total_volume

def is_integrated_pump(node_name):
    return "pump" in node_name and "valve" in node_name


def find_connected_pump(G, valve_node):
    for neighbor in G.neighbors(valve_node):
        node_class = G.nodes[neighbor].get("class") or ""
        if "pump" in node_class:
            return neighbor
    raise ValueError(f"未找到与阀 {valve_node} 唯一相连的泵节点")


def build_pump_valve_maps(G, pump_backbone):
    pumps_from_node = {}
    valve_from_node = {}
    for node in pump_backbone:
        if is_integrated_pump(node):
            pumps_from_node[node] = node
            valve_from_node[node] = node
        else:
            pump_node = find_connected_pump(G, node)
            pumps_from_node[node] = pump_node
            valve_from_node[node] = node
    return pumps_from_node, valve_from_node


def generate_pump_protocol(
    G: nx.DiGraph,
    from_vessel: str,
    to_vessel: str,
    volume: float,
    flowrate: float = 2.5,
    transfer_flowrate: float = 0.5,
) -> List[Dict[str, Any]]:
    """
    生成泵操作的动作序列
    """
    pump_action_sequence = []
    nodes = G.nodes(data=True)
    
    # 验证输入参数
    if volume <= 0:
        logger.error(f"无效的体积参数: {volume}mL")
        return pump_action_sequence
    
    if flowrate <= 0:
        flowrate = 2.5
        logger.warning(f"flowrate <= 0，使用默认值 {flowrate}mL/s")
    
    if transfer_flowrate <= 0:
        transfer_flowrate = 0.5
        logger.warning(f"transfer_flowrate <= 0，使用默认值 {transfer_flowrate}mL/s")
    
    # 验证容器存在
    if from_vessel not in G.nodes():
        logger.error(f"源容器 '{from_vessel}' 不存在")
        return pump_action_sequence
        
    if to_vessel not in G.nodes():
        logger.error(f"目标容器 '{to_vessel}' 不存在")
        return pump_action_sequence
    
    try:
        shortest_path = nx.shortest_path(G, source=from_vessel, target=to_vessel)
        debug_print(f"PUMP_TRANSFER: 路径 {from_vessel} -> {to_vessel}: {shortest_path}")
    except nx.NetworkXNoPath:
        logger.error(f"无法找到从 '{from_vessel}' 到 '{to_vessel}' 的路径")
        return pump_action_sequence

    pump_backbone = shortest_path
    if not from_vessel.startswith("pump"):
        pump_backbone = pump_backbone[1:]
    if not to_vessel.startswith("pump"):
        pump_backbone = pump_backbone[:-1]

    if not pump_backbone:
        debug_print("没有泵骨架节点，可能是直接容器连接")
        return pump_action_sequence

    if transfer_flowrate == 0:
        transfer_flowrate = flowrate

    pumps_from_node, valve_from_node = build_pump_valve_maps(G, pump_backbone)

    # 获取最小转移体积
    try:
        min_transfer_volume = min([nodes[pumps_from_node[node]]["config"]["max_volume"] for node in pump_backbone])
    except (KeyError, TypeError):
        min_transfer_volume = 25.0  # 默认值

    repeats = int(np.ceil(volume / min_transfer_volume))
    
    if repeats > 1 and (from_vessel.startswith("pump") or to_vessel.startswith("pump")):
        logger.error("Cannot transfer volume larger than min_transfer_volume between two pumps.")
        return pump_action_sequence

    volume_left = volume
    debug_print(f"PUMP_TRANSFER: 需要 {repeats} 次转移，单次最大体积 {min_transfer_volume} mL")

    # 生成泵操作序列
    for i in range(repeats):
        current_volume = min(volume_left, min_transfer_volume)
        
        # 从源容器吸液
        if not from_vessel.startswith("pump"):
            pump_action_sequence.extend([
                {
                    "device_id": valve_from_node[pump_backbone[0]],
                    "action_name": "set_valve_position",
                    "action_kwargs": {
                        "command": G.get_edge_data(pump_backbone[0], from_vessel)["port"][pump_backbone[0]]
                    }
                },
                {
                    "device_id": pumps_from_node[pump_backbone[0]],
                    "action_name": "set_position",
                    "action_kwargs": {
                        "position": float(current_volume),
                        "max_velocity": transfer_flowrate
                    }
                }
            ])
            pump_action_sequence.append({"action_name": "wait", "action_kwargs": {"time": 3}})
        
        # 泵间转移
        for nodeA, nodeB in zip(pump_backbone[:-1], pump_backbone[1:]):
            pump_action_sequence.append([
                {
                    "device_id": valve_from_node[nodeA],
                    "action_name": "set_valve_position",
                    "action_kwargs": {
                        "command": G.get_edge_data(nodeA, nodeB)["port"][nodeA]
                    }
                },
                {
                    "device_id": valve_from_node[nodeB],
                    "action_name": "set_valve_position",
                    "action_kwargs": {
                        "command": G.get_edge_data(nodeB, nodeA)["port"][nodeB],
                    }
                }
            ])
            pump_action_sequence.append([
                {
                    "device_id": pumps_from_node[nodeA],
                    "action_name": "set_position",
                    "action_kwargs": {
                        "position": 0.0,
                        "max_velocity": transfer_flowrate
                    }
                },
                {
                    "device_id": pumps_from_node[nodeB],
                    "action_name": "set_position",
                    "action_kwargs": {
                        "position": float(current_volume),
                        "max_velocity": transfer_flowrate
                    }
                }
            ])
            pump_action_sequence.append({"action_name": "wait", "action_kwargs": {"time": 3}})

        # 排液到目标容器
        if not to_vessel.startswith("pump"):
            pump_action_sequence.extend([
                {
                    "device_id": valve_from_node[pump_backbone[-1]],
                    "action_name": "set_valve_position",
                    "action_kwargs": {
                        "command": G.get_edge_data(pump_backbone[-1], to_vessel)["port"][pump_backbone[-1]]
                    }
                },
                {
                    "device_id": pumps_from_node[pump_backbone[-1]],
                    "action_name": "set_position",
                    "action_kwargs": {
                        "position": 0.0,
                        "max_velocity": flowrate
                    }
                }
            ])
            pump_action_sequence.append({"action_name": "wait", "action_kwargs": {"time": 3}})

        volume_left -= current_volume
    
    return pump_action_sequence


def generate_pump_protocol_with_rinsing(
    G: nx.DiGraph,
    from_vessel: str,
    to_vessel: str,
    volume: float = 0.0,
    amount: str = "",
    duration: float = 0.0,  # 🔧 重命名参数，避免冲突
    viscous: bool = False,
    rinsing_solvent: str = "",
    rinsing_volume: float = 0.0,
    rinsing_repeats: int = 0,
    solid: bool = False,
    flowrate: float = 2.5,
    transfer_flowrate: float = 0.5,
    rate_spec: str = "",
    event: str = "",
    through: str = "",
    **kwargs
) -> List[Dict[str, Any]]:
    """
    增强兼容性的泵转移协议生成器，支持自动体积检测
    """
    debug_print("=" * 60)
    debug_print(f"PUMP_TRANSFER: 🚀 开始生成协议")
    debug_print(f"  📍 路径: {from_vessel} -> {to_vessel}")
    debug_print(f"  🕐 时间戳: {time_module.time()}")  # 🔧 使用重命名的模块
    debug_print(f"  📊 原始参数:")
    debug_print(f"    - volume: {volume} (类型: {type(volume)})")
    debug_print(f"    - amount: '{amount}'")
    debug_print(f"    - duration: {duration}")  # 🔧 使用新的参数名
    debug_print(f"    - flowrate: {flowrate}")
    debug_print(f"    - transfer_flowrate: {transfer_flowrate}")
    debug_print(f"    - rate_spec: '{rate_spec}'")
    debug_print("=" * 60)
    
    # ========== 🔧 核心修复：智能体积处理 ==========
    
    debug_print("🔍 步骤1: 开始体积处理...")
    
    # 1. 处理体积参数
    final_volume = volume
    debug_print(f"📋 初始设置: final_volume = {final_volume}")
    
    # 🔧 修复：如果volume为0（ROS2传入的空值），从容器读取实际体积
    if volume == 0.0:
        debug_print("🎯 检测到 volume=0.0，开始自动体积检测...")
        
        # 直接从源容器读取实际体积
        actual_volume = get_vessel_liquid_volume(G, from_vessel)
        debug_print(f"📖 从容器 '{from_vessel}' 读取到体积: {actual_volume}mL")
        
        if actual_volume > 0:
            final_volume = actual_volume
            debug_print(f"✅ 成功设置体积为: {final_volume}mL")
        else:
            final_volume = 10.0  # 如果读取失败，使用默认值
            debug_print(f"⚠️ 无法从容器读取体积，使用默认值: {final_volume}mL")
    else:
        debug_print(f"📌 体积非零，直接使用: {final_volume}mL")
    
    # 处理 amount 参数
    if amount and amount.strip():
        debug_print(f"🔍 检测到 amount 参数: '{amount}'，开始解析...")
        parsed_volume = _parse_amount_to_volume(amount)
        debug_print(f"📖 从 amount 解析得到体积: {parsed_volume}mL")
        
        if parsed_volume > 0:
            final_volume = parsed_volume
            debug_print(f"✅ 使用从 amount 解析的体积: {final_volume}mL")
        elif parsed_volume == 0.0 and amount.lower().strip() == "all":
            debug_print("🎯 检测到 amount='all'，从容器读取全部体积...")
            actual_volume = get_vessel_liquid_volume(G, from_vessel)
            if actual_volume > 0:
                final_volume = actual_volume
                debug_print(f"✅ amount='all'，设置体积为: {final_volume}mL")
    
    # 最终体积验证
    debug_print(f"🔍 步骤2: 最终体积验证...")
    if final_volume <= 0:
        debug_print(f"❌ 体积无效: {final_volume}mL")
        final_volume = 10.0
        debug_print(f"⚠️ 强制设置为默认值: {final_volume}mL")
    
    debug_print(f"✅ 最终确定体积: {final_volume}mL")
    
    # 2. 处理流速参数
    debug_print(f"🔍 步骤3: 处理流速参数...")
    debug_print(f"  - 原始 flowrate: {flowrate}")
    debug_print(f"  - 原始 transfer_flowrate: {transfer_flowrate}")
    
    final_flowrate = flowrate if flowrate > 0 else 2.5
    final_transfer_flowrate = transfer_flowrate if transfer_flowrate > 0 else 0.5
    
    if flowrate <= 0:
        debug_print(f"⚠️ flowrate <= 0，修正为: {final_flowrate}mL/s")
    if transfer_flowrate <= 0:
        debug_print(f"⚠️ transfer_flowrate <= 0，修正为: {final_transfer_flowrate}mL/s")
    
    debug_print(f"✅ 修正后流速: flowrate={final_flowrate}mL/s, transfer_flowrate={final_transfer_flowrate}mL/s")
    
    # 3. 根据时间计算流速
    if duration > 0 and final_volume > 0:  # 🔧 使用duration而不是time
        debug_print(f"🔍 步骤4: 根据时间计算流速...")
        calculated_flowrate = final_volume / duration
        debug_print(f"  - 计算得到流速: {calculated_flowrate}mL/s")
        
        if flowrate <= 0 or flowrate == 2.5:
            final_flowrate = min(calculated_flowrate, 10.0)
            debug_print(f"  - 调整 flowrate 为: {final_flowrate}mL/s")
        if transfer_flowrate <= 0 or transfer_flowrate == 0.5:
            final_transfer_flowrate = min(calculated_flowrate, 5.0)
            debug_print(f"  - 调整 transfer_flowrate 为: {final_transfer_flowrate}mL/s")
    
    # 4. 根据速度规格调整
    if rate_spec:
        debug_print(f"🔍 步骤5: 根据速度规格调整...")
        debug_print(f"  - 速度规格: '{rate_spec}'")
        
        if rate_spec == "dropwise":
            final_flowrate = min(final_flowrate, 0.1)
            final_transfer_flowrate = min(final_transfer_flowrate, 0.1)
            debug_print(f"  - dropwise模式，流速调整为: {final_flowrate}mL/s")
        elif rate_spec == "slowly":
            final_flowrate = min(final_flowrate, 0.5)
            final_transfer_flowrate = min(final_transfer_flowrate, 0.3)
            debug_print(f"  - slowly模式，流速调整为: {final_flowrate}mL/s")
        elif rate_spec == "quickly":
            final_flowrate = max(final_flowrate, 5.0)
            final_transfer_flowrate = max(final_transfer_flowrate, 2.0)
            debug_print(f"  - quickly模式，流速调整为: {final_flowrate}mL/s")
    
    # 5. 处理冲洗参数
    debug_print(f"🔍 步骤6: 处理冲洗参数...")
    final_rinsing_solvent = rinsing_solvent
    final_rinsing_volume = rinsing_volume if rinsing_volume > 0 else 5.0
    final_rinsing_repeats = rinsing_repeats if rinsing_repeats > 0 else 2
    
    if rinsing_volume <= 0:
        debug_print(f"⚠️ rinsing_volume <= 0，修正为: {final_rinsing_volume}mL")
    if rinsing_repeats <= 0:
        debug_print(f"⚠️ rinsing_repeats <= 0，修正为: {final_rinsing_repeats}次")
    
    # 根据物理属性调整冲洗参数
    if viscous or solid:
        final_rinsing_repeats = max(final_rinsing_repeats, 3)
        final_rinsing_volume = max(final_rinsing_volume, 10.0)
        debug_print(f"🧪 粘稠/固体物质，调整冲洗参数：{final_rinsing_repeats}次，{final_rinsing_volume}mL")
    
    # 参数总结
    debug_print("📊 最终参数总结:")
    debug_print(f"  - 体积: {final_volume}mL")
    debug_print(f"  - 流速: {final_flowrate}mL/s")
    debug_print(f"  - 转移流速: {final_transfer_flowrate}mL/s")
    debug_print(f"  - 冲洗溶剂: '{final_rinsing_solvent}'")
    debug_print(f"  - 冲洗体积: {final_rinsing_volume}mL")
    debug_print(f"  - 冲洗次数: {final_rinsing_repeats}次")
    
    # ========== 执行基础转移 ==========
    
    debug_print("🔧 步骤7: 开始执行基础转移...")
    
    try:
        debug_print(f"  - 调用 generate_pump_protocol...")
        debug_print(f"  - 参数: G, '{from_vessel}', '{to_vessel}', {final_volume}, {final_flowrate}, {final_transfer_flowrate}")
        
        pump_action_sequence = generate_pump_protocol(
            G, from_vessel, to_vessel, final_volume,
            final_flowrate, final_transfer_flowrate
        )
        
        debug_print(f"  - generate_pump_protocol 返回结果:")
        debug_print(f"    - 动作序列长度: {len(pump_action_sequence)}")
        debug_print(f"    - 动作序列是否为空: {len(pump_action_sequence) == 0}")
        
        if not pump_action_sequence:
            debug_print("❌ 基础转移协议生成为空，可能是路径问题")
            debug_print(f"  - 源容器存在: {from_vessel in G.nodes()}")
            debug_print(f"  - 目标容器存在: {to_vessel in G.nodes()}")
            
            if from_vessel in G.nodes() and to_vessel in G.nodes():
                try:
                    path = nx.shortest_path(G, source=from_vessel, target=to_vessel)
                    debug_print(f"  - 路径存在: {path}")
                except Exception as path_error:
                    debug_print(f"  - 无法找到路径: {str(path_error)}")
            
            return [
                {
                    "device_id": "system",
                    "action_name": "log_message",
                    "action_kwargs": {
                        "message": f"⚠️ 路径问题，无法转移: {final_volume}mL 从 {from_vessel} 到 {to_vessel}"
                    }
                }
            ]
        
        debug_print(f"✅ 基础转移生成了 {len(pump_action_sequence)} 个动作")
        
        # 打印前几个动作用于调试
        if len(pump_action_sequence) > 0:
            debug_print("🔍 前几个动作预览:")
            for i, action in enumerate(pump_action_sequence[:3]):
                debug_print(f"  动作 {i+1}: {action}")
            if len(pump_action_sequence) > 3:
                debug_print(f"  ... 还有 {len(pump_action_sequence) - 3} 个动作")
            
    except Exception as e:
        debug_print(f"❌ 基础转移失败: {str(e)}")
        import traceback
        debug_print(f"详细错误: {traceback.format_exc()}")
        return [
            {
                "device_id": "system",
                "action_name": "log_message",
                "action_kwargs": {
                    "message": f"❌ 转移失败: {final_volume}mL 从 {from_vessel} 到 {to_vessel}, 错误: {str(e)}"
                }
            }
        ]
    
    # ========== 执行冲洗操作 ==========
    
    debug_print("🔧 步骤8: 检查冲洗操作...")
    
    if final_rinsing_solvent and final_rinsing_solvent.strip() and final_rinsing_repeats > 0:
        debug_print(f"🧽 开始冲洗操作，溶剂: '{final_rinsing_solvent}'")
        
        try:
            if final_rinsing_solvent.strip() != "air":
                debug_print("  - 执行液体冲洗...")
                rinsing_actions = _generate_rinsing_sequence(
                    G, from_vessel, to_vessel, final_rinsing_solvent,
                    final_rinsing_volume, final_rinsing_repeats,
                    final_flowrate, final_transfer_flowrate
                )
                pump_action_sequence.extend(rinsing_actions)
                debug_print(f"  - 添加了 {len(rinsing_actions)} 个冲洗动作")
            else:
                debug_print("  - 执行空气冲洗...")
                air_rinsing_actions = _generate_air_rinsing_sequence(
                    G, from_vessel, to_vessel, final_rinsing_volume, final_rinsing_repeats,
                    final_flowrate, final_transfer_flowrate
                )
                pump_action_sequence.extend(air_rinsing_actions)
                debug_print(f"  - 添加了 {len(air_rinsing_actions)} 个空气冲洗动作")
        except Exception as e:
            debug_print(f"⚠️ 冲洗操作失败: {str(e)}，跳过冲洗")
    else:
        debug_print(f"⏭️ 跳过冲洗操作")
        debug_print(f"  - 溶剂: '{final_rinsing_solvent}'")
        debug_print(f"  - 次数: {final_rinsing_repeats}")
        debug_print(f"  - 条件满足: {bool(final_rinsing_solvent and final_rinsing_solvent.strip() and final_rinsing_repeats > 0)}")
    
    # ========== 最终结果 ==========
    
    debug_print("=" * 60)
    debug_print(f"🎉 PUMP_TRANSFER: 协议生成完成")
    debug_print(f"  📊 总动作数: {len(pump_action_sequence)}")
    debug_print(f"  📋 最终体积: {final_volume}mL")
    debug_print(f"  🚀 执行路径: {from_vessel} -> {to_vessel}")
    
    # 最终验证
    if len(pump_action_sequence) == 0:
        debug_print("🚨 协议生成结果为空！这是异常情况")
        return [
            {
                "device_id": "system",
                "action_name": "log_message",
                "action_kwargs": {
                    "message": f"🚨 协议生成失败: 无法生成任何动作序列"
                }
            }
        ]
    
    debug_print("=" * 60)
    return pump_action_sequence


async def generate_pump_protocol_with_rinsing_async(
    G: nx.DiGraph,
    from_vessel: str,
    to_vessel: str,
    volume: float = 0.0,
    amount: str = "",
    time: float = 0.0,
    viscous: bool = False,
    rinsing_solvent: str = "",
    rinsing_volume: float = 0.0,
    rinsing_repeats: int = 0,
    solid: bool = False,
    flowrate: float = 2.5,
    transfer_flowrate: float = 0.5,
    rate_spec: str = "",
    event: str = "",
    through: str = "",
    **kwargs
) -> List[Dict[str, Any]]:
    """
    异步版本的泵转移协议生成器，避免并发问题
    """
    debug_print("=" * 60)
    debug_print(f"PUMP_TRANSFER: 🚀 开始生成协议 (异步版本)")
    debug_print(f"  📍 路径: {from_vessel} -> {to_vessel}")
    debug_print(f"  🕐 时间戳: {time_module.time()}")
    debug_print("=" * 60)
    
    # 添加唯一标识符
    protocol_id = f"pump_transfer_{int(time_module.time() * 1000000)}"
    debug_print(f"📋 协议ID: {protocol_id}")
    
    # 调用原有的同步版本
    result = generate_pump_protocol_with_rinsing(
        G, from_vessel, to_vessel, volume, amount, time, viscous,
        rinsing_solvent, rinsing_volume, rinsing_repeats, solid,
        flowrate, transfer_flowrate, rate_spec, event, through, **kwargs
    )
    
    # 为每个动作添加唯一标识
    for i, action in enumerate(result):
        if isinstance(action, dict):
            action['_protocol_id'] = protocol_id
            action['_action_sequence'] = i
            action['_timestamp'] = time_module.time()
    
    debug_print(f"📊 协议 {protocol_id} 生成完成，共 {len(result)} 个动作")
    return result

# 保持原有的同步版本兼容性
def generate_pump_protocol_with_rinsing(
    G: nx.DiGraph,
    from_vessel: str,
    to_vessel: str,
    volume: float = 0.0,
    amount: str = "",
    time: float = 0.0,
    viscous: bool = False,
    rinsing_solvent: str = "",
    rinsing_volume: float = 0.0,
    rinsing_repeats: int = 0,
    solid: bool = False,
    flowrate: float = 2.5,
    transfer_flowrate: float = 0.5,
    rate_spec: str = "",
    event: str = "",
    through: str = "",
    **kwargs
) -> List[Dict[str, Any]]:
    """
    原有的同步版本，添加防冲突机制
    """
    
    # 添加执行锁，防止并发调用
    import threading
    if not hasattr(generate_pump_protocol_with_rinsing, '_lock'):
        generate_pump_protocol_with_rinsing._lock = threading.Lock()
    
    with generate_pump_protocol_with_rinsing._lock:
        debug_print("=" * 60)
        debug_print(f"PUMP_TRANSFER: 🚀 开始生成协议 (同步版本)")
        debug_print(f"  📍 路径: {from_vessel} -> {to_vessel}")
        debug_print(f"  🕐 时间戳: {time_module.time()}")
        debug_print(f"  🔒 获得执行锁")
        debug_print("=" * 60)
        
        # 短暂延迟，避免快速重复调用
        time_module.sleep(0.01)
        
        debug_print("🔍 步骤1: 开始体积处理...")
        
        # 1. 处理体积参数
        final_volume = volume
        debug_print(f"📋 初始设置: final_volume = {final_volume}")
        
        # 🔧 修复：如果volume为0（ROS2传入的空值），从容器读取实际体积
        if volume == 0.0:
            debug_print("🎯 检测到 volume=0.0，开始自动体积检测...")
            
            # 直接从源容器读取实际体积
            actual_volume = get_vessel_liquid_volume(G, from_vessel)
            debug_print(f"📖 从容器 '{from_vessel}' 读取到体积: {actual_volume}mL")
            
            if actual_volume > 0:
                final_volume = actual_volume
                debug_print(f"✅ 成功设置体积为: {final_volume}mL")
            else:
                final_volume = 10.0  # 如果读取失败，使用默认值
                logger.warning(f"⚠️ 无法从容器读取体积，使用默认值: {final_volume}mL")
        else:
            debug_print(f"📌 体积非零，直接使用: {final_volume}mL")
        
        # 处理 amount 参数
        if amount and amount.strip():
            debug_print(f"🔍 检测到 amount 参数: '{amount}'，开始解析...")
            parsed_volume = _parse_amount_to_volume(amount)
            debug_print(f"📖 从 amount 解析得到体积: {parsed_volume}mL")
            
            if parsed_volume > 0:
                final_volume = parsed_volume
                debug_print(f"✅ 使用从 amount 解析的体积: {final_volume}mL")
            elif parsed_volume == 0.0 and amount.lower().strip() == "all":
                debug_print("🎯 检测到 amount='all'，从容器读取全部体积...")
                actual_volume = get_vessel_liquid_volume(G, from_vessel)
                if actual_volume > 0:
                    final_volume = actual_volume
                    debug_print(f"✅ amount='all'，设置体积为: {final_volume}mL")
        
        # 最终体积验证
        debug_print(f"🔍 步骤2: 最终体积验证...")
        if final_volume <= 0:
            logger.error(f"❌ 体积无效: {final_volume}mL")
            final_volume = 10.0
            logger.warning(f"⚠️ 强制设置为默认值: {final_volume}mL")
        
        debug_print(f"✅ 最终确定体积: {final_volume}mL")
        
        # 2. 处理流速参数
        debug_print(f"🔍 步骤3: 处理流速参数...")
        debug_print(f"  - 原始 flowrate: {flowrate}")
        debug_print(f"  - 原始 transfer_flowrate: {transfer_flowrate}")
        
        final_flowrate = flowrate if flowrate > 0 else 2.5
        final_transfer_flowrate = transfer_flowrate if transfer_flowrate > 0 else 0.5
        
        if flowrate <= 0:
            logger.warning(f"⚠️ flowrate <= 0，修正为: {final_flowrate}mL/s")
        if transfer_flowrate <= 0:
            logger.warning(f"⚠️ transfer_flowrate <= 0，修正为: {final_transfer_flowrate}mL/s")
        
        debug_print(f"✅ 修正后流速: flowrate={final_flowrate}mL/s, transfer_flowrate={final_transfer_flowrate}mL/s")
        
        # 3. 根据时间计算流速
        if time > 0 and final_volume > 0:
            debug_print(f"🔍 步骤4: 根据时间计算流速...")
            calculated_flowrate = final_volume / time
            debug_print(f"  - 计算得到流速: {calculated_flowrate}mL/s")
            
            if flowrate <= 0 or flowrate == 2.5:
                final_flowrate = min(calculated_flowrate, 10.0)
                debug_print(f"  - 调整 flowrate 为: {final_flowrate}mL/s")
            if transfer_flowrate <= 0 or transfer_flowrate == 0.5:
                final_transfer_flowrate = min(calculated_flowrate, 5.0)
                debug_print(f"  - 调整 transfer_flowrate 为: {final_transfer_flowrate}mL/s")
        
        # 4. 根据速度规格调整
        if rate_spec:
            debug_print(f"🔍 步骤5: 根据速度规格调整...")
            debug_print(f"  - 速度规格: '{rate_spec}'")
            
            if rate_spec == "dropwise":
                final_flowrate = min(final_flowrate, 0.1)
                final_transfer_flowrate = min(final_transfer_flowrate, 0.1)
                debug_print(f"  - dropwise模式，流速调整为: {final_flowrate}mL/s")
            elif rate_spec == "slowly":
                final_flowrate = min(final_flowrate, 0.5)
                final_transfer_flowrate = min(final_transfer_flowrate, 0.3)
                debug_print(f"  - slowly模式，流速调整为: {final_flowrate}mL/s")
            elif rate_spec == "quickly":
                final_flowrate = max(final_flowrate, 5.0)
                final_transfer_flowrate = max(final_transfer_flowrate, 2.0)
                debug_print(f"  - quickly模式，流速调整为: {final_flowrate}mL/s")
        
        # 5. 处理冲洗参数
        debug_print(f"🔍 步骤6: 处理冲洗参数...")
        final_rinsing_solvent = rinsing_solvent
        final_rinsing_volume = rinsing_volume if rinsing_volume > 0 else 5.0
        final_rinsing_repeats = rinsing_repeats if rinsing_repeats > 0 else 2
        
        if rinsing_volume <= 0:
            logger.warning(f"⚠️ rinsing_volume <= 0，修正为: {final_rinsing_volume}mL")
        if rinsing_repeats <= 0:
            logger.warning(f"⚠️ rinsing_repeats <= 0，修正为: {final_rinsing_repeats}次")
        
        # 根据物理属性调整冲洗参数
        if viscous or solid:
            final_rinsing_repeats = max(final_rinsing_repeats, 3)
            final_rinsing_volume = max(final_rinsing_volume, 10.0)
            debug_print(f"🧪 粘稠/固体物质，调整冲洗参数：{final_rinsing_repeats}次，{final_rinsing_volume}mL")
        
        # 参数总结
        debug_print("📊 最终参数总结:")
        debug_print(f"  - 体积: {final_volume}mL")
        debug_print(f"  - 流速: {final_flowrate}mL/s")
        debug_print(f"  - 转移流速: {final_transfer_flowrate}mL/s")
        debug_print(f"  - 冲洗溶剂: '{final_rinsing_solvent}'")
        debug_print(f"  - 冲洗体积: {final_rinsing_volume}mL")
        debug_print(f"  - 冲洗次数: {final_rinsing_repeats}次")
        
        # 这里应该是您现有的pump_action_sequence生成逻辑
        # 我先提供一个示例，您需要替换为实际的生成逻辑
        
        try:
            pump_action_sequence = generate_pump_protocol(
                G, from_vessel, to_vessel, final_volume,
                flowrate, transfer_flowrate
            )
            
            # 为每个动作添加唯一标识
            # for i, action in enumerate(pump_action_sequence):
            #     if isinstance(action, dict):
            #         action['_protocol_id'] = protocol_id
            #         action['_action_sequence'] = i
            #     elif isinstance(action, list):
            #         for j, sub_action in enumerate(action):
            #             if isinstance(sub_action, dict):
            #                 sub_action['_protocol_id'] = protocol_id
            #                 sub_action['_action_sequence'] = f"{i}_{j}"
            #
            # debug_print(f"📊 协议 {protocol_id} 生成完成，共 {len(pump_action_sequence)} 个动作")
            debug_print(f"🔓 释放执行锁")
            return pump_action_sequence
            
        except Exception as e:
            logger.error(f"❌ 协议生成失败: {str(e)}")
            return [
                {
                    "device_id": "system",
                    "action_name": "log_message",
                    "action_kwargs": {
                        "message": f"❌ 协议生成失败: {str(e)}"
                    },
                    '_protocol_id': protocol_id,
                    '_action_sequence': 0
                }
            ]

def _parse_amount_to_volume(amount: str) -> float:
    """解析 amount 字符串为体积"""
    debug_print(f"🔍 解析 amount: '{amount}'")
    
    if not amount:
        debug_print("  - amount 为空，返回 0.0")
        return 0.0

    amount = amount.lower().strip()
    debug_print(f"  - 处理后的 amount: '{amount}'")

    # 处理特殊关键词
    if amount == "all":
        debug_print("  - 检测到 'all'，返回 0.0（需要后续处理）")
        return 0.0  # 返回0.0，让调用者处理

    # 提取数字
    import re
    numbers = re.findall(r'[\d.]+', amount)
    debug_print(f"  - 提取到的数字: {numbers}")
    
    if numbers:
        volume = float(numbers[0])
        debug_print(f"  - 基础体积: {volume}")

        # 单位转换
        if 'ml' in amount or 'milliliter' in amount:
            debug_print(f"  - 单位: mL，最终体积: {volume}")
            return volume
        elif 'l' in amount and 'ml' not in amount:
            final_volume = volume * 1000
            debug_print(f"  - 单位: L，最终体积: {final_volume}mL")
            return final_volume
        elif 'μl' in amount or 'microliter' in amount:
            final_volume = volume / 1000
            debug_print(f"  - 单位: μL，最终体积: {final_volume}mL")
            return final_volume
        else:
            debug_print(f"  - 无单位，假设为 mL: {volume}")
            return volume

    debug_print("  - 无法解析，返回 0.0")
    return 0.0


def _generate_rinsing_sequence(G: nx.DiGraph, from_vessel: str, to_vessel: str,
                              rinsing_solvent: str, rinsing_volume: float,
                              rinsing_repeats: int, flowrate: float,
                              transfer_flowrate: float) -> List[Dict[str, Any]]:
    """生成冲洗动作序列"""
    rinsing_actions = []

    try:
        shortest_path = nx.shortest_path(G, source=from_vessel, target=to_vessel)
        pump_backbone = shortest_path[1:-1]

        if not pump_backbone:
            return rinsing_actions

        nodes = G.nodes(data=True)
        pumps_from_node, valve_from_node = build_pump_valve_maps(G, pump_backbone)
        min_transfer_volume = min([nodes[pumps_from_node[node]]["config"]["max_volume"] for node in pump_backbone])

        waste_vessel = "waste_workup"

        # 处理多种溶剂情况
        if "," in rinsing_solvent:
            rinsing_solvents = rinsing_solvent.split(",")
            if len(rinsing_solvents) != rinsing_repeats:
                rinsing_solvents = [rinsing_solvent] * rinsing_repeats
        else:
            rinsing_solvents = [rinsing_solvent] * rinsing_repeats

        for solvent in rinsing_solvents:
            solvent_vessel = f"flask_{solvent.strip()}"

            # 检查溶剂容器是否存在
            if solvent_vessel not in G.nodes():
                logger.warning(f"溶剂容器 {solvent_vessel} 不存在，跳过该溶剂冲洗")
                continue

            # 清洗泵系统
            rinsing_actions.extend(
                generate_pump_protocol(G, solvent_vessel, pump_backbone[0], min_transfer_volume, flowrate, transfer_flowrate)
            )

            if len(pump_backbone) > 1:
                rinsing_actions.extend(
                    generate_pump_protocol(G, pump_backbone[0], pump_backbone[-1], min_transfer_volume, flowrate, transfer_flowrate)
                )

            # 排到废液容器
            if waste_vessel in G.nodes():
                rinsing_actions.extend(
                    generate_pump_protocol(G, pump_backbone[-1], waste_vessel, min_transfer_volume, flowrate, transfer_flowrate)
                )

            # 第一种冲洗溶剂稀释源容器和目标容器
            if solvent == rinsing_solvents[0]:
                rinsing_actions.extend(
                    generate_pump_protocol(G, solvent_vessel, from_vessel, rinsing_volume, flowrate, transfer_flowrate)
                )
                rinsing_actions.extend(
                    generate_pump_protocol(G, solvent_vessel, to_vessel, rinsing_volume, flowrate, transfer_flowrate)
                )

    except Exception as e:
        logger.error(f"生成冲洗序列失败: {str(e)}")

    return rinsing_actions


def _generate_air_rinsing_sequence(G: nx.DiGraph, from_vessel: str, to_vessel: str,
                                  rinsing_volume: float, repeats: int,
                                  flowrate: float, transfer_flowrate: float) -> List[Dict[str, Any]]:
    """生成空气冲洗序列"""
    air_rinsing_actions = []

    try:
        air_vessel = "flask_air"
        if air_vessel not in G.nodes():
            logger.warning("空气容器 flask_air 不存在，跳过空气冲洗")
            return air_rinsing_actions

        for _ in range(repeats):
            # 空气冲洗源容器
            air_rinsing_actions.extend(
                generate_pump_protocol(G, air_vessel, from_vessel, rinsing_volume, flowrate, transfer_flowrate)
            )

            # 空气冲洗目标容器
            air_rinsing_actions.extend(
                generate_pump_protocol(G, air_vessel, to_vessel, rinsing_volume, flowrate, transfer_flowrate)
            )

    except Exception as e:
        logger.warning(f"空气冲洗失败: {str(e)}")

    return air_rinsing_actions
