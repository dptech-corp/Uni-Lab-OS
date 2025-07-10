from typing import List, Dict, Any, Optional
import networkx as nx
import logging
from .pump_protocol import generate_pump_protocol_with_rinsing

logger = logging.getLogger(__name__)

def debug_print(message):
    """调试输出"""
    print(f"[FILTER] {message}", flush=True)
    logger.info(f"[FILTER] {message}")

def find_filter_device(G: nx.DiGraph) -> str:
    """查找过滤器设备"""
    debug_print("查找过滤器设备...")
    
    # 查找过滤器设备
    for node in G.nodes():
        node_data = G.nodes[node]
        node_class = node_data.get('class', '') or ''
        
        if 'filter' in node_class.lower() or 'filter' in node.lower():
            debug_print(f"找到过滤器设备: {node}")
            return node
    
    # 如果没找到，寻找可能的过滤器名称
    possible_names = ["filter", "filter_1", "virtual_filter", "filtration_unit"]
    for name in possible_names:
        if name in G.nodes():
            debug_print(f"找到过滤器设备: {name}")
            return name
    
    raise ValueError("未找到过滤器设备")

def validate_vessel(G: nx.DiGraph, vessel: str, vessel_type: str = "容器") -> None:
    """验证容器是否存在"""
    if not vessel:
        raise ValueError(f"{vessel_type}不能为空")
    
    if vessel not in G.nodes():
        raise ValueError(f"{vessel_type} '{vessel}' 不存在于系统中")
    
    debug_print(f"✅ {vessel_type} '{vessel}' 验证通过")

def generate_filter_protocol(
    G: nx.DiGraph,
    vessel: str,
    filtrate_vessel: str = "",
    **kwargs
) -> List[Dict[str, Any]]:
    """
    生成过滤操作的协议序列
    
    Args:
        G: 设备图
        vessel: 过滤容器名称（必需）- 包含需要过滤的混合物
        filtrate_vessel: 滤液容器名称（可选）- 如果提供则收集滤液
        **kwargs: 其他参数（兼容性）
    
    Returns:
        List[Dict[str, Any]]: 过滤操作的动作序列
    """
    
    debug_print("=" * 60)
    debug_print("开始生成过滤协议")
    debug_print(f"输入参数:")
    debug_print(f"  - vessel: {vessel}")
    debug_print(f"  - filtrate_vessel: {filtrate_vessel}")
    debug_print(f"  - 其他参数: {kwargs}")
    debug_print("=" * 60)
    
    action_sequence = []
    
    # === 参数验证 ===
    debug_print("步骤1: 参数验证...")
    
    # 验证必需参数
    validate_vessel(G, vessel, "过滤容器")
    
    # 验证可选参数
    if filtrate_vessel:
        validate_vessel(G, filtrate_vessel, "滤液容器")
        debug_print("模式: 过滤并收集滤液")
    else:
        debug_print("模式: 过滤并收集固体")
    
    # === 查找设备 ===
    debug_print("步骤2: 查找设备...")
    
    try:
        filter_device = find_filter_device(G)
        debug_print(f"使用过滤器设备: {filter_device}")
        
    except Exception as e:
        debug_print(f"❌ 设备查找失败: {str(e)}")
        raise ValueError(f"设备查找失败: {str(e)}")
    
    # === 转移到过滤器（如果需要）===
    debug_print("步骤3: 转移到过滤器...")
    
    if vessel != filter_device:
        debug_print(f"需要转移: {vessel} → {filter_device}")
        
        try:
            # 使用pump protocol转移液体到过滤器
            transfer_actions = generate_pump_protocol_with_rinsing(
                G=G,
                from_vessel=vessel,
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
                debug_print(f"✅ 添加了 {len(transfer_actions)} 个转移动作")
            else:
                debug_print("⚠️ 转移协议返回空序列")
                
        except Exception as e:
            debug_print(f"❌ 转移失败: {str(e)}")
            # 继续执行，可能是直接连接的过滤器
    else:
        debug_print("过滤容器就是过滤器，无需转移")
    
    # === 执行过滤操作 ===
    debug_print("步骤4: 执行过滤操作...")
    
    # 构建过滤动作参数
    filter_kwargs = {
        "vessel": filter_device,  # 过滤器设备
        "filtrate_vessel": filtrate_vessel,  # 滤液容器（可能为空）
        "stir": kwargs.get("stir", False),
        "stir_speed": kwargs.get("stir_speed", 0.0),
        "temp": kwargs.get("temp", 25.0),
        "continue_heatchill": kwargs.get("continue_heatchill", False),
        "volume": kwargs.get("volume", 0.0)  # 0表示过滤所有
    }
    
    debug_print(f"过滤参数: {filter_kwargs}")
    
    # 过滤动作
    filter_action = {
        "device_id": filter_device,
        "action_name": "filter",
        "action_kwargs": filter_kwargs
    }
    action_sequence.append(filter_action)
    
    # 过滤后等待
    action_sequence.append({
        "action_name": "wait",
        "action_kwargs": {"time": 10.0}
    })
    
    # === 收集滤液（如果需要）===
    debug_print("步骤5: 收集滤液...")
    
    if filtrate_vessel:
        debug_print(f"收集滤液: {filter_device} → {filtrate_vessel}")
        
        try:
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
                debug_print(f"✅ 添加了 {len(collect_actions)} 个收集动作")
            else:
                debug_print("⚠️ 收集协议返回空序列")
                
        except Exception as e:
            debug_print(f"❌ 收集滤液失败: {str(e)}")
            # 继续执行，可能滤液直接流入指定容器
    else:
        debug_print("未指定滤液容器，固体保留在过滤器中")
    
    # === 最终等待 ===
    action_sequence.append({
        "action_name": "wait",
        "action_kwargs": {"time": 5.0}
    })
    
    # === 总结 ===
    debug_print("=" * 60)
    debug_print(f"过滤协议生成完成")
    debug_print(f"总动作数: {len(action_sequence)}")
    debug_print(f"过滤容器: {vessel}")
    debug_print(f"过滤器设备: {filter_device}")
    debug_print(f"滤液容器: {filtrate_vessel or '无（保留固体）'}")
    debug_print("=" * 60)
    
    return action_sequence
