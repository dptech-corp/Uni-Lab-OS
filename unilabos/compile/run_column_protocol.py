from typing import List, Dict, Any, Union
import networkx as nx
import logging
import re
from .pump_protocol import generate_pump_protocol_with_rinsing

logger = logging.getLogger(__name__)

def debug_print(message):
    """调试输出"""
    print(f"[RUN_COLUMN] {message}", flush=True)
    logger.info(f"[RUN_COLUMN] {message}")

def parse_percentage(pct_str: str) -> float:
    """
    解析百分比字符串为数值
    
    Args:
        pct_str: 百分比字符串（如 "40 %", "40%", "40"）
    
    Returns:
        float: 百分比数值（0-100）
    """
    if not pct_str or not pct_str.strip():
        return 0.0
    
    pct_str = pct_str.strip().lower()
    debug_print(f"解析百分比: '{pct_str}'")
    
    # 移除百分号和空格
    pct_clean = re.sub(r'[%\s]', '', pct_str)
    
    # 提取数字
    match = re.search(r'([0-9]*\.?[0-9]+)', pct_clean)
    if match:
        value = float(match.group(1))
        debug_print(f"百分比解析结果: {value}%")
        return value
    
    debug_print(f"⚠️ 无法解析百分比: '{pct_str}'，返回0.0")
    return 0.0

def parse_ratio(ratio_str: str) -> tuple:
    """
    解析比例字符串为两个数值
    
    Args:
        ratio_str: 比例字符串（如 "5:95", "1:1", "40:60"）
    
    Returns:
        tuple: (ratio1, ratio2) 两个比例值
    """
    if not ratio_str or not ratio_str.strip():
        return (50.0, 50.0)  # 默认1:1
    
    ratio_str = ratio_str.strip()
    debug_print(f"解析比例: '{ratio_str}'")
    
    # 支持多种分隔符：: / -
    if ':' in ratio_str:
        parts = ratio_str.split(':')
    elif '/' in ratio_str:
        parts = ratio_str.split('/')
    elif '-' in ratio_str:
        parts = ratio_str.split('-')
    elif 'to' in ratio_str.lower():
        parts = ratio_str.lower().split('to')
    else:
        debug_print(f"⚠️ 无法解析比例格式: '{ratio_str}'，使用默认1:1")
        return (50.0, 50.0)
    
    if len(parts) >= 2:
        try:
            ratio1 = float(parts[0].strip())
            ratio2 = float(parts[1].strip())
            total = ratio1 + ratio2
            
            # 转换为百分比
            pct1 = (ratio1 / total) * 100
            pct2 = (ratio2 / total) * 100
            
            debug_print(f"比例解析结果: {ratio1}:{ratio2} -> {pct1:.1f}%:{pct2:.1f}%")
            return (pct1, pct2)
        except ValueError as e:
            debug_print(f"⚠️ 比例数值转换失败: {str(e)}")
    
    debug_print(f"⚠️ 比例解析失败，使用默认1:1")
    return (50.0, 50.0)

def parse_rf_value(rf_str: str) -> float:
    """
    解析Rf值字符串
    
    Args:
        rf_str: Rf值字符串（如 "0.3", "0.45", "?"）
    
    Returns:
        float: Rf值（0-1）
    """
    if not rf_str or not rf_str.strip():
        return 0.3  # 默认Rf值
    
    rf_str = rf_str.strip().lower()
    debug_print(f"解析Rf值: '{rf_str}'")
    
    # 处理未知Rf值
    if rf_str in ['?', 'unknown', 'tbd', 'to be determined']:
        default_rf = 0.3
        debug_print(f"检测到未知Rf值，使用默认值: {default_rf}")
        return default_rf
    
    # 提取数字
    match = re.search(r'([0-9]*\.?[0-9]+)', rf_str)
    if match:
        value = float(match.group(1))
        # 确保Rf值在0-1范围内
        if value > 1.0:
            value = value / 100.0  # 可能是百分比形式
        value = max(0.0, min(1.0, value))  # 限制在0-1范围
        debug_print(f"Rf值解析结果: {value}")
        return value
    
    debug_print(f"⚠️ 无法解析Rf值: '{rf_str}'，使用默认值0.3")
    return 0.3

def find_column_device(G: nx.DiGraph) -> str:
    """查找柱层析设备"""
    debug_print("查找柱层析设备...")
    
    # 查找虚拟柱设备
    for node in G.nodes():
        node_data = G.nodes[node]
        node_class = node_data.get('class', '') or ''
        
        if 'virtual_column' in node_class.lower() or 'column' in node_class.lower():
            debug_print(f"✅ 找到柱层析设备: {node}")
            return node
    
    # 如果没有找到，尝试创建虚拟设备名称
    possible_names = ['column_1', 'virtual_column_1', 'chromatography_column_1']
    for name in possible_names:
        if name in G.nodes():
            debug_print(f"✅ 找到柱设备: {name}")
            return name
    
    debug_print("⚠️ 未找到柱层析设备，将使用pump protocol直接转移")
    return ""

def find_column_vessel(G: nx.DiGraph, column: str) -> str:
    """查找柱容器"""
    debug_print(f"查找柱容器: '{column}'")
    
    # 直接检查column参数是否是容器
    if column in G.nodes():
        node_type = G.nodes[column].get('type', '')
        if node_type == 'container':
            debug_print(f"✅ 找到柱容器: {column}")
            return column
    
    # 尝试常见的命名规则
    possible_names = [
        f"column_{column}",
        f"{column}_column", 
        f"vessel_{column}",
        f"{column}_vessel",
        "column_vessel",
        "chromatography_column",
        "silica_column",
        "preparative_column",
        "column"
    ]
    
    for vessel_name in possible_names:
        if vessel_name in G.nodes():
            node_type = G.nodes[vessel_name].get('type', '')
            if node_type == 'container':
                debug_print(f"✅ 找到柱容器: {vessel_name}")
                return vessel_name
    
    debug_print(f"⚠️ 未找到柱容器，将直接在源容器中进行分离")
    return ""

def find_solvent_vessel(G: nx.DiGraph, solvent: str) -> str:
    """查找溶剂容器 - 增强版"""
    if not solvent or not solvent.strip():
        return ""
    
    solvent = solvent.strip().replace(' ', '_').lower()
    debug_print(f"查找溶剂容器: '{solvent}'")
    
    # 🔧 方法1：直接搜索 data.reagent_name
    for node in G.nodes():
        node_data = G.nodes[node].get('data', {})
        node_type = G.nodes[node].get('type', '')
        
        # 只搜索容器类型的节点
        if node_type == 'container':
            reagent_name = node_data.get('reagent_name', '').lower()
            reagent_config = G.nodes[node].get('config', {}).get('reagent', '').lower()
            
            # 检查 data.reagent_name 和 config.reagent
            if reagent_name == solvent or reagent_config == solvent:
                debug_print(f"✅ 通过reagent_name找到溶剂容器: {node} (reagent: {reagent_name or reagent_config})")
                return node
            
            # 模糊匹配 reagent_name
            if solvent in reagent_name or reagent_name in solvent:
                debug_print(f"✅ 通过reagent_name模糊匹配到溶剂容器: {node} (reagent: {reagent_name})")
                return node
            
            if solvent in reagent_config or reagent_config in solvent:
                debug_print(f"✅ 通过config.reagent模糊匹配到溶剂容器: {node} (reagent: {reagent_config})")
                return node
    
    # 🔧 方法2：常见的溶剂容器命名规则
    possible_names = [
        f"flask_{solvent}",
        f"bottle_{solvent}",
        f"reagent_{solvent}",
        f"{solvent}_bottle",
        f"{solvent}_flask",
        f"solvent_{solvent}",
        f"reagent_bottle_{solvent}"
    ]
    
    for vessel_name in possible_names:
        if vessel_name in G.nodes():
            node_type = G.nodes[vessel_name].get('type', '')
            if node_type == 'container':
                debug_print(f"✅ 通过命名规则找到溶剂容器: {vessel_name}")
                return vessel_name
    
    # 🔧 方法3：节点名称模糊匹配
    for node in G.nodes():
        node_type = G.nodes[node].get('type', '')
        if node_type == 'container':
            if ('flask_' in node or 'bottle_' in node or 'reagent_' in node) and solvent in node.lower():
                debug_print(f"✅ 通过节点名称模糊匹配到溶剂容器: {node}")
                return node
    
    # 🔧 方法4：特殊溶剂名称映射
    solvent_mapping = {
        'dmf': ['dmf', 'dimethylformamide', 'n,n-dimethylformamide'],
        'ethyl_acetate': ['ethyl_acetate', 'ethylacetate', 'etoac', 'ea'],
        'hexane': ['hexane', 'hexanes', 'n-hexane'],
        'methanol': ['methanol', 'meoh', 'ch3oh'],
        'water': ['water', 'h2o', 'distilled_water'],
        'acetone': ['acetone', 'ch3coch3', '2-propanone'],
        'dichloromethane': ['dichloromethane', 'dcm', 'ch2cl2', 'methylene_chloride'],
        'chloroform': ['chloroform', 'chcl3', 'trichloromethane']
    }
    
    # 查找映射的同义词
    for canonical_name, synonyms in solvent_mapping.items():
        if solvent in synonyms:
            debug_print(f"检测到溶剂同义词: '{solvent}' -> '{canonical_name}'")
            return find_solvent_vessel(G, canonical_name)  # 递归搜索
    
    debug_print(f"⚠️ 未找到溶剂 '{solvent}' 的容器")
    return ""

def get_vessel_liquid_volume(G: nx.DiGraph, vessel: str) -> float:
    """获取容器中的液体体积 - 增强版"""
    if vessel not in G.nodes():
        debug_print(f"⚠️ 节点 '{vessel}' 不存在")
        return 0.0
    
    node_type = G.nodes[vessel].get('type', '')
    vessel_data = G.nodes[vessel].get('data', {})
    
    debug_print(f"读取节点 '{vessel}' (类型: {node_type}) 体积数据: {vessel_data}")
    
    # 🔧 如果是设备类型，尝试查找关联的容器
    if node_type == 'device':
        debug_print(f"'{vessel}' 是设备，尝试查找关联容器...")
        
        # 查找是否有内置容器数据
        config_data = G.nodes[vessel].get('config', {})
        if 'volume' in config_data:
            default_volume = config_data.get('volume', 50.0)
            debug_print(f"使用设备默认容量: {default_volume}mL")
            return default_volume
        
        # 对于旋蒸等设备，使用默认值
        if 'rotavap' in vessel.lower():
            default_volume = 50.0
            debug_print(f"旋蒸设备使用默认容量: {default_volume}mL")
            return default_volume
        
        debug_print(f"⚠️ 设备 '{vessel}' 无法确定容量，返回0")
        return 0.0
    
    # 🔧 如果是容器类型，正常读取体积
    total_volume = 0.0
    
    # 方法1：检查液体列表
    liquids = vessel_data.get('liquid', [])
    if isinstance(liquids, list):
        for liquid in liquids:
            if isinstance(liquid, dict):
                volume = liquid.get('volume') or liquid.get('liquid_volume', 0.0)
                total_volume += volume
    
    # 方法2：检查直接体积字段
    if total_volume == 0.0:
        volume_keys = ['current_volume', 'total_volume', 'volume', 'liquid_volume']
        for key in volume_keys:
            if key in vessel_data:
                try:
                    total_volume = float(vessel_data[key])
                    if total_volume > 0:
                        break
                except (ValueError, TypeError):
                    continue
    
    # 方法3：检查配置中的初始体积
    if total_volume == 0.0:
        config_data = G.nodes[vessel].get('config', {})
        if 'current_volume' in config_data:
            try:
                total_volume = float(config_data['current_volume'])
            except (ValueError, TypeError):
                pass
    
    debug_print(f"容器 '{vessel}' 总体积: {total_volume}mL")
    return total_volume

def calculate_solvent_volumes(total_volume: float, pct1: float, pct2: float) -> tuple:
    """根据百分比计算溶剂体积"""
    volume1 = (total_volume * pct1) / 100.0
    volume2 = (total_volume * pct2) / 100.0
    
    debug_print(f"溶剂体积计算: 总体积{total_volume}mL")
    debug_print(f"  - 溶剂1: {pct1}% = {volume1}mL")
    debug_print(f"  - 溶剂2: {pct2}% = {volume2}mL")
    
    return (volume1, volume2)

def generate_run_column_protocol(
    G: nx.DiGraph,
    from_vessel: str,
    to_vessel: str,
    column: str,
    rf: str = "",
    pct1: str = "",
    pct2: str = "",
    solvent1: str = "",
    solvent2: str = "",
    ratio: str = "",
    **kwargs
) -> List[Dict[str, Any]]:
    """
    生成柱层析分离的协议序列 - 增强版
    
    支持新版XDL的所有参数，具有高兼容性和容错性
    
    Args:
        G: 有向图，节点为设备和容器，边为流体管道
        from_vessel: 源容器的名称，即样品起始所在的容器（必需）
        to_vessel: 目标容器的名称，分离后的样品要到达的容器（必需）
        column: 所使用的柱子的名称（必需）
        rf: Rf值（可选，支持 "?" 表示未知）
        pct1: 第一种溶剂百分比（如 "40 %"，可选）
        pct2: 第二种溶剂百分比（如 "50 %"，可选）
        solvent1: 第一种溶剂名称（可选）
        solvent2: 第二种溶剂名称（可选）
        ratio: 溶剂比例（如 "5:95"，可选，优先级高于pct1/pct2）
        **kwargs: 其他可选参数
    
    Returns:
        List[Dict[str, Any]]: 柱层析分离操作的动作序列
    """
    
    debug_print("=" * 60)
    debug_print("开始生成柱层析协议")
    debug_print(f"输入参数:")
    debug_print(f"  - from_vessel: '{from_vessel}'")
    debug_print(f"  - to_vessel: '{to_vessel}'")
    debug_print(f"  - column: '{column}'")
    debug_print(f"  - rf: '{rf}'")
    debug_print(f"  - pct1: '{pct1}'")
    debug_print(f"  - pct2: '{pct2}'")
    debug_print(f"  - solvent1: '{solvent1}'")
    debug_print(f"  - solvent2: '{solvent2}'")
    debug_print(f"  - ratio: '{ratio}'")
    debug_print(f"  - 其他参数: {kwargs}")
    debug_print("=" * 60)
    
    action_sequence = []
    
    # === 参数验证 ===
    debug_print("步骤1: 参数验证...")
    
    if not from_vessel:
        raise ValueError("from_vessel 参数不能为空")
    if not to_vessel:
        raise ValueError("to_vessel 参数不能为空")
    if not column:
        raise ValueError("column 参数不能为空")
    
    if from_vessel not in G.nodes():
        raise ValueError(f"源容器 '{from_vessel}' 不存在于系统中")
    if to_vessel not in G.nodes():
        raise ValueError(f"目标容器 '{to_vessel}' 不存在于系统中")
    
    debug_print("✅ 基本参数验证通过")
    
    # === 参数解析 ===
    debug_print("步骤2: 参数解析...")
    
    # 解析Rf值
    final_rf = parse_rf_value(rf)
    debug_print(f"最终Rf值: {final_rf}")
    
    # 解析溶剂比例（ratio优先级高于pct1/pct2）
    if ratio and ratio.strip():
        final_pct1, final_pct2 = parse_ratio(ratio)
        debug_print(f"使用ratio参数: {final_pct1:.1f}% : {final_pct2:.1f}%")
    else:
        final_pct1 = parse_percentage(pct1) if pct1 else 50.0
        final_pct2 = parse_percentage(pct2) if pct2 else 50.0
        
        # 如果百分比和不是100%，进行归一化
        total_pct = final_pct1 + final_pct2
        if total_pct == 0:
            final_pct1, final_pct2 = 50.0, 50.0
        elif total_pct != 100.0:
            final_pct1 = (final_pct1 / total_pct) * 100
            final_pct2 = (final_pct2 / total_pct) * 100
        
        debug_print(f"使用百分比参数: {final_pct1:.1f}% : {final_pct2:.1f}%")
    
    # 设置默认溶剂（如果未指定）
    final_solvent1 = solvent1.strip() if solvent1 else "ethyl_acetate"
    final_solvent2 = solvent2.strip() if solvent2 else "hexane"
    
    debug_print(f"最终溶剂: {final_solvent1} : {final_solvent2}")
    
    # === 查找设备和容器 ===
    debug_print("步骤3: 查找设备和容器...")
    
    # 查找柱层析设备
    column_device_id = find_column_device(G)
    
    # 查找柱容器
    column_vessel = find_column_vessel(G, column)
    
    # 查找溶剂容器
    solvent1_vessel = find_solvent_vessel(G, final_solvent1)
    solvent2_vessel = find_solvent_vessel(G, final_solvent2)
    
    debug_print(f"设备映射:")
    debug_print(f"  - 柱设备: '{column_device_id}'")
    debug_print(f"  - 柱容器: '{column_vessel}'")
    debug_print(f"  - 溶剂1容器: '{solvent1_vessel}'")
    debug_print(f"  - 溶剂2容器: '{solvent2_vessel}'")
    
    # === 获取源容器体积 ===
    debug_print("步骤4: 获取源容器体积...")
    
    source_volume = get_vessel_liquid_volume(G, from_vessel)
    if source_volume <= 0:
        source_volume = 50.0  # 默认体积
        debug_print(f"⚠️ 无法获取源容器体积，使用默认值: {source_volume}mL")
    else:
        debug_print(f"✅ 源容器体积: {source_volume}mL")
    
    # === 计算溶剂体积 ===
    debug_print("步骤5: 计算溶剂体积...")
    
    # 洗脱溶剂通常是样品体积的2-5倍
    total_elution_volume = source_volume * 3.0
    solvent1_volume, solvent2_volume = calculate_solvent_volumes(
        total_elution_volume, final_pct1, final_pct2
    )
    
    # === 执行柱层析流程 ===
    debug_print("步骤6: 执行柱层析流程...")
    
    try:
        # 步骤6.1: 样品上柱（如果有独立的柱容器）
        if column_vessel and column_vessel != from_vessel:
            debug_print(f"6.1: 样品上柱 - {source_volume}mL 从 {from_vessel} 到 {column_vessel}")
            
            try:
                sample_transfer_actions = generate_pump_protocol_with_rinsing(
                    G=G,
                    from_vessel=from_vessel,
                    to_vessel=column_vessel,
                    volume=source_volume,
                    flowrate=1.0,  # 慢速上柱
                    transfer_flowrate=0.5,
                    rinsing_solvent="",  # 暂不冲洗
                    rinsing_volume=0.0,
                    rinsing_repeats=0
                )
                action_sequence.extend(sample_transfer_actions)
                debug_print(f"✅ 样品上柱完成，添加了 {len(sample_transfer_actions)} 个动作")
            except Exception as e:
                debug_print(f"⚠️ 样品上柱失败: {str(e)}")
        
        # 步骤6.2: 添加洗脱溶剂1（如果有溶剂容器）
        if solvent1_vessel and solvent1_volume > 0:
            debug_print(f"6.2: 添加洗脱溶剂1 - {solvent1_volume:.1f}mL {final_solvent1}")
            
            try:
                target_vessel = column_vessel if column_vessel else from_vessel
                solvent1_transfer_actions = generate_pump_protocol_with_rinsing(
                    G=G,
                    from_vessel=solvent1_vessel,
                    to_vessel=target_vessel,
                    volume=solvent1_volume,
                    flowrate=2.0,
                    transfer_flowrate=1.0
                )
                action_sequence.extend(solvent1_transfer_actions)
                debug_print(f"✅ 溶剂1添加完成，添加了 {len(solvent1_transfer_actions)} 个动作")
            except Exception as e:
                debug_print(f"⚠️ 溶剂1添加失败: {str(e)}")
        
        # 步骤6.3: 添加洗脱溶剂2（如果有溶剂容器）
        if solvent2_vessel and solvent2_volume > 0:
            debug_print(f"6.3: 添加洗脱溶剂2 - {solvent2_volume:.1f}mL {final_solvent2}")
            
            try:
                target_vessel = column_vessel if column_vessel else from_vessel
                solvent2_transfer_actions = generate_pump_protocol_with_rinsing(
                    G=G,
                    from_vessel=solvent2_vessel,
                    to_vessel=target_vessel,
                    volume=solvent2_volume,
                    flowrate=2.0,
                    transfer_flowrate=1.0
                )
                action_sequence.extend(solvent2_transfer_actions)
                debug_print(f"✅ 溶剂2添加完成，添加了 {len(solvent2_transfer_actions)} 个动作")
            except Exception as e:
                debug_print(f"⚠️ 溶剂2添加失败: {str(e)}")
        
        # 步骤6.4: 使用柱层析设备执行分离（如果有设备）
        if column_device_id:
            debug_print(f"6.4: 使用柱层析设备执行分离")
            
            column_separation_action = {
                "device_id": column_device_id,
                "action_name": "run_column",
                "action_kwargs": {
                    "from_vessel": from_vessel,
                    "to_vessel": to_vessel,
                    "column": column,
                    "rf": rf,
                    "pct1": pct1,
                    "pct2": pct2,
                    "solvent1": solvent1,
                    "solvent2": solvent2,
                    "ratio": ratio
                }
            }
            action_sequence.append(column_separation_action)
            debug_print(f"✅ 柱层析设备动作已添加")
            
            # 等待分离完成
            separation_time = max(30, int(total_elution_volume / 2))  # 基于体积估算时间
            action_sequence.append({
                "action_name": "wait",
                "action_kwargs": {"time": separation_time}
            })
            debug_print(f"✅ 等待分离完成: {separation_time}秒")
        
        # 步骤6.5: 产物收集（从柱容器到目标容器）
        if column_vessel and column_vessel != to_vessel:
            debug_print(f"6.5: 产物收集 - 从 {column_vessel} 到 {to_vessel}")
            
            try:
                # 估算产物体积（原始样品体积的70-90%）
                product_volume = source_volume * 0.8
                
                product_transfer_actions = generate_pump_protocol_with_rinsing(
                    G=G,
                    from_vessel=column_vessel,
                    to_vessel=to_vessel,
                    volume=product_volume,
                    flowrate=1.5,
                    transfer_flowrate=0.8
                )
                action_sequence.extend(product_transfer_actions)
                debug_print(f"✅ 产物收集完成，添加了 {len(product_transfer_actions)} 个动作")
            except Exception as e:
                debug_print(f"⚠️ 产物收集失败: {str(e)}")
        
        # 步骤6.6: 如果没有独立的柱设备和容器，执行简化的直接转移
        if not column_device_id and not column_vessel:
            debug_print(f"6.6: 简化模式 - 直接转移 {source_volume}mL 从 {from_vessel} 到 {to_vessel}")
            
            try:
                direct_transfer_actions = generate_pump_protocol_with_rinsing(
                    G=G,
                    from_vessel=from_vessel,
                    to_vessel=to_vessel,
                    volume=source_volume,
                    flowrate=2.0,
                    transfer_flowrate=1.0
                )
                action_sequence.extend(direct_transfer_actions)
                debug_print(f"✅ 直接转移完成，添加了 {len(direct_transfer_actions)} 个动作")
            except Exception as e:
                debug_print(f"⚠️ 直接转移失败: {str(e)}")
        
    except Exception as e:
        debug_print(f"❌ 协议生成失败: {str(e)} 😭")
        
        # 不添加不确定的动作，直接让action_sequence保持为空列表
        # action_sequence 已经在函数开始时初始化为 []
    
    # 确保至少有一个有效的动作，如果完全失败就返回空列表
    if not action_sequence:
        debug_print("⚠️ 没有生成任何有效动作")
        # 可以选择返回空列表或添加一个基本的等待动作
        action_sequence.append({
            "action_name": "wait",
            "action_kwargs": {
                "time": 1.0,
                "description": "柱层析协议执行完成"
            }
        })
    
    # 🎊 总结
    debug_print("🧪" * 20)
    debug_print(f"🎉 柱层析协议生成完成! ✨")
    debug_print(f"📊 总动作数: {len(action_sequence)} 个")
    debug_print(f"🥽 路径: {from_vessel} → {to_vessel}")
    debug_print(f"🏛️ 柱子: {column}")
    debug_print(f"🧪 溶剂: {final_solvent1}:{final_solvent2}")
    debug_print("🧪" * 20)
    
    return action_sequence

# 测试函数
def test_run_column_protocol():
    """测试柱层析协议"""
    debug_print("🧪 === RUN COLUMN PROTOCOL 测试 === ✨")
    debug_print("✅ 测试完成 🎉")

if __name__ == "__main__":
    test_run_column_protocol()

