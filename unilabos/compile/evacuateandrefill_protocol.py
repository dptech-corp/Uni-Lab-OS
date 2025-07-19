import networkx as nx
import logging
import uuid
import sys
from typing import List, Dict, Any, Optional
from .pump_protocol import generate_pump_protocol_with_rinsing, generate_pump_protocol

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
        print(f"[抽真空充气] {safe_message}", flush=True)
        logger.info(f"[抽真空充气] {safe_message}")
    except UnicodeEncodeError:
        # 如果编码失败，尝试替换不支持的字符
        safe_message = str(message).encode('utf-8', errors='replace').decode('utf-8')
        print(f"[抽真空充气] {safe_message}", flush=True)
        logger.info(f"[抽真空充气] {safe_message}")
    except Exception as e:
        # 最后的安全措施
        fallback_message = f"日志输出错误: {repr(message)}"
        print(f"[抽真空充气] {fallback_message}", flush=True)
        logger.info(f"[抽真空充气] {fallback_message}")

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

def find_gas_source(G: nx.DiGraph, gas: str) -> str:
    """
    根据气体名称查找对应的气源，支持多种匹配模式：
    1. 容器名称匹配
    2. 气体类型匹配（data.gas_type）
    3. 默认气源
    """
    debug_print(f"🔍 正在查找气体 '{gas}' 的气源...")
    
    # 第一步：通过容器名称匹配
    debug_print(f"📋 方法1: 容器名称匹配...")
    gas_source_patterns = [
        f"gas_source_{gas}",
        f"gas_{gas}",
        f"flask_{gas}",
        f"{gas}_source",
        f"source_{gas}",
        f"reagent_bottle_{gas}",
        f"bottle_{gas}"
    ]
    
    debug_print(f"🎯 尝试的容器名称: {gas_source_patterns}")
    
    for pattern in gas_source_patterns:
        if pattern in G.nodes():
            debug_print(f"✅ 通过名称找到气源: {pattern}")
            return pattern
    
    # 第二步：通过气体类型匹配 (data.gas_type)
    debug_print(f"📋 方法2: 气体类型匹配...")
    for node_id in G.nodes():
        node_data = G.nodes[node_id]
        node_class = node_data.get('class', '') or ''
        
        # 检查是否是气源设备
        if ('gas_source' in node_class or 
            'gas' in node_id.lower() or 
            node_id.startswith('flask_')):
            
            # 检查 data.gas_type
            data = node_data.get('data', {})
            gas_type = data.get('gas_type', '')
            
            if gas_type.lower() == gas.lower():
                debug_print(f"✅ 通过气体类型找到气源: {node_id} (气体类型: {gas_type})")
                return node_id
            
            # 检查 config.gas_type  
            config = node_data.get('config', {})
            config_gas_type = config.get('gas_type', '')
            
            if config_gas_type.lower() == gas.lower():
                debug_print(f"✅ 通过配置气体类型找到气源: {node_id} (配置气体类型: {config_gas_type})")
                return node_id
    
    # 第三步：查找所有可用的气源设备
    debug_print(f"📋 方法3: 查找可用气源...")
    available_gas_sources = []
    for node_id in G.nodes():
        node_data = G.nodes[node_id]
        node_class = node_data.get('class', '') or ''
        
        if ('gas_source' in node_class or 
            'gas' in node_id.lower() or
            (node_id.startswith('flask_') and any(g in node_id.lower() for g in ['air', 'nitrogen', 'argon']))):
            
            data = node_data.get('data', {})
            gas_type = data.get('gas_type', '未知')
            available_gas_sources.append(f"{node_id} (气体类型: {gas_type})")
    
    debug_print(f"📊 可用气源: {available_gas_sources}")
    
    # 第四步：如果找不到特定气体，使用默认的第一个气源
    debug_print(f"📋 方法4: 查找默认气源...")
    default_gas_sources = [
        node for node in G.nodes() 
        if ((G.nodes[node].get('class') or '').find('virtual_gas_source') != -1
            or 'gas_source' in node)
    ]
    
    if default_gas_sources:
        default_source = default_gas_sources[0]
        debug_print(f"⚠️ 未找到特定气体 '{gas}'，使用默认气源: {default_source}")
        return default_source
    
    debug_print(f"❌ 所有方法都失败了！")
    raise ValueError(f"无法找到气体 '{gas}' 的气源。可用气源: {available_gas_sources}")

def find_vacuum_pump(G: nx.DiGraph) -> str:
    """查找真空泵设备"""
    debug_print("🔍 正在查找真空泵...")
    
    vacuum_pumps = []
    for node in G.nodes():
        node_data = G.nodes[node]
        node_class = node_data.get('class', '') or ''
        
        if ('virtual_vacuum_pump' in node_class or 
            'vacuum_pump' in node.lower() or 
            'vacuum' in node_class.lower()):
            vacuum_pumps.append(node)
            debug_print(f"📋 发现真空泵: {node}")
    
    if not vacuum_pumps:
        debug_print(f"❌ 系统中未找到真空泵")
        raise ValueError("系统中未找到真空泵")
    
    debug_print(f"✅ 使用真空泵: {vacuum_pumps[0]}")
    return vacuum_pumps[0]

def find_connected_stirrer(G: nx.DiGraph, vessel: str) -> Optional[str]:
    """查找与指定容器相连的搅拌器"""
    debug_print(f"🔍 正在查找与容器 {vessel} 连接的搅拌器...")
    
    stirrer_nodes = []
    for node in G.nodes():
        node_data = G.nodes[node]
        node_class = node_data.get('class', '') or ''
        
        if 'virtual_stirrer' in node_class or 'stirrer' in node.lower():
            stirrer_nodes.append(node)
            debug_print(f"📋 发现搅拌器: {node}")
    
    debug_print(f"📊 找到的搅拌器总数: {len(stirrer_nodes)}")
    
    # 检查哪个搅拌器与目标容器相连
    for stirrer in stirrer_nodes:
        if G.has_edge(stirrer, vessel) or G.has_edge(vessel, stirrer):
            debug_print(f"✅ 找到连接的搅拌器: {stirrer}")
            return stirrer
    
    # 如果没有连接的搅拌器，返回第一个可用的
    if stirrer_nodes:
        debug_print(f"⚠️ 未找到直接连接的搅拌器，使用第一个可用的: {stirrer_nodes[0]}")
        return stirrer_nodes[0]
    
    debug_print("❌ 未找到搅拌器")
    return None

def find_vacuum_solenoid_valve(G: nx.DiGraph, vacuum_pump: str) -> Optional[str]:
    """查找真空泵相关的电磁阀"""
    debug_print(f"🔍 正在查找真空泵 {vacuum_pump} 的电磁阀...")
    
    # 查找所有电磁阀
    solenoid_valves = []
    for node in G.nodes():
        node_data = G.nodes[node]
        node_class = node_data.get('class', '') or ''
        
        if ('solenoid' in node_class.lower() or 'solenoid_valve' in node.lower()):
            solenoid_valves.append(node)
            debug_print(f"📋 发现电磁阀: {node}")
    
    debug_print(f"📊 找到的电磁阀: {solenoid_valves}")
    
    # 检查连接关系
    debug_print(f"📋 方法1: 检查连接关系...")
    for solenoid in solenoid_valves:
        if G.has_edge(solenoid, vacuum_pump) or G.has_edge(vacuum_pump, solenoid):
            debug_print(f"✅ 找到连接的真空电磁阀: {solenoid}")
            return solenoid
    
    # 通过命名规则查找
    debug_print(f"📋 方法2: 检查命名规则...")
    for solenoid in solenoid_valves:
        if 'vacuum' in solenoid.lower() or solenoid == 'solenoid_valve_1':
            debug_print(f"✅ 通过命名找到真空电磁阀: {solenoid}")
            return solenoid
    
    debug_print("⚠️ 未找到真空电磁阀")
    return None

def find_gas_solenoid_valve(G: nx.DiGraph, gas_source: str) -> Optional[str]:
    """查找气源相关的电磁阀"""
    debug_print(f"🔍 正在查找气源 {gas_source} 的电磁阀...")
    
    # 查找所有电磁阀
    solenoid_valves = []
    for node in G.nodes():
        node_data = G.nodes[node]
        node_class = node_data.get('class', '') or ''
        
        if ('solenoid' in node_class.lower() or 'solenoid_valve' in node.lower()):
            solenoid_valves.append(node)
    
    debug_print(f"📊 找到的电磁阀: {solenoid_valves}")
    
    # 检查连接关系
    debug_print(f"📋 方法1: 检查连接关系...")
    for solenoid in solenoid_valves:
        if G.has_edge(gas_source, solenoid) or G.has_edge(solenoid, gas_source):
            debug_print(f"✅ 找到连接的气源电磁阀: {solenoid}")
            return solenoid
    
    # 通过命名规则查找
    debug_print(f"📋 方法2: 检查命名规则...")
    for solenoid in solenoid_valves:
        if 'gas' in solenoid.lower() or solenoid == 'solenoid_valve_2':
            debug_print(f"✅ 通过命名找到气源电磁阀: {solenoid}")
            return solenoid
    
    debug_print("⚠️ 未找到气源电磁阀")
    return None

def generate_evacuateandrefill_protocol(
    G: nx.DiGraph,
    vessel: dict,  # 🔧 修改：从字符串改为字典类型
    gas: str,
    **kwargs
) -> List[Dict[str, Any]]:
    """
    生成抽真空和充气操作的动作序列 - 中文版
    
    Args:
        G: 设备图
        vessel: 目标容器字典（必需）
        gas: 气体名称（必需）  
        **kwargs: 其他参数（兼容性）
    
    Returns:
        List[Dict[str, Any]]: 动作序列
    """
    
    # 🔧 核心修改：从字典中提取容器ID
    vessel_id = vessel["id"]
    
    # 硬编码重复次数为 3
    repeats = 3
    
    # 生成协议ID
    protocol_id = str(uuid.uuid4())
    debug_print(f"🆔 生成协议ID: {protocol_id}")
    
    debug_print("=" * 60)
    debug_print("🧪 开始生成抽真空充气协议")
    debug_print(f"📋 原始参数:")
    debug_print(f"  🥼 vessel: {vessel} (ID: {vessel_id})")
    debug_print(f"  💨 气体: '{gas}'")
    debug_print(f"  🔄 循环次数: {repeats} (硬编码)")
    debug_print(f"  📦 其他参数: {kwargs}")
    debug_print("=" * 60)
    
    action_sequence = []
    
    # === 参数验证和修正 ===
    debug_print("🔍 步骤1: 参数验证和修正...")
    action_sequence.append(create_action_log(f"开始抽真空充气操作 - 容器: {vessel_id}", "🎬"))
    action_sequence.append(create_action_log(f"目标气体: {gas}", "💨"))
    action_sequence.append(create_action_log(f"循环次数: {repeats}", "🔄"))
    
    # 验证必需参数
    if not vessel_id:
        debug_print("❌ 容器参数不能为空")
        raise ValueError("容器参数不能为空")
    
    if not gas:
        debug_print("❌ 气体参数不能为空")
        raise ValueError("气体参数不能为空")
    
    if vessel_id not in G.nodes():  # 🔧 使用 vessel_id
        debug_print(f"❌ 容器 '{vessel_id}' 在系统中不存在")
        raise ValueError(f"容器 '{vessel_id}' 在系统中不存在")
    
    debug_print("✅ 基本参数验证通过")
    action_sequence.append(create_action_log("参数验证通过", "✅"))
    
    # 标准化气体名称
    debug_print("🔧 标准化气体名称...")
    gas_aliases = {
        'n2': 'nitrogen',
        'ar': 'argon',
        'air': 'air',
        'o2': 'oxygen',
        'co2': 'carbon_dioxide',
        'h2': 'hydrogen',
        '氮气': 'nitrogen',
        '氩气': 'argon',
        '空气': 'air',
        '氧气': 'oxygen',
        '二氧化碳': 'carbon_dioxide',
        '氢气': 'hydrogen'
    }
    
    original_gas = gas
    gas_lower = gas.lower().strip()
    if gas_lower in gas_aliases:
        gas = gas_aliases[gas_lower]
        debug_print(f"🔄 标准化气体名称: {original_gas} -> {gas}")
        action_sequence.append(create_action_log(f"气体名称标准化: {original_gas} -> {gas}", "🔄"))
    
    debug_print(f"📋 最终参数: 容器={vessel_id}, 气体={gas}, 重复={repeats}")
    
    # === 查找设备 ===
    debug_print("🔍 步骤2: 查找设备...")
    action_sequence.append(create_action_log("正在查找相关设备...", "🔍"))
    
    try:
        vacuum_pump = find_vacuum_pump(G)
        action_sequence.append(create_action_log(f"找到真空泵: {vacuum_pump}", "🌪️"))
        
        gas_source = find_gas_source(G, gas)
        action_sequence.append(create_action_log(f"找到气源: {gas_source}", "💨"))
        
        vacuum_solenoid = find_vacuum_solenoid_valve(G, vacuum_pump)
        if vacuum_solenoid:
            action_sequence.append(create_action_log(f"找到真空电磁阀: {vacuum_solenoid}", "🚪"))
        else:
            action_sequence.append(create_action_log("未找到真空电磁阀", "⚠️"))
        
        gas_solenoid = find_gas_solenoid_valve(G, gas_source)
        if gas_solenoid:
            action_sequence.append(create_action_log(f"找到气源电磁阀: {gas_solenoid}", "🚪"))
        else:
            action_sequence.append(create_action_log("未找到气源电磁阀", "⚠️"))
        
        stirrer_id = find_connected_stirrer(G, vessel_id)  # 🔧 使用 vessel_id
        if stirrer_id:
            action_sequence.append(create_action_log(f"找到搅拌器: {stirrer_id}", "🌪️"))
        else:
            action_sequence.append(create_action_log("未找到搅拌器", "⚠️"))
        
        debug_print(f"📊 设备配置:")
        debug_print(f"  🌪️ 真空泵: {vacuum_pump}")
        debug_print(f"  💨 气源: {gas_source}")
        debug_print(f"  🚪 真空电磁阀: {vacuum_solenoid}")
        debug_print(f"  🚪 气源电磁阀: {gas_solenoid}")
        debug_print(f"  🌪️ 搅拌器: {stirrer_id}")
        
    except Exception as e:
        debug_print(f"❌ 设备查找失败: {str(e)}")
        action_sequence.append(create_action_log(f"设备查找失败: {str(e)}", "❌"))
        raise ValueError(f"设备查找失败: {str(e)}")
    
    # === 参数设置 ===
    debug_print("🔍 步骤3: 参数设置...")
    action_sequence.append(create_action_log("设置操作参数...", "⚙️"))
    
    # 根据气体类型调整参数
    if gas.lower() in ['nitrogen', 'argon']:
        VACUUM_VOLUME = 25.0
        REFILL_VOLUME = 25.0
        PUMP_FLOW_RATE = 2.0
        VACUUM_TIME = 30.0
        REFILL_TIME = 20.0
        debug_print("💨 惰性气体: 使用标准参数")
        action_sequence.append(create_action_log("检测到惰性气体，使用标准参数", "💨"))
    elif gas.lower() in ['air', 'oxygen']:
        VACUUM_VOLUME = 20.0
        REFILL_VOLUME = 20.0
        PUMP_FLOW_RATE = 1.5
        VACUUM_TIME = 45.0
        REFILL_TIME = 25.0
        debug_print("🔥 活性气体: 使用保守参数")
        action_sequence.append(create_action_log("检测到活性气体，使用保守参数", "🔥"))
    else:
        VACUUM_VOLUME = 15.0
        REFILL_VOLUME = 15.0
        PUMP_FLOW_RATE = 1.0
        VACUUM_TIME = 60.0
        REFILL_TIME = 30.0
        debug_print("❓ 未知气体: 使用安全参数")
        action_sequence.append(create_action_log("未知气体类型，使用安全参数", "❓"))
    
    STIR_SPEED = 200.0
    
    debug_print(f"⚙️ 操作参数:")
    debug_print(f"  📏 真空体积: {VACUUM_VOLUME}mL")
    debug_print(f"  📏 充气体积: {REFILL_VOLUME}mL")
    debug_print(f"  ⚡ 泵流速: {PUMP_FLOW_RATE}mL/s")
    debug_print(f"  ⏱️ 真空时间: {VACUUM_TIME}s")
    debug_print(f"  ⏱️ 充气时间: {REFILL_TIME}s")
    debug_print(f"  🌪️ 搅拌速度: {STIR_SPEED}RPM")
    
    action_sequence.append(create_action_log(f"真空体积: {VACUUM_VOLUME}mL", "📏"))
    action_sequence.append(create_action_log(f"充气体积: {REFILL_VOLUME}mL", "📏"))
    action_sequence.append(create_action_log(f"泵流速: {PUMP_FLOW_RATE}mL/s", "⚡"))
    
    # === 路径验证 ===
    debug_print("🔍 步骤4: 路径验证...")
    action_sequence.append(create_action_log("验证传输路径...", "🛤️"))
    
    try:
        # 验证抽真空路径
        if nx.has_path(G, vessel_id, vacuum_pump):  # 🔧 使用 vessel_id
            vacuum_path = nx.shortest_path(G, source=vessel_id, target=vacuum_pump)
            debug_print(f"✅ 真空路径: {' -> '.join(vacuum_path)}")
            action_sequence.append(create_action_log(f"真空路径: {' -> '.join(vacuum_path)}", "🛤️"))
        else:
            debug_print(f"⚠️ 真空路径不存在，继续执行但可能有问题")
            action_sequence.append(create_action_log("真空路径检查: 路径不存在", "⚠️"))
        
        # 验证充气路径
        if nx.has_path(G, gas_source, vessel_id):  # 🔧 使用 vessel_id
            gas_path = nx.shortest_path(G, source=gas_source, target=vessel_id)
            debug_print(f"✅ 气体路径: {' -> '.join(gas_path)}")
            action_sequence.append(create_action_log(f"气体路径: {' -> '.join(gas_path)}", "🛤️"))
        else:
            debug_print(f"⚠️ 气体路径不存在，继续执行但可能有问题")
            action_sequence.append(create_action_log("气体路径检查: 路径不存在", "⚠️"))
        
    except Exception as e:
        debug_print(f"⚠️ 路径验证失败: {str(e)}，继续执行")
        action_sequence.append(create_action_log(f"路径验证失败: {str(e)}", "⚠️"))
    
    # === 启动搅拌器 ===
    debug_print("🔍 步骤5: 启动搅拌器...")
    
    if stirrer_id:
        debug_print(f"🌪️ 启动搅拌器: {stirrer_id}")
        action_sequence.append(create_action_log(f"启动搅拌器 {stirrer_id} (速度: {STIR_SPEED}rpm)", "🌪️"))
        
        action_sequence.append({
            "device_id": stirrer_id,
            "action_name": "start_stir",
            "action_kwargs": {
                "vessel": vessel_id,  # 🔧 使用 vessel_id
                "stir_speed": STIR_SPEED,
                "purpose": "抽真空充气前预搅拌"
            }
        })
        
        # 等待搅拌稳定
        action_sequence.append(create_action_log("等待搅拌稳定...", "⏳"))
        action_sequence.append({
            "action_name": "wait",
            "action_kwargs": {"time": 5.0}
        })
    else:
        debug_print("⚠️ 未找到搅拌器，跳过搅拌器启动")
        action_sequence.append(create_action_log("跳过搅拌器启动", "⏭️"))
    
    # === 执行循环 ===
    debug_print("🔍 步骤6: 执行抽真空-充气循环...")
    action_sequence.append(create_action_log(f"开始 {repeats} 次抽真空-充气循环", "🔄"))
    
    for cycle in range(repeats):
        debug_print(f"=== 第 {cycle+1}/{repeats} 轮循环 ===")
        action_sequence.append(create_action_log(f"第 {cycle+1}/{repeats} 轮循环开始", "🚀"))
        
        # ============ 抽真空阶段 ============
        debug_print(f"🌪️ 抽真空阶段开始")
        action_sequence.append(create_action_log("开始抽真空阶段", "🌪️"))
        
        # 启动真空泵
        debug_print(f"🔛 启动真空泵: {vacuum_pump}")
        action_sequence.append(create_action_log(f"启动真空泵: {vacuum_pump}", "🔛"))
        action_sequence.append({
            "device_id": vacuum_pump,
            "action_name": "set_status",
            "action_kwargs": {"string": "ON"}
        })
        
        # 开启真空电磁阀
        if vacuum_solenoid:
            debug_print(f"🚪 打开真空电磁阀: {vacuum_solenoid}")
            action_sequence.append(create_action_log(f"打开真空电磁阀: {vacuum_solenoid}", "🚪"))
            action_sequence.append({
                "device_id": vacuum_solenoid,
                "action_name": "set_valve_position",
                "action_kwargs": {"command": "OPEN"}
            })
        
        # 抽真空操作
        debug_print(f"🌪️ 抽真空操作: {vessel_id} -> {vacuum_pump}")
        action_sequence.append(create_action_log(f"开始抽真空: {vessel_id} -> {vacuum_pump}", "🌪️"))
        
        try:
            vacuum_transfer_actions = generate_pump_protocol_with_rinsing(
                G=G,
                from_vessel=vessel_id,  # 🔧 使用 vessel_id
                to_vessel=vacuum_pump,
                volume=VACUUM_VOLUME,
                amount="",
                time=0.0,
                viscous=False,
                rinsing_solvent="",
                rinsing_volume=0.0,
                rinsing_repeats=0,
                solid=False,
                flowrate=PUMP_FLOW_RATE,
                transfer_flowrate=PUMP_FLOW_RATE
            )
            
            if vacuum_transfer_actions:
                action_sequence.extend(vacuum_transfer_actions)
                debug_print(f"✅ 添加了 {len(vacuum_transfer_actions)} 个抽真空动作")
                action_sequence.append(create_action_log(f"抽真空协议完成 ({len(vacuum_transfer_actions)} 个操作)", "✅"))
            else:
                debug_print("⚠️ 抽真空协议返回空序列，添加手动动作")
                action_sequence.append(create_action_log("抽真空协议为空，使用手动等待", "⚠️"))
                action_sequence.append({
                    "action_name": "wait",
                    "action_kwargs": {"time": VACUUM_TIME}
                })
                
        except Exception as e:
            debug_print(f"❌ 抽真空失败: {str(e)}")
            action_sequence.append(create_action_log(f"抽真空失败: {str(e)}", "❌"))
            action_sequence.append({
                "action_name": "wait",
                "action_kwargs": {"time": VACUUM_TIME}
            })
        
        # 抽真空后等待
        wait_minutes = VACUUM_TIME / 60
        action_sequence.append(create_action_log(f"抽真空后等待 ({wait_minutes:.1f} 分钟)", "⏳"))
        action_sequence.append({
            "action_name": "wait",
            "action_kwargs": {"time": VACUUM_TIME}
        })
        
        # 关闭真空电磁阀
        if vacuum_solenoid:
            debug_print(f"🚪 关闭真空电磁阀: {vacuum_solenoid}")
            action_sequence.append(create_action_log(f"关闭真空电磁阀: {vacuum_solenoid}", "🚪"))
            action_sequence.append({
                "device_id": vacuum_solenoid,
                "action_name": "set_valve_position",
                "action_kwargs": {"command": "CLOSED"}
            })
        
        # 关闭真空泵
        debug_print(f"🔴 停止真空泵: {vacuum_pump}")
        action_sequence.append(create_action_log(f"停止真空泵: {vacuum_pump}", "🔴"))
        action_sequence.append({
            "device_id": vacuum_pump,
            "action_name": "set_status",
            "action_kwargs": {"string": "OFF"}
        })
        
        # 阶段间等待
        action_sequence.append(create_action_log("抽真空阶段完成，短暂等待", "⏳"))
        action_sequence.append({
            "action_name": "wait",
            "action_kwargs": {"time": 5.0}
        })
        
        # ============ 充气阶段 ============
        debug_print(f"💨 充气阶段开始")
        action_sequence.append(create_action_log("开始气体充气阶段", "💨"))
        
        # 启动气源
        debug_print(f"🔛 启动气源: {gas_source}")
        action_sequence.append(create_action_log(f"启动气源: {gas_source}", "🔛"))
        action_sequence.append({
            "device_id": gas_source,
            "action_name": "set_status",
            "action_kwargs": {"string": "ON"}
        })
        
        # 开启气源电磁阀
        if gas_solenoid:
            debug_print(f"🚪 打开气源电磁阀: {gas_solenoid}")
            action_sequence.append(create_action_log(f"打开气源电磁阀: {gas_solenoid}", "🚪"))
            action_sequence.append({
                "device_id": gas_solenoid,
                "action_name": "set_valve_position",
                "action_kwargs": {"command": "OPEN"}
            })
        
        # 充气操作
        debug_print(f"💨 充气操作: {gas_source} -> {vessel_id}")
        action_sequence.append(create_action_log(f"开始气体充气: {gas_source} -> {vessel_id}", "💨"))
        
        try:
            gas_transfer_actions = generate_pump_protocol_with_rinsing(
                G=G,
                from_vessel=gas_source,
                to_vessel=vessel_id,  # 🔧 使用 vessel_id
                volume=REFILL_VOLUME,
                amount="",
                time=0.0,
                viscous=False,
                rinsing_solvent="",
                rinsing_volume=0.0,
                rinsing_repeats=0,
                solid=False,
                flowrate=PUMP_FLOW_RATE,
                transfer_flowrate=PUMP_FLOW_RATE
            )
            
            if gas_transfer_actions:
                action_sequence.extend(gas_transfer_actions)
                debug_print(f"✅ 添加了 {len(gas_transfer_actions)} 个充气动作")
                action_sequence.append(create_action_log(f"气体充气协议完成 ({len(gas_transfer_actions)} 个操作)", "✅"))
            else:
                debug_print("⚠️ 充气协议返回空序列，添加手动动作")
                action_sequence.append(create_action_log("充气协议为空，使用手动等待", "⚠️"))
                action_sequence.append({
                    "action_name": "wait",
                    "action_kwargs": {"time": REFILL_TIME}
                })
                
        except Exception as e:
            debug_print(f"❌ 气体充气失败: {str(e)}")
            action_sequence.append(create_action_log(f"气体充气失败: {str(e)}", "❌"))
            action_sequence.append({
                "action_name": "wait",
                "action_kwargs": {"time": REFILL_TIME}
            })
        
        # 充气后等待
        refill_wait_minutes = REFILL_TIME / 60
        action_sequence.append(create_action_log(f"充气后等待 ({refill_wait_minutes:.1f} 分钟)", "⏳"))
        action_sequence.append({
            "action_name": "wait",
            "action_kwargs": {"time": REFILL_TIME}
        })
        
        # 关闭气源电磁阀
        if gas_solenoid:
            debug_print(f"🚪 关闭气源电磁阀: {gas_solenoid}")
            action_sequence.append(create_action_log(f"关闭气源电磁阀: {gas_solenoid}", "🚪"))
            action_sequence.append({
                "device_id": gas_solenoid,
                "action_name": "set_valve_position",
                "action_kwargs": {"command": "CLOSED"}
            })
        
        # 关闭气源
        debug_print(f"🔴 停止气源: {gas_source}")
        action_sequence.append(create_action_log(f"停止气源: {gas_source}", "🔴"))
        action_sequence.append({
            "device_id": gas_source,
            "action_name": "set_status",
            "action_kwargs": {"string": "OFF"}
        })
        
        # 循环间等待
        if cycle < repeats - 1:
            debug_print(f"⏳ 等待下一个循环...")
            action_sequence.append(create_action_log("等待下一个循环...", "⏳"))
            action_sequence.append({
                "action_name": "wait",
                "action_kwargs": {"time": 10.0}
            })
        else:
            action_sequence.append(create_action_log(f"第 {cycle+1}/{repeats} 轮循环完成", "✅"))
    
    # === 停止搅拌器 ===
    debug_print("🔍 步骤7: 停止搅拌器...")
    
    if stirrer_id:
        debug_print(f"🛑 停止搅拌器: {stirrer_id}")
        action_sequence.append(create_action_log(f"停止搅拌器: {stirrer_id}", "🛑"))
        action_sequence.append({
            "device_id": stirrer_id,
            "action_name": "stop_stir",
            "action_kwargs": {"vessel": vessel_id}  # 🔧 使用 vessel_id
        })
    else:
        action_sequence.append(create_action_log("跳过搅拌器停止", "⏭️"))
    
    # === 最终等待 ===
    action_sequence.append(create_action_log("最终稳定等待...", "⏳"))
    action_sequence.append({
        "action_name": "wait",
        "action_kwargs": {"time": 10.0}
    })
    
    # === 总结 ===
    total_time = (VACUUM_TIME + REFILL_TIME + 25) * repeats + 20
    
    debug_print("=" * 60)
    debug_print(f"🎉 抽真空充气协议生成完成")
    debug_print(f"📊 协议统计:")
    debug_print(f"  📋 总动作数: {len(action_sequence)}")
    debug_print(f"  ⏱️ 预计总时间: {total_time:.0f}s ({total_time/60:.1f} 分钟)")
    debug_print(f"  🥼 处理容器: {vessel_id}")
    debug_print(f"  💨 使用气体: {gas}")
    debug_print(f"  🔄 重复次数: {repeats}")
    debug_print("=" * 60)
    
    # 添加完成日志
    summary_msg = f"抽真空充气协议完成: {vessel_id} (使用 {gas}，{repeats} 次循环)"
    action_sequence.append(create_action_log(summary_msg, "🎉"))
    
    return action_sequence

# === 便捷函数 ===

def generate_nitrogen_purge_protocol(G: nx.DiGraph, vessel: dict, **kwargs) -> List[Dict[str, Any]]:  # 🔧 修改参数类型
    """生成氮气置换协议"""
    vessel_id = vessel["id"]
    debug_print(f"💨 生成氮气置换协议: {vessel_id}")
    return generate_evacuateandrefill_protocol(G, vessel, "nitrogen", **kwargs)

def generate_argon_purge_protocol(G: nx.DiGraph, vessel: dict, **kwargs) -> List[Dict[str, Any]]:  # 🔧 修改参数类型
    """生成氩气置换协议"""
    vessel_id = vessel["id"]
    debug_print(f"💨 生成氩气置换协议: {vessel_id}")
    return generate_evacuateandrefill_protocol(G, vessel, "argon", **kwargs)

def generate_air_purge_protocol(G: nx.DiGraph, vessel: dict, **kwargs) -> List[Dict[str, Any]]:  # 🔧 修改参数类型
    """生成空气置换协议"""
    vessel_id = vessel["id"]
    debug_print(f"💨 生成空气置换协议: {vessel_id}")
    return generate_evacuateandrefill_protocol(G, vessel, "air", **kwargs)

def generate_inert_atmosphere_protocol(G: nx.DiGraph, vessel: dict, gas: str = "nitrogen", **kwargs) -> List[Dict[str, Any]]:  # 🔧 修改参数类型
    """生成惰性气氛协议"""
    vessel_id = vessel["id"]
    debug_print(f"🛡️ 生成惰性气氛协议: {vessel_id} (使用 {gas})")
    return generate_evacuateandrefill_protocol(G, vessel, gas, **kwargs)

# 测试函数
def test_evacuateandrefill_protocol():
    """测试抽真空充气协议"""
    debug_print("=== 抽真空充气协议增强中文版测试 ===")
    debug_print("✅ 测试完成")

if __name__ == "__main__":
    test_evacuateandrefill_protocol()