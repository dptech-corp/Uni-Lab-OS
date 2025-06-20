from typing import List, Dict, Any
import networkx as nx
from .pump_protocol import generate_pump_protocol


def get_vessel_liquid_volume(G: nx.DiGraph, vessel: str) -> float:
    """获取容器中的液体体积"""
    if vessel not in G.nodes():
        return 0.0
    
    vessel_data = G.nodes[vessel].get('data', {})
    liquids = vessel_data.get('liquid', [])
    
    total_volume = 0.0
    for liquid in liquids:
        if isinstance(liquid, dict):
            # 支持两种格式：新格式 (name, volume) 和旧格式 (liquid_type, liquid_volume)
            volume = liquid.get('volume') or liquid.get('liquid_volume', 0.0)
            total_volume += volume
    
    return total_volume


def find_column_device(G: nx.DiGraph, column: str) -> str:
    """查找柱层析设备"""
    # 首先检查是否有虚拟柱设备
    column_nodes = [node for node in G.nodes() 
                   if (G.nodes[node].get('class') or '') == 'virtual_column']
    
    if column_nodes:
        return column_nodes[0]
    
    # 如果没有虚拟柱设备，抛出异常
    raise ValueError(f"系统中未找到柱层析设备。请确保配置了 virtual_column 设备")


def find_column_vessel(G: nx.DiGraph, column: str) -> str:
    """查找柱容器"""
    # 直接使用 column 参数作为容器名称
    if column in G.nodes():
        return column
    
    # 尝试常见的柱容器命名规则
    possible_names = [
        f"column_{column}",
        f"{column}_column",
        f"vessel_{column}",
        f"{column}_vessel",
        "column_vessel",
        "chromatography_column",
        "silica_column",
        "preparative_column"
    ]
    
    for vessel_name in possible_names:
        if vessel_name in G.nodes():
            return vessel_name
    
    raise ValueError(f"未找到柱容器 '{column}'。尝试了以下名称: {[column] + possible_names}")


def find_eluting_solvent_vessel(G: nx.DiGraph, eluting_solvent: str) -> str:
    """查找洗脱溶剂容器"""
    if not eluting_solvent:
        return ""
    
    # 按照命名规则查找溶剂瓶
    solvent_vessel_id = f"flask_{eluting_solvent}"
    
    if solvent_vessel_id in G.nodes():
        return solvent_vessel_id
    
    # 如果直接匹配失败，尝试模糊匹配
    for node in G.nodes():
        if node.startswith('flask_') and eluting_solvent.lower() in node.lower():
            return node
    
    # 如果还是找不到，列出所有可用的溶剂瓶
    available_flasks = [node for node in G.nodes() 
                       if node.startswith('flask_') 
                       and G.nodes[node].get('type') == 'container']
    
    raise ValueError(f"找不到洗脱溶剂 '{eluting_solvent}' 对应的溶剂瓶。可用溶剂瓶: {available_flasks}")


def generate_run_column_protocol(
    G: nx.DiGraph,
    from_vessel: str,
    to_vessel: str,
    column: str
) -> List[Dict[str, Any]]:
    """
    生成柱层析分离的协议序列
    
    Args:
        G: 有向图，节点为设备和容器，边为流体管道
        from_vessel: 源容器的名称，即样品起始所在的容器
        to_vessel: 目标容器的名称，分离后的样品要到达的容器
        column: 所使用的柱子的名称
    
    Returns:
        List[Dict[str, Any]]: 柱层析分离操作的动作序列
    """
    action_sequence = []
    
    print(f"RUN_COLUMN: 开始生成柱层析协议")
    print(f"  - 源容器: {from_vessel}")
    print(f"  - 目标容器: {to_vessel}")
    print(f"  - 柱子: {column}")
    
    # 验证源容器和目标容器存在
    if from_vessel not in G.nodes():
        raise ValueError(f"源容器 '{from_vessel}' 不存在于系统中")
    
    if to_vessel not in G.nodes():
        raise ValueError(f"目标容器 '{to_vessel}' 不存在于系统中")
    
    # 查找柱层析设备
    column_device_id = None
    column_nodes = [node for node in G.nodes() 
                   if (G.nodes[node].get('class') or '') == 'virtual_column']
    
    if column_nodes:
        column_device_id = column_nodes[0]
        print(f"RUN_COLUMN: 找到柱层析设备: {column_device_id}")
    else:
        print(f"RUN_COLUMN: 警告 - 未找到柱层析设备")
    
    # 获取源容器中的液体体积
    source_volume = get_vessel_liquid_volume(G, from_vessel)
    print(f"RUN_COLUMN: 源容器 {from_vessel} 中有 {source_volume} mL 液体")
    
    # === 第一步：样品转移到柱子（如果柱子是容器） ===
    if column in G.nodes() and G.nodes[column].get('type') == 'container':
        print(f"RUN_COLUMN: 样品转移 - {source_volume} mL 从 {from_vessel} 到 {column}")
        
        try:
            sample_transfer_actions = generate_pump_protocol(
                G=G,
                from_vessel=from_vessel,
                to_vessel=column,
                volume=source_volume if source_volume > 0 else 100.0,
                flowrate=2.0
            )
            action_sequence.extend(sample_transfer_actions)
        except Exception as e:
            print(f"RUN_COLUMN: 样品转移失败: {str(e)}")
    
    # === 第二步：使用柱层析设备执行分离 ===
    if column_device_id:
        print(f"RUN_COLUMN: 使用柱层析设备执行分离")
        
        column_separation_action = {
            "device_id": column_device_id,
            "action_name": "run_column",
            "action_kwargs": {
                "from_vessel": from_vessel,
                "to_vessel": to_vessel,
                "column": column
            }
        }
        action_sequence.append(column_separation_action)
        
        # 等待柱层析设备完成分离
        action_sequence.append({
            "action_name": "wait",
            "action_kwargs": {"time": 60}
        })
    
    # === 第三步：从柱子转移到目标容器（如果需要） ===
    if column in G.nodes() and column != to_vessel:
        print(f"RUN_COLUMN: 产物转移 - 从 {column} 到 {to_vessel}")
        
        try:
            product_transfer_actions = generate_pump_protocol(
                G=G,
                from_vessel=column,
                to_vessel=to_vessel,
                volume=source_volume * 0.8 if source_volume > 0 else 80.0,  # 假设有一些损失
                flowrate=1.5
            )
            action_sequence.extend(product_transfer_actions)
        except Exception as e:
            print(f"RUN_COLUMN: 产物转移失败: {str(e)}")
    
    print(f"RUN_COLUMN: 生成了 {len(action_sequence)} 个动作")
    return action_sequence


# 便捷函数：常用柱层析方案
def generate_flash_column_protocol(
    G: nx.DiGraph,
    from_vessel: str,
    to_vessel: str,
    column_material: str = "silica_gel",
    mobile_phase: str = "ethyl_acetate",
    mobile_phase_volume: float = 100.0
) -> List[Dict[str, Any]]:
    """快速柱层析：高流速分离"""
    return generate_run_column_protocol(
        G, from_vessel, to_vessel, column_material, 
        mobile_phase, mobile_phase_volume, 1, "", 0.0, 3.0
    )


def generate_preparative_column_protocol(
    G: nx.DiGraph,
    from_vessel: str,
    to_vessel: str,
    column_material: str = "silica_gel",
    equilibration_solvent: str = "hexane",
    eluting_solvent: str = "ethyl_acetate",
    eluting_volume: float = 50.0,
    eluting_repeats: int = 3
) -> List[Dict[str, Any]]:
    """制备柱层析：带平衡和多次洗脱"""
    return generate_run_column_protocol(
        G, from_vessel, to_vessel, column_material,
        eluting_solvent, eluting_volume, eluting_repeats,
        equilibration_solvent, 30.0, 1.5
    )


def generate_gradient_column_protocol(
    G: nx.DiGraph,
    from_vessel: str,
    to_vessel: str,
    column_material: str = "silica_gel",
    gradient_solvents: List[str] = None,
    gradient_volumes: List[float] = None
) -> List[Dict[str, Any]]:
    """梯度洗脱柱层析：多种溶剂系统"""
    if gradient_solvents is None:
        gradient_solvents = ["hexane", "ethyl_acetate", "methanol"]
    if gradient_volumes is None:
        gradient_volumes = [50.0, 50.0, 30.0]
    
    action_sequence = []
    
    # 每种溶剂单独执行一次柱层析
    for i, (solvent, volume) in enumerate(zip(gradient_solvents, gradient_volumes)):
        print(f"RUN_COLUMN: 梯度洗脱第 {i+1}/{len(gradient_solvents)} 步: {volume} mL {solvent}")
        
        # 第一步使用源容器，后续步骤使用柱子作为源
        step_from_vessel = from_vessel if i == 0 else column_material
        # 最后一步使用目标容器，其他步骤使用柱子作为目标
        step_to_vessel = to_vessel if i == len(gradient_solvents) - 1 else column_material
        
        step_actions = generate_run_column_protocol(
            G, step_from_vessel, step_to_vessel, column_material,
            solvent, volume, 1, "", 0.0, 1.0
        )
        action_sequence.extend(step_actions)
        
        # 在梯度步骤之间加入等待时间
        if i < len(gradient_solvents) - 1:
            action_sequence.append({
                "action_name": "wait",
                "action_kwargs": {"time": 20}
            })
    
    return action_sequence


def generate_reverse_phase_column_protocol(
    G: nx.DiGraph,
    from_vessel: str,
    to_vessel: str,
    column_material: str = "C18",
    aqueous_phase: str = "water",
    organic_phase: str = "methanol",
    gradient_ratio: float = 0.5
) -> List[Dict[str, Any]]:
    """反相柱层析：C18柱，水-有机相梯度"""
    # 先用水相平衡
    equilibration_volume = 20.0
    # 然后用有机相洗脱
    eluting_volume = 30.0 * gradient_ratio
    
    return generate_run_column_protocol(
        G, from_vessel, to_vessel, column_material,
        organic_phase, eluting_volume, 2,
        aqueous_phase, equilibration_volume, 0.8
    )


def generate_ion_exchange_column_protocol(
    G: nx.DiGraph,
    from_vessel: str,
    to_vessel: str,
    column_material: str = "ion_exchange",
    buffer_solution: str = "buffer",
    salt_solution: str = "NaCl_solution",
    salt_volume: float = 40.0
) -> List[Dict[str, Any]]:
    """离子交换柱层析：缓冲液平衡，盐溶液洗脱"""
    return generate_run_column_protocol(
        G, from_vessel, to_vessel, column_material,
        salt_solution, salt_volume, 1,
        buffer_solution, 25.0, 0.5
    )


# 测试函数
def test_run_column_protocol():
    """测试柱层析协议的示例"""
    print("=== RUN COLUMN PROTOCOL 测试 ===")
    print("测试完成")


if __name__ == "__main__":
    test_run_column_protocol()