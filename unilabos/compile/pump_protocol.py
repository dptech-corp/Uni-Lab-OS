import numpy as np
import networkx as nx


def generate_pump_protocol(
    G: nx.DiGraph, 
    from_vessel: str, 
    to_vessel: str, 
    volume: float, 
    flowrate: float = 0.5,
    transfer_flowrate: float = 0, 
) -> list[dict]:
    """
    生成泵操作的动作序列。
    
    :param G: 有向图, 节点为容器和注射泵, 边为流体管道, A→B边的属性为管道接A端的阀门位置
    :param from_vessel: 容器A
    :param to_vessel: 容器B
    :param volume: 转移的体积
    :param flowrate: 最终注入容器B时的流速
    :param transfer_flowrate: 泵骨架中转移流速（若不指定，默认与注入流速相同）
    :return: 泵操作的动作序列
    """
    
    # 生成泵操作的动作序列
    pump_action_sequence = []
    nodes = G.nodes(data=True)
    # 从from_vessel到to_vessel的最短路径
    shortest_path = nx.shortest_path(G, source=from_vessel, target=to_vessel)
    print(shortest_path)

    pump_backbone = shortest_path
    if not from_vessel.startswith("pump"):
        pump_backbone = pump_backbone[1:]
    if not to_vessel.startswith("pump"):
        pump_backbone = pump_backbone[:-1]
    
    if transfer_flowrate == 0:
        transfer_flowrate = flowrate
    
    min_transfer_volume = min([nodes[pump]["max_volume"] for pump in pump_backbone])
    repeats = int(np.ceil(volume / min_transfer_volume))
    if repeats > 1 and (from_vessel.startswith("pump") or to_vessel.startswith("pump")):
        raise ValueError("Cannot transfer volume larger than min_transfer_volume between two pumps.")
    
    volume_left = volume
    
    # 生成泵操作的动作序列
    for i in range(repeats):
        # 单泵依次执行阀指令、活塞指令，将液体吸入与之相连的第一台泵
        if not from_vessel.startswith("pump"):
            pump_action_sequence.extend([
                {
                    "device_id": pump_backbone[0], 
                    "action_name": "set_valve_position",
                    "action_kwargs": {
                        "command": G.get_edge_data(pump_backbone[0], from_vessel)["port"][pump_backbone[0]]
                    }
                },
                {
                    "device_id": pump_backbone[0], 
                    "action_name": "set_position",
                    "action_kwargs": {
                        "position": float(min(volume_left, min_transfer_volume)),
                        "max_velocity": transfer_flowrate
                    }
                }
            ])
            pump_action_sequence.append({"action_name": "wait", "action_kwargs": {"time": 5}})
        for pumpA, pumpB in zip(pump_backbone[:-1], pump_backbone[1:]):
            # 相邻两泵同时切换阀门至连通位置
            pump_action_sequence.append([
            {
                "device_id": pumpA,
                "action_name": "set_valve_position",
                "action_kwargs": {
                    "command": G.get_edge_data(pumpA, pumpB)["port"][pumpA]
                }
            },
            {
                "device_id": pumpB, 
                "action_name": "set_valve_position",
                "action_kwargs": {
                    "command": G.get_edge_data(pumpB, pumpA)["port"][pumpB],
                }
            }
            ])
            # 相邻两泵液体转移：泵A排出液体，泵B吸入液体
            pump_action_sequence.append([
            {
                "device_id": pumpA,
                "action_name": "set_position",
                "action_kwargs": {
                    "position": 0.0,
                    "max_velocity": transfer_flowrate
                }
            },
            {
                "device_id": pumpB, 
                "action_name": "set_position",
                "action_kwargs": {
                    "position": float(min(volume_left, min_transfer_volume)),
                    "max_velocity": transfer_flowrate
                }
            }
            ])
            pump_action_sequence.append({"action_name": "wait", "action_kwargs": {"time": 5}})
        
        if not to_vessel.startswith("pump"):
            # 单泵依次执行阀指令、活塞指令，将最后一台泵液体缓慢加入容器B
            pump_action_sequence.extend([
                {
                    "device_id": pump_backbone[-1], 
                    "action_name": "set_valve_position",
                    "action_kwargs": {
                        "command": G.get_edge_data(pump_backbone[-1], to_vessel)["port"][pump_backbone[-1]]
                    }
                },
                {
                    "device_id": pump_backbone[-1], 
                    "action_name": "set_position",
                    "action_kwargs": {
                        "position": 0.0,
                        "max_velocity": flowrate
                    }
                }
            ])
            pump_action_sequence.append({"action_name": "wait", "action_kwargs": {"time": 5}})
        
        volume_left -= min_transfer_volume
    return pump_action_sequence


# Pump protocol compilation
def generate_pump_protocol_with_rinsing(
    G: nx.DiGraph, 
    from_vessel: str, 
    to_vessel: str, 
    volume: float, 
    amount: str = "",
    time: float = 0,
    viscous: bool = False,
    rinsing_solvent: str = "air",
    rinsing_volume: float = 5.0,
    rinsing_repeats: int = 2,
    solid: bool = False,
    flowrate: float = 2.5,
    transfer_flowrate: float = 0.5, 
) -> list[dict]:
    """
    Generates a pump protocol for transferring a specified volume between vessels, including rinsing steps with a chosen solvent. This function constructs a sequence of pump actions based on the provided parameters and the shortest path in a directed graph.
    
    Args:
        G (nx.DiGraph): The directed graph representing the vessels and connections. 有向图, 节点为容器和注射泵, 边为流体管道, A→B边的属性为管道接A端的阀门位置
        from_vessel (str): The name of the vessel to transfer from.
        to_vessel (str): The name of the vessel to transfer to.
        volume (float): The volume to transfer.
        amount (str, optional): Additional amount specification (default is "").
        time (float, optional): Time over which to perform the transfer (default is 0).
        viscous (bool, optional): Indicates if the fluid is viscous (default is False).
        rinsing_solvent (str, optional): The solvent to use for rinsing (default is "air").
        rinsing_volume (float, optional): The volume of rinsing solvent to use (default is 5.0).
        rinsing_repeats (int, optional): The number of times to repeat rinsing (default is 2).
        solid (bool, optional): Indicates if the transfer involves a solid (default is False).
        flowrate (float, optional): The flow rate for the transfer (default is 2.5). 最终注入容器B时的流速
        transfer_flowrate (float, optional): The flow rate for the transfer action (default is 0.5). 泵骨架中转移流速（若不指定，默认与注入流速相同）
    
    Returns:
        list[dict]: A sequence of pump actions to be executed for the transfer and rinsing process. 泵操作的动作序列.
    
    Raises:
        AssertionError: If the number of rinsing solvents does not match the number of rinsing repeats.
    
    Examples:
        pump_protocol = generate_pump_protocol_with_rinsing(G, "vessel_A", "vessel_B", 0.1, rinsing_solvent="water")
    """
    air_vessel = "flask_air"
    waste_vessel = f"waste_workup"
    
    shortest_path = nx.shortest_path(G, source=from_vessel, target=to_vessel)
    pump_backbone = shortest_path[1: -1]
    nodes = G.nodes(data=True)
    min_transfer_volume = float(min([nodes[pump]["max_volume"] for pump in pump_backbone]))
    if time != 0:
        flowrate = transfer_flowrate = volume / time
    
    pump_action_sequence = generate_pump_protocol(G, from_vessel, to_vessel, float(volume), flowrate, transfer_flowrate)
    if rinsing_solvent != "air":
        if "," in rinsing_solvent:
            rinsing_solvents = rinsing_solvent.split(",")
            assert len(rinsing_solvents) == rinsing_repeats, "Number of rinsing solvents must match number of rinsing repeats."
        else:
            rinsing_solvents = [rinsing_solvent] * rinsing_repeats
        
        for rinsing_solvent in rinsing_solvents:
            solvent_vessel = f"flask_{rinsing_solvent}"
            # 清洗泵
            pump_action_sequence.extend(
                generate_pump_protocol(G, solvent_vessel, pump_backbone[0], min_transfer_volume, flowrate, transfer_flowrate) +
                generate_pump_protocol(G, pump_backbone[0], pump_backbone[-1], min_transfer_volume, flowrate, transfer_flowrate) +
                generate_pump_protocol(G, pump_backbone[-1], waste_vessel, min_transfer_volume, flowrate, transfer_flowrate)
            )
            # 如果转移的是溶液，第一种冲洗溶剂请选用溶液的溶剂，稀释泵内、转移管道内的溶液。后续冲洗溶剂不需要此操作。
            if rinsing_solvent == rinsing_solvents[0]:
                pump_action_sequence.extend(generate_pump_protocol(G, solvent_vessel, from_vessel, rinsing_volume, flowrate, transfer_flowrate))
                pump_action_sequence.extend(generate_pump_protocol(G, solvent_vessel, to_vessel, rinsing_volume, flowrate, transfer_flowrate))
            pump_action_sequence.extend(generate_pump_protocol(G, air_vessel, solvent_vessel, rinsing_volume, flowrate, transfer_flowrate))
            pump_action_sequence.extend(generate_pump_protocol(G, air_vessel, waste_vessel, rinsing_volume, flowrate, transfer_flowrate))
    pump_action_sequence.extend(generate_pump_protocol(G, air_vessel, from_vessel, rinsing_volume, flowrate, transfer_flowrate) * 2)
    pump_action_sequence.extend(generate_pump_protocol(G, air_vessel, to_vessel, rinsing_volume, flowrate, transfer_flowrate) * 2)
    
    return pump_action_sequence
# End Protocols
