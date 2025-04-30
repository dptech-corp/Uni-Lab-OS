import numpy as np
import networkx as nx


def generate_evaporate_protocol(
    G: nx.DiGraph, 
    vessel: str,
    pressure: float,
    temp: float,
    time: float,
    stir_speed: float
) -> list[dict]:
    """
    Generate a protocol to evaporate a solution from a vessel.
    
    :param G: Directed graph. Nodes are containers and pumps, edges are fluidic connections.
    :param vessel: Vessel to clean.
    :param solvent: Solvent to clean vessel with.
    :param volume: Volume of solvent to clean vessel with.
    :param temp: Temperature to heat vessel to while cleaning.
    :param repeats: Number of cleaning cycles to perform.
    :return: List of actions to clean vessel.
    """
    
    # 生成泵操作的动作序列
    pump_action_sequence = []
    reactor_volume = 500.0
    transfer_flowrate = flowrate = 2.5
    
    # 开启冷凝器
    pump_action_sequence.append({
        "device_id": "rotavap_chiller",
        "action_name": "set_temperature",
        "action_kwargs": {
            "command": "-40"
        }
    })
    # TODO: 通过温度反馈改为 HeatChillToTemp，而非等待固定时间
    pump_action_sequence.append({
        "action_name": "wait",
        "action_kwargs": {
            "time": 1800
        }
    })
    
    # 开启旋蒸真空泵、旋转，在液体转移后运行time时间
    pump_action_sequence.append({
        "device_id": "rotavap_controller",
        "action_name": "set_pump_time",
        "action_kwargs": {
            "command": str(time + reactor_volume / flowrate * 3)
        }
    })
    pump_action_sequence.append({
        "device_id": "rotavap_controller",
        "action_name": "set_pump_time",
        "action_kwargs": {
            "command": str(time + reactor_volume / flowrate * 3)
        }
    })
    
    # 液体转入旋转蒸发器
    pump_action_sequence.append({
        "device_id": "", 
        "action_name": "PumpTransferProtocol",
        "action_kwargs": {
            "from_vessel": vessel,
            "to_vessel": "rotavap",
            "volume": reactor_volume,
            "time": reactor_volume / flowrate,
            # "transfer_flowrate": transfer_flowrate,
        }
    })
    
    pump_action_sequence.append({
        "action_name": "wait",
        "action_kwargs": {
            "time": time
        }
    })
    return pump_action_sequence
