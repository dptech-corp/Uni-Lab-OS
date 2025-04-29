import numpy as np
import networkx as nx


def generate_separate_protocol(
    G: nx.DiGraph, 
    purpose: str,  # 'wash' or 'extract'. 'wash' means that product phase will not be the added solvent phase, 'extract' means product phase will be the added solvent phase. If no solvent is added just use 'extract'.
    product_phase: str, # 'top' or 'bottom'. Phase that product will be in.
    from_vessel: str, #Contents of from_vessel are transferred to separation_vessel and separation is performed.
    separation_vessel: str, # Vessel in which separation of phases will be carried out.
    to_vessel: str, # Vessel to send product phase to.
    waste_phase_to_vessel: str, # Optional. Vessel to send waste phase to.
    solvent: str, # Optional. Solvent to add to separation vessel after contents of from_vessel has been transferred to create two phases.
    solvent_volume: float = 50, # Optional. Volume of solvent to add (mL).
    through: str = "", # Optional. Solid chemical to send product phase through on way to to_vessel, e.g. 'celite'.
    repeats: int = 1, # Optional. Number of separations to perform.
    stir_time: float = 30, # Optional. Time stir for after adding solvent, before separation of phases.
    stir_speed: float = 300, # Optional. Speed to stir at after adding solvent, before separation of phases.
    settling_time: float = 300 # Optional. Time
) -> list[dict]:
    """
    Generate a protocol to clean a vessel with a solvent.
    
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
    waste_vessel = waste_phase_to_vessel
    
    # TODO：通过物料管理系统找到溶剂的容器
    if "," in solvent:
        solvents = solvent.split(",")
        assert len(solvents) == repeats, "Number of solvents must match number of repeats."
    else:
        solvents = [solvent] * repeats
    
    # TODO: 通过设备连接图找到分离容器的控制器、底部出口
    separator_controller = f"{separation_vessel}_controller"
    separation_vessel_bottom = f"flask_{separation_vessel}"
    
    transfer_flowrate = flowrate = 2.5
    
    if from_vessel != separation_vessel:
        pump_action_sequence.append(
            {
                "device_id": "", 
                "action_name": "PumpTransferProtocol",
                "action_kwargs": {
                    "from_vessel": from_vessel,
                    "to_vessel": separation_vessel,
                    "volume": reactor_volume,
                    "time": reactor_volume / flowrate,
                    # "transfer_flowrate": transfer_flowrate,
                }
            }
        )
        
        # for i in range(2):
        #     pump_action_sequence.append(
        #         {
        #             "device_id": "", 
        #             "action_name": "CleanProtocol",
        #             "action_kwargs": {
        #                 "vessel": from_vessel,
        #                 "solvent": "H2O", # Solvent to clean vessel with.
        #                 "volume": solvent_volume, # Optional. Volume of solvent to clean vessel with.
        #                 "temp": 25.0, # Optional. Temperature to heat vessel to while cleaning.
        #                 "repeats": 1
        #             }
        #         }
        #     )
        #     pump_action_sequence.append(
        #         {
        #             "device_id": "", 
        #             "action_name": "CleanProtocol",
        #             "action_kwargs": {
        #                 "vessel": from_vessel,
        #                 "solvent": "CH2Cl2", # Solvent to clean vessel with.
        #                 "volume": solvent_volume, # Optional. Volume of solvent to clean vessel with.
        #                 "temp": 25.0, # Optional. Temperature to heat vessel to while cleaning.
        #                 "repeats": 1
        #             }
        #         }
        #     )
    
    # 生成泵操作的动作序列
    for i in range(repeats):
        # 找到当次萃取所用溶剂
        solvent_thistime = solvents[i]
        solvent_vessel = f"flask_{solvent_thistime}"
        
        pump_action_sequence.append(
            {
                "device_id": "", 
                "action_name": "PumpTransferProtocol",
                "action_kwargs": {
                    "from_vessel": solvent_vessel,
                    "to_vessel": separation_vessel,
                    "volume": solvent_volume,
                    "time": solvent_volume / flowrate,
                    # "transfer_flowrate": transfer_flowrate,
                }
            }
        )
        pump_action_sequence.extend([
            # 搅拌、静置
            {
                "device_id": separator_controller, 
                "action_name": "stir",
                "action_kwargs": {
                    "stir_time": stir_time,
                    "stir_speed": stir_speed,
                    "settling_time": settling_time
                }
            },
            # 分液（判断电导突跃）
            {
                "device_id": separator_controller, 
                "action_name": "valve_open",
                "action_kwargs": {
                    "command": "delta > 0.05"
                }
            }
        ])
        
        if product_phase == "bottom":
            # 产物转移到目标瓶
            pump_action_sequence.append(
                {
                    "device_id": "", 
                    "action_name": "PumpTransferProtocol",
                    "action_kwargs": {
                        "from_vessel": separation_vessel_bottom,
                        "to_vessel": to_vessel,
                        "volume": 250.0,
                        "time": 250.0 / flowrate,
                        # "transfer_flowrate": transfer_flowrate,
                    }
                }
            )
            # 放出上面那一相，60秒后关阀门
            pump_action_sequence.append(
                {
                    "device_id": separator_controller, 
                    "action_name": "valve_open",
                    "action_kwargs": {
                        "command": "time > 60"
                    }
                }
            )
            # 弃去上面那一相进废液
            pump_action_sequence.append(
                {
                    "device_id": "", 
                    "action_name": "PumpTransferProtocol",
                    "action_kwargs": {
                        "from_vessel": separation_vessel_bottom,
                        "to_vessel": waste_vessel,
                        "volume": 250.0,
                        "time": 250.0 / flowrate,
                        # "transfer_flowrate": transfer_flowrate,
                    }
                }
            )
        elif product_phase == "top":
            # 弃去下面那一相进废液
            pump_action_sequence.append(
                {
                    "device_id": "", 
                    "action_name": "PumpTransferProtocol",
                    "action_kwargs": {
                        "from_vessel": separation_vessel_bottom,
                        "to_vessel": waste_vessel,
                        "volume": 250.0,
                        "time": 250.0 / flowrate,
                        # "transfer_flowrate": transfer_flowrate,
                    }
                }
            )
            # 放出上面那一相
            pump_action_sequence.append(
                {
                    "device_id": separator_controller, 
                    "action_name": "valve_open",
                    "action_kwargs": {
                        "command": "time > 60"
                    }
                }
            )
            # 产物转移到目标瓶
            pump_action_sequence.append(
                {
                    "device_id": "", 
                    "action_name": "PumpTransferProtocol",
                    "action_kwargs": {
                        "from_vessel": separation_vessel_bottom,
                        "to_vessel": to_vessel,
                        "volume": 250.0,
                        "time": 250.0 / flowrate,
                        # "transfer_flowrate": transfer_flowrate,
                    }
                }
            )
        elif product_phase == "organic":
            pass
        
        # 如果不是最后一次，从中转瓶转移回分液漏斗
        if i < repeats - 1:
            pump_action_sequence.append(
                {
                    "device_id": "", 
                    "action_name": "PumpTransferProtocol",
                    "action_kwargs": {
                        "from_vessel": to_vessel,
                        "to_vessel": separation_vessel,
                        "volume": 250.0,
                        "time": 250.0 / flowrate,
                        # "transfer_flowrate": transfer_flowrate,
                    }
                }
            )
    return pump_action_sequence
