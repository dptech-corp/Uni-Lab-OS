from pydantic import BaseModel, Field
import pint


class Point3D(BaseModel):
    x: float = Field(..., title="X coordinate")
    y: float = Field(..., title="Y coordinate")
    z: float = Field(..., title="Z coordinate")

# Start Protocols

class PumpTransferProtocol(BaseModel):
    from_vessel: str
    to_vessel: str
    volume: float
    amount: str = ""
    time: float = 0
    viscous: bool = False
    rinsing_solvent: str = "air"
    rinsing_volume: float = 5000
    rinsing_repeats: int = 2
    solid: bool = False
    flowrate: float = 500
    transfer_flowrate: float = 2500


class CleanProtocol(BaseModel):
    vessel: str
    solvent: str
    volume: float
    temp: float
    repeats: int = 1


class SeparateProtocol(BaseModel):
    purpose: str  # 'wash' or 'extract'. 'wash' means that product phase will not be the added solvent phase, 'extract' means product phase will be the added solvent phase. If no solvent is added just use 'extract'.
    product_phase: str # 'top' or 'bottom'. Phase that product will be in.
    from_vessel: str #Contents of from_vessel are transferred to separation_vessel and separation is performed.
    separation_vessel: str # Vessel in which separation of phases will be carried out.
    to_vessel: str # Vessel to send product phase to.
    waste_phase_to_vessel: str # Optional. Vessel to send waste phase to.
    solvent: str # Optional. Solvent to add to separation vessel after contents of from_vessel has been transferred to create two phases.
    solvent_volume: float # Optional. Volume of solvent to add.
    through: str # Optional. Solid chemical to send product phase through on way to to_vessel, e.g. 'celite'.
    repeats: int # Optional. Number of separations to perform.
    stir_time: float # Optional. Time stir for after adding solvent, before separation of phases.
    stir_speed: float # Optional. Speed to stir at after adding solvent, before separation of phases.
    settling_time: float # Optional. Time


class EvaporateProtocol(BaseModel):
    vessel: str
    pressure: float
    temp: float
    time: float
    stir_speed: float


class EvacuateAndRefillProtocol(BaseModel):
    vessel: str
    gas: str
    repeats: int


class AGVTransferProtocol(BaseModel):
    from_repo: dict
    to_repo: dict
    from_repo_position: str
    to_repo_position: str
#=============新添加的新的协议================
class AddProtocol(BaseModel):
    vessel: str
    reagent: str
    volume: float
    mass: float
    amount: str
    time: float
    stir: bool
    stir_speed: float
    viscous: bool
    purpose: str

class CentrifugeProtocol(BaseModel):
    vessel: str
    speed: float
    time: float
    temp: float  # 移除默认值

class FilterProtocol(BaseModel):
    vessel: str
    filtrate_vessel: str  # 移除默认值
    stir: bool  # 移除默认值
    stir_speed: float  # 移除默认值
    temp: float  # 移除默认值
    continue_heatchill: bool  # 移除默认值
    volume: float  # 移除默认值

class HeatChillProtocol(BaseModel):
    vessel: str
    temp: float
    time: float
    stir: bool
    stir_speed: float
    purpose: str

class HeatChillStartProtocol(BaseModel):
    vessel: str
    temp: float
    purpose: str

class HeatChillStopProtocol(BaseModel):
    vessel: str

class StirProtocol(BaseModel):
    stir_time: float
    stir_speed: float
    settling_time: float

class StartStirProtocol(BaseModel):
    vessel: str
    stir_speed: float
    purpose: str

class StopStirProtocol(BaseModel):
    vessel: str

class TransferProtocol(BaseModel):
    from_vessel: str
    to_vessel: str
    volume: float
    amount: str = ""
    time: float = 0
    viscous: bool = False
    rinsing_solvent: str = ""
    rinsing_volume: float = 0.0
    rinsing_repeats: int = 0
    solid: bool = False

class CleanVesselProtocol(BaseModel):
    vessel: str                 # 要清洗的容器名称
    solvent: str               # 用于清洗容器的溶剂名称
    volume: float              # 清洗溶剂的体积，可选参数
    temp: float                # 清洗时的温度，可选参数
    repeats: int = 1           # 清洗操作的重复次数，默认为 1

class DissolveProtocol(BaseModel):
    vessel: str                 # 装有要溶解物质的容器名称
    solvent: str               # 用于溶解物质的溶剂名称
    volume: float              # 溶剂的体积，可选参数
    amount: str = ""           # 要溶解物质的量，可选参数
    temp: float = 25.0         # 溶解时的温度，可选参数
    time: float = 0.0          # 溶解的时间，可选参数
    stir_speed: float = 0.0    # 搅拌速度，可选参数

class FilterThroughProtocol(BaseModel):
    from_vessel: str              # 源容器的名称，即物质起始所在的容器
    to_vessel: str                # 目标容器的名称，物质过滤后要到达的容器
    filter_through: str           # 过滤时所通过的介质，如滤纸、柱子等
    eluting_solvent: str = ""     # 洗脱溶剂的名称，可选参数
    eluting_volume: float = 0.0   # 洗脱溶剂的体积，可选参数
    eluting_repeats: int = 0      # 洗脱操作的重复次数，默认为 0
    residence_time: float = 0.0   # 物质在过滤介质中的停留时间，可选参数

class RunColumnProtocol(BaseModel):
    from_vessel: str              # 源容器的名称，即样品起始所在的容器
    to_vessel: str                # 目标容器的名称，分离后的样品要到达的容器
    column: str                   # 所使用的柱子的名称

class WashSolidProtocol(BaseModel):
    vessel: str                   # 装有固体物质的容器名称
    solvent: str                  # 用于清洗固体的溶剂名称
    volume: float                 # 清洗溶剂的体积
    filtrate_vessel: str = ""     # 滤液要收集到的容器名称，可选参数
    temp: float = 25.0            # 清洗时的温度，可选参数
    stir: bool = False            # 是否在清洗过程中搅拌，默认为 False
    stir_speed: float = 0.0       # 搅拌速度，可选参数
    time: float = 0.0             # 清洗的时间，可选参数
    repeats: int = 1              # 清洗操作的重复次数，默认为 1

__all__ = ["Point3D", "PumpTransferProtocol", "CleanProtocol", "SeparateProtocol", "EvaporateProtocol", "EvacuateAndRefillProtocol", "AGVTransferProtocol", "CentrifugeProtocol", "AddProtocol", "FilterProtocol", "HeatChillProtocol", "HeatChillStartProtocol", "HeatChillStopProtocol", "StirProtocol", "StartStirProtocol", "StopStirProtocol", "TransferProtocol", "CleanVesselProtocol", "DissolveProtocol", "FilterThroughProtocol", "RunColumnProtocol", "WashSolidProtocol"]
# End Protocols
