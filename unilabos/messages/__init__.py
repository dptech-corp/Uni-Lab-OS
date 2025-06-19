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
    purpose: str
    product_phase: str
    from_vessel: str
    separation_vessel: str
    to_vessel: str
    waste_phase_to_vessel: str
    solvent: str
    solvent_volume: float
    through: str
    repeats: int
    stir_time: float
    stir_speed: float
    settling_time: float


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
    temp: float

class FilterProtocol(BaseModel):
    vessel: str
    filtrate_vessel: str
    stir: bool
    stir_speed: float
    temp: float
    continue_heatchill: bool
    volume: float

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
    vessel: str
    solvent: str
    volume: float
    temp: float
    repeats: int = 1

class DissolveProtocol(BaseModel):
    vessel: str
    solvent: str
    volume: float
    amount: str = ""
    temp: float = 25.0
    time: float = 0.0
    stir_speed: float = 0.0

class FilterThroughProtocol(BaseModel):
    from_vessel: str
    to_vessel: str
    filter_through: str
    eluting_solvent: str = ""
    eluting_volume: float = 0.0
    eluting_repeats: int = 0
    residence_time: float = 0.0

class RunColumnProtocol(BaseModel):
    from_vessel: str
    to_vessel: str
    column: str

class WashSolidProtocol(BaseModel):
    vessel: str
    solvent: str
    volume: float
    filtrate_vessel: str = ""
    temp: float = 25.0
    stir: bool = False
    stir_speed: float = 0.0
    time: float = 0.0
    repeats: int = 1

__all__ = [
    "Point3D", "PumpTransferProtocol", "CleanProtocol", "SeparateProtocol", 
    "EvaporateProtocol", "EvacuateAndRefillProtocol", "AGVTransferProtocol", 
    "CentrifugeProtocol", "AddProtocol", "FilterProtocol", 
    "HeatChillProtocol", "HeatChillStartProtocol", "HeatChillStopProtocol", 
    "StirProtocol", "StartStirProtocol", "StopStirProtocol", 
    "TransferProtocol", "CleanVesselProtocol", "DissolveProtocol", 
    "FilterThroughProtocol", "RunColumnProtocol", "WashSolidProtocol"
]
# End Protocols
