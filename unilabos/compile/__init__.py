from unilabos.messages import *
from .pump_protocol import generate_pump_protocol, generate_pump_protocol_with_rinsing
from .clean_protocol import generate_clean_protocol
from .separate_protocol import generate_separate_protocol
from .evaporate_protocol import generate_evaporate_protocol
from .evacuateandrefill_protocol import generate_evacuateandrefill_protocol
from .agv_transfer_protocol import generate_agv_transfer_protocol
from .add_protocol import generate_add_protocol
from .centrifuge_protocol import generate_centrifuge_protocol
from .filter_protocol import generate_filter_protocol
from .heatchill_protocol import (
    generate_heat_chill_protocol, 
    generate_heat_chill_start_protocol, 
    generate_heat_chill_stop_protocol,
    generate_heat_chill_to_temp_protocol  # 保留导入，但不注册为协议
)
from .stir_protocol import generate_stir_protocol, generate_start_stir_protocol, generate_stop_stir_protocol
from .transfer_protocol import generate_transfer_protocol
from .clean_vessel_protocol import generate_clean_vessel_protocol
from .dissolve_protocol import generate_dissolve_protocol
from .filter_through_protocol import generate_filter_through_protocol
from .run_column_protocol import generate_run_column_protocol
from .wash_solid_protocol import generate_wash_solid_protocol


# Define a dictionary of protocol generators.
action_protocol_generators = {
    PumpTransferProtocol: generate_pump_protocol_with_rinsing,
    CleanProtocol: generate_clean_protocol,
    SeparateProtocol: generate_separate_protocol,
    EvaporateProtocol: generate_evaporate_protocol,
    EvacuateAndRefillProtocol: generate_evacuateandrefill_protocol,
    AGVTransferProtocol: generate_agv_transfer_protocol,
    CentrifugeProtocol: generate_centrifuge_protocol,
    AddProtocol: generate_add_protocol,
    FilterProtocol: generate_filter_protocol,
    HeatChillProtocol: generate_heat_chill_protocol,
    HeatChillStartProtocol: generate_heat_chill_start_protocol,
    HeatChillStopProtocol: generate_heat_chill_stop_protocol,
    # HeatChillToTempProtocol: generate_heat_chill_to_temp_protocol,  # **移除这行**
    StirProtocol: generate_stir_protocol,
    StartStirProtocol: generate_start_stir_protocol,
    StopStirProtocol: generate_stop_stir_protocol,
    TransferProtocol: generate_transfer_protocol,
    CleanVesselProtocol: generate_clean_vessel_protocol,
    DissolveProtocol: generate_dissolve_protocol,
    FilterThroughProtocol: generate_filter_through_protocol,
    RunColumnProtocol: generate_run_column_protocol,
    WashSolidProtocol: generate_wash_solid_protocol,
}
