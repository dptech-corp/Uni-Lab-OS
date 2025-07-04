1. 用到的仪器
                virtual_multiway_valve(√)                                                     八通阀门
                virtual_transfer_pump(√)                                                        转移泵
                virtual_centrifuge()                                                            离心机
                virtual_rotavap()                                                               旋蒸仪
                virtual_heatchill()                                                             加热器
                virtual_stirrer()                                                               搅拌器
                virtual_solenoid_valve()                                                        电磁阀
                virtual_vacuum_pump(√)                       vacuum_pump.mock                            真空泵
                virtual_gas_source(√)                                                                    气源
                virtual_filter()                                                                过滤器
                virtual_column(√)                                                               层析柱
                separator()                         homemade_grbl_conductivity                  分液漏斗
2. 用到的protocol
    PumpTransferProtocol: generate_pump_protocol_with_rinsing,                      (√)
    这个重复了，删掉CleanProtocol: generate_clean_protocol,
    SeparateProtocol: generate_separate_protocol,                           (×)
    EvaporateProtocol: generate_evaporate_protocol,                                 (√)
    EvacuateAndRefillProtocol: generate_evacuateandrefill_protocol,                 (√)
    CentrifugeProtocol: generate_centrifuge_protocol,                               (√)
    AddProtocol: generate_add_protocol,                                             (√)
    FilterProtocol: generate_filter_protocol,                                       (√)
    HeatChillProtocol: generate_heat_chill_protocol,                                (√)
    HeatChillStartProtocol: generate_heat_chill_start_protocol,                     (√)
    HeatChillStopProtocol: generate_heat_chill_stop_protocol,                       (√)
    HeatChillToTempProtocol:
    StirProtocol: generate_stir_protocol,                                           (√)
    StartStirProtocol: generate_start_stir_protocol,                                (√)
    StopStirProtocol: generate_stop_stir_protocol,                                  (√)
    这个重复了，删掉TransferProtocol: generate_transfer_protocol,
    CleanVesselProtocol: generate_clean_vessel_protocol,                            (√)
    DissolveProtocol: generate_dissolve_protocol,                                   (√)
    FilterThroughProtocol: generate_filter_through_protocol,                        (√)
    RunColumnProtocol: generate_run_column_protocol,                                (√)<RunColumn Rf="?" column="column" from_vessel="rotavap" ratio="5:95" solvent1="methanol" solvent2="chloroform" to_vessel="rotavap"/>

上下文体积搜索
3. 还没创建的protocol
    ResetHandling                         写完了                               <ResetHandling solvent="methanol"/>
    Dry                                  写完了                                 <Dry compound="product" vessel="filter"/>
    AdjustPH                               写完了                                <AdjustPH pH="8.0" reagent="hydrochloric acid" vessel="main_reactor"/>
    Recrystallize                          写完了                              <Recrystallize ratio="?" solvent1="dichloromethane" solvent2="methanol" vessel="filter" volume="?"/>
    TakeSample                                                                  <TakeSample id="a" vessel="rotavap"/>
    Hydrogenate                                                                 <Hydrogenate temp="45 °C" time="?" vessel="main_reactor"/>