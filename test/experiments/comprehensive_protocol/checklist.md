1. 用到的仪器
                virtual_multiway_valve()                                                        八通阀门
                virtual_transfer_pump()                                                         转移泵
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
                AddProtocol()
                TransferProtocol()                  应该用pump_protocol.py删掉transfer
                StartStirProtocol()
                StopStirProtocol()
                StirProtocol()
                RunColumnProtocol()
                CentrifugeProtocol()
                FilterProtocol()
                CleanVesselProtocol()
                DissolveProtocol()
                FilterThroughProtocol()
                WashSolidProtocol()
                SeparateProtocol()
                EvaporateProtocol()
                HeatChillProtocol()
                HeatChillStartProtocol()
                HeatChillStopProtocol()
                EvacuateAndRefillProtocol()
