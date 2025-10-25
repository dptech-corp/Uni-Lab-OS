import json
import re
from pymodbus.client import ModbusTcpClient
from unilabos.device_comms.modbus_plc.modbus import WorderOrder, Coil, DiscreteInputs, HoldRegister, InputRegister, DataType
from pymodbus.constants import Endian
import time
import threading
import csv
import os
from datetime import datetime
from typing import Callable
from unilabos.device_comms.modbus_plc.client import TCPClient, ModbusNode, PLCWorkflow, ModbusWorkflow, WorkflowAction, BaseClient
from unilabos.device_comms.modbus_plc.modbus import DeviceType, Base as ModbusNodeBase, DataType, WorderOrder

class Coin_Cell_Assembly:


if __name__ == '__main__':
    coin_cell_assmbly = Coin_Cell_Assembly(address="192.168.1.20", port="502")

    #params = {
    #    "elec_num": 32
    #}
    #str_data = json.dumps(params, ensure_ascii=False)
    #print('param:', coin_cell_assmbly.func_pack_device_write_batch_elec_param(params))
    #time.sleep(1)

    print(coin_cell_assmbly.func_pack_device_write_per_elec_param(
        elec_use_num=4, 
        elec_num=5, 
        elec_vol=55, 
        assembly_type=25, 
        assembly_pressure=550))
    time.sleep(1)


'''
        print('start:', coin_cell_assmbly.func_pack_device_start())
    time.sleep(1)





    

    
    print('start:', coin_cell_assmbly.func_pack_device_start())
    time.sleep(1)
    
    print('stop:', coin_cell_assmbly.func_pack_device_stop())
    time.sleep(1)
    
    while True:
        # cmd coil
        print('start cmd:', coin_cell_assmbly.sys_start_cmd(True))
        time.sleep(1)
        print('stop cmd:', coin_cell_assmbly.sys_stop_cmd(False))
        time.sleep(1)
        print('reset cmd:', coin_cell_assmbly.sys_reset_cmd(True))
        time.sleep(1)
        print('hand cmd:', coin_cell_assmbly.sys_hand_cmd(False))
        time.sleep(1)
        print('auto cmd:', coin_cell_assmbly.sys_auto_cmd(True))
        time.sleep(1)
        print('init cmd:', coin_cell_assmbly.sys_init_cmd(False))
        time.sleep(1)
        print('send msg succ cmd:', coin_cell_assmbly.unilab_send_msg_succ_cmd(False))
        time.sleep(1)
        print('rec msg succ cmd:', coin_cell_assmbly.unilab_rec_msg_succ_cmd(True))
        time.sleep(1)

        # cmd reg
        print('elec use num msg:', coin_cell_assmbly.unilab_send_msg_electrolyte_use_num(8))
        time.sleep(1)
        print('elec num msg:', coin_cell_assmbly.unilab_send_msg_electrolyte_num(4))
        time.sleep(1)
        print('elec vol msg:', coin_cell_assmbly.unilab_send_msg_electrolyte_vol(3.3))
        time.sleep(1)
        print('assembly type msg:', coin_cell_assmbly.unilab_send_msg_assembly_type(1))
        time.sleep(1)   
        print('assembly pressure msg:', coin_cell_assmbly.unilab_send_msg_assembly_pressure(1))
        time.sleep(1)   

        # status coil
        print('start status:',coin_cell_assmbly.sys_start_status)
        time.sleep(1)
        print('stop status:',coin_cell_assmbly.sys_stop_status)
        time.sleep(1)
        print('reset status:',coin_cell_assmbly.sys_reset_status)
        time.sleep(1)
        print('hand status:',coin_cell_assmbly.sys_hand_status)
        time.sleep(1)
        print('auto status:', coin_cell_assmbly.sys_auto_status)
        time.sleep(1)
        print('init ok:', coin_cell_assmbly.sys_init_status)
        time.sleep(1)
        print('request rec msg:', coin_cell_assmbly.request_rec_msg_status)
        time.sleep(1)
        print('request send msg:', coin_cell_assmbly.request_send_msg_status)
        time.sleep(1)

        # status reg
        print('assembly coin cell num:', coin_cell_assmbly.data_assembly_coin_cell_num)
        time.sleep(1)
        print('assembly coin assembly per time:', coin_cell_assmbly.data_assembly_time)
        time.sleep(1)
        print('open circuit vol:', coin_cell_assmbly.data_open_circuit_voltage)
        time.sleep(1)
        print('axis x pos:', coin_cell_assmbly.data_axis_x_pos)
        time.sleep(1)
        print('axis y pos:', coin_cell_assmbly.data_axis_y_pos)
        time.sleep(1)
        print('axis z pos:', coin_cell_assmbly.data_axis_z_pos)
        time.sleep(1)
        print('pole weight:', coin_cell_assmbly.data_pole_weight)
        time.sleep(1)
        print('assembly pressure:', coin_cell_assmbly.data_assembly_coin_cell_num)
        time.sleep(1)  
        print('assembly electrolyte vol:', coin_cell_assmbly.data_electrolyte_volume)
        time.sleep(1)  
        print('assembly coin num:', coin_cell_assmbly.data_coin_num)
        time.sleep(1)   
        print('coin cell code:', coin_cell_assmbly.data_coin_cell_code)
        time.sleep(1)
        print('elec code:', coin_cell_assmbly.data_electrolyte_code)
        time.sleep(1)
        print('glove box pressure:', coin_cell_assmbly.data_glove_box_pressure)
        time.sleep(1)
        print('glove box o2:', coin_cell_assmbly.data_glove_box_o2_content)
        time.sleep(1)
        print('glove box water:', coin_cell_assmbly.data_glove_box_water_content)
        time.sleep(1)

'''