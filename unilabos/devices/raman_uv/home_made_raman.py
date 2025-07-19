import json
import serial
import struct
import crcmod
import tkinter as tk
from tkinter import messagebox
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import time

class RamanObj:
    def __init__(self, port_laser, port_ccd, baudrate_laser=9600, baudrate_ccd=921600):
        self.port_laser = port_laser
        self.port_ccd = port_ccd

        self.baudrate_laser = baudrate_laser
        self.baudrate_ccd = baudrate_ccd
        self.success    = False
        self.status     = "None"
        self.ser_laser = serial.Serial( self.port_laser, 
                                        self.baudrate_laser,
                                        bytesize=serial.EIGHTBITS,
                                        parity=serial.PARITY_NONE,
                                        stopbits=serial.STOPBITS_ONE, 
                                        timeout=1)
        
        self.ser_ccd = serial.Serial(   self.port_ccd, 
                                        self.baudrate_ccd,
                                        bytesize=serial.EIGHTBITS,
                                        parity=serial.PARITY_NONE,
                                        stopbits=serial.STOPBITS_ONE, 
                                        timeout=1)
        
    def laser_on_power(self, output_voltage_laser):

        def calculate_crc(data):
            """
            计算Modbus CRC校验码
            """
            crc16 = crcmod.predefined.mkCrcFun('modbus')
            return crc16(data)


        device_id = 0x01  # 从机地址
        register_address = 0x0298  # 寄存器地址
        output_voltage = int(output_voltage_laser)  # 输出值

        # 将输出值转换为需要发送的格式
        output_value = int(output_voltage)  # 确保是整数
        high_byte = (output_value >> 8) & 0xFF
        low_byte = output_value & 0xFF

        # 构造Modbus RTU数据包
        function_code = 0x06  # 写单个寄存器
        message = struct.pack('>BBHH', device_id, function_code, register_address, output_value)
        crc = calculate_crc(message)
        crc_bytes = struct.pack('<H', crc)
        full_message = message + crc_bytes

        try:
            # 打开串口
            self.ser_laser.write(full_message)

            # 读取响应
            response = self.ser_laser.read(8)  # Modbus响应通常为8字节
            print(f"接收到的响应: {response.hex().upper()}")

            # 校验响应CRC
            if len(response) >= 5:
                response_crc = calculate_crc(response[:-2])
                received_crc = struct.unpack('<H', response[-2:])[0]
                if response_crc == received_crc:
                    print("响应CRC校验成功")
                else:
                    print("响应CRC校验失败")
            else:
                print("响应长度不足，无法校验")

        except Exception as e:
            print(f"发生错误: {e}")

        return

    def ccd_time(self, int_time):
        # 发送命令
        def send_command(ser, command):
            try:
                ser.write(command)
                print(f"\u53d1\u9001: {command.hex()}")
            except Exception as e:
                print(f"\u53d1\u9001\u5931\u8d25: {e}")

        # 接收数据
        def receive_data(ser, expected_bytes):
            try:
                data = ser.read(expected_bytes)
                print(f"\u63a5\u6536: {data.hex()}")
                return data
            except Exception as e:
                print(f"\u63a5\u6536\u5931\u8d25: {e}")
                return None

        # 读取CCD数据并绘图
        def read_and_plot(serial_port, int_time):

            # 发送数据清空指令
            clear_command = bytes([0xAA, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x55])
            send_command(serial_port, clear_command)
            time.sleep(0.1)
            clear_command = bytes([0xAA, 0x10, 0x00, 0x00, 0x00, 0x00, 0x10, 0x55])
            send_command(serial_port, clear_command)
            receive_data(serial_port, 4136)
            time.sleep(0.1)
            clear_command = bytes([0xAA, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x55])
            send_command(serial_port, clear_command)
            time.sleep(0.1)

            # 发送数据读取指令
            clear_command = bytes([0xAA, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x55])
            send_command(serial_port, clear_command)
            int_time = int(int_time)
            time.sleep(int_time)
            read_command = bytes([0xAA, 0x10, 0x00, 0x00, 0x00, 0x00, 0x10, 0x55])
            send_command(serial_port, read_command)

            # 接收 4136 字节的数据
            data = receive_data(serial_port, 4136)

            if not data or len(data) != 4136:
                messagebox.showerror("\u9519\u8bef", "\u63a5\u6536\u6570\u636e\u5931\u8d25\u6216\u6570\u636e\u957f\u5ea6\u4e0d\u6b63\u786e")
                return

            # 将接收到的字节数据转换为 16 位无符号整型数组
            values = [struct.unpack(">H", data[i:i + 2])[0] for i in range(0, len(data), 2)]
            return values
        
        try:
            ser = self.ser_ccd
            values = read_and_plot(ser, int_time)  # 修正传递serial对象
            print(f"\u8fd4\u56de\u503c: {values}")
            return values
        except Exception as e:
            messagebox.showerror("\u9519\u8bef", f"\u4e32\u53e3\u521d\u59cb\u5316\u5931\u8d25: {e}")
            return None

    def raman_without_background(self,int_time, laser_power):
        self.laser_on_power(0)
        time.sleep(0.1)
        ccd_data_background = self.ccd_time(int_time)
        time.sleep(0.1)
        self.laser_on_power(laser_power)
        time.sleep(0.2)
        ccd_data_total = self.ccd_time(int_time)
        self.laser_on_power(0)
        ccd_data = [x - y for x, y in zip(ccd_data_total, ccd_data_background)]
        return ccd_data

    def raman_without_background_average(self, sample_name , int_time, laser_power, average):
        ccd_data = [0] * 4136
        for i in range(average):
            ccd_data_temp = self.raman_without_background(int_time, laser_power)
            ccd_data = [x + y for x, y in zip(ccd_data, ccd_data_temp)]
        ccd_data = [x / average for x in ccd_data]
        #保存数据 用时间命名
        t = time.strftime("%Y-%m-%d-%H-%M-%S-%MS", time.localtime())
        folder_path = r"C:\auto\raman_data"
        with open(f'{folder_path}/{sample_name}_{t}.txt', 'w') as f:
            for i in ccd_data:
                f.write(str(i) + '\n')
        return ccd_data

    def raman_cmd(self, command:str):
        self.success = False
        try:
            cmd_dict = json.loads(command)
            ccd_data = self.raman_without_background_average(**cmd_dict)
            self.success = True

            # return ccd_data
        except Exception as e:
            # return None
            raise f"error: {e}"

if __name__ == "__main__":
    raman = RamanObj(port_laser='COM20', port_ccd='COM2')
    ccd_data = raman.raman_without_background_average('test44',0.5,3000,1)

    plt.plot(ccd_data)
    plt.xlabel("Pixel")
    plt.ylabel("Intensity")
    plt.title("Raman Spectrum")
    plt.show()






