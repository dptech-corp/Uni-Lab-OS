import sys
import threading
import serial
import serial.tools.list_ports
import re
import time
from typing import Optional, List, Dict, Tuple

class ChinweDevice:
    """
    ChinWe设备控制类
    提供串口通信、电机控制、传感器数据读取等功能
    """
    
    def __init__(self, port: str, baudrate: int = 115200, debug: bool = False):
        """
        初始化ChinWe设备
        
        Args:
            port: 串口名称，如果为None则自动检测
            baudrate: 波特率，默认115200
        """
        self.debug = debug
        self.port = port
        self.baudrate = baudrate
        self.serial_port: Optional[serial.Serial] = None
        self._voltage: float = 0.0
        self._ec_value: float = 0.0
        self._ec_adc_value: int = 0
        self._is_connected = False
        self.connect()
    
    @property
    def is_connected(self) -> bool:
        """获取连接状态"""
        return self._is_connected and self.serial_port and self.serial_port.is_open
    
    @property
    def voltage(self) -> float:
        """获取电源电压值"""
        return self._voltage
    
    @property
    def ec_value(self) -> float:
        """获取电导率值 (ms/cm)"""
        return self._ec_value

    @property
    def ec_adc_value(self) -> int:
        """获取EC ADC原始值"""
        return self._ec_adc_value
    

    @property
    def device_status(self) -> Dict[str, any]:
        """
        获取设备状态信息
        
        Returns:
            包含设备状态的字典
        """
        return {
            "connected": self.is_connected,
            "port": self.port,
            "baudrate": self.baudrate,
            "voltage": self.voltage,
            "ec_value": self.ec_value,
            "ec_adc_value": self.ec_adc_value
        }
    
    def connect(self, port: Optional[str] = None, baudrate: Optional[int] = None) -> bool:
        """
        连接到串口设备
        
        Args:
            port: 串口名称，如果为None则使用初始化时的port或自动检测
            baudrate: 波特率，如果为None则使用初始化时的baudrate
            
        Returns:
            连接是否成功
        """
        if self.is_connected:
            return True
            
        target_port = port or self.port
        target_baudrate = baudrate or self.baudrate
        
        try:
            self.serial_port = serial.Serial(target_port, target_baudrate, timeout=0.5)
            self._is_connected = True
            self.port = target_port
            self.baudrate = target_baudrate
            connect_allow_times = 5
            while not self.serial_port.is_open and connect_allow_times > 0:
                time.sleep(0.5)
                connect_allow_times -= 1
                print(f"尝试连接到 {target_port} @ {target_baudrate}，剩余尝试次数: {connect_allow_times}", self.debug)
                raise ValueError("串口未打开，请检查设备连接")
            print(f"已连接到 {target_port} @ {target_baudrate}", self.debug)
            threading.Thread(target=self._read_data, daemon=True).start()
            return True
        except Exception as e:
            print(f"ChinweDevice连接失败: {e}")
            self._is_connected = False
            return False
    
    def disconnect(self) -> bool:
        """
        断开串口连接
        
        Returns:
            断开是否成功
        """
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
                self._is_connected = False
                print("已断开串口连接")
                return True
            except Exception as e:
                print(f"断开连接失败: {e}")
                return False
        return True
    
    def _send_motor_command(self, command: str) -> bool:
        """
        发送电机控制命令
        
        Args:
            command: 电机命令字符串，例如 "M 1 CW 1.5"
            
        Returns:
            发送是否成功
        """
        if not self.is_connected:
            print("设备未连接")
            return False
            
        try:
            self.serial_port.write((command + "\n").encode('utf-8'))
            print(f"发送命令: {command}")
            return True
        except Exception as e:
            print(f"发送命令失败: {e}")
            return False
    
    def rotate_motor(self, motor_id: int, turns: float, clockwise: bool = True) -> bool:
        """
        使电机转动指定圈数
        
        Args:
            motor_id: 电机ID（1, 2, 3...）
            turns: 转动圈数，支持小数
            clockwise: True为顺时针，False为逆时针
            
        Returns:
            命令发送是否成功
        """
        if clockwise:
            command = f"M {motor_id} CW {turns}"
        else:
            command = f"M {motor_id} CCW {turns}"
        return self._send_motor_command(command)

    def set_motor_speed(self, motor_id: int, speed: float) -> bool:
        """
        设置电机转速（如果设备支持）
        
        Args:
            motor_id: 电机ID（1, 2, 3...）
            speed: 转速值
            
        Returns:
            命令发送是否成功
        """
        command = f"M {motor_id} SPEED {speed}"
        return self._send_motor_command(command)

    def _read_data(self) -> List[str]:
        """
        读取串口数据并解析
        
        Returns:
            读取到的数据行列表
        """
        print("开始读取串口数据...")
        if not self.is_connected:
            return []
            
        data_lines = []
        try:
            while self.serial_port.in_waiting:
                time.sleep(0.1)  # 等待数据稳定
                try:
                    line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        data_lines.append(line)
                        self._parse_sensor_data(line)
                except Exception as ex:
                    print(f"解码数据错误: {ex}")
        except Exception as e:
            print(f"读取串口数据错误: {e}")
            
        return data_lines
    
    def _parse_sensor_data(self, line: str) -> None:
        """
        解析传感器数据
        
        Args:
            line: 接收到的数据行
        """
        # 解析电源电压
        if "电源电压" in line:
            try:
                val = float(line.split("：")[1].replace("V", "").strip())
                self._voltage = val
                if self.debug:
                    print(f"电源电压更新: {val}V")
            except Exception:
                pass

        # 解析电导率和ADC原始值（支持两种格式）
        if "电导率" in line and "ADC原始值" in line:
            try:
                # 支持格式如：电导率：2.50ms/cm, ADC原始值：2052
                ec_match = re.search(r"电导率[:：]\s*([\d\.]+)", line)
                adc_match = re.search(r"ADC原始值[:：]\s*(\d+)", line)
                if ec_match:
                    ec_val = float(ec_match.group(1))
                    self._ec_value = ec_val
                    if self.debug:
                        print(f"电导率更新: {ec_val:.2f} ms/cm")
                if adc_match:
                    adc_val = int(adc_match.group(1))
                    self._ec_adc_value = adc_val
                    if self.debug:
                        print(f"EC ADC原始值更新: {adc_val}")
            except Exception:
                pass
        # 仅电导率，无ADC原始值
        elif "电导率" in line:
            try:
                val = float(line.split("：")[1].replace("ms/cm", "").strip())
                self._ec_value = val
                if self.debug:
                    print(f"电导率更新: {val:.2f} ms/cm")
            except Exception:
                pass
        # 仅ADC原始值（如有分开回传场景）
        elif "ADC原始值" in line:
            try:
                adc_val = int(line.split("：")[1].strip())
                self._ec_adc_value = adc_val
                if self.debug:
                    print(f"EC ADC原始值更新: {adc_val}")
            except Exception:
                pass
    
    def spin_when_ec_ge_0():
        pass
    

def main():
    """测试函数"""
    print("=== ChinWe设备测试 ===")
    
    # 创建设备实例
    device = ChinweDevice("/dev/tty.usbserial-A5069RR4", debug=True)
    try:
        # 测试5: 发送电机命令
        print("\n5. 发送电机命令测试:")
        print("  5.3 使用通用函数控制电机20顺时针转2圈:")
        device.rotate_motor(2, 20.0, clockwise=True)
        time.sleep(0.5)
    finally:
        time.sleep(10)
        # 测试7: 断开连接
        print("\n7. 断开连接:")
        device.disconnect()
if __name__ == "__main__":
    main()
