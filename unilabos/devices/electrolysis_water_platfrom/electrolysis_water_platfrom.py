# -*- coding: utf-8 -*-
import serial
import time
import csv
import threading
import os
from collections import deque
from typing import Dict, Any, Optional
from pylabrobot.resources import Deck

from unilabos.devices.workstation.workstation_base import WorkstationBase


class ElectrolysisWaterPlatform(WorkstationBase):
    """
    电解水平台工作站
    基于 WorkstationBase 的电解水实验平台，支持串口通信和数据采集
    """
    
    def __init__(
        self, 
        deck: Deck,
        port: str = "COM10",
        baudrate: int = 115200,
        csv_path: Optional[str] = None,
        timeout: float = 0.2,
        **kwargs
    ):
        super().__init__(deck, **kwargs)
        
        # ========== 配置 ==========
        self.port = port
        self.baudrate = baudrate
        # 如果没有指定路径，默认保存在代码文件所在目录
        if csv_path is None:
            current_dir = os.path.dirname(os.path.abspath(__file__))
            self.csv_path = os.path.join(current_dir, "stm32_data.csv")
        else:
            self.csv_path = csv_path
        self.ser_timeout = timeout
        self.chunk_read = 128
        
        # 串口对象
        self.ser: Optional[serial.Serial] = None
        self.stop_flag = False
        
        # 线程对象
        self.rx_thread: Optional[threading.Thread] = None
        self.tx_thread: Optional[threading.Thread] = None
        
        # ==== 接收（下位机->上位机）：固定 1+13+1 = 15 字节 ====
        self.RX_HEAD = 0x3E
        self.RX_TAIL = 0x3E
        self.RX_FRAME_LEN = 1 + 13 + 1  # 15
        
        # ==== 发送（上位机->下位机）：固定 1+9+1 = 11 字节 ====
        self.TX_HEAD = 0x3E
        self.TX_TAIL = 0xE3  # 协议图中标注 E3 作为帧尾
        self.TX_FRAME_LEN = 1 + 9 + 1  # 11
    
    def open_serial(self, port: Optional[str] = None, baudrate: Optional[int] = None, timeout: Optional[float] = None) -> Optional[serial.Serial]:
        """打开串口"""
        port = port or self.port
        baudrate = baudrate or self.baudrate
        timeout = timeout or self.ser_timeout
        try:
            ser = serial.Serial(port, baudrate, timeout=timeout)
            print(f"[OK] 串口 {port} 已打开，波特率 {baudrate}")
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            self.ser = ser
            return ser
        except serial.SerialException as e:
            print(f"[ERR] 无法打开串口 {port}: {e}")
            return None

    def close_serial(self):
        """关闭串口"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("[INFO] 串口已关闭")

    @staticmethod
    def u16_be(h: int, l: int) -> int:
        """将两个字节组合成16位无符号整数（大端序）"""
        return ((h & 0xFF) << 8) | (l & 0xFF)

    @staticmethod
    def split_u16_be(val: int) -> tuple:
        """返回 (高字节, 低字节)，输入会夹到 0..65535"""
        v = int(max(0, min(65535, int(val))))
        return (v >> 8) & 0xFF, v & 0xFF

    # ================== 接收：固定15字节 ==================
    def parse_rx_payload(self, dat13: bytes) -> Optional[Dict[str, Any]]:
        """解析 13 字节数据区（下位机发送到上位机）"""
        if len(dat13) != 13:
            return None
        current_mA      = self.u16_be(dat13[0], dat13[1])
        voltage_mV      = self.u16_be(dat13[2], dat13[3])
        temperature_raw = self.u16_be(dat13[4], dat13[5])
        tds_ppm         = self.u16_be(dat13[6], dat13[7])
        gas_sccm        = self.u16_be(dat13[8], dat13[9])
        liquid_mL       = self.u16_be(dat13[10], dat13[11])
        ph_raw          = dat13[12] & 0xFF

        return {
            "Current_mA": current_mA,
            "Voltage_mV": voltage_mV,
            "Temperature_C": round(temperature_raw / 100.0, 2),
            "TDS_ppm": tds_ppm,
            "GasFlow_sccm": gas_sccm,
            "LiquidFlow_mL": liquid_mL,
            "pH": round(ph_raw / 10.0, 2)
        }

    def try_parse_rx_frame(self, frame15: bytes) -> Optional[Dict[str, Any]]:
        """尝试解析接收帧"""
        if len(frame15) != self.RX_FRAME_LEN:
            return None
        if frame15[0] != self.RX_HEAD or frame15[-1] != self.RX_TAIL:
            return None
        return self.parse_rx_payload(frame15[1:-1])

    def rx_thread_fn(self):
        """接收线程函数"""
        headers = ["Timestamp", "Current_mA", "Voltage_mV",
                   "Temperature_C", "TDS_ppm", "GasFlow_sccm", "LiquidFlow_mL", "pH"]

        new_file = not os.path.exists(self.csv_path)
        f = open(self.csv_path, mode='a', newline='', encoding='utf-8')
        writer = csv.writer(f)
        if new_file:
            writer.writerow(headers)
            f.flush()

        buf = deque(maxlen=8192)
        print(f"[RX] 开始接收（帧长 {self.RX_FRAME_LEN} 字节）；写入：{self.csv_path}")

        try:
            while not self.stop_flag and self.ser and self.ser.is_open:
                chunk = self.ser.read(self.chunk_read)
                if chunk:
                    buf.extend(chunk)
                    while True:
                        # 找帧头
                        try:
                            start = next(i for i, b in enumerate(buf) if b == self.RX_HEAD)
                        except StopIteration:
                            buf.clear()
                            break
                        if start > 0:
                            for _ in range(start):
                                buf.popleft()
                        if len(buf) < self.RX_FRAME_LEN:
                            break
                        candidate = bytes([buf[i] for i in range(self.RX_FRAME_LEN)])
                        if candidate[-1] == self.RX_TAIL:
                            parsed = self.try_parse_rx_frame(candidate)
                            for _ in range(self.RX_FRAME_LEN):
                                buf.popleft()
                            if parsed:
                                ts = time.strftime("%Y-%m-%d %H:%M:%S")
                                row = [ts,
                                       parsed["Current_mA"], parsed["Voltage_mV"],
                                       parsed["Temperature_C"], parsed["TDS_ppm"],
                                       parsed["GasFlow_sccm"], parsed["LiquidFlow_mL"],
                                       parsed["pH"]]
                                writer.writerow(row)
                                f.flush()
                                # 若不想打印可注释下一行
                                # print(f"[{ts}] I={parsed['Current_mA']} mA, V={parsed['Voltage_mV']} mV, "
                                #       f"T={parsed['Temperature_C']} °C, TDS={parsed['TDS_ppm']}, "
                                #       f"Gas={parsed['GasFlow_sccm']} sccm, Liq={parsed['LiquidFlow_mL']} mL, pH={parsed['pH']}")
                        else:
                            # 头不变，尾不对，丢1字节继续对齐
                            buf.popleft()
                else:
                    time.sleep(0.01)
        finally:
            f.close()
            print("[RX] 接收线程退出，CSV 已关闭")

    # ================== 发送：固定11字节 ==================
    def build_tx_frame(self, mode: int, current_ma: int, voltage_mv: int, temp_c: float, ki: float, pump_percent: float) -> bytes:
        """
        发送帧：HEAD + [mode, I_hi, I_lo, V_hi, V_lo, T_hi, T_lo, Ki_byte, Pump_byte] + TAIL
        - mode: 0=恒压, 1=恒流
        - current_ma: mA (0..65535)
        - voltage_mv: mV (0..65535)
        - temp_c: ℃，将 *100 后拆分为高/低字节
        - ki: 0.0..20.0  -> byte = round(ki * 10) 夹到 0..200
        - pump_percent: 0..100 -> byte = round(pump * 2) 夹到 0..200
        """
        mode_b = 1 if int(mode) == 1 else 0

        i_hi, i_lo = self.split_u16_be(current_ma)
        v_hi, v_lo = self.split_u16_be(voltage_mv)

        t100 = int(round(float(temp_c) * 100.0))
        t_hi, t_lo = self.split_u16_be(t100)

        ki_b = int(max(0, min(200, round(float(ki) * 10))))
        pump_b = int(max(0, min(200, round(float(pump_percent) * 2))))

        return bytes((
            self.TX_HEAD,
            mode_b,
            i_hi, i_lo,
            v_hi, v_lo,
            t_hi, t_lo,
            ki_b,
            pump_b,
            self.TX_TAIL
        ))

    def tx_thread_fn(self):
        """
        发送线程函数
        用户输入 6 个用逗号分隔的数值：
        mode,current_mA,voltage_mV,set_temp_C,Ki,pump_percent
        例如： 0,1000,500,0,0,50
        """
        print("\n输入 6 个值（用英文逗号分隔），顺序为：")
        print("mode,current_mA,voltage_mV,set_temp_C,Ki,pump_percent")
        print("示例恒压：0,500,1000,25,0,100   （stop 结束）\n")
        print("示例恒流：1,1000,500,25,0,100   （stop 结束）\n")
        print("示例恒流：1,2000,500,25,0,100   （stop 结束）\n")
        # 1,2000,500,25,0,100

        while not self.stop_flag and self.ser and self.ser.is_open:
            try:
                line = input(">>> ").strip()
            except EOFError:
                self.stop_flag = True
                break

            if not line:
                continue
            if line.lower() == "stop":
                self.stop_flag = True
                print("[SYS] 停止程序")
                break

            try:
                parts = [p.strip() for p in line.split(",")]
                if len(parts) != 6:
                    raise ValueError("需要 6 个逗号分隔的数值")
                mode = int(parts[0])
                i_ma = int(float(parts[1]))
                v_mv = int(float(parts[2]))
                t_c  = float(parts[3])
                ki   = float(parts[4])
                pump = float(parts[5])

                frame = self.build_tx_frame(mode, i_ma, v_mv, t_c, ki, pump)
                self.ser.write(frame)
                print("[TX]", " ".join(f"{b:02X}" for b in frame))
            except Exception as e:
                print("[TX] 输入/打包失败：", e)
                print("格式：mode,current_mA,voltage_mV,set_temp_C,Ki,pump_percent")
                continue
    
    def start(self):
        """启动电解水平台"""
        self.ser = self.open_serial()
        if self.ser:
            try:
                self.rx_thread = threading.Thread(target=self.rx_thread_fn, daemon=True)
                self.tx_thread = threading.Thread(target=self.tx_thread_fn, daemon=True)
                self.rx_thread.start()
                self.tx_thread.start()
                print("[INFO] 电解水平台已启动")
                self.tx_thread.join()  # 等待用户输入线程结束（输入 stop）
            finally:
                self.close_serial()
    
    def stop(self):
        """停止电解水平台"""
        self.stop_flag = True
        if self.rx_thread and self.rx_thread.is_alive():
            self.rx_thread.join(timeout=2.0)
        if self.tx_thread and self.tx_thread.is_alive():
            self.tx_thread.join(timeout=2.0)
        self.close_serial()
        print("[INFO] 电解水平台已停止")


# ================== 主入口 ==================
if __name__ == "__main__":
    # 创建一个简单的 Deck 用于测试
    from pylabrobot.resources import Deck
    
    deck = Deck()
    platform = ElectrolysisWaterPlatform(deck)
    platform.start()
