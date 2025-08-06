import asyncio
from threading import Lock, Event
from enum import Enum
from dataclasses import dataclass
import time
import traceback
from typing import Any, Union, Optional, overload

import serial.tools.list_ports
from serial import Serial
from serial.serialutil import SerialException


class RunzeSyringePumpMode(Enum):
    Normal = 0
    AccuratePos = 1
    AccuratePosVel = 2


pulse_freq_grades = {
    6000: "0" ,
    5600: "1" ,
    5000: "2" ,
    4400: "3" ,
    3800: "4" ,
    3200: "5" ,
    2600: "6" ,
    2200: "7" ,
    2000: "8" ,
    1800: "9" ,
    1600: "10",
    1400: "11",
    1200: "12",
    1000: "13",
    800 : "14",
    600 : "15",
    400 : "16",
    200 : "17",
    190 : "18",
    180 : "19",
    170 : "20",
    160 : "21",
    150 : "22",
    140 : "23",
    130 : "24",
    120 : "25",
    110 : "26",
    100 : "27",
    90  : "28",
    80  : "29",
    70  : "30",
    60  : "31",
    50  : "32",
    40  : "33",
    30  : "34",
    20  : "35",
    18  : "36",
    16  : "37",
    14  : "38",
    12  : "39",
    10  : "40",
}


class RunzeSyringePumpConnectionError(Exception):
    pass


@dataclass(frozen=True, kw_only=True)
class RunzeSyringePumpInfo:
    port: str
    address: str = "1"
    
    max_volume: float = 25.0
    mode: RunzeSyringePumpMode = RunzeSyringePumpMode.Normal

    def create(self):
        return RunzeSyringePump(self.port, self.address, self.max_volume, self.mode)


class RunzeSyringePump:
    def __init__(self, port: str, address: str = "1", max_volume: float = 25.0, mode: RunzeSyringePumpMode = None):
        self.port = port
        self.address = address
        
        self.max_volume = max_volume
        self.total_steps = self.total_steps_vel = 6000
        
        self._status = "Idle"
        self._mode = mode
        self._max_velocity = 0
        self._valve_position = "I"
        self._position = 0
        
        try:
            # if port in serial_ports and serial_ports[port].is_open:
            #     self.hardware_interface = serial_ports[port]
            # else:
            #     serial_ports[port] = self.hardware_interface = Serial(
            #         baudrate=9600,
            #         port=port
            #     )
            self.hardware_interface = Serial(
                baudrate=9600,
                port=port
            )
            
        except (OSError, SerialException) as e:
            # raise RunzeSyringePumpConnectionError from e
            self.hardware_interface = port

        self._busy = False
        self._closing = False
        self._error_event = Event()
        self._query_lock = Lock()
        self._run_lock = Lock()
    
    def _adjust_total_steps(self):
        self.total_steps = 6000 if self.mode == RunzeSyringePumpMode.Normal else 48000
        self.total_steps_vel = 48000 if self.mode == RunzeSyringePumpMode.AccuratePosVel else 6000
    
    def send_command(self, full_command: str):
        full_command_data = bytearray(full_command, 'ascii')
        response = self.hardware_interface.write(full_command_data)
        time.sleep(0.05)
        output = self._receive(self.hardware_interface.read_until(b"\n"))
        return output

    def _query(self, command: str):
        with self._query_lock:
            if self._closing:
                raise RunzeSyringePumpConnectionError

            run = 'R' if not "?" in command else ''
            full_command = f"/{self.address}{command}{run}\r\n"
            
            output = self.send_command(full_command)[3:-3]
        return output

    def _parse(self, data: bytes, dtype: Optional[type] = None):
        response = data.decode()

        if dtype == bool:
            return response == "1"
        elif dtype == int:
            return int(response)
        else:
            return response

    def _receive(self, data: bytes):
        ascii_string = "".join(chr(byte) for byte in data)
        was_busy = self._busy
        self._busy = ((data[0] & (1 << 5)) < 1) or ascii_string.startswith("@")
        return ascii_string

    def _run(self, command: str):
        with self._run_lock:
            try:
                response = self._query(command)
                while True:
                    time.sleep(0.5)  # Wait for 0.5 seconds before polling again

                    status = self.get_status()
                    if status == 'Idle':
                        break
            finally:
                pass
        return response

    def initialize(self):
        print("Initializing Runze Syringe Pump")
        response = self._run("Z")
        # if self.mode:
        #     self.set_mode(self.mode)
        # else:
        #     # self.mode = RunzeSyringePumpMode.Normal
        #     # self.set_mode(self.mode)
        #     self.mode = self.get_mode()
        return response
    
    # Settings

    def set_baudrate(self, baudrate):
        if baudrate == 9600:
            return self._run("U41")
        elif baudrate == 38400:
            return self._run("U47")
        else:
            raise ValueError("Unsupported baudrate")
    
    # Device Status
    @property
    def status(self) -> str:
        return self._status
    
    def _standardize_status(self, status_raw):
        return "Idle" if status_raw == "`" else "Busy"
    
    def get_status(self):
        status_raw = self._query("Q")
        self._status = self._standardize_status(status_raw)
        return self._status
    
    # Mode Settings and Queries
    
    @property
    def mode(self) -> int:
        return self._mode
    
    # def set_mode(self, mode: RunzeSyringePumpMode):
    #     self.mode = mode
    #     self._adjust_total_steps()
    #     command = f"N{mode.value}"
    #     return self._run(command)
    
    # def get_mode(self):
    #     response = self._query("?28")
    #     status_raw, mode = response[0], int(response[1])
    #     self.mode = RunzeSyringePumpMode._value2member_map_[mode]
    #     self._adjust_total_steps()
    #     return self.mode

    # Speed Settings and Queries
    
    @property
    def max_velocity(self) -> float:
        return self._max_velocity
    
    def set_max_velocity(self, velocity: float):
        self._max_velocity = velocity
        pulse_freq = int(velocity / self.max_volume * self.total_steps_vel)
        pulse_freq = min(6000, pulse_freq)
        return self._run(f"V{pulse_freq}")

    def get_max_velocity(self):
        response = self._query("?2")
        status_raw, pulse_freq = response[0], int(response[1:])
        self._status = self._standardize_status(status_raw)
        self._max_velocity = pulse_freq / self.total_steps_vel * self.max_volume
        return self._max_velocity
    
    def set_velocity_grade(self, velocity: Union[int, str]):
        return self._run(f"S{velocity}")
    
    def get_velocity_grade(self):
        response = self._query("?2")
        status_raw, pulse_freq = response[0], int(response[1:])
        g = "-1"
        for freq, grade in pulse_freq_grades.items():
            if pulse_freq >= freq:
                g = grade
                break
        return g

    def get_velocity_init(self):
        response = self._query("?1")
        status_raw, pulse_freq = response[0], int(response[1:])
        self._status = self._standardize_status(status_raw)
        velocity = pulse_freq / self.total_steps_vel * self.max_volume
        return pulse_freq, velocity

    def get_velocity_end(self):
        response = self._query("?3")
        status_raw, pulse_freq = response[0], int(response[1:])
        self._status = self._standardize_status(status_raw)
        velocity = pulse_freq / self.total_steps_vel * self.max_volume
        return pulse_freq, velocity
    
    # Operations
    
    # Valve Setpoint and Queries

    @property
    def valve_position(self) -> str:
        return self._valve_position
    
    def set_valve_position(self, position: Union[int, str, float]):
        if type(position) == float:
            position = round(position / 120)
        command = f"I{position}" if type(position) == int or ord(position) <= 57 else position.upper()
        response = self._run(command)
        self._valve_position = f"{position}" if type(position) == int or ord(position) <= 57 else position.upper()
        return response

    def get_valve_position(self) -> str:
        response = self._query("?6")
        status_raw, pos_valve = response[0], response[1].upper()
        self._valve_position = pos_valve
        self._status = self._standardize_status(status_raw)
        return pos_valve
    
    # Plunger Setpoint and Queries
    
    @property
    def position(self) -> float:
        return self._position

    def get_position(self):
        response = self._query("?0")
        status_raw, pos_step = response[0], int(response[1:])
        self._status = self._standardize_status(status_raw)
        return pos_step / self.total_steps * self.max_volume

    def set_position(self, position: float, max_velocity: float = None):
        """
        Move to absolute volume (unit: ml)

        Args:
            position (float): absolute position of the plunger, unit: ml
            max_velocity (float): maximum velocity of the plunger, unit: ml/s

        Returns:
            None
        """
        if max_velocity is not None:
            self.set_max_velocity(max_velocity)
            pulse_freq = int(max_velocity / self.max_volume * self.total_steps_vel)
            pulse_freq = min(6000, pulse_freq)
            velocity_cmd = f"V{pulse_freq}"
        else:
            velocity_cmd = ""
        pos_step = int(position / self.max_volume * self.total_steps)
        return self._run(f"{velocity_cmd}A{pos_step}")
    
    def pull_plunger(self, volume: float):
        """
        Pull a fixed volume (unit: ml)

        Args:
            volume (float): absolute position of the plunger, unit: mL

        Returns:
            None
        """
        pos_step = int(volume / self.max_volume * self.total_steps)
        return self._run(f"P{pos_step}")
    
    def push_plunger(self, volume: float):
        """
        Push a fixed volume (unit: ml)

        Args:
            volume (float): absolute position of the plunger, unit: mL

        Returns:
            None
        """
        pos_step = int(volume / self.max_volume * self.total_steps)
        return self._run(f"D{pos_step}")

    def get_plunger_position(self):
        response = self._query("?4")
        status, pos_step = response[0], int(response[1:])
        return pos_step / self.total_steps * self.max_volume

    def stop_operation(self):
        return self._run("T")
    
    # Queries

    def query_command_buffer_status(self):
        return self._query("?10")

    def query_backlash_position(self):
        return self._query("?12")

    def query_aux_input_status_1(self):
        return self._query("?13")

    def query_aux_input_status_2(self):
        return self._query("?14")

    def query_software_version(self):
        return self._query("?23")

    def wait_error(self):
        self._error_event.wait()

    def close(self):
        if self._closing:
            raise RunzeSyringePumpConnectionError

        self._closing = True
        self.hardware_interface.close()

    @staticmethod
    def list():
        for item in serial.tools.list_ports.comports():
            yield RunzeSyringePumpInfo(port=item.device)


if __name__ == "__main__":
    r = RunzeSyringePump("/dev/tty.usbserial-D30JUGG5", "1", 25.0)
    r.initialize()