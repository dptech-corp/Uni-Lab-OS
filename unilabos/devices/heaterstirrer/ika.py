import time
import asyncio
from contextlib import contextmanager

import serial


class IkaNamurClient:
  """IKA NAMUR 串口客户端（RS-232 9600 7E1，CRLF）。

  提供基础的 send() 以及常用指令的便捷方法。
  """

  def __init__(self, port: str = "COM7", baud: int = 9600, timeout: float = 1.0):
    self.port = port
    self.baud = baud
    self.timeout = timeout
    self._ser: serial.Serial | None = None

  def open(self) -> None:
    if self._ser and self._ser.is_open:
      return
    self._ser = serial.Serial(
      self.port,
      self.baud,
      bytesize=serial.SEVENBITS,
      parity=serial.PARITY_EVEN,
      stopbits=serial.STOPBITS_ONE,
      timeout=self.timeout,
      write_timeout=2,
    )
    time.sleep(0.1)

  def close(self) -> None:
    if self._ser and self._ser.is_open:
      self._ser.close()

  def send(self, *tokens: str) -> str:
    if not self._ser or not self._ser.is_open:
      self.open()
    line = " ".join(tokens).upper()
    payload = (line + "\r\n").encode("ascii")
    self._ser.reset_input_buffer()
    self._ser.write(payload)
    time.sleep(0.25)
    buf = bytearray()
    end = time.time() + self.timeout
    while time.time() < end or self._ser.in_waiting:
      data = self._ser.read(self._ser.in_waiting or 1)
      if data:
        buf.extend(data)
      else:
        time.sleep(0.02)
    return buf.decode("ascii", errors="ignore").strip()

  # 便捷方法
  def read_name(self) -> str:
    return self.send("IN_NAME")

  def read_speed(self) -> str:
    return self.send("IN_PV_4")

  def read_speed_setpoint(self) -> str:
    return self.send("IN_SP_4")

  def set_speed(self, rpm: int) -> str:
    return self.send("OUT_SP_4", str(rpm))

  def start(self) -> str:
    return self.send("START_4")

  def stop(self) -> str:
    return self.send("STOP_4")


@contextmanager
def ika_client(port: str = "COM7", baud: int = 9600, timeout: float = 1.0):
  cli = IkaNamurClient(port, baud, timeout)
  try:
    cli.open()
    yield cli
  finally:
    cli.close()


class HeaterStirrer_IKA:
  """IKA 加热搅拌器（NAMUR 协议）统一接口，供 unilabos 调用。"""

  def __init__(self, port: str = "COM7", baudrate: int = 9600, timeout: float = 1.0):
    self._status = "Idle"
    self._stir_speed = 0.0
    self._temp_target = 20.0
    self._cli = IkaNamurClient(port=port, baud=baudrate, timeout=timeout)
    self._cli.open()

  @property
  def status(self) -> str:
    self._status = "Idle" if self._stir_speed == 0 else "Running"
    return self._status

  @property
  def stir_speed(self) -> float:
    return self._stir_speed

  def set_stir_speed(self, speed: float):
    speed_int = int(float(speed))
    self._cli.set_speed(speed_int)
    if speed_int > 0:
      self._cli.start()
    else:
      self._cli.stop()
    self._stir_speed = float(speed_int)

  @property
  def temp_target(self) -> float:
    return self._temp_target

  def set_temp_target(self, temp: float):
    self._temp_target = float(temp)
    self._cli.send("OUT_SP_1", f"{int(self._temp_target)}")
    self._cli.send("START_1")

  @property
  def temp(self) -> float:
    # 具体型号若支持查询实际温度，可在此扩展 NAMUR 读指令
    return self._temp_target

  # 兼容 stir_protocol.py 的动作接口
  def _extract_vessel_id(self, vessel) -> str:
    if isinstance(vessel, dict):
      return str(vessel.get("id", ""))
    return str(vessel)

  async def start_stir(self, vessel, stir_speed: float, purpose: str = "") -> bool:
    """开始持续搅拌（协议动作）

    - vessel: 可为字符串或形如 {"id": "..."} 的字典
    - stir_speed: 目标转速 RPM
    - purpose: 可选用途描述（仅用于上层日志）
    """
    _ = self._extract_vessel_id(vessel)  # 当前实现不强依赖容器，仅做形参兼容
    try:
      speed_int = int(float(stir_speed))
    except (ValueError, TypeError):
      speed_int = 0
    # 同步串口调用放入线程，避免阻塞事件循环
    await asyncio.to_thread(self.set_stir_speed, speed_int)
    return True

  async def stop_stir(self, vessel) -> bool:
    """停止搅拌（协议动作）"""
    _ = self._extract_vessel_id(vessel)
    await asyncio.to_thread(self.set_stir_speed, 0)
    return True

  async def stir(self, stir_time: float, stir_speed: float, settling_time: float, **kwargs) -> bool:
    """定时搅拌 + 沉降（协议动作）

    - stir_time: 搅拌时间（秒）
    - stir_speed: 搅拌速度（RPM）
    - settling_time: 沉降时间（秒）
    其余 kwargs（如 vessel/time/time_spec/event）按协议形态传入，此处可忽略。
    """
    try:
      total_stir_seconds = max(0.0, float(stir_time))
    except (ValueError, TypeError):
      total_stir_seconds = 0.0
    try:
      speed_int = int(float(stir_speed))
    except (ValueError, TypeError):
      speed_int = 0
    try:
      total_settle_seconds = max(0.0, float(settling_time))
    except (ValueError, TypeError):
      total_settle_seconds = 0.0

    # 开始搅拌
    await asyncio.to_thread(self.set_stir_speed, speed_int)
    if total_stir_seconds > 0:
      await asyncio.sleep(total_stir_seconds)

    # 停止搅拌进入沉降
    await asyncio.to_thread(self.set_stir_speed, 0)
    if total_settle_seconds > 0:
      await asyncio.sleep(total_settle_seconds)

    return True

  def close(self):
    self._cli.close()