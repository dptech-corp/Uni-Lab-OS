# station_simple.py
import logging
import json
import os
from typing import Optional

from .drivers.SharedRS485Bus import SharedRS485Bus
from .XYZStepperController import XYZStepperController
from .drivers.SOPAPipetteYYQ import SOPAPipetteYYQ

logger = logging.getLogger("liquid_station.simple_station")


class Station:
    """
    简化版 Station：
    - 共享 RS485 总线
    - XYZ 三轴控制器 (XYZStepperController)
    - SOPA 移液枪 (SOPAPipetteYYQ)
    """

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 115200,
        points_file: str = "unilabos/devices/laiyu_xyz_pipette/points.json",   # 新增：点位文件路径
    ):
        self.port = port
        self.baudrate = baudrate

        self.bus: SharedRS485Bus = SharedRS485Bus(port=self.port, baudrate=self.baudrate)
        self.xyz: Optional[XYZStepperController] = None
        self.pip: Optional[SOPAPipetteYYQ] = None

        self.points_file = points_file
        self.points: dict[str, dict[str, float]] = {}
        self._load_points()  # 新增：启动时加载点位

    # ===== 点位加载 =====
    def _load_points(self):
        """从 JSON 文件加载预定义点位"""
        if not os.path.exists(self.points_file):
            logger.warning("点位文件不存在: %s", self.points_file)
            self.points = {}
            return

        try:
            with open(self.points_file, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception as e:
            logger.error("加载点位文件失败 %s: %s", self.points_file, e)
            self.points = {}
            return

        # 简单校验一下结构
        ok_points: dict[str, dict[str, float]] = {}
        for name, p in data.items():
            try:
                x = float(p["x"])
                y = float(p["y"])
                z = float(p["z"])
                ok_points[name] = {"x": x, "y": y, "z": z}
            except Exception:
                logger.warning("点位 %s 格式不正确，已忽略: %s", name, p)

        self.points = ok_points
        logger.info("已加载点位 %d 个", len(self.points))

    # ===== 连接与断开 =====
    def connect(self):
        logger.info("Connecting station on %s ...", self.port)
        self.bus.open()
        self.xyz = XYZStepperController(self.bus)
        self.pip = SOPAPipetteYYQ(self.bus)
        logger.info("Station connected.")

    def disconnect(self):
        self.bus.close()

    def _ensure_connected(self):
        """如果尚未连接设备，则自动连接"""
        if self.xyz is None or self.pip is None:
            self.connect()

    # ===== 对外简单 API =====
    def move_xyz(
        self,
        x: float | None,
        y: float | None,
        z: float | None,
        speed: int = 500,
        acc: int = 6000,
    ):
        self._ensure_connected()

        # 把可能是字符串的 x/y/z 转成 float 或 None
        def _to_float_or_none(v):
            if v is None:
                return None
            # 允许传入 "0"、"100.5" 这类字符串
            return float(v)

        x_f = _to_float_or_none(x)
        y_f = _to_float_or_none(y)
        z_f = _to_float_or_none(z)

        self.xyz.move_xyz_work(x_f, y_f, z_f, speed, acc)

    def aspirate(self, volume_uL: float):
        self._ensure_connected()
        if not self.pip:
            raise RuntimeError("Pipette not initialized")
        self.pip.aspirate(volume_uL)

    def dispense(self, volume_uL: float):
        self._ensure_connected()
        if not self.pip:
            raise RuntimeError("Pipette not initialized")
        self.pip.dispense(volume_uL)

    def eject_tip(self):
        self._ensure_connected()
        if not self.pip:
            raise RuntimeError("Pipette not initialized")
        self.pip.eject_tip()

    # ===== 新增：按点位名移动 =====
    def move_to_point(
        self,
        name: str,
        speed: int = 500,
        acc: int = 6000,
        z_offset: float = 0.0,
    ):
        self._ensure_connected()

        if name not in self.points:
            raise ValueError(f"未知点位: {name}，已加载点位数量={len(self.points)}")

        p = self.points[name]
        x = float(p["x"])
        y = float(p["y"])
        z = float(p["z"]) + float(z_offset)

        logger.info(
            "Move to point %s: x=%.3f, y=%.3f, z=%.3f (z_offset=%.3f)",
            name, x, y, z, z_offset,
        )
        self.xyz.move_xyz_work(x, y, z, speed, acc)