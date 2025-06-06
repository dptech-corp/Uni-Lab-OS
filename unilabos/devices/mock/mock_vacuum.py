import time
import threading


class MockVacuum:
    """
    模拟真空泵设备类

    这个类模拟了一个实验室真空泵的行为，包括真空度控制、
    压力监测、运行状态管理等功能。参考了现有的 VacuumPumpMock 实现。
    """

    def __init__(self, port: str = "MOCK"):
        """
        初始化MockVacuum实例

        Args:
            port (str): 设备端口，默认为"MOCK"表示模拟设备
        """
        self.port = port

        # 设备基本状态属性
        self._status: str = "Idle"  # 设备状态：Idle, Running, Error, Stopped
        self._power_state: str = "Off"  # 电源状态：On, Off
        self._pump_state: str = "Stopped"  # 泵运行状态：Running, Stopped, Paused

        # 真空相关属性
        self._vacuum_level: float = 1013.25  # 当前真空度 (mbar) - 大气压开始
        self._target_vacuum: float = 50.0  # 目标真空度 (mbar)
        self._min_vacuum: float = 1.0  # 最小真空度 (mbar)
        self._max_vacuum: float = 1013.25  # 最大真空度 (mbar) - 大气压

        # 泵性能相关属性
        self._pump_speed: float = 0.0  # 泵速 (L/s)
        self._max_pump_speed: float = 100.0  # 最大泵速 (L/s)
        self._pump_efficiency: float = 95.0  # 泵效率百分比

        # 运行控制线程
        self._vacuum_thread = None
        self._running = False
        self._thread_lock = threading.Lock()

    # ==================== 状态属性 ====================
    # 这些属性会被Uni-Lab系统自动识别并定时对外广播

    @property
    def status(self) -> str:
        """
        设备状态 - 会被自动识别的设备属性

        Returns:
            str: 当前设备状态 (Idle, Running, Error, Stopped)
        """
        return self._status

    @property
    def power_state(self) -> str:
        """
        电源状态

        Returns:
            str: 电源状态 (On, Off)
        """
        return self._power_state

    @property
    def pump_state(self) -> str:
        """
        泵运行状态

        Returns:
            str: 泵状态 (Running, Stopped, Paused)
        """
        return self._pump_state

    @property
    def vacuum_level(self) -> float:
        """
        当前真空度

        Returns:
            float: 当前真空度 (mbar)
        """
        return self._vacuum_level

    @property
    def target_vacuum(self) -> float:
        """
        目标真空度

        Returns:
            float: 目标真空度 (mbar)
        """
        return self._target_vacuum

    @property
    def pump_speed(self) -> float:
        """
        泵速

        Returns:
            float: 泵速 (L/s)
        """
        return self._pump_speed

    @property
    def pump_efficiency(self) -> float:
        """
        泵效率

        Returns:
            float: 泵效率百分比
        """
        return self._pump_efficiency

    @property
    def max_pump_speed(self) -> float:
        """
        最大泵速

        Returns:
            float: 最大泵速 (L/s)
        """
        return self._max_pump_speed

    # ==================== 设备控制方法 ====================
    # 这些方法需要在注册表中添加，会作为ActionServer接受控制指令

    def power_control(self, power_state: str = "On") -> str:
        """
        电源控制方法

        Args:
            power_state (str): 电源状态，可选值："On", "Off"

        Returns:
            str: 操作结果状态 ("Success", "Error")
        """
        if power_state not in ["On", "Off"]:
            self._status = "Error: Invalid power state"
            return "Error"

        self._power_state = power_state

        if power_state == "On":
            self._status = "Power On"
            self._start_vacuum_operation()
        else:
            self._status = "Power Off"
            self.stop_vacuum()

        return "Success"

    def set_vacuum_level(self, vacuum_level: float) -> str:
        """
        设置目标真空度

        Args:
            vacuum_level (float): 目标真空度 (mbar)

        Returns:
            str: 操作结果状态 ("Success", "Error")
        """
        try:
            vacuum_level = float(vacuum_level)
        except ValueError:
            self._status = "Error: Invalid vacuum level"
            return "Error"
        if self._power_state != "On":
            self._status = "Error: Power Off"
            return "Error"

        if vacuum_level < self._min_vacuum or vacuum_level > self._max_vacuum:
            self._status = f"Error: Vacuum level out of range ({self._min_vacuum}-{self._max_vacuum})"
            return "Error"

        self._target_vacuum = vacuum_level
        self._status = "Setting Vacuum Level"

        return "Success"

    def start_vacuum(self) -> str:
        """
        启动真空泵

        Returns:
            str: 操作结果状态 ("Success", "Error")
        """
        if self._power_state != "On":
            self._status = "Error: Power Off"
            return "Error"

        self._pump_state = "Running"
        self._status = "Starting Vacuum Pump"
        self._start_vacuum_operation()

        return "Success"

    def stop_vacuum(self) -> str:
        """
        停止真空泵

        Returns:
            str: 操作结果状态 ("Success", "Error")
        """
        self._pump_state = "Stopped"
        self._status = "Stopping Vacuum Pump"
        self._stop_vacuum_operation()
        self._pump_speed = 0.0

        return "Success"

    def pause_vacuum(self) -> str:
        """
        暂停真空泵

        Returns:
            str: 操作结果状态 ("Success", "Error")
        """
        if self._pump_state != "Running":
            self._status = "Error: Pump not running"
            return "Error"

        self._pump_state = "Paused"
        self._status = "Vacuum Pump Paused"
        self._stop_vacuum_operation()

        return "Success"

    def resume_vacuum(self) -> str:
        """
        恢复真空泵运行

        Returns:
            str: 操作结果状态 ("Success", "Error")
        """
        if self._pump_state != "Paused":
            self._status = "Error: Pump not paused"
            return "Error"

        if self._power_state != "On":
            self._status = "Error: Power Off"
            return "Error"

        self._pump_state = "Running"
        self._status = "Resuming Vacuum Pump"
        self._start_vacuum_operation()

        return "Success"

    def vent_to_atmosphere(self) -> str:
        """
        通大气 - 将真空度恢复到大气压

        Returns:
            str: 操作结果状态 ("Success", "Error")
        """
        self._target_vacuum = self._max_vacuum  # 设置为大气压
        self._status = "Venting to Atmosphere"
        return "Success"

    def emergency_stop(self) -> str:
        """
        紧急停止

        Returns:
            str: 操作结果状态 ("Success", "Error")
        """
        self._status = "Emergency Stop"
        self._pump_state = "Stopped"
        self._stop_vacuum_operation()
        self._pump_speed = 0.0

        return "Success"

    # ==================== 内部控制方法 ====================

    def _start_vacuum_operation(self):
        """
        启动真空操作线程

        这个方法启动一个后台线程来模拟真空泵的实际运行过程。
        """
        with self._thread_lock:
            if not self._running and self._power_state == "On":
                self._running = True
                self._vacuum_thread = threading.Thread(target=self._vacuum_operation_loop)
                self._vacuum_thread.daemon = True
                self._vacuum_thread.start()

    def _stop_vacuum_operation(self):
        """
        停止真空操作线程

        安全地停止后台运行线程并等待其完成。
        """
        with self._thread_lock:
            self._running = False
            if self._vacuum_thread and self._vacuum_thread.is_alive():
                self._vacuum_thread.join(timeout=2.0)

    def _vacuum_operation_loop(self):
        """
        真空操作主循环

        这个方法在后台线程中运行，模拟真空泵的工作过程：
          1. 检查电源状态和运行状态
          2. 如果泵状态为 "Running"，根据目标真空调整泵速和真空度
          3. 否则等待
        """
        while self._running and self._power_state == "On":
            try:
                with self._thread_lock:
                    # 只有泵状态为 Running 时才进行更新
                    if self._pump_state == "Running":
                        vacuum_diff = self._vacuum_level - self._target_vacuum

                        if abs(vacuum_diff) < 1.0:  # 真空度接近目标值
                            self._status = "At Target Vacuum"
                            self._pump_speed = self._max_pump_speed * 0.2  # 维持真空的最小泵速
                        elif vacuum_diff > 0:  # 需要抽真空（降低压力）
                            self._status = "Pumping Down"
                            if vacuum_diff > 500:
                                self._pump_speed = self._max_pump_speed
                            elif vacuum_diff > 100:
                                self._pump_speed = self._max_pump_speed * 0.8
                            elif vacuum_diff > 50:
                                self._pump_speed = self._max_pump_speed * 0.6
                            else:
                                self._pump_speed = self._max_pump_speed * 0.4

                            # 根据泵速和效率计算真空降幅
                            pump_rate = (self._pump_speed / self._max_pump_speed) * self._pump_efficiency / 100.0
                            vacuum_reduction = pump_rate * 10.0  # 每秒最大降低10 mbar
                            self._vacuum_level = max(self._target_vacuum, self._vacuum_level - vacuum_reduction)
                        else:  # 目标真空度高于当前值，需要通气
                            self._status = "Venting"
                            self._pump_speed = 0.0
                            self._vacuum_level = min(self._target_vacuum, self._vacuum_level + 5.0)

                        # 限制真空度范围
                        self._vacuum_level = max(self._min_vacuum, min(self._max_vacuum, self._vacuum_level))
                    else:
                        # 当泵状态不是 Running 时，可保持原状态
                        self._status = "Vacuum Pump Not Running"
                # 释放锁后等待1秒钟
                time.sleep(1.0)
            except Exception as e:
                with self._thread_lock:
                    self._status = f"Error in vacuum operation: {str(e)}"
                break

        # 循环结束后的清理工作
        if self._pump_state == "Running":
            self._status = "Idle"
            # 停止泵后，真空度逐渐回升到大气压
            while self._vacuum_level < self._max_vacuum * 0.9:
                with self._thread_lock:
                    self._vacuum_level += 2.0
                time.sleep(0.1)

    def get_status_info(self) -> dict:
        """
        获取完整的设备状态信息

        Returns:
            dict: 包含所有设备状态的字典
        """
        return {
            "status": self._status,
            "power_state": self._power_state,
            "pump_state": self._pump_state,
            "vacuum_level": self._vacuum_level,
            "target_vacuum": self._target_vacuum,
            "pump_speed": self._pump_speed,
            "pump_efficiency": self._pump_efficiency,
            "max_pump_speed": self._max_pump_speed,
        }


# 用于测试的主函数
if __name__ == "__main__":
    vacuum = MockVacuum()

    # 测试基本功能
    print("启动真空泵测试...")
    vacuum.power_control("On")
    print(f"初始状态: {vacuum.get_status_info()}")

    # 设置目标真空度并启动
    vacuum.set_vacuum_level(10.0)  # 设置为10mbar
    vacuum.start_vacuum()

    # 模拟运行15秒
    for i in range(15):
        time.sleep(1)
        print(
            f"第{i+1}秒: 真空度={vacuum.vacuum_level:.1f}mbar, 泵速={vacuum.pump_speed:.1f}L/s, 状态={vacuum.status}"
        )
        # 测试通大气
    print("测试通大气...")
    vacuum.vent_to_atmosphere()

    # 继续运行5秒观察通大气过程
    for i in range(5):
        time.sleep(1)
        print(f"通大气第{i+1}秒: 真空度={vacuum.vacuum_level:.1f}mbar, 状态={vacuum.status}")

    vacuum.emergency_stop()
    print("测试完成")
