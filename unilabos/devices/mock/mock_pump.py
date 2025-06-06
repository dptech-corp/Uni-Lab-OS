import time
import threading


class MockPump:
    """
    模拟泵设备类

    这个类模拟了一个实验室泵设备的行为，包括流量控制、压力监测、
    运行状态管理等功能。所有的控制参数都使用字符串类型以提供更好的
    可读性和扩展性。
    """

    def __init__(self, port: str = "MOCK"):
        """
        初始化MockPump实例

        Args:
            port (str): 设备端口，默认为"MOCK"表示模拟设备
        """
        self.port = port

        # 设备基本状态属性
        self._status: str = "Idle"  # 设备状态：Idle, Running, Error, Stopped
        self._power_state: str = "Off"  # 电源状态：On, Off
        self._pump_state: str = "Stopped"  # 泵运行状态：Running, Stopped, Paused

        # 流量相关属性
        self._flow_rate: float = 0.0  # 当前流速 (mL/min)
        self._target_flow_rate: float = 0.0  # 目标流速 (mL/min)
        self._max_flow_rate: float = 100.0  # 最大流速 (mL/min)
        self._total_volume: float = 0.0  # 累计流量 (mL)

        # 压力相关属性
        self._pressure: float = 0.0  # 当前压力 (bar)
        self._max_pressure: float = 10.0  # 最大压力 (bar)

        # 方向控制属性
        self._direction: str = "Forward"  # 泵方向：Forward, Reverse

        # 运行控制线程
        self._pump_thread = None
        self._running = False
        self._thread_lock = threading.Lock()

    # ==================== 状态属性 ====================
    # 这些属性会被Uni-Lab系统自动识别并定时对外广播

    @property
    def status(self) -> str:
        return self._status

    @property
    def power_state(self) -> str:
        return self._power_state

    @property
    def pump_state(self) -> str:
        return self._pump_state

    @property
    def flow_rate(self) -> float:
        return self._flow_rate

    @property
    def target_flow_rate(self) -> float:
        return self._target_flow_rate

    @property
    def pressure(self) -> float:
        return self._pressure

    @property
    def total_volume(self) -> float:
        return self._total_volume

    @property
    def direction(self) -> str:
        return self._direction

    @property
    def max_flow_rate(self) -> float:
        return self._max_flow_rate

    @property
    def max_pressure(self) -> float:
        return self._max_pressure

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
        else:
            self._status = "Power Off"
            # 关机时停止所有运行
            self.stop_pump()

        return "Success"

    def set_flow_rate(self, flow_rate: float) -> str:
        """
        设置目标流速

        Args:
            flow_rate (float): 目标流速 (mL/min)

        Returns:
            str: 操作结果状态 ("Success", "Error")
        """
        try:
            flow_rate = float(flow_rate)
        except ValueError:
            self._status = "Error: Invalid flow rate"
            return "Error"

        if self._power_state != "On":
            self._status = "Error: Power Off"
            return "Error"

        if flow_rate < 0 or flow_rate > self._max_flow_rate:
            self._status = f"Error: Flow rate out of range (0-{self._max_flow_rate})"
            return "Error"

        self._target_flow_rate = flow_rate
        self._status = "Setting Flow Rate"

        # 如果设置了非零流速，自动启动泵
        if flow_rate > 0:
            # 自动切换泵状态为 "Running" 以触发操作循环
            self._pump_state = "Running"
            self._start_pump_operation()
        else:
            self.stop_pump()

        return "Success"

    def start_pump(self) -> str:
        """
        启动泵运行

        Returns:
            str: 操作结果状态 ("Success", "Error")
        """
        if self._power_state != "On":
            self._status = "Error: Power Off"
            return "Error"

        if self._target_flow_rate <= 0:
            self._status = "Error: No target flow rate set"
            return "Error"

        self._pump_state = "Running"
        self._status = "Starting Pump"
        self._start_pump_operation()

        return "Success"

    def stop_pump(self) -> str:
        """
        停止泵运行

        Returns:
            str: 操作结果状态 ("Success", "Error")
        """
        self._pump_state = "Stopped"
        self._status = "Stopping Pump"
        self._stop_pump_operation()
        self._flow_rate = 0.0
        self._pressure = 0.0

        return "Success"

    def pause_pump(self) -> str:
        """
        暂停泵运行

        Returns:
            str: 操作结果状态 ("Success", "Error")
        """
        if self._pump_state != "Running":
            self._status = "Error: Pump not running"
            return "Error"

        self._pump_state = "Paused"
        self._status = "Pump Paused"
        self._stop_pump_operation()

        return "Success"

    def resume_pump(self) -> str:
        """
        恢复泵运行

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
        self._status = "Resuming Pump"
        self._start_pump_operation()

        return "Success"

    def set_direction(self, direction: str = "Forward") -> str:
        """
        设置泵方向

        Args:
            direction (str): 泵方向，可选值："Forward", "Reverse"

        Returns:
            str: 操作结果状态 ("Success", "Error")
        """
        if direction not in ["Forward", "Reverse"]:
            self._status = "Error: Invalid direction"
            return "Error"

        # 如果泵正在运行，需要先停止
        was_running = self._pump_state == "Running"
        if was_running:
            self.stop_pump()
            time.sleep(0.5)  # 等待停止完成

        self._direction = direction
        self._status = f"Direction set to {direction}"

        # 如果之前在运行，重新启动
        if was_running:
            self.start_pump()

        return "Success"

    def reset_volume_counter(self) -> str:
        """
        重置累计流量计数器

        Returns:
            str: 操作结果状态 ("Success", "Error")
        """
        self._total_volume = 0.0
        self._status = "Volume counter reset"
        return "Success"

    def emergency_stop(self) -> str:
        """
        紧急停止

        Returns:
            str: 操作结果状态 ("Success", "Error")
        """
        self._status = "Emergency Stop"
        self._pump_state = "Stopped"
        self._stop_pump_operation()
        self._flow_rate = 0.0
        self._pressure = 0.0
        self._target_flow_rate = 0.0

        return "Success"

    # ==================== 内部控制方法 ====================

    def _start_pump_operation(self):
        """
        启动泵运行线程

        这个方法启动一个后台线程来模拟泵的实际运行过程，
        包括流速控制、压力变化和累计流量计算。
        """
        with self._thread_lock:
            if not self._running and self._power_state == "On":
                self._running = True
                self._pump_thread = threading.Thread(target=self._pump_operation_loop)
                self._pump_thread.daemon = True
                self._pump_thread.start()

    def _stop_pump_operation(self):
        """
        停止泵运行线程

        安全地停止后台运行线程并等待其完成。
        """
        with self._thread_lock:
            self._running = False
            if self._pump_thread and self._pump_thread.is_alive():
                self._pump_thread.join(timeout=2.0)

    def _pump_operation_loop(self):
        """
        泵运行主循环

        这个方法在后台线程中运行，模拟真实泵的工作过程：
        1. 逐步调整流速到目标值
        2. 根据流速计算压力
        3. 累计流量统计
        4. 状态更新
        """
        while self._running and self._power_state == "On" and self._pump_state == "Running":
            try:
                # 模拟流速调节过程（逐步接近目标流速）
                flow_diff = self._target_flow_rate - self._flow_rate

                if abs(flow_diff) < 0.1:  # 流速接近目标值
                    self._flow_rate = self._target_flow_rate
                    self._status = "At Target Flow Rate"
                else:
                    # 模拟流速调节，每秒调整10%的差值
                    adjustment = flow_diff * 0.1
                    self._flow_rate += adjustment
                    self._status = "Adjusting Flow Rate"

                # 确保流速在合理范围内
                self._flow_rate = max(0.0, min(self._max_flow_rate, self._flow_rate))

                # 模拟压力变化（压力与流速成正比，加上一些随机波动）
                base_pressure = (self._flow_rate / self._max_flow_rate) * self._max_pressure
                pressure_variation = 0.1 * base_pressure * (time.time() % 1.0 - 0.5)  # ±5%波动
                self._pressure = max(0.0, base_pressure + pressure_variation)

                # 累计流量计算（每秒更新）
                if self._flow_rate > 0:
                    volume_increment = self._flow_rate / 60.0  # 转换为mL/s
                    if self._direction == "Reverse":
                        volume_increment = -volume_increment
                    self._total_volume += volume_increment

                # 压力保护检查
                if self._pressure > self._max_pressure * 0.95:
                    self._status = "Warning: High Pressure"

                # 等待1秒后继续下一次循环
                time.sleep(1.0)

            except Exception as e:
                self._status = f"Error in pump operation: {str(e)}"
                break

        # 循环结束时的清理工作
        if self._pump_state == "Running":
            self._status = "Idle"

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
            "flow_rate": self._flow_rate,
            "target_flow_rate": self._target_flow_rate,
            "pressure": self._pressure,
            "total_volume": self._total_volume,
            "direction": self._direction,
            "max_flow_rate": self._max_flow_rate,
            "max_pressure": self._max_pressure,
        }


# 用于测试的主函数
if __name__ == "__main__":
    pump = MockPump()

    # 测试基本功能
    print("启动泵设备测试...")
    pump.power_control("On")
    print(f"初始状态: {pump.get_status_info()}")

    # 设置流速并启动
    pump.set_flow_rate(50.0)
    pump.start_pump()

    # 模拟运行10秒
    for i in range(10):
        time.sleep(1)
        print(f"第{i+1}秒: 流速={pump.flow_rate:.1f}mL/min, 压力={pump.pressure:.2f}bar, 状态={pump.status}")

    # 测试方向切换
    print("切换泵方向...")
    pump.set_direction("Reverse")

    # 继续运行5秒
    for i in range(5):
        time.sleep(1)
        print(f"反向第{i+1}秒: 累计流量={pump.total_volume:.1f}mL, 方向={pump.direction}")

    pump.emergency_stop()
    print("测试完成")
