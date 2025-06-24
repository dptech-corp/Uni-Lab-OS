import time
import threading
import json


class MockRotavap:
    """
    模拟旋转蒸发器设备类

    这个类模拟了一个实验室旋转蒸发器的行为，包括旋转控制、
    真空泵控制、温度控制等功能。参考了现有的 RotavapOne 实现。
    """

    def __init__(self, port: str = "MOCK"):
        """
        初始化MockRotavap实例

        Args:
            port (str): 设备端口，默认为"MOCK"表示模拟设备
        """
        self.port = port

        # 设备基本状态属性
        self._status: str = "Idle"  # 设备状态：Idle, Running, Error, Stopped

        # 旋转相关属性
        self._rotate_state: str = "Stopped"  # 旋转状态：Running, Stopped
        self._rotate_time: float = 0.0  # 旋转剩余时间 (秒)
        self._rotate_speed: float = 0.0  # 旋转速度 (rpm)
        self._max_rotate_speed: float = 300.0  # 最大旋转速度 (rpm)

        # 真空泵相关属性
        self._pump_state: str = "Stopped"  # 泵状态：Running, Stopped
        self._pump_time: float = 0.0  # 泵剩余时间 (秒)
        self._vacuum_level: float = 0.0  # 真空度 (mbar)
        self._target_vacuum: float = 50.0  # 目标真空度 (mbar)

        # 温度相关属性
        self._temperature: float = 25.0  # 水浴温度 (°C)
        self._target_temperature: float = 25.0  # 目标温度 (°C)
        self._max_temperature: float = 180.0  # 最大温度 (°C)

        # 运行控制线程
        self._operation_thread = None
        self._running = False
        self._thread_lock = threading.Lock()

        # 操作成功标志
        self.success: str = "True"  # 使用字符串而不是布尔值

    # ==================== 状态属性 ====================
    # 这些属性会被Uni-Lab系统自动识别并定时对外广播

    @property
    def status(self) -> str:
        return self._status

    @property
    def rotate_state(self) -> str:
        return self._rotate_state

    @property
    def rotate_time(self) -> float:
        return self._rotate_time

    @property
    def rotate_speed(self) -> float:
        return self._rotate_speed

    @property
    def pump_state(self) -> str:
        return self._pump_state

    @property
    def pump_time(self) -> float:
        return self._pump_time

    @property
    def vacuum_level(self) -> float:
        return self._vacuum_level

    @property
    def temperature(self) -> float:
        return self._temperature

    @property
    def target_temperature(self) -> float:
        return self._target_temperature

    # ==================== 设备控制方法 ====================
    # 这些方法需要在注册表中添加，会作为ActionServer接受控制指令

    def set_timer(self, command: str) -> str:
        """
        设置定时器 - 兼容现有RotavapOne接口

        Args:
            command (str): JSON格式的命令字符串，包含rotate_time和pump_time

        Returns:
            str: 操作结果状态 ("Success", "Error")
        """

        try:
            timer = json.loads(command)
            rotate_time = timer.get("rotate_time", 0)
            pump_time = timer.get("pump_time", 0)

            self.success = "False"
            self._rotate_time = float(rotate_time)
            self._pump_time = float(pump_time)
            self.success = "True"

            self._status = "Timer Set"
            return "Success"

        except (json.JSONDecodeError, ValueError, KeyError) as e:
            self._status = f"Error: Invalid command format - {str(e)}"
            self.success = "False"
            return "Error"

    def set_rotate_time(self, time_seconds: float) -> str:
        """
        设置旋转时间

        Args:
            time_seconds (float): 旋转时间 (秒)

        Returns:
            str: 操作结果状态 ("Success", "Error")
        """

        self.success = "False"
        self._rotate_time = max(0.0, float(time_seconds))
        self.success = "True"
        self._status = "Rotate time set"
        return "Success"

    def set_pump_time(self, time_seconds: float) -> str:
        """
        设置泵时间

        Args:
            time_seconds (float): 泵时间 (秒)

        Returns:
            str: 操作结果状态 ("Success", "Error")
        """

        self.success = "False"
        self._pump_time = max(0.0, float(time_seconds))
        self.success = "True"
        self._status = "Pump time set"
        return "Success"

    def set_rotate_speed(self, speed: float) -> str:
        """
        设置旋转速度

        Args:
            speed (float): 旋转速度 (rpm)

        Returns:
            str: 操作结果状态 ("Success", "Error")
        """

        if speed < 0 or speed > self._max_rotate_speed:
            self._status = f"Error: Speed out of range (0-{self._max_rotate_speed})"
            return "Error"

        self._rotate_speed = speed
        self._status = "Rotate speed set"
        return "Success"

    def set_temperature(self, temperature: float) -> str:
        """
        设置水浴温度

        Args:
            temperature (float): 目标温度 (°C)

        Returns:
            str: 操作结果状态 ("Success", "Error")
        """

        if temperature < 0 or temperature > self._max_temperature:
            self._status = f"Error: Temperature out of range (0-{self._max_temperature})"
            return "Error"

        self._target_temperature = temperature
        self._status = "Temperature set"

        # 启动操作线程以开始温度控制
        self._start_operation()
        
        return "Success"

    def start_rotation(self) -> str:
        """
        启动旋转

        Returns:
            str: 操作结果状态 ("Success", "Error")
        """

        if self._rotate_time <= 0:
            self._status = "Error: No rotate time set"
            return "Error"

        self._rotate_state = "Running"
        self._status = "Rotation started"
        return "Success"

    def start_pump(self) -> str:
        """
        启动真空泵

        Returns:
            str: 操作结果状态 ("Success", "Error")
        """

        if self._pump_time <= 0:
            self._status = "Error: No pump time set"
            return "Error"

        self._pump_state = "Running"
        self._status = "Pump started"
        return "Success"

    def stop_all_operations(self) -> str:
        """
        停止所有操作

        Returns:
            str: 操作结果状态 ("Success", "Error")
        """
        self._rotate_state = "Stopped"
        self._pump_state = "Stopped"
        self._stop_operation()
        self._rotate_time = 0.0
        self._pump_time = 0.0
        self._vacuum_level = 0.0
        self._status = "All operations stopped"
        return "Success"

    def emergency_stop(self) -> str:
        """
        紧急停止

        Returns:
            str: 操作结果状态 ("Success", "Error")
        """
        self._status = "Emergency Stop"
        self.stop_all_operations()
        return "Success"

    # ==================== 内部控制方法 ====================

    def _start_operation(self):
        """
        启动操作线程

        这个方法启动一个后台线程来模拟旋蒸的实际运行过程。
        """
        with self._thread_lock:
            if not self._running:
                self._running = True
                self._operation_thread = threading.Thread(target=self._operation_loop)
                self._operation_thread.daemon = True
                self._operation_thread.start()

    def _stop_operation(self):
        """
        停止操作线程

        安全地停止后台运行线程并等待其完成。
        """
        with self._thread_lock:
            self._running = False
            if self._operation_thread and self._operation_thread.is_alive():
                self._operation_thread.join(timeout=2.0)

    def _operation_loop(self):
        """
        操作主循环

        这个方法在后台线程中运行，模拟真实旋蒸的工作过程：
        1. 时间倒计时
        2. 温度控制
        3. 真空度控制
        4. 状态更新
        """
        while self._running:
            try:
                # 处理旋转时间倒计时
                if self._rotate_time > 0:
                    self._rotate_state = "Running"
                    self._rotate_time = max(0.0, self._rotate_time - 1.0)
                else:
                    self._rotate_state = "Stopped"

                # 处理泵时间倒计时
                if self._pump_time > 0:
                    self._pump_state = "Running"
                    self._pump_time = max(0.0, self._pump_time - 1.0)
                    # 模拟真空度变化
                    if self._vacuum_level > self._target_vacuum:
                        self._vacuum_level = max(self._target_vacuum, self._vacuum_level - 5.0)
                else:
                    self._pump_state = "Stopped"
                    # 真空度逐渐回升
                    self._vacuum_level = min(1013.25, self._vacuum_level + 2.0)

                # 模拟温度控制
                temp_diff = self._target_temperature - self._temperature
                if abs(temp_diff) > 0.5:
                    if temp_diff > 0:
                        self._temperature += min(1.0, temp_diff * 0.1)
                    else:
                        self._temperature += max(-1.0, temp_diff * 0.1)

                # 更新整体状态
                if self._rotate_state == "Running" or self._pump_state == "Running":
                    self._status = "Operating"
                elif self._rotate_time > 0 or self._pump_time > 0:
                    self._status = "Ready"
                else:
                    self._status = "Idle"

                # 等待1秒后继续下一次循环
                time.sleep(1.0)

            except Exception as e:
                self._status = f"Error in operation: {str(e)}"
                break

        # 循环结束时的清理工作
            self._status = "Idle"

    def get_status_info(self) -> dict:
        """
        获取完整的设备状态信息

        Returns:
            dict: 包含所有设备状态的字典
        """
        return {
            "status": self._status,
            "rotate_state": self._rotate_state,
            "rotate_time": self._rotate_time,
            "rotate_speed": self._rotate_speed,
            "pump_state": self._pump_state,
            "pump_time": self._pump_time,
            "vacuum_level": self._vacuum_level,
            "temperature": self._temperature,
            "target_temperature": self._target_temperature,
            "success": self.success,
        }


# 用于测试的主函数
if __name__ == "__main__":
    rotavap = MockRotavap()

    # 测试基本功能
    print("启动旋转蒸发器测试...")
    print(f"初始状态: {rotavap.get_status_info()}")

    # 设置定时器
    timer_command = '{"rotate_time": 300, "pump_time": 600}'
    rotavap.set_timer(timer_command)

    # 设置温度和转速
    rotavap.set_temperature(60.0)
    rotavap.set_rotate_speed(120.0)

    # 启动操作
    rotavap.start_rotation()
    rotavap.start_pump()

    # 模拟运行10秒
    for i in range(10):
        time.sleep(1)
        print(
            f"第{i+1}秒: 旋转={rotavap.rotate_time:.0f}s, 泵={rotavap.pump_time:.0f}s, "
            f"温度={rotavap.temperature:.1f}°C, 真空={rotavap.vacuum_level:.1f}mbar"
        )

    rotavap.emergency_stop()
    print("测试完成")
