import time
import threading


class MockSeparator:
    def __init__(self, port: str = "MOCK"):
        self.port = port

        # 基本状态属性
        self._power_state: str = "Off"  # 电源：On 或 Off
        self._status: str = "Idle"  # 当前总体状态
        self._valve_state: str = "Closed"  # 阀门状态：Open 或 Closed
        self._settling_time: float = 0.0  # 静置时间（秒）

        # 搅拌相关属性
        self._shake_time: float = 0.0  # 剩余摇摆时间（秒）
        self._shake_status: str = "Not Shaking"  # 摇摆状态

        # 用于后台模拟 shake 动作
        self._operation_thread = None
        self._thread_lock = threading.Lock()

    @property
    def power_state(self) -> str:
        return self._power_state

    @property
    def valve_state(self) -> str:
        return self._valve_state

    @property
    def settling_time(self) -> float:
        return self._settling_time

    @property
    def status(self) -> str:
        return self._status

    @property
    def shake_time(self) -> float:
        with self._thread_lock:
            return self._shake_time

    @property
    def shake_status(self) -> str:
        with self._thread_lock:
            return self._shake_status

    def power_control(self, power_state: str) -> str:
        """
        电源控制：只接受 "On" 或 "Off"
        """
        if power_state not in ["On", "Off"]:
            self._status = "Error: Invalid power state"
            return "Error"

        self._power_state = power_state
        if power_state == "On":
            self._status = "Powered On"
        else:
            self._status = "Powered Off"
            self.stop_operations()
        return "Success"

    def shake(self, shake_time: float) -> str:
        """
        模拟 shake（搅拌）操作：
         - 进入 "Shaking" 状态，倒计时 shake_time 秒
         - shake 结束后，进入 "Settling" 状态，静置时间固定为 5 秒
         - 最后恢复为 Idle
        """
        try:
            shake_time = float(shake_time)
        except ValueError:
            self._status = "Error: Invalid shake time"
            return "Error"

        if self._power_state != "On":
            self._status = "Error: Power Off"
            return "Error"

        with self._thread_lock:
            self._status = "Shaking"
            self._settling_time = 0.0
            self._shake_time = shake_time
            self._shake_status = "Shaking"

        def _run_shake():
            remaining = shake_time
            while remaining > 0:
                time.sleep(1)
                remaining -= 1
                with self._thread_lock:
                    self._shake_time = remaining
            with self._thread_lock:
                self._status = "Settling"
                self._settling_time = 60.0  # 固定静置时间为60秒
                self._shake_status = "Settling"
            while True:
                with self._thread_lock:
                    if self._settling_time <= 0:
                        self._status = "Idle"
                        self._shake_status = "Idle"
                        break
                time.sleep(1)
                with self._thread_lock:
                    self._settling_time = max(0.0, self._settling_time - 1)

        self._operation_thread = threading.Thread(target=_run_shake)
        self._operation_thread.daemon = True
        self._operation_thread.start()
        return "Success"

    def set_valve(self, command: str) -> str:
        """
        阀门控制命令：传入 "open" 或 "close"
        """
        if self._power_state != "On":
            self._status = "Error: Power Off"
            return "Error"

        command = command.lower()
        if command == "open":
            self._valve_state = "Open"
            self._status = "Valve Opened"
        elif command == "close":
            self._valve_state = "Closed"
            self._status = "Valve Closed"
        else:
            self._status = "Error: Invalid valve command"
            return "Error"
        return "Success"

    def stop_operations(self) -> str:
        """
        停止任何正在执行的操作
        """
        with self._thread_lock:
            self._settling_time = 0.0
            self._status = "Idle"
            self._shake_status = "Idle"
            self._shake_time = 0.0
        return "Success"

    def get_status_info(self) -> dict:
        """
        获取当前设备状态信息
        """
        with self._thread_lock:
            return {
                "status": self._status,
                "power_state": self._power_state,
                "valve_state": self._valve_state,
                "settling_time": self._settling_time,
                "shake_time": self._shake_time,
                "shake_status": self._shake_status,
            }


# 主函数用于测试
if __name__ == "__main__":
    separator = MockSeparator()

    print("启动简单版分离器测试...")
    print(separator.power_control("On"))
    print("初始状态:", separator.get_status_info())

    # 触发 shake 操作，模拟 10 秒的搅拌
    print("执行 shake 操作...")
    print(separator.shake(10.0))

    # 循环显示状态变化
    for i in range(20):
        time.sleep(1)
        info = separator.get_status_info()
        print(
            f"第{i+1}秒: 状态={info['status']}, 静置时间={info['settling_time']:.1f}秒, "
            f"阀门状态={info['valve_state']}, shake_time={info['shake_time']:.1f}, "
            f"shake_status={info['shake_status']}"
        )

    # 模拟打开阀门
    print("打开阀门...", separator.set_valve("open"))
    print("最终状态:", separator.get_status_info())
