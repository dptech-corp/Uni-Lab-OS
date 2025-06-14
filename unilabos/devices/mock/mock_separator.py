import time
import threading
from datetime import datetime, timedelta

class MockSeparator:
    def __init__(self, port: str = "MOCK"):
        self.port = port

        # 基本状态属性
        self._status: str = "Idle"  # 当前总体状态
        self._valve_state: str = "Closed"  # 阀门状态：Open 或 Closed
        self._settling_time: float = 0.0  # 静置时间（秒）

        # 搅拌相关属性
        self._shake_time: float = 0.0  # 剩余摇摆时间（秒）
        self._shake_status: str = "Not Shaking"  # 摇摆状态

        # 用于后台模拟 shake 动作
        self._operation_thread = None
        self._thread_lock = threading.Lock()
        self._running = False

        # Separate action 相关属性
        self._current_device: str = "MockSeparator1"
        self._purpose: str = ""  # wash or extract
        self._product_phase: str = ""  # top or bottom
        self._from_vessel: str = ""
        self._separation_vessel: str = ""
        self._to_vessel: str = ""
        self._waste_phase_to_vessel: str = ""
        self._solvent: str = ""
        self._solvent_volume: float = 0.0
        self._through: str = ""
        self._repeats: int = 1
        self._stir_time: float = 0.0
        self._stir_speed: float = 0.0
        self._time_spent = timedelta()
        self._time_remaining = timedelta()
        self._start_time = datetime.now()  # 添加这一行

    @property
    def current_device(self) -> str:
        return self._current_device

    @property
    def purpose(self) -> str:
        return self._purpose

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
    
    @property
    def product_phase(self) -> str:
        return self._product_phase

    @property
    def from_vessel(self) -> str:
        return self._from_vessel

    @property
    def separation_vessel(self) -> str:
        return self._separation_vessel

    @property
    def to_vessel(self) -> str:
        return self._to_vessel

    @property
    def waste_phase_to_vessel(self) -> str:
        return self._waste_phase_to_vessel

    @property
    def solvent(self) -> str:
        return self._solvent

    @property
    def solvent_volume(self) -> float:
        return self._solvent_volume

    @property
    def through(self) -> str:
        return self._through

    @property
    def repeats(self) -> int:
        return self._repeats

    @property
    def stir_time(self) -> float:
        return self._stir_time

    @property
    def stir_speed(self) -> float:
        return self._stir_speed

    @property
    def time_spent(self) -> float:
        if self._running:
            self._time_spent = datetime.now() - self._start_time
        return self._time_spent.total_seconds()

    @property
    def time_remaining(self) -> float:
        if self._running:
            elapsed = (datetime.now() - self._start_time).total_seconds()
            total_time = (self._stir_time + self._settling_time + 10) * self._repeats
            remain = max(0, total_time - elapsed)
            self._time_remaining = timedelta(seconds=remain)
        return self._time_remaining.total_seconds()

    def separate(self, purpose: str, product_phase: str, from_vessel: str,
            separation_vessel: str, to_vessel: str, waste_phase_to_vessel: str = "",
            solvent: str = "", solvent_volume: float = 0.0, through: str = "",
            repeats: int = 1, stir_time: float = 0.0, stir_speed: float = 0.0,
            settling_time: float = 60.0) -> dict:
        """
        执行分离操作
        """
        with self._thread_lock:
            # 检查是否已经在运行
            if self._running:
                return {
                    "success": False,
                    "status": "Error: Operation already in progress"
                }
        # 必填参数验证
        if not all([from_vessel, separation_vessel, to_vessel]):
            self._status = "Error: Missing required vessel parameters"
            return {"success": False}
        # 验证参数
        if purpose not in ["wash", "extract"]:
            self._status = "Error: Invalid purpose"
            return {"success": False}
        
        if product_phase not in ["top", "bottom"]:
            self._status = "Error: Invalid product phase"
            return {"success": False}
            # 数值参数验证
        try:
            solvent_volume = float(solvent_volume)
            repeats = int(repeats)
            stir_time = float(stir_time)
            stir_speed = float(stir_speed)
            settling_time = float(settling_time)
        except ValueError:
            self._status = "Error: Invalid numeric parameters"
            return {"success": False}

        # 设置参数
        self._purpose = purpose
        self._product_phase = product_phase
        self._from_vessel = from_vessel
        self._separation_vessel = separation_vessel
        self._to_vessel = to_vessel
        self._waste_phase_to_vessel = waste_phase_to_vessel
        self._solvent = solvent
        self._solvent_volume = float(solvent_volume)
        self._through = through
        self._repeats = int(repeats)
        self._stir_time = float(stir_time)
        self._stir_speed = float(stir_speed)
        self._settling_time = float(settling_time)

        # 重置计时器
        self._start_time = datetime.now()
        self._time_spent = timedelta()
        total_time = (self._stir_time + self._settling_time + 10) * self._repeats
        self._time_remaining = timedelta(seconds=total_time)

        # 启动分离操作
        self._status = "Starting Separation"
        self._running = True

        # 在锁内创建和启动线程
        self._operation_thread = threading.Thread(target=self._operation_loop)
        self._operation_thread.daemon = True
        self._operation_thread.start()

        # 等待确认操作已经开始
        time.sleep(0.1)  # 短暂等待确保操作线程已启动

        return {
            "success": True,
            "status": self._status,
            "current_device": self._current_device,
            "time_spent": self._time_spent.total_seconds(),
            "time_remaining": self._time_remaining.total_seconds()
        }

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
    
    def _operation_loop(self):
        """分离操作主循环"""
        try:
            current_repeat = 1
            
            # 立即更新状态，确保不会停留在Starting Separation
            with self._thread_lock:
                self._status = f"Separation Cycle {current_repeat}/{self._repeats}"
            
            while self._running and current_repeat <= self._repeats:
                # 第一步：搅拌
                if self._stir_time > 0:
                    with self._thread_lock:
                        self._status = f"Stirring (Repeat {current_repeat}/{self._repeats})"
                    remaining_stir = self._stir_time
                    while remaining_stir > 0 and self._running:
                        time.sleep(1)
                        remaining_stir -= 1

                # 第二步：静置
                if self._settling_time > 0:
                    with self._thread_lock:
                        self._status = f"Settling (Repeat {current_repeat}/{self._repeats})"
                    remaining_settle = self._settling_time
                    while remaining_settle > 0 and self._running:
                        time.sleep(1)
                        remaining_settle -= 1

                # 第三步：打开阀门排出
                with self._thread_lock:
                    self._valve_state = "Open"
                    self._status = f"Draining (Repeat {current_repeat}/{self._repeats})"
                
                # 模拟排出时间（5秒）
                time.sleep(10)
                
                # 关闭阀门
                with self._thread_lock:
                    self._valve_state = "Closed"

                # 检查是否继续下一次重复
                if current_repeat < self._repeats:
                    current_repeat += 1
                else:
                    with self._thread_lock:
                        self._status = "Separation Complete"
                    break

        except Exception as e:
            with self._thread_lock:
                self._status = f"Error in separation: {str(e)}"
        finally:
            with self._thread_lock:
                self._running = False
                self._valve_state = "Closed"
                if self._status == "Starting Separation":
                    self._status = "Error: Operation failed to start"
                elif self._status != "Separation Complete":
                    self._status = "Stopped"

    def stop_operations(self) -> str:
        """停止任何正在执行的操作"""
        with self._thread_lock:
            self._running = False
            if self._operation_thread and self._operation_thread.is_alive():
                self._operation_thread.join(timeout=1.0)
            self._operation_thread = None
            self._settling_time = 0.0
            self._status = "Idle"
            self._shake_status = "Idle"
            self._shake_time = 0.0
            self._time_remaining = timedelta()
        return "Success"

    def get_status_info(self) -> dict:
        """获取当前设备状态信息"""
        with self._thread_lock:
            current_time = datetime.now()
            if self._start_time:
                self._time_spent = current_time - self._start_time
            
            return {
                "status": self._status,
                "valve_state": self._valve_state,
                "settling_time": self._settling_time,
                "shake_time": self._shake_time,
                "shake_status": self._shake_status,
                "current_device": self._current_device,
                "purpose": self._purpose,
                "product_phase": self._product_phase,
                "from_vessel": self._from_vessel,
                "separation_vessel": self._separation_vessel,
                "to_vessel": self._to_vessel,
                "waste_phase_to_vessel": self._waste_phase_to_vessel,
                "solvent": self._solvent,
                "solvent_volume": self._solvent_volume,
                "through": self._through,
                "repeats": self._repeats,
                "stir_time": self._stir_time,
                "stir_speed": self._stir_speed,
                "time_spent": self._time_spent.total_seconds(),
                "time_remaining": self._time_remaining.total_seconds()
            }


# 主函数用于测试
if __name__ == "__main__":
    separator = MockSeparator()

    print("启动简单版分离器测试...")
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
