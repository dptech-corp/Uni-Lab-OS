import time
import threading
from datetime import datetime, timedelta

class MockPump:
    def __init__(self, port: str = "MOCK"):
        self.port = port

        # 设备基本状态属性
        self._current_device = "MockPump1"  # 设备标识符
        self._status: str = "Idle"  # 设备状态：Idle, Running, Error, Stopped
        self._pump_state: str = "Stopped"  # 泵运行状态：Running, Stopped, Paused

        # 流量相关属性
        self._flow_rate: float = 0.0  # 当前流速 (mL/min)
        self._target_flow_rate: float = 0.0  # 目标流速 (mL/min)
        self._max_flow_rate: float = 100.0  # 最大流速 (mL/min)
        self._total_volume: float = 0.0  # 累计流量 (mL)

        # 压力相关属性
        self._pressure: float = 0.0  # 当前压力 (bar)
        self._max_pressure: float = 10.0  # 最大压力 (bar)

        # 运行控制线程
        self._pump_thread = None
        self._running = False
        self._thread_lock = threading.Lock()

        # 新增 PumpTransfer 相关属性
        self._from_vessel: str = ""
        self._to_vessel: str = ""
        self._transfer_volume: float = 0.0
        self._amount: str = ""
        self._transfer_time: float = 0.0
        self._is_viscous: bool = False
        self._rinsing_solvent: str = ""
        self._rinsing_volume: float = 0.0
        self._rinsing_repeats: int = 0
        self._is_solid: bool = False
        
        # 时间追踪
        self._start_time: datetime = None
        self._time_spent: timedelta = timedelta()
        self._time_remaining: timedelta = timedelta()

    # ==================== 状态属性 ====================
    # 这些属性会被Uni-Lab系统自动识别并定时对外广播

    @property
    def status(self) -> str:
        return self._status
    
    @property
    def current_device(self) -> str:
        """当前设备标识符"""
        return self._current_device

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
    def max_flow_rate(self) -> float:
        return self._max_flow_rate

    @property
    def max_pressure(self) -> float:
        return self._max_pressure
    
        # 添加新的属性访问器
    @property
    def from_vessel(self) -> str:
        return self._from_vessel

    @property
    def to_vessel(self) -> str:
        return self._to_vessel

    @property
    def transfer_volume(self) -> float:
        return self._transfer_volume

    @property
    def amount(self) -> str:
        return self._amount

    @property
    def transfer_time(self) -> float:
        return self._transfer_time

    @property
    def is_viscous(self) -> bool:
        return self._is_viscous

    @property
    def rinsing_solvent(self) -> str:
        return self._rinsing_solvent

    @property
    def rinsing_volume(self) -> float:
        return self._rinsing_volume

    @property
    def rinsing_repeats(self) -> int:
        return self._rinsing_repeats

    @property
    def is_solid(self) -> bool:
        return self._is_solid

    # 修改这两个属性装饰器
    @property
    def time_spent(self) -> float:
        """已用时间（秒）"""
        if isinstance(self._time_spent, timedelta):
            return self._time_spent.total_seconds()
        return float(self._time_spent)

    @property
    def time_remaining(self) -> float:
        """剩余时间（秒）"""
        if isinstance(self._time_remaining, timedelta):
            return self._time_remaining.total_seconds()
        return float(self._time_remaining)

    # ==================== 设备控制方法 ====================
    # 这些方法需要在注册表中添加，会作为ActionServer接受控制指令
    def pump_transfer(self, from_vessel: str, to_vessel: str, volume: float,
                    amount: str = "", time: float = 0.0, viscous: bool = False,
                    rinsing_solvent: str = "", rinsing_volume: float = 0.0,
                    rinsing_repeats: int = 0, solid: bool = False) -> dict:
        """Execute pump transfer operation"""
        # Stop any existing operation first
        self._stop_pump_operation()
        
        # Set transfer parameters
        self._from_vessel = from_vessel
        self._to_vessel = to_vessel
        self._transfer_volume = float(volume)
        self._amount = amount
        self._transfer_time = float(time)
        self._is_viscous = viscous
        self._rinsing_solvent = rinsing_solvent
        self._rinsing_volume = float(rinsing_volume)
        self._rinsing_repeats = int(rinsing_repeats)
        self._is_solid = solid

        # Calculate flow rate
        if self._transfer_time > 0 and self._transfer_volume > 0:
            self._target_flow_rate = (self._transfer_volume / self._transfer_time) * 60.0
        else:
            self._target_flow_rate = 10.0 if not self._is_viscous else 5.0

        # Reset timers and counters
        self._start_time = datetime.now()
        self._time_spent = timedelta()
        self._time_remaining = timedelta(seconds=self._transfer_time)
        self._total_volume = 0.0
        self._flow_rate = 0.0
        
        # Start pump operation
        self._pump_state = "Running"
        self._status = "Starting Transfer"
        self._running = True
        
        # Start pump operation thread
        self._pump_thread = threading.Thread(target=self._pump_operation_loop)
        self._pump_thread.daemon = True
        self._pump_thread.start()

        # Wait briefly to ensure thread starts
        time.sleep(0.1)

        return {
            "success": True,
            "status": self._status,
            "current_device": self._current_device,
            "time_spent": 0.0,
            "time_remaining": float(self._transfer_time)
        }

    def pause_pump(self) -> str:

        if self._pump_state != "Running":
            self._status = "Error: Pump not running"
            return "Error"

        self._pump_state = "Paused"
        self._status = "Pump Paused"
        self._stop_pump_operation()

        return "Success"

    def resume_pump(self) -> str:

        if self._pump_state != "Paused":
            self._status = "Error: Pump not paused"
            return "Error"

        self._pump_state = "Running"
        self._status = "Resuming Pump"
        self._start_pump_operation()

        return "Success"

    def reset_volume_counter(self) -> str:
        self._total_volume = 0.0
        self._status = "Volume counter reset"
        return "Success"

    def emergency_stop(self) -> str:
        self._status = "Emergency Stop"
        self._pump_state = "Stopped"
        self._stop_pump_operation()
        self._flow_rate = 0.0
        self._pressure = 0.0
        self._target_flow_rate = 0.0

        return "Success"

    # ==================== 内部控制方法 ====================

    def _start_pump_operation(self):
        with self._thread_lock:
            if not self._running:
                self._running = True
                self._pump_thread = threading.Thread(target=self._pump_operation_loop)
                self._pump_thread.daemon = True
                self._pump_thread.start()

    def _stop_pump_operation(self):
        with self._thread_lock:
            self._running = False
            if self._pump_thread and self._pump_thread.is_alive():
                self._pump_thread.join(timeout=2.0)

    def _pump_operation_loop(self):
        """泵运行主循环"""
        print("Pump operation loop started")  # Debug print
        
        while self._running and self._pump_state == "Running":
            try:
                # Calculate flow rate adjustment
                flow_diff = self._target_flow_rate - self._flow_rate
                
                # Adjust flow rate more aggressively (50% of difference)
                adjustment = flow_diff * 0.5
                self._flow_rate += adjustment
                
                # Ensure flow rate is within bounds
                self._flow_rate = max(0.1, min(self._max_flow_rate, self._flow_rate))
                
                # Update status based on flow rate
                if abs(flow_diff) < 0.1:
                    self._status = "Running at Target Flow Rate"
                else:
                    self._status = "Adjusting Flow Rate"

                # Calculate volume increment
                volume_increment = (self._flow_rate / 60.0)  # mL/s
                self._total_volume += volume_increment

                # Update time tracking
                self._time_spent = datetime.now() - self._start_time
                if self._transfer_time > 0:
                    remaining = self._transfer_time - self._time_spent.total_seconds()
                    self._time_remaining = timedelta(seconds=max(0, remaining))

                # Check completion
                if self._total_volume >= self._transfer_volume:
                    self._status = "Transfer Completed"
                    self._pump_state = "Stopped"
                    self._running = False
                    break

                # Update pressure
                self._pressure = (self._flow_rate / self._max_flow_rate) * self._max_pressure

                print(f"Debug - Flow: {self._flow_rate:.1f}, Volume: {self._total_volume:.1f}")  # Debug print
                
                time.sleep(1.0)

            except Exception as e:
                print(f"Error in pump operation: {str(e)}")
                self._status = "Error in pump operation"
                self._pump_state = "Stopped"
                self._running = False
                break

    def get_status_info(self) -> dict:
        """
        获取完整的设备状态信息

        Returns:
            dict: 包含所有设备状态的字典
        """
        return {
        "status": self._status,
        "pump_state": self._pump_state,
        "flow_rate": self._flow_rate,
        "target_flow_rate": self._target_flow_rate,
        "pressure": self._pressure,
        "total_volume": self._total_volume,
        "max_flow_rate": self._max_flow_rate,
        "max_pressure": self._max_pressure,
        "current_device": self._current_device,
        "from_vessel": self._from_vessel,
        "to_vessel": self._to_vessel,
        "transfer_volume": self._transfer_volume,
        "amount": self._amount,
        "transfer_time": self._transfer_time,
        "is_viscous": self._is_viscous,
        "rinsing_solvent": self._rinsing_solvent,
        "rinsing_volume": self._rinsing_volume,
        "rinsing_repeats": self._rinsing_repeats,
        "is_solid": self._is_solid,
        "time_spent": self._time_spent.total_seconds(),
        "time_remaining": self._time_remaining.total_seconds()
        }


# 用于测试的主函数
if __name__ == "__main__":
    pump = MockPump()

    # 测试基本功能
    print("启动泵设备测试...")
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


    pump.emergency_stop()
    print("测试完成")
