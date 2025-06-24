import time
import threading
from datetime import datetime, timedelta

class MockStirrer_new:
    def __init__(self, port: str = "MOCK"):
        self.port = port

        # 基本状态属性
        self._status: str = "Idle"
        self._vessel: str = ""
        self._purpose: str = ""

        # 搅拌相关属性
        self._stir_speed: float = 0.0
        self._target_stir_speed: float = 0.0
        self._max_stir_speed: float = 2000.0
        self._stir_state: str = "Stopped"
        
        # 计时相关
        self._stir_time: float = 0.0
        self._settling_time: float = 0.0
        self._start_time = datetime.now()
        self._time_remaining = timedelta()
        
        # 运行控制
        self._operation_thread = None
        self._running = False
        self._thread_lock = threading.Lock()
        
        # 创建操作线程
        self._operation_thread = threading.Thread(target=self._operation_loop)
        self._operation_thread.daemon = True
        self._operation_thread.start()

    # ==================== 状态属性 ====================
    @property
    def status(self) -> str:
        return self._status

    @property
    def stir_speed(self) -> float:
        return self._stir_speed

    @property
    def target_stir_speed(self) -> float:
        return self._target_stir_speed

    @property
    def stir_state(self) -> str:
        return self._stir_state
    
    @property
    def vessel(self) -> str:
        return self._vessel

    @property
    def purpose(self) -> str:
        return self._purpose

    @property
    def stir_time(self) -> float:
        return self._stir_time

    @property
    def settling_time(self) -> float:
        return self._settling_time

    @property
    def max_stir_speed(self) -> float:
        return self._max_stir_speed
        
    @property
    def progress(self) -> float:
        """返回当前操作的进度（0-100）"""
        if not self._running:
            return 0.0
        elapsed = (datetime.now() - self._start_time).total_seconds()
        total_time = self._stir_time + self._settling_time
        if total_time <= 0:
            return 100.0
        return min(100.0, (elapsed / total_time) * 100)

    # ==================== Action Server 方法 ====================
    def start_stir(self, vessel: str, stir_speed: float = 0.0, purpose: str = "") -> dict:
        """
        StartStir.action 对应的方法
        """
        with self._thread_lock:
            if self._running:
                return {
                    "success": False,
                    "message": "Operation already in progress"
                }
            
            try:
                # 重置所有参数
                self._vessel = vessel
                self._purpose = purpose
                self._stir_time = 0.0  # 连续搅拌模式下不设置搅拌时间
                self._settling_time = 0.0
                self._start_time = datetime.now()  # 重置开始时间
                
                if stir_speed > 0:
                    self._target_stir_speed = min(stir_speed, self._max_stir_speed)
                
                self._stir_state = "Running"
                self._status = "Stirring Started"
                self._running = True
                
                return {
                    "success": True,
                    "message": "Stirring started successfully"
                }
                
            except Exception as e:
                return {
                    "success": False,
                    "message": f"Error: {str(e)}"
                }

    def stir(self, stir_time: float, stir_speed: float, settling_time: float) -> dict:
        """
        Stir.action 对应的方法
        """
        with self._thread_lock:
            try:
                # 如果已经在运行，先停止当前操作
                if self._running:
                    self._running = False
                    self._stir_state = "Stopped"
                    self._target_stir_speed = 0.0
                    time.sleep(0.1)  # 给一个短暂的停止时间
            
        
                # 重置所有参数
                self._stir_time = float(stir_time)
                self._settling_time = float(settling_time)
                self._target_stir_speed = min(float(stir_speed), self._max_stir_speed)
                self._start_time = datetime.now()  # 重置开始时间
                self._stir_state = "Running"
                self._status = "Stirring"
                self._running = True
                
                return {"success": True}
                
            except ValueError:
                self._status = "Error: Invalid parameters"
                return {"success": False}

    def stop_stir(self, vessel: str) -> dict:
        """
        StopStir.action 对应的方法
        """
        with self._thread_lock:
            if vessel != self._vessel:
                return {
                    "success": False,
                    "message": "Vessel mismatch"
                }
            
            self._running = False
            self._stir_state = "Stopped"
            self._target_stir_speed = 0.0
            self._status = "Stirring Stopped"
            
            return {
                "success": True,
                "message": "Stirring stopped successfully"
            }

    # ==================== 内部控制方法 ====================

    def _operation_loop(self):
        """操作主循环"""
        while True:
            try:
                current_time = datetime.now()
                
                with self._thread_lock:  # 添加锁保护
                    if self._stir_state == "Running":
                        # 实际搅拌逻辑
                        speed_diff = self._target_stir_speed - self._stir_speed
                        if abs(speed_diff) > 0.1:
                            adjustment = speed_diff * 0.1
                            self._stir_speed += adjustment
                        else:
                            self._stir_speed = self._target_stir_speed
                        
                        # 更新进度
                        if self._running:
                            if self._stir_time > 0:  # 定时搅拌模式
                                elapsed = (current_time - self._start_time).total_seconds()
                                if elapsed >= self._stir_time + self._settling_time:
                                    self._running = False
                                    self._stir_state = "Stopped"
                                    self._target_stir_speed = 0.0
                                    self._stir_speed = 0.0
                                    self._status = "Stirring Complete"
                                elif elapsed >= self._stir_time:
                                    self._status = "Settling"
                            else:  # 连续搅拌模式
                                self._status = "Stirring"
                    else:
                        # 停止状态下慢慢降低速度
                        if self._stir_speed > 0:
                            self._stir_speed = max(0, self._stir_speed - 20.0)
                
                time.sleep(0.1)
                        
            except Exception as e:
                print(f"Error in operation loop: {str(e)}")  # 添加错误输出
                self._status = f"Error: {str(e)}"
                time.sleep(1.0)  # 错误发生时等待较长时间

    def get_status_info(self) -> dict:
        """获取设备状态信息"""
        return {
            "status": self._status,
            "vessel": self._vessel,
            "purpose": self._purpose,
            "stir_speed": self._stir_speed,
            "target_stir_speed": self._target_stir_speed,
            "stir_state": self._stir_state,
            "stir_time": self._stir_time,         # 添加
            "settling_time": self._settling_time,  # 添加
            "progress": self.progress,
            "max_stir_speed": self._max_stir_speed
        }