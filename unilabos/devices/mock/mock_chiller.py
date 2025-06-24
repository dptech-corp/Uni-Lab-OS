import time
import threading


class MockChiller:
    def __init__(self, port: str = "MOCK"):
        self.port = port
        self._current_temperature: float = 25.0  # 室温开始
        self._target_temperature: float = 25.0
        self._status: str = "Idle"
        self._is_cooling: bool = False
        self._is_heating: bool = False
        self._vessel = "Unknown"
        self._purpose = "Unknown"

        # 模拟温度变化的线程
        self._temperature_thread = None
        self._running = True
        self._temperature_thread = threading.Thread(target=self._temperature_control_loop)
        self._temperature_thread.daemon = True
        self._temperature_thread.start()

    @property
    def current_temperature(self) -> float:
        """当前温度 - 会被自动识别的设备属性"""
        return self._current_temperature

    @property
    def target_temperature(self) -> float:
        """目标温度"""
        return self._target_temperature

    @property
    def status(self) -> str:
        """设备状态 - 会被自动识别的设备属性"""
        return self._status

    @property
    def is_cooling(self) -> bool:
        """是否正在冷却"""
        return self._is_cooling

    @property
    def is_heating(self) -> bool:
        """是否正在加热"""
        return self._is_heating

    @property
    def vessel(self) -> str:
        """当前操作的容器名称"""
        return self._vessel

    @property
    def purpose(self) -> str:
        """当前操作目的"""
        return self._purpose

    def heat_chill_start(self, vessel: str, temp: float, purpose: str):
        """设置目标温度并记录容器和目的"""
        self._vessel = str(vessel)
        self._purpose = str(purpose)
        self._target_temperature = float(temp)

        diff = self._target_temperature - self._current_temperature
        if abs(diff) < 0.1:
            self._status = "At Target Temperature"
            self._is_cooling = False
            self._is_heating = False
        elif diff < 0:
            self._status = "Cooling"
            self._is_cooling = True
            self._is_heating = False
        else:
            self._status = "Heating"
            self._is_heating = True
            self._is_cooling = False

        self._start_temperature_control()
        return True
    
    def heat_chill_stop(self, vessel: str):
        """停止加热/制冷"""
        if vessel != self._vessel:
            return {"success": False, "status": f"Wrong vessel: expected {self._vessel}, got {vessel}"}
            
        # 停止温度控制线程，锁定当前温度
        self._stop_temperature_control()
        
        # 更新状态
        self._status = "Stopped"
        self._is_cooling = False
        self._is_heating = False
        
        # 重新启动线程但保持温度
        self._running = True
        self._temperature_thread = threading.Thread(target=self._temperature_control_loop)
        self._temperature_thread.daemon = True
        self._temperature_thread.start()
        
        return {"success": True, "status": self._status}

    def _start_temperature_control(self):
        """启动温度控制线程"""
        self._running = True
        if self._temperature_thread is None or not self._temperature_thread.is_alive():
            self._temperature_thread = threading.Thread(target=self._temperature_control_loop)
            self._temperature_thread.daemon = True
            self._temperature_thread.start()

    def _stop_temperature_control(self):
        """停止温度控制"""
        self._running = False
        if self._temperature_thread:
            self._temperature_thread.join(timeout=1.0)

    def _temperature_control_loop(self):
        """温度控制循环 - 模拟真实冷却器的温度变化"""
        while self._running:
            # 如果状态是 Stopped，不改变温度
            if self._status == "Stopped":
                time.sleep(1.0)
                continue
                
            temp_diff = self._target_temperature - self._current_temperature

            if abs(temp_diff) < 0.1:
                self._status = "At Target Temperature"
                self._is_cooling = False
                self._is_heating = False
            elif temp_diff < 0:
                self._status = "Cooling"
                self._is_cooling = True
                self._is_heating = False
                self._current_temperature -= 0.5
            else:
                self._status = "Heating"
                self._is_heating = True
                self._is_cooling = False
                self._current_temperature += 0.3

            time.sleep(1.0)

    def emergency_stop(self):
        """紧急停止"""
        self._status = "Emergency Stop"
        self._stop_temperature_control()
        self._is_cooling = False
        self._is_heating = False

    def get_status_info(self) -> dict:
        """获取完整状态信息"""
        return {
            "current_temperature": self._current_temperature,
            "target_temperature": self._target_temperature,
            "status": self._status,
            "is_cooling": self._is_cooling,
            "is_heating": self._is_heating,
            "vessel": self._vessel,
            "purpose": self._purpose,
        }


# 用于测试的主函数
if __name__ == "__main__":
    chiller = MockChiller()

    # 测试基本功能
    print("启动冷却器测试...")
    print(f"初始状态: {chiller.get_status_info()}")

    # 模拟运行10秒
    for i in range(10):
        time.sleep(1)
        print(f"第{i+1}秒: 当前温度={chiller.current_temperature:.1f}°C, 状态={chiller.status}")

    chiller.emergency_stop()
    print("测试完成")
