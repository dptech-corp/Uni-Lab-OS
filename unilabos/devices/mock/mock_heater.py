import time
import threading

class MockHeater:
    def __init__(self, port: str = "MOCK"):
        self.port = port
        self._current_temperature: float = 25.0  # 室温开始
        self._target_temperature: float = 25.0
        self._status: str = "Idle"
        self._is_heating: bool = False
        self._heating_power: float = 0.0  # 加热功率百分比 0-100
        self._max_temperature: float = 300.0  # 最大加热温度
        
        # 新增加的属性
        self._vessel: str = "Unknown"
        self._purpose: str = "Unknown"
        self._stir: bool = False
        self._stir_speed: float = 0.0

        # 模拟加热过程的线程
        self._heating_thread = None
        self._running = True
        self._heating_thread = threading.Thread(target=self._heating_control_loop)
        self._heating_thread.daemon = True
        self._heating_thread.start()

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
    def is_heating(self) -> bool:
        """是否正在加热"""
        return self._is_heating

    @property
    def heating_power(self) -> float:
        """加热功率百分比"""
        return self._heating_power

    @property
    def max_temperature(self) -> float:
        """最大加热温度"""
        return self._max_temperature
    
    @property
    def vessel(self) -> str:
        """当前操作的容器名称"""
        return self._vessel

    @property
    def purpose(self) -> str:
        """操作目的"""
        return self._purpose

    @property
    def stir(self) -> bool:
        """是否搅拌"""
        return self._stir

    @property
    def stir_speed(self) -> float:
        """搅拌速度"""
        return self._stir_speed

    def heat_chill_start(self, vessel: str, temp: float, purpose: str) -> dict:
        """开始加热/制冷过程"""
        self._vessel = str(vessel)
        self._purpose = str(purpose)
        self._target_temperature = float(temp)

        diff = self._target_temperature - self._current_temperature
        if abs(diff) < 0.1:
            self._status = "At Target Temperature"
            self._is_heating = False
        elif diff > 0:
            self._status = "Heating"
            self._is_heating = True
        else:
            self._status = "Cooling Down"
            self._is_heating = False

        return {"success": True, "status": self._status}

    def heat_chill_stop(self, vessel: str) -> dict:
        """停止加热/制冷"""
        if vessel != self._vessel:
            return {"success": False, "status": f"Wrong vessel: expected {self._vessel}, got {vessel}"}
            
        self._status = "Stopped"
        self._is_heating = False
        self._heating_power = 0.0
        
        return {"success": True, "status": self._status}

    def heat_chill(self, vessel: str, temp: float, time: float, 
                  stir: bool = False, stir_speed: float = 0.0, 
                  purpose: str = "Unknown") -> dict:
        """完整的加热/制冷控制"""
        self._vessel = str(vessel)
        self._target_temperature = float(temp)
        self._purpose = str(purpose)
        self._stir = stir
        self._stir_speed = stir_speed

        diff = self._target_temperature - self._current_temperature
        if abs(diff) < 0.1:
            self._status = "At Target Temperature"
            self._is_heating = False
        elif diff > 0:
            self._status = "Heating"
            self._is_heating = True
        else:
            self._status = "Cooling Down"
            self._is_heating = False

        return {"success": True, "status": self._status}

    def set_temperature(self, temperature: float):
        """设置目标温度 - 需要在注册表添加的设备动作"""
        try:
            temperature = float(temperature)
        except ValueError:
            self._status = "Error: Invalid temperature value"
            return False

        if temperature > self._max_temperature:
            self._status = f"Error: Temperature exceeds maximum ({self._max_temperature}°C)"
            return False

        self._target_temperature = temperature
        self._status = "Setting Temperature"

        # 启动加热控制
        self._start_heating_control()
        return True

    def set_heating_power(self, power: float):
        """设置加热功率"""
        try:
            power = float(power)
        except ValueError:
            self._status = "Error: Invalid power value"
            return False

        self._heating_power = max(0.0, min(100.0, power))  # 限制在0-100%
        return True

    def _start_heating_control(self):
        """启动加热控制线程"""
        if not self._running:
            self._running = True
            self._heating_thread = threading.Thread(target=self._heating_control_loop)
            self._heating_thread.daemon = True
            self._heating_thread.start()

    def _stop_heating_control(self):
        """停止加热控制"""
        self._running = False
        if self._heating_thread:
            self._heating_thread.join(timeout=1.0)

    def _heating_control_loop(self):
        """加热控制循环"""
        while self._running:
            # 如果状态是 Stopped，不改变温度
            if self._status == "Stopped":
                time.sleep(1.0)
                continue
                
            temp_diff = self._target_temperature - self._current_temperature

            if abs(temp_diff) < 0.1:
                self._status = "At Target Temperature"
                self._is_heating = False
                self._heating_power = 10.0
            elif temp_diff > 0:
                self._status = "Heating"
                self._is_heating = True
                self._heating_power = min(100.0, abs(temp_diff) * 2)
                self._current_temperature += 0.5
            else:
                self._status = "Cooling Down"
                self._is_heating = False
                self._heating_power = 0.0
                self._current_temperature -= 0.2

            time.sleep(1.0)

    def emergency_stop(self):
        """紧急停止"""
        self._status = "Emergency Stop"
        self._stop_heating_control()
        self._is_heating = False
        self._heating_power = 0.0

    def get_status_info(self) -> dict:
        """获取完整状态信息"""
        return {
            "current_temperature": self._current_temperature,
            "target_temperature": self._target_temperature,
            "status": self._status,
            "is_heating": self._is_heating,
            "heating_power": self._heating_power,
            "max_temperature": self._max_temperature,
            "vessel": self._vessel,
            "purpose": self._purpose,
            "stir": self._stir,
            "stir_speed": self._stir_speed
        }

# 用于测试的主函数
if __name__ == "__main__":
    heater = MockHeater()

    print("启动加热器测试...")
    print(f"初始状态: {heater.get_status_info()}")

    # 设置目标温度为80度
    heater.set_temperature(80.0)

    # 模拟运行15秒
    try:
        for i in range(15):
            time.sleep(1)
            status = heater.get_status_info()
            print(
                f"\r温度: {status['current_temperature']:.1f}°C / {status['target_temperature']:.1f}°C | "
                f"功率: {status['heating_power']:.1f}% | 状态: {status['status']}",
                end=""
            )
    except KeyboardInterrupt:
        heater.emergency_stop()
        print("\n测试被手动停止")

    print("\n测试完成")