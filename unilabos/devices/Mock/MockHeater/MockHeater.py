import time  
import threading  
  
  
class MockHeater:  
    def __init__(self, port: str = "MOCK"):  
        self.port = port  
        self._current_temperature: float = 25.0  # 室温开始  
        self._target_temperature: float = 25.0  
        self._status: str = "Idle"  
        self._is_heating: bool = False  
        self._power_on: bool = False  
        self._heating_power: float = 0.0  # 加热功率百分比 0-100  
        self._max_temperature: float = 300.0  # 最大加热温度  
          
        # 模拟加热过程的线程  
        self._heating_thread = None  
        self._running = False  
      
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
    def power_on(self) -> bool:  
        """电源状态"""  
        return self._power_on  
      
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
      
    def set_temperature(self, temperature: float):
        """设置目标温度 - 需要在注册表添加的设备动作"""
        try:
            temperature = float(temperature)
        except ValueError:
            self._status = "Error: Invalid temperature value"
            return False

        if not self._power_on:
            self._status = "Error: Power Off"
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

        if not self._power_on:
            self._status = "Error: Power Off"
            return False

        self._heating_power = max(0.0, min(100.0, power))  # 限制在0-100%
        return True
      
    def power_on_off(self, power_state: str):
        """开关机控制，接收字符串命令 "On" 或 "Off" """
        power_state = power_state.capitalize()
        if power_state not in ["On", "Off"]:
            self._status = "Error: Invalid power state"
            return "Error"
        self._power_on = True if power_state == "On" else False
        if self._power_on:
            self._status = "Power On"
            self._start_heating_control()
        else:
            self._status = "Power Off"
            self._stop_heating_control()
            self._is_heating = False
            self._heating_power = 0.0
        return "Success"
      
    def _start_heating_control(self):  
        """启动加热控制线程"""  
        if not self._running and self._power_on:  
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
        """加热控制循环 - 模拟真实加热器的温度变化"""  
        while self._running and self._power_on:  
            temp_diff = self._target_temperature - self._current_temperature  
              
            if abs(temp_diff) < 0.5:  # 温度接近目标值  
                self._status = "At Target Temperature"  
                self._is_heating = False  
                self._heating_power = 10.0  # 维持温度的最小功率  
            elif temp_diff > 0:  # 需要加热  
                self._status = "Heating"  
                self._is_heating = True  
                # 根据温差调整加热功率  
                if temp_diff > 50:  
                    self._heating_power = 100.0  
                elif temp_diff > 20:  
                    self._heating_power = 80.0  
                elif temp_diff > 10:  
                    self._heating_power = 60.0  
                else:  
                    self._heating_power = 40.0  
                  
                # 模拟加热过程，加热速度与功率成正比  
                heating_rate = self._heating_power / 100.0 * 2.0  # 最大每秒升温2度  
                self._current_temperature += heating_rate  
            else:  # 目标温度低于当前温度，自然冷却  
                self._status = "Cooling Down"  
                self._is_heating = False  
                self._heating_power = 0.0  
                # 模拟自然冷却，每秒降低0.2度  
                self._current_temperature -= 0.2  
              
            # 限制温度范围  
            self._current_temperature = max(20.0, min(self._max_temperature, self._current_temperature))  
              
            time.sleep(1.0)  # 每秒更新一次  
      
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
            "power_on": self._power_on,  
            "is_heating": self._is_heating,  
            "heating_power": self._heating_power,  
            "max_temperature": self._max_temperature  
        }  
  
  
# 用于测试的主函数  
if __name__ == "__main__":  
    heater = MockHeater()  
      
    # 测试基本功能  
    print("启动加热器测试...")  
    heater.power_on_off(True)  
    print(f"初始状态: {heater.get_status_info()}")  
      
    # 设置目标温度为80度  
    heater.set_temperature(80.0)  
      
    # 模拟运行15秒  
    for i in range(15):  
        time.sleep(1)  
        print(f"第{i+1}秒: 当前温度={heater.current_temperature:.1f}°C, 功率={heater.heating_power:.1f}%, 状态={heater.status}")  
      
    heater.emergency_stop()  
    print("测试完成")