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
        self._power_on: bool = False  
          
        # 模拟温度变化的线程  
        self._temperature_thread = None  
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
    def is_cooling(self) -> bool:  
        """是否正在冷却"""  
        return self._is_cooling  
      
    @property  
    def is_heating(self) -> bool:  
        """是否正在加热"""  
        return self._is_heating  
      
    def set_temperature(self, temperature: float):
        """设置目标温度 - 需要在注册表添加的设备动作"""
        if not self._power_on:
            self._status = "Error: Power Off"
            return False

        # 将传入温度转换为 float，并限制在允许范围内
        temperature = float(temperature)
        self._target_temperature = temperature

        # 立即更新状态
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

        # 启动温度控制
        self._start_temperature_control()
        return True
      
    def power_on_off(self, power_state: str):
        """开关机控制"""
        if power_state == "on":
            self._power_on = True
            # 不在这里直接设置状态和加热/制冷标志
            self._start_temperature_control()
        else:
            self._power_on = False
            self._status = "Power Off"
            self._stop_temperature_control()
            self._is_cooling = False
            self._is_heating = False
      
    def _start_temperature_control(self):  
        """启动温度控制线程"""
        if self._power_on:  # 移除 not self._running 检查
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
        while self._running and self._power_on:
            temp_diff = self._target_temperature - self._current_temperature
            
            if abs(temp_diff) < 0.1:  # 将判断范围从0.5改小到0.1
                self._status = "At Target Temperature"
                self._is_cooling = False
                self._is_heating = False
            elif temp_diff < 0:  # 需要冷却
                self._status = "Cooling"
                self._is_cooling = True
                self._is_heating = False
                # 模拟冷却过程，每秒降低0.5度
                self._current_temperature -= 0.5
            else:  # 需要加热
                self._status = "Heating"
                self._is_heating = True
                self._is_cooling = False
                # 模拟加热过程，每秒升高0.3度
                self._current_temperature += 0.3
            
            # 限制温度范围
            self._current_temperature = max(-20.0, min(80.0, self._current_temperature))
            
            time.sleep(1.0)  # 每秒更新一次
      
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
            "power_on": self._power_on,  
            "is_cooling": self._is_cooling,  
            "is_heating": self._is_heating  
        }  
  
  
# 用于测试的主函数  
if __name__ == "__main__":  
    chiller = MockChiller()  
      
    # 测试基本功能  
    print("启动冷却器测试...")  
    chiller.power_on_off("on")  
    print(f"初始状态: {chiller.get_status_info()}")  
      
    # 设置目标温度为5度  
    chiller.set_temperature(5.0)  
      
    # 模拟运行10秒  
    for i in range(10):  
        time.sleep(1)  
        print(f"第{i+1}秒: 当前温度={chiller.current_temperature:.1f}°C, 状态={chiller.status}")  
      
    chiller.emergency_stop()  
    print("测试完成")