import time  
import threading  
  
  
class MockStirrer:  
    """  
    模拟搅拌器设备类  
      
    这个类模拟了一个实验室搅拌器的行为，包括搅拌速度控制、  
    温度监测、加热控制等功能。参考了现有的 HeaterStirrer_DaLong 实现。  
    """  
      
    def __init__(self, port: str = "MOCK"):  
        """  
        初始化MockStirrer实例  
          
        Args:  
            port (str): 设备端口，默认为"MOCK"表示模拟设备  
        """  
        self.port = port  
          
        # 设备基本状态属性  
        self._status: str = "Idle"  # 设备状态：Idle, Running, Error, Stopped  
        self._power_state: str = "Off"  # 电源状态：On, Off  
          
        # 搅拌相关属性  
        self._stir_speed: float = 0.0  # 当前搅拌速度 (rpm)  
        self._target_stir_speed: float = 0.0  # 目标搅拌速度 (rpm)  
        self._max_stir_speed: float = 2000.0  # 最大搅拌速度 (rpm)  
        self._stir_state: str = "Stopped"  # 搅拌状态：Running, Stopped  
          
        # 温度相关属性  
        self._temperature: float = 25.0  # 当前温度 (°C)  
        self._target_temperature: float = 25.0  # 目标温度 (°C)  
        self._max_temperature: float = 300.0  # 最大温度 (°C)  
        self._heating_state: str = "Off"  # 加热状态：On, Off  
        self._heating_power: float = 0.0  # 加热功率百分比 0-100  
          
        # 运行控制线程  
        self._operation_thread = None  
        self._running = False  
        self._thread_lock = threading.Lock()  
      
    # ==================== 状态属性 ====================  
    # 这些属性会被Uni-Lab系统自动识别并定时对外广播  
      
    @property  
    def status(self) -> str:  
        """  
        设备状态 - 会被自动识别的设备属性  
          
        Returns:  
            str: 当前设备状态 (Idle, Running, Error, Stopped)  
        """  
        return self._status  
      
    @property  
    def power_state(self) -> str:  
        """  
        电源状态  
          
        Returns:  
            str: 电源状态 (On, Off)  
        """  
        return self._power_state  
      
    @property  
    def stir_speed(self) -> float:  
        """  
        当前搅拌速度  
          
        Returns:  
            float: 当前搅拌速度 (rpm)  
        """  
        return self._stir_speed  
      
    @property  
    def target_stir_speed(self) -> float:  
        """  
        目标搅拌速度  
          
        Returns:  
            float: 目标搅拌速度 (rpm)  
        """  
        return self._target_stir_speed  
      
    @property  
    def stir_state(self) -> str:  
        """  
        搅拌状态  
          
        Returns:  
            str: 搅拌状态 (Running, Stopped)  
        """  
        return self._stir_state  
      
    @property  
    def temperature(self) -> float:  
        """  
        当前温度  
          
        Returns:  
            float: 当前温度 (°C)  
        """  
        return self._temperature  
      
    @property  
    def target_temperature(self) -> float:  
        """  
        目标温度  
          
        Returns:  
            float: 目标温度 (°C)  
        """  
        return self._target_temperature  
      
    @property  
    def heating_state(self) -> str:  
        """  
        加热状态  
          
        Returns:  
            str: 加热状态 (On, Off)  
        """  
        return self._heating_state  
      
    @property  
    def heating_power(self) -> float:  
        """  
        加热功率  
          
        Returns:  
            float: 加热功率百分比 (0-100)  
        """  
        return self._heating_power  
      
    @property  
    def max_stir_speed(self) -> float:  
        """  
        最大搅拌速度  
          
        Returns:  
            float: 最大搅拌速度 (rpm)  
        """  
        return self._max_stir_speed  
      
    @property  
    def max_temperature(self) -> float:  
        """  
        最大温度  
          
        Returns:  
            float: 最大温度 (°C)  
        """  
        return self._max_temperature  
      
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
            self._start_operation()  
        else:  
            self._status = "Power Off"  
            self.stop_all_operations()  
              
        return "Success"  
      
    def set_stir_speed(self, speed: float) -> str:  
        """  
        设置搅拌速度  
          
        Args:  
            speed (float): 目标搅拌速度 (rpm)  
          
        Returns:  
            str: 操作结果状态 ("Success", "Error")  
        """  
        speed = float(speed)  # 确保传入的速度是浮点数
        if self._power_state != "On":  
            self._status = "Error: Power Off"  
            return "Error"  
          
        if speed < 0 or speed > self._max_stir_speed:  
            self._status = f"Error: Speed out of range (0-{self._max_stir_speed})"  
            return "Error"  
          
        self._target_stir_speed = speed  
        self._status = "Setting Stir Speed"  
          
        # 如果设置了非零速度，启动搅拌  
        if speed > 0:  
            self._stir_state = "Running"  
        else:  
            self._stir_state = "Stopped"  
              
        return "Success"  
      
    def set_temperature(self, temperature: float) -> str:  
        """  
        设置目标温度  
          
        Args:  
            temperature (float): 目标温度 (°C)  
          
        Returns:  
            str: 操作结果状态 ("Success", "Error")  
        """  
        temperature = float(temperature)  # 确保传入的温度是浮点数
        if self._power_state != "On":  
            self._status = "Error: Power Off"  
            return "Error"  
          
        if temperature < 0 or temperature > self._max_temperature:  
            self._status = f"Error: Temperature out of range (0-{self._max_temperature})"  
            return "Error"  
          
        self._target_temperature = temperature  
        self._status = "Setting Temperature"  
          
        return "Success"  
      
    def start_stirring(self) -> str:  
        """  
        启动搅拌  
          
        Returns:  
            str: 操作结果状态 ("Success", "Error")  
        """  
        if self._power_state != "On":  
            self._status = "Error: Power Off"  
            return "Error"  
          
        if self._target_stir_speed <= 0:  
            self._status = "Error: No target speed set"  
            return "Error"  
          
        self._stir_state = "Running"  
        self._status = "Stirring Started"  
        return "Success"  
      
    def stop_stirring(self) -> str:  
        """  
        停止搅拌  
          
        Returns:  
            str: 操作结果状态 ("Success", "Error")  
        """  
        self._stir_state = "Stopped"  
        self._target_stir_speed = 0.0  
        self._status = "Stirring Stopped"  
        return "Success"  
      
    def heating_control(self, heating_state: str = "On") -> str:  
        """  
        加热控制  
          
        Args:  
            heating_state (str): 加热状态，可选值："On", "Off"  
          
        Returns:  
            str: 操作结果状态 ("Success", "Error")  
        """  
        if self._power_state != "On":  
            self._status = "Error: Power Off"  
            return "Error"  
          
        if heating_state not in ["On", "Off"]:  
            self._status = "Error: Invalid heating state"  
            return "Error"  
          
        self._heating_state = heating_state  
          
        if heating_state == "On":  
            self._status = "Heating On"  
        else:  
            self._status = "Heating Off"  
            self._heating_power = 0.0  
              
        return "Success"  
      
    def stop_all_operations(self) -> str:  
        """  
        停止所有操作  
          
        Returns:  
            str: 操作结果状态 ("Success", "Error")  
        """  
        self._stir_state = "Stopped"  
        self._heating_state = "Off"  
        self._stop_operation()  
        self._stir_speed = 0.0  
        self._target_stir_speed = 0.0  
        self._heating_power = 0.0  
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
          
        这个方法启动一个后台线程来模拟搅拌器的实际运行过程。  
        """  
        with self._thread_lock:  
            if not self._running and self._power_state == "On":  
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
          
        这个方法在后台线程中运行，模拟真实搅拌器的工作过程：  
        1. 搅拌速度控制  
        2. 温度控制和加热  
        3. 状态更新  
        """  
        while self._running and self._power_state == "On":  
            try:  
                # 处理搅拌速度控制  
                if self._stir_state == "Running":  
                    speed_diff = self._target_stir_speed - self._stir_speed  
                      
                    if abs(speed_diff) < 1.0:  # 速度接近目标值  
                        self._stir_speed = self._target_stir_speed  
                        if self._stir_speed > 0:  
                            self._status = "Stirring at Target Speed"  
                    else:  
                        # 模拟速度调节，每秒调整10%的差值  
                        adjustment = speed_diff * 0.1  
                        self._stir_speed += adjustment  
                        self._status = "Adjusting Stir Speed"  
                      
                    # 确保速度在合理范围内  
                    self._stir_speed = max(0.0, min(self._max_stir_speed, self._stir_speed))  
                else:  
                    # 搅拌停止时，速度逐渐降为0  
                    if self._stir_speed > 0:  
                        self._stir_speed = max(0.0, self._stir_speed - 50.0)  # 每秒减少50rpm  
                  
                # 处理温度控制  
                if self._heating_state == "On":  
                    temp_diff = self._target_temperature - self._temperature  
                      
                    if abs(temp_diff) < 0.5:  # 温度接近目标值  
                        self._heating_power = 20.0  # 维持温度的最小功率  
                    elif temp_diff > 0:  # 需要加热  
                        # 根据温差调整加热功率  
                        if temp_diff > 50:  
                            self._heating_power = 100.0  
                        elif temp_diff > 20:  
                            self._heating_power = 80.0  
                        elif temp_diff > 10:  
                            self._heating_power = 60.0  
                        else:  
                            self._heating_power = 40.0  
                          
                        # 模拟加热过程  
                        heating_rate = self._heating_power / 100.0 * 1.5  # 最大每秒升温1.5度  
                        self._temperature += heating_rate  
                    else:  # 目标温度低于当前温度  
                        self._heating_power = 0.0  
                        # 自然冷却  
                        self._temperature -= 0.1  
                else:  
                    self._heating_power = 0.0  
                    # 自然冷却到室温  
                    if self._temperature > 25.0:  
                        self._temperature -= 0.2  
                  
                # 限制温度范围  
                self._temperature = max(20.0, min(self._max_temperature, self._temperature))  
                  
                # 更新整体状态
                if self._stir_state == "Running" and self._heating_state == "On":  
                    self._status = "Stirring and Heating"  
                elif self._stir_state == "Running":  
                    self._status = "Stirring Only"  
                elif self._heating_state == "On":  
                    self._status = "Heating Only"  
                else:  
                    self._status = "Idle"  
                  
                # 等待1秒后继续下一次循环  
                time.sleep(1.0)  
                  
            except Exception as e:  
                self._status = f"Error in operation: {str(e)}"  
                break  
          
        # 循环结束时的清理工作  
        if self._power_state == "On":  
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
            "stir_speed": self._stir_speed,  
            "target_stir_speed": self._target_stir_speed,  
            "stir_state": self._stir_state,  
            "temperature": self._temperature,  
            "target_temperature": self._target_temperature,  
            "heating_state": self._heating_state,  
            "heating_power": self._heating_power,  
            "max_stir_speed": self._max_stir_speed,  
            "max_temperature": self._max_temperature  
        }  
  
  
# 用于测试的主函数  
if __name__ == "__main__":  
    stirrer = MockStirrer()  
      
    # 测试基本功能  
    print("启动搅拌器测试...")  
    stirrer.power_control("On")  
    print(f"初始状态: {stirrer.get_status_info()}")  
      
    # 设置搅拌速度和温度  
    stirrer.set_stir_speed(800.0)  
    stirrer.set_temperature(60.0)  
    stirrer.heating_control("On")  
      
    # 模拟运行15秒  
    for i in range(15):  
        time.sleep(1)  
        print(f"第{i+1}秒: 速度={stirrer.stir_speed:.0f}rpm, 温度={stirrer.temperature:.1f}°C, 功率={stirrer.heating_power:.1f}%, 状态={stirrer.status}")  
      
    stirrer.emergency_stop()  
    print("测试完成")  