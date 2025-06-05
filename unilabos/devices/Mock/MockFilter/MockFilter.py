import time  
import threading  
  
  
class MockFilter:  
    def __init__(self, port: str = "MOCK"):  
        self.port = port  
        self._status: str = "Idle"  
        self._is_filtering: bool = False  
        self._filter_efficiency: float = 95.0  # 过滤效率百分比  
        self._flow_rate: float = 0.0  # 流速 L/min  
        self._pressure_drop: float = 0.0  # 压降 Pa  
        self._filter_life: float = 100.0  # 滤芯寿命百分比  
        self._power_on: bool = False  
          
        # 模拟过滤过程的线程  
        self._filter_thread = None  
        self._running = False  
      
    @property  
    def status(self) -> str:  
        """设备状态 - 会被自动识别的设备属性"""  
        return self._status  
      
    @property  
    def is_filtering(self) -> bool:  
        """是否正在过滤"""  
        return self._is_filtering  
      
    @property  
    def filter_efficiency(self) -> float:  
        """过滤效率"""  
        return self._filter_efficiency  
      
    @property  
    def flow_rate(self) -> float:  
        """流速"""  
        return self._flow_rate  
      
    @property  
    def pressure_drop(self) -> float:  
        """压降"""  
        return self._pressure_drop  
      
    @property  
    def filter_life(self) -> float:  
        """滤芯寿命"""  
        return self._filter_life  
      
    @property  
    def power_on(self) -> bool:  
        """电源状态"""  
        return self._power_on  
      
    def start_filtering(self, flow_rate: float = 1.0):  
        """开始过滤 - 需要在注册表添加的设备动作"""  
        if not self._power_on:  
            self._status = "Error: Power Off"  
            return False  
              
        self._flow_rate = flow_rate  
        self._status = "Starting Filter"  
        self._start_filter_process()  
        return True  
      
    def stop_filtering(self):  
        """停止过滤"""  
        self._status = "Stopping Filter"  
        self._stop_filter_process()  
        self._flow_rate = 0.0  
        self._is_filtering = False  
        self._status = "Idle"  
        return True  
      
    def power_on_off(self, power_state: str):  
        """开关机控制"""  
        if power_state == "on":
            self._power_on = True
            self._status = "Power On" 
        else:  
            self._power_on = False
            self._status = "Power Off"  
            self._stop_filter_process()  
            self._is_filtering = False  
            self._flow_rate = 0.0  
      
    def replace_filter(self):  
        """更换滤芯"""  
        self._filter_life = 100.0  
        self._filter_efficiency = 95.0  
        self._status = "Filter Replaced"  
        return True  
      
    def _start_filter_process(self):  
        """启动过滤过程线程"""  
        if not self._running and self._power_on:  
            self._running = True  
            self._is_filtering = True  
            self._filter_thread = threading.Thread(target=self._filter_loop)  
            self._filter_thread.daemon = True  
            self._filter_thread.start()  
      
    def _stop_filter_process(self):  
        """停止过滤过程"""  
        self._running = False  
        if self._filter_thread:  
            self._filter_thread.join(timeout=1.0)  
      
    def _filter_loop(self):  
        """过滤过程循环 - 模拟真实过滤器的工作过程"""  
        while self._running and self._power_on and self._is_filtering:  
            self._status = "Filtering"  
              
            # 模拟滤芯磨损  
            if self._filter_life > 0:  
                self._filter_life -= 0.1  # 每秒减少0.1%寿命  
                  
            # 根据滤芯寿命调整效率和压降  
            life_factor = self._filter_life / 100.0  
            self._filter_efficiency = 95.0 * life_factor + 50.0 * (1 - life_factor)  
            self._pressure_drop = 100.0 + (200.0 * (1 - life_factor))  # 压降随磨损增加  
              
            # 检查滤芯是否需要更换  
            if self._filter_life <= 10.0:  
                self._status = "Filter Needs Replacement"  
              
            time.sleep(1.0)  # 每秒更新一次  
      
    def emergency_stop(self):  
        """紧急停止"""  
        self._status = "Emergency Stop"  
        self._stop_filter_process()  
        self._is_filtering = False  
        self._flow_rate = 0.0  
      
    def get_status_info(self) -> dict:  
        """获取完整状态信息"""  
        return {  
            "status": self._status,  
            "is_filtering": self._is_filtering,  
            "filter_efficiency": self._filter_efficiency,  
            "flow_rate": self._flow_rate,  
            "pressure_drop": self._pressure_drop,  
            "filter_life": self._filter_life,  
            "power_on": self._power_on  
        }  
  
  
# 用于测试的主函数  
if __name__ == "__main__":  
    filter_device = MockFilter()  
      
    # 测试基本功能  
    print("启动过滤器测试...")  
    filter_device.power_on_off("on")  
    print(f"初始状态: {filter_device.get_status_info()}")  
      
    # 开始过滤  
    filter_device.start_filtering(2.0)  
      
    # 模拟运行10秒  
    for i in range(10):  
        time.sleep(1)  
        print(f"第{i+1}秒: 效率={filter_device.filter_efficiency:.1f}%, 寿命={filter_device.filter_life:.1f}%, 状态={filter_device.status}")  
      
    filter_device.emergency_stop()  
    print("测试完成")