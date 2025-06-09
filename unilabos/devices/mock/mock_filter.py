import time
import threading


class MockFilter:
    def __init__(self, port: str = "MOCK"):
        # 基本参数初始化
        self.port = port
        self._status: str = "Idle"
        self._is_filtering: bool = False
        
        # 过滤性能参数
        self._flow_rate: float = 1.0          # 流速(L/min)
        self._pressure_drop: float = 0.0      # 压降(Pa)
        self._filter_life: float = 100.0      # 滤芯寿命(%)
        
        # 过滤操作参数
        self._vessel: str = ""                # 源容器
        self._filtrate_vessel: str = ""       # 目标容器
        self._stir: bool = False              # 是否搅拌
        self._stir_speed: float = 0.0         # 搅拌速度
        self._temperature: float = 25.0        # 温度(℃)
        self._continue_heatchill: bool = False # 是否继续加热/制冷
        self._target_volume: float = 0.0      # 目标过滤体积(L)
        self._filtered_volume: float = 0.0    # 已过滤体积(L)
        self._progress: float = 0.0           # 过滤进度(%)
        
        # 线程控制
        self._filter_thread = None
        self._running = False

    @property
    def status(self) -> str:
        return self._status

    @property
    def is_filtering(self) -> bool:
        return self._is_filtering

    @property
    def flow_rate(self) -> float:
        return self._flow_rate

    @property
    def pressure_drop(self) -> float:
        return self._pressure_drop

    @property
    def filter_life(self) -> float:
        return self._filter_life
    # 新增 property
    @property
    def vessel(self) -> str:
        return self._vessel

    @property
    def filtrate_vessel(self) -> str:
        return self._filtrate_vessel

    @property
    def filtered_volume(self) -> float:
        return self._filtered_volume

    @property
    def progress(self) -> float:
        return self._progress
    
    @property
    def stir(self) -> bool:
        return self._stir

    @property
    def stir_speed(self) -> float:
        return self._stir_speed

    @property
    def temperature(self) -> float:
        return self._temperature

    @property
    def continue_heatchill(self) -> bool:
        return self._continue_heatchill

    @property
    def target_volume(self) -> float:
        return self._target_volume

    def filter(self, vessel: str, filtrate_vessel: str, stir: bool = False, stir_speed: float = 0.0, temp: float = 25.0, continue_heatchill: bool = False, volume: float = 0.0) -> dict:
        """新的过滤操作"""
        # 停止任何正在进行的过滤
        if self._is_filtering:
            self.stop_filtering()
        # 验证参数
        if volume <= 0:
            return {"success": False, "message": "Target volume must be greater than 0"}
        # 设置新的过滤参数
        self._vessel = vessel
        self._filtrate_vessel = filtrate_vessel
        self._stir = stir
        self._stir_speed = stir_speed
        self._temperature = temp
        self._continue_heatchill = continue_heatchill
        self._target_volume = volume
        # 重置过滤状态
        self._filtered_volume = 0.0
        self._progress = 0.0
        self._status = "Starting Filter"
        # 启动过滤过程
        self._flow_rate = 1.0  # 设置默认流速
        self._start_filter_process()
        
        return {"success": True, "message": "Filter started"}

    def stop_filtering(self):
        """停止过滤"""
        self._status = "Stopping Filter"
        self._stop_filter_process()
        self._flow_rate = 0.0
        self._is_filtering = False
        self._status = "Stopped"
        return True

    def replace_filter(self):
        """更换滤芯"""
        self._filter_life = 100.0
        self._status = "Filter Replaced"
        return True

    def _start_filter_process(self):
        """启动过滤过程线程"""
        if not self._running:
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
        """过滤进程主循环"""
        update_interval = 1.0  # 更新间隔(秒)
        
        while self._running and self._is_filtering:
            try:
                self._status = "Filtering"
                
                # 计算这一秒过滤的体积 (L/min -> L/s)
                volume_increment = (self._flow_rate / 60.0) * update_interval
                
                # 更新已过滤体积
                self._filtered_volume += volume_increment
                
                # 更新进度 (避免除零错误)
                if self._target_volume > 0:
                    self._progress = min(100.0, (self._filtered_volume / self._target_volume) * 100.0)
                
                # 更新滤芯寿命 (每过滤1L减少0.5%寿命)
                self._filter_life = max(0.0, self._filter_life - (volume_increment * 0.5))
                
                # 更新压降 (根据滤芯寿命和流速动态计算)
                life_factor = self._filter_life / 100.0  # 将寿命转换为0-1的因子
                flow_factor = self._flow_rate / 2.0     # 将流速标准化(假设2L/min是标准流速)
                base_pressure = 100.0                    # 基础压降
                # 压降随滤芯寿命降低而增加，随流速增加而增加
                self._pressure_drop = base_pressure * (2 - life_factor) * flow_factor
                
                # 检查是否完成目标体积
                if self._target_volume > 0 and self._filtered_volume >= self._target_volume:
                    self._status = "Completed"
                    self._progress = 100.0
                    self.stop_filtering()
                    break
                    
                # 检查滤芯寿命
                if self._filter_life <= 10.0:
                    self._status = "Filter Needs Replacement"
                
                time.sleep(update_interval)
                
            except Exception as e:
                print(f"Error in filter loop: {e}")
                self.emergency_stop()
                break

    def emergency_stop(self):
        """紧急停止"""
        self._status = "Emergency Stop"
        self._stop_filter_process()
        self._is_filtering = False
        self._flow_rate = 0.0

    def get_status_info(self) -> dict:
        """扩展的状态信息"""
        return {
            "status": self._status,
            "is_filtering": self._is_filtering,
            "flow_rate": self._flow_rate,
            "pressure_drop": self._pressure_drop,
            "filter_life": self._filter_life,
            "vessel": self._vessel,
            "filtrate_vessel": self._filtrate_vessel,
            "filtered_volume": self._filtered_volume,
            "target_volume": self._target_volume,
            "progress": self._progress,
            "temperature": self._temperature,
            "stir": self._stir,
            "stir_speed": self._stir_speed
        }


# 用于测试的主函数
if __name__ == "__main__":
    filter_device = MockFilter()

    # 测试基本功能
    print("启动过滤器测试...")
    print(f"初始状态: {filter_device.get_status_info()}")



    # 模拟运行10秒
    for i in range(10):
        time.sleep(1)
        print(
            f"第{i+1}秒: "
            f"寿命={filter_device.filter_life:.1f}%, 状态={filter_device.status}"
        )

    filter_device.emergency_stop()
    print("测试完成")
