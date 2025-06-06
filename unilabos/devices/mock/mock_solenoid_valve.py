import time


class MockSolenoidValve:
    """
    模拟电磁阀设备类 - 简化版本

    这个类提供了电磁阀的基本功能：开启、关闭和状态查询
    """

    def __init__(self, port: str = "MOCK"):
        """
        初始化MockSolenoidValve实例

        Args:
            port (str): 设备端口，默认为"MOCK"表示模拟设备
        """
        self.port = port
        self._status: str = "Idle"
        self._valve_status: str = "Closed"  # 阀门位置：Open, Closed

    @property
    def status(self) -> str:
        """设备状态 - 会被自动识别的设备属性"""
        return self._status

    @property
    def valve_status(self) -> str:
        """阀门状态"""
        return self._valve_status

    def set_valve_status(self, status: str) -> str:
        """
        设置阀门位置

        Args:
            position (str): 阀门位置，可选值："Open", "Closed"

        Returns:
            str: 操作结果状态 ("Success", "Error")
        """
        if status not in ["Open", "Closed"]:
            self._status = "Error: Invalid position"
            return "Error"

        self._status = "Moving"
        time.sleep(1)  # 模拟阀门动作时间

        self._valve_status = status
        self._status = "Idle"
        return "Success"

    def open_valve(self) -> str:
        """打开阀门"""
        return self.set_valve_status("Open")

    def close_valve(self) -> str:
        """关闭阀门"""
        return self.set_valve_status("Closed")

    def get_valve_status(self) -> str:
        """获取阀门位置"""
        return self._valve_status

    def is_open(self) -> bool:
        """检查阀门是否打开"""
        return self._valve_status == "Open"

    def is_closed(self) -> bool:
        """检查阀门是否关闭"""
        return self._valve_status == "Closed"


# 用于测试的主函数
if __name__ == "__main__":
    valve = MockSolenoidValve()

    print("启动电磁阀测试...")
    print(f"初始状态: 位置={valve.valve_status}, 状态={valve.status}")

    # 测试开启阀门
    valve.open_valve()
    print(f"开启后: 位置={valve.valve_status}, 状态={valve.status}")

    # 测试关闭阀门
    valve.close_valve()
    print(f"关闭后: 位置={valve.valve_status}, 状态={valve.status}")

    print("测试完成")
