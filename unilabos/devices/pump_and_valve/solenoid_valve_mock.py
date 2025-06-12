import time


class SolenoidValveMock:
    def __init__(self, port: str = "COM6"):
        self._status = "Idle"
        self._valve_position = "OPEN"

    @property
    def status(self) -> str:
        return self._status

    @property
    def valve_position(self) -> str:
        return self._valve_position

    def get_valve_position(self) -> str:
        return self._valve_position

    def set_valve_position(self, position):
        self._status = "Busy"
        time.sleep(5)

        self._valve_position = position
        time.sleep(5)
        self._status = "Idle"

    def open(self):
        self._valve_position = "OPEN"

    def close(self):
        self._valve_position = "CLOSED"

    def is_open(self):
        return self._valve_position

    def is_closed(self):
        return not self._valve_position
