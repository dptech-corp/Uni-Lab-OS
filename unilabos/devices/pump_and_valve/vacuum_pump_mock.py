import time


class VacuumPumpMock:
    def __init__(self, port: str = "COM6"):
        self._status = "OPEN"

    @property
    def status(self) -> str:
        return self._status

    def get_status(self) -> str:
        return self._status

    def set_status(self, position):
        time.sleep(5)

        self._status = position
        time.sleep(5)

    def open(self):
        self._status = "OPEN"

    def close(self):
        self._status = "CLOSED"

    def is_open(self):
        return self._status

    def is_closed(self):
        return not self._status
