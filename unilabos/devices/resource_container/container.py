
class HotelContainer:
    def __init__(self, rotation: dict, device_config: dict):
        self.rotation = rotation
        self.device_config = device_config
        self.status = 'idle'

    def get_rotation(self):
        return self.rotation
