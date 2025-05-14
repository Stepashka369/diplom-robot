from msg.sensor_info import SensorInfo
from dataclasses import dataclass

@dataclass
class IrSensorsInfo:
    front_center: SensorInfo
    front_left: SensorInfo
    front_right: SensorInfo
    back_center: SensorInfo

    def to_dict(self):
        return {
            'front_center': self.front_center.to_dict(),
            'front_left': self.front_left.to_dict(),
            'front_right': self.front_right.to_dict(),
            'back_center': self.back_center.to_dict()
        }   
