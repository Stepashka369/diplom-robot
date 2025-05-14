from dataclasses import dataclass

@dataclass
class SensorInfo:
    min_range: float
    max_range: float
    range: float

    def to_dict(self):
        return {
            'min_range': self.min_range,
            'max_range': self.max_range,
            'range': self.range
        }
