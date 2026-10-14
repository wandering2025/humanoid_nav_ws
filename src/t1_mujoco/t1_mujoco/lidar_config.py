"""Frozen W08 geometry shared with the system-Python URDF wrapper."""
from dataclasses import dataclass
import math

MOUNT = (0.10, 0.0, 0.20)
FRAME = 'laser_stabilized'
RAW_FRAME = 'lidar_link'


@dataclass(frozen=True)
class LidarConfig:
    noise_std: float = 0.0
    dropout: float = 0.0
    seed: int = 0
    publish_points: bool = False
    angles: int = 360
    elevations: tuple = (-20., -15., -10., -5., 0., 5., 10., 15., 20.)
    range_min: float = 0.15
    range_max: float = 12.0
    height_min: float = 0.15
    height_max: float = 1.20
    divisor: int = 5

    def __post_init__(self):
        if not math.isfinite(self.noise_std) or self.noise_std < 0:
            raise ValueError('lidar_noise_std must be finite and nonnegative')
        if not math.isfinite(self.dropout) or not 0 <= self.dropout <= 1:
            raise ValueError('lidar_dropout must be in [0,1]')
        if type(self.seed) is not int or self.seed < 0:
            raise ValueError('lidar seed must be a nonnegative integer')
