from dataclasses import dataclass
from math import degrees, isfinite

from .kinematics import JointAngles


@dataclass(frozen=True)
class JointCalibration:
    offset_deg: float = 90.0
    scale: float = 1.0
    min_deg: float = 0.0
    max_deg: float = 180.0

    def __post_init__(self):
        if not all(
            isfinite(value)
            for value in (self.offset_deg, self.scale, self.min_deg, self.max_deg)
        ):
            raise ValueError("servo calibration values must be finite")
        if self.min_deg > self.max_deg:
            raise ValueError("servo calibration minimum must not exceed maximum")

    def map_radians(self, radians_value: float) -> float:
        if not isfinite(radians_value):
            raise ValueError("joint angle must be finite")
        command = self.offset_deg + self.scale * degrees(radians_value)
        if command < self.min_deg or command > self.max_deg:
            raise ValueError(f"servo command {command:.2f} outside [{self.min_deg}, {self.max_deg}]")
        return command


@dataclass(frozen=True)
class ServoCalibration:
    base: JointCalibration = JointCalibration()
    shoulder: JointCalibration = JointCalibration()
    elbow: JointCalibration = JointCalibration()

    def map(self, joints: JointAngles) -> tuple[float, float, float]:
        return (
            self.base.map_radians(joints.base),
            self.shoulder.map_radians(joints.shoulder),
            self.elbow.map_radians(joints.elbow),
        )


def encode_joint_command(degrees_triplet: tuple[float, float, float]) -> bytes:
    if not all(isfinite(value) for value in degrees_triplet):
        raise ValueError("joint command values must be finite")
    a, b, c = degrees_triplet
    return f"J,{a:.1f},{b:.1f},{c:.1f}\n".encode("ascii")
