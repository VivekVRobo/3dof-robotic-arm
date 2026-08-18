from dataclasses import dataclass
from math import degrees

from .kinematics import JointAngles


@dataclass(frozen=True)
class JointCalibration:
    offset_deg: float = 90.0
    scale: float = 1.0
    min_deg: float = 0.0
    max_deg: float = 180.0

    def map_radians(self, radians_value: float) -> float:
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
    a, b, c = degrees_triplet
    return f"J,{a:.1f},{b:.1f},{c:.1f}\n".encode("ascii")
