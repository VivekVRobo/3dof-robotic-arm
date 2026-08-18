from dataclasses import dataclass
from math import atan2, cos, hypot, sin, sqrt


@dataclass(frozen=True)
class ArmGeometry:
    l1: float = 100.0
    l2: float = 100.0
    base_height: float = 30.0

    def __post_init__(self):
        if self.l1 <= 0 or self.l2 <= 0:
            raise ValueError("link lengths must be positive")


@dataclass(frozen=True)
class JointAngles:
    base: float
    shoulder: float
    elbow: float


def forward_kinematics(q: JointAngles, g: ArmGeometry) -> tuple[float, float, float]:
    radial = g.l1 * cos(q.shoulder) + g.l2 * cos(q.shoulder + q.elbow)
    z = g.base_height + g.l1 * sin(q.shoulder) + g.l2 * sin(q.shoulder + q.elbow)
    return radial * cos(q.base), radial * sin(q.base), z


def inverse_kinematics(x: float, y: float, z: float, g: ArmGeometry, *, elbow_up: bool = False) -> JointAngles:
    radial = hypot(x, y)
    vertical = z - g.base_height
    c2 = (radial * radial + vertical * vertical - g.l1 * g.l1 - g.l2 * g.l2) / (2.0 * g.l1 * g.l2)
    if c2 < -1.0 - 1e-9 or c2 > 1.0 + 1e-9:
        raise ValueError("target is outside the arm workspace")
    c2 = max(-1.0, min(1.0, c2))
    s2_mag = sqrt(max(0.0, 1.0 - c2 * c2))
    s2 = s2_mag if elbow_up else -s2_mag
    elbow = atan2(s2, c2)
    shoulder = atan2(vertical, radial) - atan2(g.l2 * s2, g.l1 + g.l2 * c2)
    base = atan2(y, x)
    return JointAngles(base, shoulder, elbow)
