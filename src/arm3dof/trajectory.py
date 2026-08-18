from .kinematics import ArmGeometry, JointAngles, inverse_kinematics


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def cartesian_line(start: tuple[float, float, float], end: tuple[float, float, float], steps: int) -> list[tuple[float, float, float]]:
    if steps < 2:
        raise ValueError("steps must be >= 2")
    return [
        tuple(lerp(a, b, i / (steps - 1)) for a, b in zip(start, end))
        for i in range(steps)
    ]


def joint_path_for_cartesian_line(start, end, steps: int, geometry: ArmGeometry, *, elbow_up: bool = False) -> list[JointAngles]:
    return [inverse_kinematics(*point, geometry, elbow_up=elbow_up) for point in cartesian_line(start, end, steps)]
