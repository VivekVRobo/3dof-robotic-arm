import argparse
import math
from dataclasses import dataclass


@dataclass(frozen=True)
class ArmGeometry:
    base_height: float = 60.0
    upper_arm: float = 120.0
    forearm: float = 120.0


GEOMETRY = ArmGeometry()


def clamp_unit(value: float) -> float:
    return max(-1.0, min(1.0, value))


def forward_kinematics(base_deg: float, shoulder_deg: float, elbow_deg: float, g: ArmGeometry = GEOMETRY):
    q0 = math.radians(base_deg)
    q1 = math.radians(shoulder_deg)
    q2 = math.radians(elbow_deg)

    radial = g.upper_arm * math.cos(q1) + g.forearm * math.cos(q1 + q2)
    z = g.base_height + g.upper_arm * math.sin(q1) + g.forearm * math.sin(q1 + q2)
    x = radial * math.cos(q0)
    y = radial * math.sin(q0)
    return x, y, z


def inverse_kinematics(x: float, y: float, z: float, elbow_up: bool = False, g: ArmGeometry = GEOMETRY):
    base = math.atan2(y, x)
    r = math.hypot(x, y)
    z_planar = z - g.base_height
    d2 = r * r + z_planar * z_planar

    cos_elbow = (d2 - g.upper_arm**2 - g.forearm**2) / (2 * g.upper_arm * g.forearm)
    if cos_elbow < -1.0 or cos_elbow > 1.0:
        raise ValueError("Target is outside the arm workspace")

    elbow = math.acos(clamp_unit(cos_elbow))
    if elbow_up:
        elbow = -elbow

    shoulder = math.atan2(z_planar, r) - math.atan2(
        g.forearm * math.sin(elbow),
        g.upper_arm + g.forearm * math.cos(elbow),
    )

    return tuple(map(math.degrees, (base, shoulder, elbow)))


def interpolate(start, end, steps: int):
    if steps < 2:
        raise ValueError("steps must be at least 2")
    for i in range(steps):
        t = i / (steps - 1)
        yield tuple(a + (b - a) * t for a, b in zip(start, end))


def main():
    parser = argparse.ArgumentParser(description="3-DOF robotic arm kinematics")
    sub = parser.add_subparsers(dest="command", required=True)

    fk = sub.add_parser("fk")
    fk.add_argument("base", type=float)
    fk.add_argument("shoulder", type=float)
    fk.add_argument("elbow", type=float)

    ik = sub.add_parser("ik")
    ik.add_argument("x", type=float)
    ik.add_argument("y", type=float)
    ik.add_argument("z", type=float)
    ik.add_argument("--elbow-up", action="store_true")

    path = sub.add_parser("path")
    path.add_argument("sx", type=float)
    path.add_argument("sy", type=float)
    path.add_argument("sz", type=float)
    path.add_argument("ex", type=float)
    path.add_argument("ey", type=float)
    path.add_argument("ez", type=float)
    path.add_argument("--steps", type=int, default=20)

    args = parser.parse_args()

    if args.command == "fk":
        xyz = forward_kinematics(args.base, args.shoulder, args.elbow)
        print("x={:.2f} y={:.2f} z={:.2f}".format(*xyz))
    elif args.command == "ik":
        joints = inverse_kinematics(args.x, args.y, args.z, args.elbow_up)
        print("base={:.2f} shoulder={:.2f} elbow={:.2f}".format(*joints))
    else:
        start = (args.sx, args.sy, args.sz)
        end = (args.ex, args.ey, args.ez)
        for i, point in enumerate(interpolate(start, end, args.steps)):
            try:
                joints = inverse_kinematics(*point)
                print(i, "xyz=({:.1f},{:.1f},{:.1f}) joints=({:.2f},{:.2f},{:.2f})".format(*point, *joints))
            except ValueError as exc:
                print(i, f"xyz={point} ERROR: {exc}")


if __name__ == "__main__":
    main()
