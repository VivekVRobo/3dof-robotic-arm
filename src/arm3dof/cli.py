import argparse
from math import degrees, radians

from .kinematics import ArmGeometry, JointAngles, forward_kinematics, inverse_kinematics
from .trajectory import joint_path_for_cartesian_line


def geometry_from_args(args) -> ArmGeometry:
    return ArmGeometry(args.l1, args.l2, args.base_height)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="3-DOF robotic arm kinematics")
    parser.add_argument("--l1", type=float, default=100.0)
    parser.add_argument("--l2", type=float, default=100.0)
    parser.add_argument("--base-height", type=float, default=30.0)
    sub = parser.add_subparsers(dest="command", required=True)

    ik = sub.add_parser("ik")
    ik.add_argument("x", type=float); ik.add_argument("y", type=float); ik.add_argument("z", type=float)
    ik.add_argument("--elbow-up", action="store_true")

    fk = sub.add_parser("fk")
    fk.add_argument("base_deg", type=float); fk.add_argument("shoulder_deg", type=float); fk.add_argument("elbow_deg", type=float)

    path = sub.add_parser("path")
    for name in ("x0", "y0", "z0", "x1", "y1", "z1"):
        path.add_argument(name, type=float)
    path.add_argument("--steps", type=int, default=8)
    return parser


def run() -> None:
    args = build_parser().parse_args()
    g = geometry_from_args(args)
    if args.command == "ik":
        q = inverse_kinematics(args.x, args.y, args.z, g, elbow_up=args.elbow_up)
        print(f"base={degrees(q.base):.3f} shoulder={degrees(q.shoulder):.3f} elbow={degrees(q.elbow):.3f}")
    elif args.command == "fk":
        q = JointAngles(radians(args.base_deg), radians(args.shoulder_deg), radians(args.elbow_deg))
        x, y, z = forward_kinematics(q, g)
        print(f"x={x:.3f} y={y:.3f} z={z:.3f}")
    else:
        joints = joint_path_for_cartesian_line(
            (args.x0, args.y0, args.z0), (args.x1, args.y1, args.z1), args.steps, g
        )
        for i, q in enumerate(joints):
            print(i, *(f"{degrees(v):.3f}" for v in (q.base, q.shoulder, q.elbow)))


if __name__ == "__main__":
    run()
