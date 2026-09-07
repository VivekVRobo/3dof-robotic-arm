#!/usr/bin/env python3
"""Generate deterministic software-only validation evidence for the arm model."""
from __future__ import annotations

import argparse
import json
from dataclasses import asdict
from pathlib import Path

from arm3dof.kinematics import ArmGeometry
from arm3dof.validation import endpoint_angle_sensitivity, validate_fk_ik_grid


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--l1", type=float, default=100.0)
    parser.add_argument("--l2", type=float, default=100.0)
    parser.add_argument("--base-height", type=float, default=30.0)
    parser.add_argument("--grid", type=int, default=17, help="points per joint for FK/IK grid")
    parser.add_argument("--sensitivity-grid", type=int, default=11)
    parser.add_argument("--angle-perturbation-deg", type=float, default=1.0)
    parser.add_argument("--output", type=Path, default=Path("artifacts/numerical_validation.json"))
    args = parser.parse_args()

    geometry = ArmGeometry(args.l1, args.l2, args.base_height)
    consistency = validate_fk_ik_grid(geometry, points_per_joint=args.grid)
    sensitivity = endpoint_angle_sensitivity(
        geometry,
        perturbation_deg=args.angle_perturbation_deg,
        points_per_joint=args.sensitivity_grid,
    )

    report = {
        "evidence_type": "software_numerical_validation",
        "hardware_evidence": False,
        "claim_boundary": (
            "Results validate the ideal analytic rigid-link model and geometric angle sensitivity only; "
            "they are not measured servo or physical endpoint accuracy."
        ),
        "geometry": {"l1": geometry.l1, "l2": geometry.l2, "base_height": geometry.base_height},
        "fk_ik_consistency": asdict(consistency),
        "angle_sensitivity": asdict(sensitivity),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if consistency.ik_failures == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
