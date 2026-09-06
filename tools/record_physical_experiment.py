#!/usr/bin/env python3
"""Record physical endpoint-validation data for the 3-DOF arm.

The script deliberately keeps measured evidence separate from dry-run data.
It computes IK targets, accepts observed Cartesian coordinates, and writes
machine-readable CSV plus a Markdown report with summary error metrics.
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from arm3dof.kinematics import ArmGeometry, JointAngles, forward_kinematics, inverse_kinematics


@dataclass(frozen=True)
class Observation:
    index: int
    target_x: float
    target_y: float
    target_z: float
    observed_x: float
    observed_y: float
    observed_z: float
    q0_deg: float
    q1_deg: float
    q2_deg: float

    @property
    def dx(self) -> float:
        return self.observed_x - self.target_x

    @property
    def dy(self) -> float:
        return self.observed_y - self.target_y

    @property
    def dz(self) -> float:
        return self.observed_z - self.target_z

    @property
    def error(self) -> float:
        return math.sqrt(self.dx * self.dx + self.dy * self.dy + self.dz * self.dz)


def _positive_float(value: str) -> float:
    number = float(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("value must be > 0")
    return number


def _prompt_positive(label: str) -> float:
    while True:
        raw = input(f"{label}: ").strip()
        try:
            value = float(raw)
        except ValueError:
            print("Enter a numeric value.")
            continue
        if value <= 0:
            print("Value must be greater than zero.")
            continue
        return value


def _default_targets(g: ArmGeometry) -> list[tuple[float, float, float]]:
    """Generate 12 guaranteed-reachable targets from joint-space presets."""
    presets_deg = [
        (-35, 20, -55), (-20, 28, -65), (0, 20, -50), (20, 28, -65),
        (35, 20, -55), (-30, 38, -75), (-10, 45, -70), (10, 45, -70),
        (30, 38, -75), (-22, 15, -40), (0, 52, -80), (22, 15, -40),
    ]
    targets: list[tuple[float, float, float]] = []
    for base_deg, shoulder_deg, elbow_deg in presets_deg:
        q = JointAngles(*(math.radians(v) for v in (base_deg, shoulder_deg, elbow_deg)))
        targets.append(forward_kinematics(q, g))
    return targets


def _load_targets(path: Path) -> list[tuple[float, float, float]]:
    targets: list[tuple[float, float, float]] = []
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        required = {"x", "y", "z"}
        if not reader.fieldnames or not required.issubset(set(reader.fieldnames)):
            raise ValueError("target CSV must contain x,y,z columns")
        for row in reader:
            targets.append((float(row["x"]), float(row["y"]), float(row["z"])))
    if not targets:
        raise ValueError("target CSV contains no targets")
    return targets


def _parse_observation(raw: str) -> tuple[float, float, float] | None:
    text = raw.strip()
    if text.lower() in {"skip", "s"}:
        return None
    parts = [part.strip() for part in text.split(",")]
    if len(parts) != 3:
        raise ValueError("enter x,y,z or 'skip'")
    return float(parts[0]), float(parts[1]), float(parts[2])


def _summary(observations: Iterable[Observation]) -> dict[str, float]:
    rows = list(observations)
    if not rows:
        raise ValueError("at least one observation is required")
    errors = [row.error for row in rows]
    return {
        "count": float(len(rows)),
        "mean_error": statistics.fmean(errors),
        "median_error": statistics.median(errors),
        "rms_error": math.sqrt(statistics.fmean(error * error for error in errors)),
        "max_error": max(errors),
        "rmse_x": math.sqrt(statistics.fmean(row.dx * row.dx for row in rows)),
        "rmse_y": math.sqrt(statistics.fmean(row.dy * row.dy for row in rows)),
        "rmse_z": math.sqrt(statistics.fmean(row.dz * row.dz for row in rows)),
    }


def _write_csv(path: Path, observations: list[Observation], evidence_type: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "evidence_type", "index", "target_x", "target_y", "target_z",
        "observed_x", "observed_y", "observed_z", "dx", "dy", "dz",
        "euclidean_error", "q0_deg", "q1_deg", "q2_deg",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in observations:
            writer.writerow({
                "evidence_type": evidence_type,
                "index": row.index,
                "target_x": f"{row.target_x:.3f}",
                "target_y": f"{row.target_y:.3f}",
                "target_z": f"{row.target_z:.3f}",
                "observed_x": f"{row.observed_x:.3f}",
                "observed_y": f"{row.observed_y:.3f}",
                "observed_z": f"{row.observed_z:.3f}",
                "dx": f"{row.dx:.3f}",
                "dy": f"{row.dy:.3f}",
                "dz": f"{row.dz:.3f}",
                "euclidean_error": f"{row.error:.3f}",
                "q0_deg": f"{row.q0_deg:.3f}",
                "q1_deg": f"{row.q1_deg:.3f}",
                "q2_deg": f"{row.q2_deg:.3f}",
            })


def _write_report(
    path: Path,
    observations: list[Observation],
    geometry: ArmGeometry,
    evidence_type: str,
) -> None:
    metrics = _summary(observations)
    physical = evidence_type == "physical-measurement"
    status = "PHYSICAL MEASUREMENT" if physical else "SIMULATED DRY RUN — NOT HARDWARE EVIDENCE"
    lines = [
        "# 3-DOF Arm Endpoint Validation Report",
        "",
        f"> **Evidence status:** {status}",
        "",
        f"Generated: {datetime.now(timezone.utc).isoformat()}",
        "",
        "## Geometry",
        "",
        "| Parameter | Value |",
        "|---|---:|",
        f"| Base height H | {geometry.base_height:.3f} |",
        f"| Link L1 | {geometry.l1:.3f} |",
        f"| Link L2 | {geometry.l2:.3f} |",
        "",
        "All dimensions and endpoint coordinates use the same user-selected length unit.",
        "",
        "## Summary",
        "",
        "| Metric | Value |",
        "|---|---:|",
        f"| Samples | {int(metrics['count'])} |",
        f"| Mean Euclidean error | {metrics['mean_error']:.3f} |",
        f"| Median Euclidean error | {metrics['median_error']:.3f} |",
        f"| RMS Euclidean error | {metrics['rms_error']:.3f} |",
        f"| Maximum Euclidean error | {metrics['max_error']:.3f} |",
        f"| X-axis RMSE | {metrics['rmse_x']:.3f} |",
        f"| Y-axis RMSE | {metrics['rmse_y']:.3f} |",
        f"| Z-axis RMSE | {metrics['rmse_z']:.3f} |",
        "",
        "## Measurements",
        "",
        "| # | Target (x,y,z) | Observed (x,y,z) | Error | q0° | q1° | q2° |",
        "|---:|---|---|---:|---:|---:|---:|",
    ]
    for row in observations:
        lines.append(
            f"| {row.index} | ({row.target_x:.2f}, {row.target_y:.2f}, {row.target_z:.2f}) "
            f"| ({row.observed_x:.2f}, {row.observed_y:.2f}, {row.observed_z:.2f}) "
            f"| {row.error:.3f} | {row.q0_deg:.2f} | {row.q1_deg:.2f} | {row.q2_deg:.2f} |"
        )
    lines.extend([
        "",
        "## Evidence notes",
        "",
        "- The script computes kinematic targets and error metrics; it does not independently verify ruler, caliper, camera, or motion-capture measurements.",
        "- Physical rows are only as reliable as the measurement procedure and coordinate-frame setup used by the operator.",
        "- Servo calibration, backlash, compliance, payload, and fixture error should be documented alongside any published result.",
    ])
    if physical and len(observations) < 10:
        lines.extend(["", "> Fewer than 10 physical samples were recorded; collect 10–15+ targets before using this as a portfolio-level validation result."])
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _dry_run_observations(g: ArmGeometry, targets: list[tuple[float, float, float]]) -> list[Observation]:
    # Deterministic synthetic offsets exist only to exercise calculations and I/O.
    offsets = [(1.0, -0.5, 0.8), (-0.7, 0.9, -0.4), (0.4, 0.3, -0.6), (-0.8, -0.2, 0.5)]
    rows: list[Observation] = []
    for index, target in enumerate(targets, start=1):
        q = inverse_kinematics(*target, g)
        offset = offsets[(index - 1) % len(offsets)]
        observed = tuple(target[i] + offset[i] for i in range(3))
        rows.append(Observation(
            index, *target, *observed,
            *(math.degrees(v) for v in (q.base, q.shoulder, q.elbow)),
        ))
    return rows


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-height", type=_positive_float)
    parser.add_argument("--l1", type=_positive_float)
    parser.add_argument("--l2", type=_positive_float)
    parser.add_argument("--targets-csv", type=Path, help="CSV with x,y,z columns; default uses 12 reachable targets")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "artifacts")
    parser.add_argument("--elbow-up", action="store_true")
    parser.add_argument("--dry-run", action="store_true", help="exercise calculations with synthetic observations; never labels output as physical evidence")
    parser.add_argument("--show-raw-serial", action="store_true", help="show uncalibrated J,q0,q1,q2 degree command template")
    args = parser.parse_args()

    if args.dry_run:
        geometry = ArmGeometry(
            l1=args.l1 or 100.0,
            l2=args.l2 or 100.0,
            base_height=args.base_height or 30.0,
        )
    else:
        print("Measure all geometry in one consistent unit, e.g. millimetres.")
        geometry = ArmGeometry(
            l1=args.l1 or _prompt_positive("L1 shoulder-to-elbow"),
            l2=args.l2 or _prompt_positive("L2 elbow-to-tip"),
            base_height=args.base_height or _prompt_positive("H base height"),
        )

    targets = _load_targets(args.targets_csv) if args.targets_csv else _default_targets(geometry)

    if args.dry_run:
        observations = _dry_run_observations(geometry, targets)
        output_dir = args.output_dir / "dry_run"
        csv_path = output_dir / "simulated_positions.csv"
        report_path = output_dir / "validation_report.md"
        evidence_type = "simulated-dry-run"
    else:
        observations = []
        print(f"\nPrepared {len(targets)} reachable targets. Record 10–15+ for portfolio evidence.")
        print("For each target, move the arm using your calibrated control path, measure endpoint x,y,z, then enter x,y,z. Type 'skip' to omit a target.\n")
        for index, target in enumerate(targets, start=1):
            try:
                q = inverse_kinematics(*target, geometry, elbow_up=args.elbow_up)
            except ValueError as exc:
                print(f"[{index}] target skipped: {exc}")
                continue
            q_deg = tuple(math.degrees(v) for v in (q.base, q.shoulder, q.elbow))
            print(f"[{index}/{len(targets)}] target = ({target[0]:.2f}, {target[1]:.2f}, {target[2]:.2f})")
            print(f"    mathematical IK = q0 {q_deg[0]:.2f}°, q1 {q_deg[1]:.2f}°, q2 {q_deg[2]:.2f}°")
            if args.show_raw_serial:
                print(f"    RAW/UNCALIBRATED template: J,{q_deg[0]:.2f},{q_deg[1]:.2f},{q_deg[2]:.2f}")
                print("    Do not send raw math angles unless your servo calibration maps them safely.")
            while True:
                raw = input("    observed x,y,z (or skip): ")
                try:
                    observed = _parse_observation(raw)
                    break
                except ValueError as exc:
                    print(f"    {exc}")
            if observed is None:
                continue
            observations.append(Observation(index, *target, *observed, *q_deg))

        if not observations:
            print("No observations recorded; no physical evidence files were written.")
            return 2
        output_dir = args.output_dir
        csv_path = output_dir / "measured_positions.csv"
        report_path = output_dir / "physical_validation_report.md"
        evidence_type = "physical-measurement"

    _write_csv(csv_path, observations, evidence_type)
    _write_report(report_path, observations, geometry, evidence_type)
    metrics = _summary(observations)
    print(f"Wrote {csv_path}")
    print(f"Wrote {report_path}")
    print(f"Samples={int(metrics['count'])} RMS={metrics['rms_error']:.3f} max={metrics['max_error']:.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
