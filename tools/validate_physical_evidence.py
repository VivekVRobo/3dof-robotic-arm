#!/usr/bin/env python3
"""Validate the completeness of a publishable 3-DOF physical evidence bundle.

This is a provenance/completeness gate, not a truth detector. It can verify that the
repository contains enough structured measurements and experiment metadata to make a
reviewable claim, but it cannot prove that a human-entered coordinate came from a real
instrument. Photos/video and the documented measurement method remain part of review.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import Counter
from pathlib import Path

REQUIRED_COLUMNS = {
    "evidence_type",
    "index",
    "target_x",
    "target_y",
    "target_z",
    "observed_x",
    "observed_y",
    "observed_z",
    "euclidean_error",
}

REQUIRED_MANIFEST_FIELDS = (
    "arm_revision",
    "measurement_unit",
    "measurement_method",
    "measurement_instrument",
    "measurement_uncertainty",
    "power_supply",
    "servo_calibration_reference",
    "coordinate_frame_reference",
    "demo_media_reference",
)


def _nonempty(value: object) -> bool:
    return isinstance(value, str) and bool(value.strip())


def _target_key(row: dict[str, str]) -> tuple[float, float, float]:
    return tuple(round(float(row[name]), 3) for name in ("target_x", "target_y", "target_z"))


def validate_bundle(
    csv_path: Path,
    manifest_path: Path,
    *,
    min_unique_targets: int = 12,
    min_repeated_targets: int = 3,
    min_trials_per_repeated_target: int = 3,
    error_tolerance: float = 0.02,
) -> dict[str, object]:
    errors: list[str] = []

    with csv_path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        fieldnames = set(reader.fieldnames or [])
        missing_columns = sorted(REQUIRED_COLUMNS - fieldnames)
        if missing_columns:
            raise ValueError(f"measurement CSV missing columns: {', '.join(missing_columns)}")
        rows = list(reader)

    if not rows:
        errors.append("measurement CSV contains no rows")

    evidence_types = {row.get("evidence_type", "") for row in rows}
    if evidence_types != {"physical-measurement"}:
        errors.append("all rows must be labelled evidence_type=physical-measurement")

    target_counts: Counter[tuple[float, float, float]] = Counter()
    numeric_errors: list[float] = []
    for number, row in enumerate(rows, start=2):
        try:
            key = _target_key(row)
            target = tuple(float(row[name]) for name in ("target_x", "target_y", "target_z"))
            observed = tuple(float(row[name]) for name in ("observed_x", "observed_y", "observed_z"))
            numeric_error = float(row["euclidean_error"])
            if not all(math.isfinite(value) for value in (*target, *observed, numeric_error)):
                raise ValueError
            if numeric_error < 0:
                raise ValueError
            calculated_error = math.sqrt(
                sum((observed[i] - target[i]) ** 2 for i in range(3))
            )
            if abs(calculated_error - numeric_error) > error_tolerance:
                errors.append(
                    f"CSV row {number} euclidean_error={numeric_error:.4f} does not match "
                    f"recomputed value {calculated_error:.4f} within {error_tolerance}"
                )
        except (TypeError, ValueError):
            errors.append(f"CSV row {number} contains invalid/non-finite numeric evidence")
            continue
        target_counts[key] += 1
        numeric_errors.append(calculated_error)

    unique_targets = len(target_counts)
    repeated_targets = sum(
        count >= min_trials_per_repeated_target for count in target_counts.values()
    )

    if unique_targets < min_unique_targets:
        errors.append(
            f"need at least {min_unique_targets} unique Cartesian targets; found {unique_targets}"
        )
    if repeated_targets < min_repeated_targets:
        errors.append(
            f"need at least {min_repeated_targets} targets repeated "
            f"{min_trials_per_repeated_target}+ times; found {repeated_targets}"
        )

    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("schema_version") != 1:
        errors.append("manifest schema_version must be 1")
    if manifest.get("evidence_type") != "physical-measurement":
        errors.append("manifest evidence_type must be physical-measurement")

    for field in REQUIRED_MANIFEST_FIELDS:
        if not _nonempty(manifest.get(field)):
            errors.append(f"manifest field {field!r} must be a non-empty string")

    if manifest.get("dry_run") is not False:
        errors.append("manifest must explicitly set dry_run=false")

    summary: dict[str, object] = {
        "schema_version": 1,
        "bundle_status": "publishable-structure" if not errors else "blocked",
        "sample_count": len(rows),
        "unique_target_count": unique_targets,
        "repeated_target_count": repeated_targets,
        "repeat_trial_minimum": min_trials_per_repeated_target,
        "rms_endpoint_error": (
            math.sqrt(sum(value * value for value in numeric_errors) / len(numeric_errors))
            if numeric_errors
            else None
        ),
        "max_endpoint_error": max(numeric_errors) if numeric_errors else None,
        "errors": errors,
        "scope_note": (
            "This gate validates evidence structure/provenance fields only; it does not independently "
            "authenticate human-entered measurements or media."
        ),
    }
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", type=Path, default=Path("artifacts/measured_positions.csv"))
    parser.add_argument(
        "--manifest",
        type=Path,
        default=Path("artifacts/physical_experiment_manifest.json"),
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("artifacts/physical_evidence_summary.json"),
    )
    args = parser.parse_args()

    try:
        summary = validate_bundle(args.csv, args.manifest)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        print(f"physical evidence validation failed: {exc}")
        return 2

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return 0 if summary["bundle_status"] == "publishable-structure" else 1


if __name__ == "__main__":
    raise SystemExit(main())
