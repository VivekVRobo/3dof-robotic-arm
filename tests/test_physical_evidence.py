import csv
import json
from pathlib import Path

from tools.validate_physical_evidence import validate_bundle


def _write_manifest(path: Path, *, dry_run: bool = False) -> None:
    path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "evidence_type": "physical-measurement",
                "dry_run": dry_run,
                "arm_revision": "rev-a",
                "measurement_unit": "mm",
                "measurement_method": "fixed Cartesian grid measured from base origin",
                "measurement_instrument": "digital caliper + marked grid",
                "measurement_uncertainty": "+/- 1.0 mm",
                "power_supply": "6 V regulated bench supply, 5 A current limit",
                "servo_calibration_reference": "docs/calibration-run.md",
                "coordinate_frame_reference": "evidence/frame-photo.jpg",
                "demo_media_reference": "evidence/demo.mp4",
            }
        ),
        encoding="utf-8",
    )


def _write_measurements(path: Path, *, evidence_type: str = "physical-measurement") -> None:
    fields = [
        "evidence_type",
        "index",
        "target_x",
        "target_y",
        "target_z",
        "observed_x",
        "observed_y",
        "observed_z",
        "euclidean_error",
    ]
    rows = []
    index = 1
    for target_id in range(12):
        trials = 3 if target_id < 3 else 1
        tx = 80.0 + target_id * 5.0
        ty = -20.0 + target_id * 2.0
        tz = 70.0 + target_id
        for trial in range(trials):
            ox = tx + 1.0 + trial * 0.1
            oy = ty - 0.5
            oz = tz + 0.5
            error = ((ox - tx) ** 2 + (oy - ty) ** 2 + (oz - tz) ** 2) ** 0.5
            rows.append(
                {
                    "evidence_type": evidence_type,
                    "index": index,
                    "target_x": tx,
                    "target_y": ty,
                    "target_z": tz,
                    "observed_x": ox,
                    "observed_y": oy,
                    "observed_z": oz,
                    "euclidean_error": f"{error:.4f}",
                }
            )
            index += 1

    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def test_publishable_structure_requires_unique_targets_and_repeat_trials(tmp_path):
    csv_path = tmp_path / "measured_positions.csv"
    manifest_path = tmp_path / "manifest.json"
    _write_measurements(csv_path)
    _write_manifest(manifest_path)

    summary = validate_bundle(csv_path, manifest_path)

    assert summary["bundle_status"] == "publishable-structure"
    assert summary["sample_count"] == 18
    assert summary["unique_target_count"] == 12
    assert summary["repeated_target_count"] == 3
    assert summary["errors"] == []


def test_dry_run_manifest_is_blocked(tmp_path):
    csv_path = tmp_path / "measured_positions.csv"
    manifest_path = tmp_path / "manifest.json"
    _write_measurements(csv_path)
    _write_manifest(manifest_path, dry_run=True)

    summary = validate_bundle(csv_path, manifest_path)

    assert summary["bundle_status"] == "blocked"
    assert "manifest must explicitly set dry_run=false" in summary["errors"]


def test_simulated_rows_are_blocked(tmp_path):
    csv_path = tmp_path / "measured_positions.csv"
    manifest_path = tmp_path / "manifest.json"
    _write_measurements(csv_path, evidence_type="simulated-dry-run")
    _write_manifest(manifest_path)

    summary = validate_bundle(csv_path, manifest_path)

    assert summary["bundle_status"] == "blocked"
    assert "all rows must be labelled evidence_type=physical-measurement" in summary["errors"]
