# Physical Endpoint Validation Protocol

This protocol turns the software kinematics reference into reproducible physical evidence without pretending that synthetic or unmeasured data is hardware validation.

## Goal

Measure how closely the real 3-DOF arm reaches commanded Cartesian targets after geometry and servo calibration.

The primary metric is endpoint Euclidean error:

\[
\Delta = \sqrt{(x_{obs}-x)^2 + (y_{obs}-y)^2 + (z_{obs}-z)^2}
\]

The harness also reports mean, median, RMS, maximum error and per-axis RMSE.

## Required equipment

- assembled 3-DOF arm on a rigid base;
- ruler or caliper for `H`, `L1`, `L2`;
- repeatable Cartesian reference frame marked on the work surface;
- endpoint marker or pointer attached consistently to the arm tip;
- safe servo power supply and calibrated servo limits;
- optional camera/tripod, grid board, or other measurement aid.

## Geometry definitions

Measure every dimension in one unit, preferably millimetres:

- `H`: work-surface origin to shoulder-axis height;
- `L1`: shoulder rotation axis to elbow rotation axis;
- `L2`: elbow rotation axis to the endpoint used for measurement.

Do not use nominal CAD dimensions if printed parts, horn offsets, adapters, or the gripper change the effective pivot-to-pivot distances.

## Coordinate frame

Before recording samples:

1. mark the base-axis projection as `(0, 0)` on the work surface;
2. define +X straight forward from the arm's calibrated zero heading;
3. define +Y to the left or right and keep that choice fixed for the full experiment;
4. define +Z upward;
5. measure the endpoint relative to this same origin for every sample.

Photograph or sketch the frame so the table can be independently interpreted later.

## Procedure

1. Calibrate safe servo zero positions, direction signs and joint limits first.
2. Run `pip install -e .` in the repository environment.
3. Start the harness:

```bash
python tools/record_physical_experiment.py
```

4. Enter the measured `L1`, `L2`, and `H` values.
5. For each generated reachable target, the tool prints the mathematical IK angles.
6. Move the robot using the project's **calibrated** control path. Raw mathematical angles are not automatically safe servo commands.
7. Measure the endpoint and enter `x_obs,y_obs,z_obs`.
8. Record at least **12 unique targets** spread across the workspace.
9. Repeat at least **3 of those targets 3 or more times** so repeatability/backlash is visible instead of inferred from one pass.
10. Save a short video or photos showing the setup, coordinate frame and measurement method.

The physical run writes:

```text
artifacts/measured_positions.csv
artifacts/physical_validation_report.md
```

## Evidence provenance manifest

Copy the template:

```text
docs/physical_experiment_manifest.example.json
```

to:

```text
artifacts/physical_experiment_manifest.json
```

and replace every placeholder with the real experiment metadata: arm revision, units, measurement method/instrument/uncertainty, power supply, servo-calibration reference, coordinate-frame reference and media reference.

## Portfolio publication gate

After the real measurement CSV and provenance manifest exist, run:

```bash
python tools/validate_physical_evidence.py
```

The gate requires:

- every CSV row to be labelled `physical-measurement`;
- at least 12 unique Cartesian targets;
- at least 3 targets with 3+ trials each;
- recomputed Euclidean errors to agree with the stored values;
- non-empty experiment provenance fields;
- `dry_run=false` explicitly in the manifest.

A passing gate means the evidence bundle has a **publishable structure**. It does not independently prove that manually entered coordinates or media are authentic; a reviewer must still inspect the physical setup and methodology.

## Dry-run verification

To verify calculations and report generation without creating fake evidence:

```bash
python tools/record_physical_experiment.py --dry-run
```

Dry-run artifacts are written under `artifacts/dry_run/` and are explicitly labelled **SIMULATED DRY RUN — NOT HARDWARE EVIDENCE**. They must never pass the physical publication gate.

## Custom target set

Pass a CSV containing `x,y,z` columns:

```bash
python tools/record_physical_experiment.py --targets-csv experiments/targets.csv
```

Repeated rows in a custom target file are useful for the repeatability subset. Use only targets that are safe for the calibrated physical joint limits and workspace.

## Portfolio evidence checklist

A physical-validation claim is ready to publish only when all of the following exist:

- measured `H`, `L1`, `L2`;
- documented coordinate frame;
- calibrated servo zero/direction/limits;
- at least 12 unique measured targets;
- at least 3 repeated targets with 3+ trials each;
- generated CSV and report;
- completed physical experiment manifest;
- passing `validate_physical_evidence.py` summary;
- photo/video evidence of the real setup;
- notes on measurement uncertainty, backlash, payload and known failure modes.

The harness computes the metrics; it does not certify the measurement instrument or prove that entered coordinates came from hardware. Keep that provenance visible in the README and report.
