# Physical IK Validation Run

> Copy this template for a real arm experiment. Do not populate measurement fields from dry-run or synthetic data.

## Run identity

- Run ID:
- Date/time + timezone:
- Git commit SHA:
- Operator:
- Arm revision:
- Power supply:
- Servo models:
- End-effector / pointer configuration:

## Measured geometry

All dimensions use the same unit.

| Parameter | Measured value | Method / instrument | Uncertainty / note |
|---|---:|---|---|
| Base height `H` | | | |
| Link 1 `L1` | | | |
| Link 2 `L2` | | | |

## Servo calibration

| Joint | Mechanical zero | Direction sign | Safe min | Safe max | Servo offset / mapping note |
|---|---:|---:|---:|---:|---|
| Base | | | | | |
| Shoulder | | | | | |
| Elbow | | | | | |

## Coordinate frame

- Origin definition:
- +X direction:
- +Y direction:
- +Z direction:
- Frame photo/sketch path:

## Measurement method

Describe how endpoint XYZ is measured, including ruler/caliper/grid/camera method and how the endpoint reference point is kept consistent.

## Target set

Target source:

- [ ] built-in harness targets
- [ ] `experiments/targets.csv`
- [ ] other documented target set

Number of unique targets:
Number of repeat trials:

## Execution

Command:

```text
python tools/record_physical_experiment.py <args>
```

Artifacts:

- [ ] `artifacts/measured_positions.csv`
- [ ] `artifacts/physical_validation_report.md`
- [ ] setup photo(s)
- [ ] short demo video or stable link
- [ ] calibration notes

## Results

Copy only values generated from the real measured CSV.

| Metric | Value | Units |
|---|---:|---|
| Mean endpoint error | | |
| Median endpoint error | | |
| RMS endpoint error | | |
| Maximum endpoint error | | |
| X-axis RMSE | | |
| Y-axis RMSE | | |
| Z-axis RMSE | | |

## Repeatability / backlash

For repeated targets, report spread separately from one-pass accuracy.

| Target | Trials | Mean endpoint error | Repeatability spread / note |
|---|---:|---:|---|
| | | | |

## Failure cases

Record unreachable commands, servo saturation, backlash, structural flex, measurement ambiguity, power issues, thermal behavior, or targets that were skipped.

## Payload / operating conditions

- Payload during validation:
- Servo supply voltage:
- Current limit / protection:
- Approximate ambient condition:
- Motion speed / delay settings:

## Claim decision

- [ ] Real geometry measured
- [ ] Servo zero/direction/limits calibrated
- [ ] At least 12 real reachable targets measured
- [ ] Error report generated from archived CSV
- [ ] Repeatability/backlash sampled
- [ ] Setup media archived
- [ ] Limitations documented

Only after these boxes are backed by real artifacts should physical accuracy numbers be promoted into the main README.

## Recruiter review path

`geometry + calibration -> target -> IK -> servo command -> real motion -> observed XYZ -> CSV -> error report -> repeatability -> limitations`
