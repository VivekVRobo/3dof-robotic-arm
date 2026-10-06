# v0.1.0 Release Readiness

This checklist defines the minimum bar for the first tagged **software-reference** release of `3dof-robotic-arm`.

The release is intentionally scoped to the analytic kinematics, planning, calibration, serial-control and numerical-validation software. It must not imply physical endpoint accuracy, repeatability, payload capability or hardware safety evidence that has not been measured.

## Required before tagging

- [ ] Python CI is green on the exact release commit.
- [ ] `pytest -q` passes on the exact release commit.
- [ ] Package installation with `pip install -e .` succeeds.
- [ ] CLI examples in the README execute as documented.
- [ ] `tools/numerical_validation.py` completes and emits a valid software-only artifact.
- [ ] Numerical artifacts retain `hardware_evidence: false` or equivalent truth-boundary metadata.
- [ ] README evidence tables match the actual repository state.
- [ ] No generated dry-run data is presented as physical evidence.
- [ ] `LICENSE` is present and accurate.
- [ ] Release notes clearly state that physical geometry/calibration/endpoint accuracy remain pending.

## Allowed v0.1.0 claims

The first release may claim:

- analytic forward and inverse kinematics;
- Cartesian waypoint generation;
- joint-limit validation;
- servo calibration/mapping support;
- serial command support and Arduino receiver reference;
- deterministic software tests;
- reproducible numerical FK → IK → FK validation;
- physical-measurement tooling for later evidence collection.

## Claims that remain blocked

Do not claim any of the following until real evidence is committed and reviewed:

- measured endpoint accuracy;
- repeatability or backlash values;
- physical servo precision;
- payload capacity;
- collision-free physical operation;
- calibrated mechanical zero/limits;
- hardware safety certification.

## Promotion rule

Tag `v0.1.0` only from a clean commit that satisfies this checklist. Physical results should be promoted separately after the protocol in `docs/PHYSICAL_VALIDATION.md` is completed with real measurements, setup evidence and provenance.