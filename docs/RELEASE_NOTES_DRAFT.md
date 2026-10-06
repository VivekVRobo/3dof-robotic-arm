# v0.1.0 Release Notes Draft

`3dof-robotic-arm` v0.1.0 is intended as the first **software-reference** release of the project.

## Included

- analytic forward and inverse kinematics for base yaw, shoulder pitch and elbow pitch;
- Cartesian waypoint generation;
- joint-limit validation;
- servo calibration/mapping support;
- serial control path and Arduino receiver reference;
- command-line examples;
- unit tests;
- deterministic numerical FK → IK → FK validation;
- physical endpoint experiment tooling and evidence protocol.

## Evidence boundary

This release validates the software/kinematic reference only. It does **not** claim measured endpoint accuracy, repeatability, backlash, payload performance, calibrated mechanical geometry or physical safety certification.

Those claims remain blocked until real measurements satisfy `docs/PHYSICAL_VALIDATION.md` and the resulting evidence is reviewed.

## Verification before publication

Publish only after the exact tag commit satisfies `docs/RELEASE_READINESS.md` and its CI run is green.