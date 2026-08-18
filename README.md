# 3-DOF Robotic Arm

A compact 3-degree-of-freedom robotic-arm reference project focused on kinematics, workspace validation, trajectory generation, and servo-command output.

> **Status:** kinematics/software implementation. Link lengths, servo zero offsets, mechanical limits, and safe speeds must be calibrated against the physical arm before motion testing.

## Model

The arm is modeled as:

- Joint 1: base yaw
- Joint 2: shoulder pitch
- Joint 3: elbow pitch

For target `(x, y, z)`, base yaw is computed from `atan2(y, x)`. The shoulder/elbow solution uses planar two-link inverse kinematics after subtracting the base height.

## Features

- Forward kinematics
- Analytic inverse kinematics
- Reachability checks
- Joint-limit validation
- Linear Cartesian waypoint generation
- CLI for testing targets before hardware motion

## Quick start

```bash
python arm_kinematics.py fk 0 45 45
python arm_kinematics.py ik 180 40 120
python arm_kinematics.py path 140 0 100 190 60 130 --steps 20
```

Angles are degrees; distances are millimetres by default.

## Default geometry

The code starts with example dimensions only:

- Base height: 60 mm
- Upper arm: 120 mm
- Forearm: 120 mm

Replace these with measurements from your actual mechanism.

## Hardware integration checklist

1. Measure link lengths center-to-center.
2. Determine servo zero angles and direction signs.
3. Measure safe joint limits mechanically.
4. Validate FK against known poses.
5. Run IK targets with servos disconnected.
6. Add slow interpolation and an emergency stop before powered testing.

## Portfolio upgrades to add later

- CAD render / annotated dimensions
- Servo driver firmware
- Calibration procedure and measured error
- Workspace visualization
- Pick-and-place demo video
- Repeatability measurements
