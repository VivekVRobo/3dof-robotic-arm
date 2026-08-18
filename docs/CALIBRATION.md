# Mechanical and Servo Calibration

1. Disconnect links if necessary and identify each servo's safe electrical midpoint.
2. Assemble the arm so the mathematical zero pose corresponds to known physical angles.
3. Measure actual link pivot-to-pivot lengths and update `ArmGeometry`.
4. Determine each joint's offset, direction (`scale` +1/-1), and safe min/max servo angle.
5. Test one joint at a time before coordinated motion.
6. Verify FK-predicted end-effector position against several measured poses.

Calibration values should come from the real arm. The default 90° offsets in software are neutral placeholders, not measured values.
