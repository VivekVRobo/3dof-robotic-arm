# Worked FK/IK example

This example is a **software-only numerical check** of the repository's analytic kinematics. It does not measure the physical arm.

## Geometry and convention

Use the implementation's default analytic model with explicit values:

- Upper-arm link length: **L1 = 100 mm**
- Forearm link length: **L2 = 100 mm**
- Base height: **H = 30 mm**
- Cartesian target: **(x, y, z) = (120, 40, 90) mm**
- IK branch: **elbow_up=False** (the negative elbow-angle branch)

The base angle **q0** is yaw about the base's Z axis. Shoulder **q1** is measured from the horizontal radial plane, and elbow **q2** is relative to the upper arm. The Python API uses radians; the command-line interface accepts and prints joint angles in degrees.

## Reproduce the inverse-kinematics result

From the repository root, after installing the project with pip install -e .:

~~~bash
python arm_kinematics.py --l1 100 --l2 100 --base-height 30 ik 120 40 90
~~~

Expected CLI output (angles rounded to three decimal places):

~~~text
base=18.435 shoulder=70.950 elbow=-91.146
~~~

## Reconstruct the target with forward kinematics

Pass the displayed angles to the FK command:

~~~bash
python arm_kinematics.py --l1 100 --l2 100 --base-height 30 fk 18.435 70.950 -91.146
~~~

Expected CLI output (Cartesian coordinates in millimetres, rounded to three decimals):

~~~text
x=120.000 y=40.000 z=90.000
~~~

The command-line display is rounded, so it is a readable demonstration rather than the precision assertion. The regression test uses the full-precision angles returned by the actual IK implementation and checks each reconstructed coordinate against the original target with an absolute tolerance of **1e-7 mm**.

## Source of truth and verification

- [Analytic FK/IK implementation](../src/arm3dof/kinematics.py)
- [Worked-example regression test](../tests/test_kinematics.py)
- [Kinematics convention](KINEMATICS.md)

Run the focused test with:

~~~bash
pytest -q tests/test_kinematics.py
~~~

This validates numerical consistency of the ideal rigid-link model only. It is not evidence of physical endpoint accuracy, calibration, servo precision, backlash, repeatability, payload capability, collision clearance, or hardware safety.
