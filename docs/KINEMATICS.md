# Kinematics

For joint angles `(q0, q1, q2)`:

```text
r = L1 cos(q1) + L2 cos(q1 + q2)
x = r cos(q0)
y = r sin(q0)
z = H + L1 sin(q1) + L2 sin(q1 + q2)
```

Inverse kinematics first solves base yaw with `atan2(y, x)`, then solves the planar two-link arm with the law of cosines. Two elbow branches are possible when the target is inside the workspace.

The solver rejects points outside the ideal geometric workspace. Real hardware needs stricter joint limits and collision constraints.
