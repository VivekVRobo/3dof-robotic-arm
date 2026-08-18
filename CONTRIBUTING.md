# Contributing

- Keep geometry calculations deterministic and unit-tested.
- State angle units explicitly at APIs and protocol boundaries.
- Add tests for new IK branches, limits, or calibration behavior.
- Do not publish guessed dimensions as measured hardware geometry.
- Run `pytest -q` and `python -m compileall -q src arm_kinematics.py` before submitting changes.
