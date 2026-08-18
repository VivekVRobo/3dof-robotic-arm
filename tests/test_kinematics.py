from math import isclose, radians
import pytest

from arm3dof.kinematics import ArmGeometry, JointAngles, forward_kinematics, inverse_kinematics


def test_fk_ik_roundtrip():
    g = ArmGeometry(100, 80, 30)
    original = JointAngles(radians(20), radians(35), radians(-50))
    p = forward_kinematics(original, g)
    solved = inverse_kinematics(*p, g, elbow_up=False)
    p2 = forward_kinematics(solved, g)
    assert all(isclose(a, b, abs_tol=1e-7) for a, b in zip(p, p2))


def test_unreachable_target():
    with pytest.raises(ValueError):
        inverse_kinematics(1000, 0, 0, ArmGeometry())
