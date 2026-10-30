from math import degrees, isclose, radians
import pytest

from arm3dof.kinematics import ArmGeometry, JointAngles, forward_kinematics, inverse_kinematics


def test_fk_ik_roundtrip():
    g = ArmGeometry(100, 80, 30)
    original = JointAngles(radians(20), radians(35), radians(-50))
    p = forward_kinematics(original, g)
    solved = inverse_kinematics(*p, g, elbow_up=False)
    p2 = forward_kinematics(solved, g)
    assert all(isclose(a, b, abs_tol=1e-7) for a, b in zip(p, p2))


def test_documented_fk_ik_worked_example():
    geometry = ArmGeometry(l1=100.0, l2=100.0, base_height=30.0)
    target = (120.0, 40.0, 90.0)

    solved = inverse_kinematics(*target, geometry, elbow_up=False)
    actual_angles_deg = tuple(degrees(angle) for angle in (solved.base, solved.shoulder, solved.elbow))
    expected_angles_deg = (18.4349488229, 70.9499295243, -91.1459919984)

    assert all(abs(actual - expected) <= 1e-6 for actual, expected in zip(actual_angles_deg, expected_angles_deg))

    reconstructed = forward_kinematics(solved, geometry)
    assert max(abs(actual - expected) for actual, expected in zip(reconstructed, target)) <= 1e-7


def test_unreachable_target():
    with pytest.raises(ValueError):
        inverse_kinematics(1000, 0, 0, ArmGeometry())
