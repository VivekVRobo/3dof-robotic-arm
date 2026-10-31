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


@pytest.mark.parametrize("bad_value", [float("nan"), float("inf"), float("-inf")])
def test_arm_geometry_rejects_non_finite_values(bad_value):
    for kwargs in (
        {"l1": bad_value},
        {"l2": bad_value},
        {"base_height": bad_value},
    ):
        with pytest.raises(ValueError, match="finite"):
            ArmGeometry(**kwargs)


@pytest.mark.parametrize("bad_value", [float("nan"), float("inf"), float("-inf")])
def test_joint_angles_reject_non_finite_values(bad_value):
    for angles in (
        (bad_value, 0.0, 0.0),
        (0.0, bad_value, 0.0),
        (0.0, 0.0, bad_value),
    ):
        with pytest.raises(ValueError, match="finite"):
            JointAngles(*angles)


@pytest.mark.parametrize("bad_value", [float("nan"), float("inf"), float("-inf")])
def test_inverse_kinematics_rejects_non_finite_target_coordinates(bad_value):
    for index in range(3):
        target = [10.0, 0.0, 100.0]
        target[index] = bad_value
        with pytest.raises(ValueError, match="finite"):
            inverse_kinematics(*target, ArmGeometry())
