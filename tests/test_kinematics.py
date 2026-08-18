import math

from arm_kinematics import forward_kinematics, inverse_kinematics


def close(a, b, tol=1e-6):
    return math.isclose(a, b, abs_tol=tol)


def test_straight_pose():
    x, y, z = forward_kinematics(0, 0, 0)
    assert close(x, 240.0)
    assert close(y, 0.0)
    assert close(z, 60.0)


def test_fk_ik_round_trip():
    target = forward_kinematics(25, 35, 50)
    joints = inverse_kinematics(*target)
    rebuilt = forward_kinematics(*joints)
    assert all(close(a, b, 1e-5) for a, b in zip(target, rebuilt))


def test_unreachable_target():
    try:
        inverse_kinematics(1000, 0, 0)
    except ValueError:
        return
    raise AssertionError("Expected unreachable target to raise ValueError")
