from math import radians
import pytest

from arm3dof.kinematics import JointAngles
from arm3dof.servo import JointCalibration, ServoCalibration, encode_joint_command


def test_default_mapping():
    mapped = ServoCalibration().map(JointAngles(0.0, radians(10), radians(-10)))
    assert mapped == (90.0, 100.0, 80.0)


def test_limit_rejection():
    calibration = JointCalibration(offset_deg=90, min_deg=20, max_deg=160)
    with pytest.raises(ValueError):
        calibration.map_radians(radians(100))


def test_protocol():
    assert encode_joint_command((90, 80, 70)) == b"J,90.0,80.0,70.0\n"


@pytest.mark.parametrize("bad_value", [float("nan"), float("inf"), float("-inf")])
def test_joint_calibration_rejects_non_finite_configuration(bad_value):
    with pytest.raises(ValueError, match="finite"):
        JointCalibration(offset_deg=bad_value)
    with pytest.raises(ValueError, match="finite"):
        JointCalibration(scale=bad_value)
    with pytest.raises(ValueError, match="finite"):
        JointCalibration(min_deg=bad_value)
    with pytest.raises(ValueError, match="finite"):
        JointCalibration(max_deg=bad_value)


def test_joint_calibration_rejects_reversed_limits():
    with pytest.raises(ValueError, match="minimum"):
        JointCalibration(min_deg=160, max_deg=20)


@pytest.mark.parametrize("bad_value", [float("nan"), float("inf"), float("-inf")])
def test_map_rejects_non_finite_joint_angle(bad_value):
    with pytest.raises(ValueError, match="finite"):
        JointCalibration().map_radians(bad_value)


@pytest.mark.parametrize("bad_value", [float("nan"), float("inf"), float("-inf")])
def test_protocol_rejects_non_finite_command(bad_value):
    for index in range(3):
        command = [90.0, 80.0, 70.0]
        command[index] = bad_value
        with pytest.raises(ValueError, match="finite"):
            encode_joint_command(tuple(command))
