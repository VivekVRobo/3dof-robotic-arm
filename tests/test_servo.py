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
