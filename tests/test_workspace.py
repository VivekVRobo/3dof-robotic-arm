from math import isclose

from arm3dof.kinematics import ArmGeometry, JointAngles
from arm3dof.workspace import characterize_workspace, position_jacobian


def test_position_jacobian_has_expected_base_column_at_zero_yaw():
    g = ArmGeometry(100.0, 100.0, 30.0)
    q = JointAngles(0.0, 0.0, 0.0)
    base, shoulder, elbow = position_jacobian(q, g)
    assert base == (0.0, 200.0, 0.0)
    assert shoulder == (0.0, 0.0, 200.0)
    assert elbow == (0.0, 0.0, 100.0)


def test_workspace_characterization_is_deterministic_and_bounded():
    g = ArmGeometry(100.0, 100.0, 30.0)
    workspace, jacobian = characterize_workspace(g, points_per_joint=5)
    assert workspace.samples == 125
    assert jacobian.samples == 125
    assert workspace.horizontal_radius_max <= 200.0 + 1e-9
    assert workspace.shoulder_distance_max <= 200.0 + 1e-9
    assert workspace.z_max > workspace.z_min
    assert jacobian.abs_det_max > 0.0
    assert 0.0 <= jacobian.near_singular_fraction <= 1.0


def test_straight_elbow_configuration_is_singular_in_position_jacobian():
    g = ArmGeometry(100.0, 100.0, 30.0)
    q = JointAngles(0.0, 0.0, 0.0)
    columns = position_jacobian(q, g)
    c0, c1, c2 = columns
    det = (
        c0[0] * (c1[1] * c2[2] - c1[2] * c2[1])
        - c1[0] * (c0[1] * c2[2] - c0[2] * c2[1])
        + c2[0] * (c0[1] * c1[2] - c0[2] * c1[1])
    )
    assert isclose(det, 0.0, abs_tol=1e-12)
