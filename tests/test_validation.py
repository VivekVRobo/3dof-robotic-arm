from arm3dof.kinematics import ArmGeometry
from arm3dof.validation import endpoint_angle_sensitivity, validate_fk_ik_grid


def test_fk_ik_grid_reconstructs_reachable_points():
    summary = validate_fk_ik_grid(ArmGeometry(), points_per_joint=5)
    assert summary.samples == 125
    assert summary.ik_failures == 0
    assert summary.max_reconstruction_error < 1e-8


def test_angle_sensitivity_is_positive_and_bounded_by_arm_scale():
    geometry = ArmGeometry(l1=100.0, l2=100.0, base_height=30.0)
    summary = endpoint_angle_sensitivity(geometry, perturbation_deg=1.0, points_per_joint=4)
    assert summary.samples == 4**3 * 6
    assert 0.0 < summary.mean_endpoint_shift
    assert summary.mean_endpoint_shift <= summary.rms_endpoint_shift <= summary.max_endpoint_shift
    assert summary.max_endpoint_shift < 10.0


def test_larger_angle_perturbation_increases_sensitivity():
    geometry = ArmGeometry()
    small = endpoint_angle_sensitivity(geometry, perturbation_deg=0.5, points_per_joint=3)
    large = endpoint_angle_sensitivity(geometry, perturbation_deg=1.0, points_per_joint=3)
    assert large.mean_endpoint_shift > small.mean_endpoint_shift
