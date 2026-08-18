import pytest

from arm3dof.trajectory import cartesian_line


def test_cartesian_line_endpoints():
    points = cartesian_line((0, 0, 0), (10, 20, 30), 3)
    assert points[0] == (0.0, 0.0, 0.0)
    assert points[-1] == (10.0, 20.0, 30.0)
    assert points[1] == (5.0, 10.0, 15.0)


def test_steps_validation():
    with pytest.raises(ValueError):
        cartesian_line((0, 0, 0), (1, 1, 1), 1)
