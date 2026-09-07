"""Deterministic workspace and Jacobian characterization for the ideal 3-DOF arm.

The outputs are software-model evidence only. They describe the analytic rigid-link
model over explicit joint ranges and do not imply physical reach, calibration,
collision clearance, payload capability, or servo accuracy.
"""
from __future__ import annotations

from dataclasses import dataclass
from math import cos, hypot, pi, sin
from statistics import median

from .kinematics import ArmGeometry, JointAngles, forward_kinematics


@dataclass(frozen=True)
class WorkspaceSummary:
    samples: int
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float
    horizontal_radius_min: float
    horizontal_radius_max: float
    shoulder_distance_min: float
    shoulder_distance_max: float


@dataclass(frozen=True)
class JacobianSummary:
    samples: int
    abs_det_min: float
    abs_det_p05: float
    abs_det_median: float
    abs_det_p95: float
    abs_det_max: float
    near_singular_relative_threshold: float
    near_singular_samples: int
    near_singular_fraction: float


def _linspace(start: float, stop: float, count: int) -> list[float]:
    if count < 2:
        raise ValueError("count must be >= 2")
    step = (stop - start) / (count - 1)
    return [start + index * step for index in range(count)]


def _percentile(values: list[float], fraction: float) -> float:
    if not values:
        raise ValueError("values must not be empty")
    if not 0.0 <= fraction <= 1.0:
        raise ValueError("fraction must be in [0, 1]")
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = fraction * (len(ordered) - 1)
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def _det3(columns: tuple[tuple[float, float, float], ...]) -> float:
    c0, c1, c2 = columns
    return (
        c0[0] * (c1[1] * c2[2] - c1[2] * c2[1])
        - c1[0] * (c0[1] * c2[2] - c0[2] * c2[1])
        + c2[0] * (c0[1] * c1[2] - c0[2] * c1[1])
    )


def position_jacobian(q: JointAngles, g: ArmGeometry) -> tuple[tuple[float, float, float], ...]:
    """Return the 3x3 Cartesian-position Jacobian as column vectors.

    Joint coordinates are base yaw, shoulder pitch, and elbow pitch in radians.
    """
    b, s, e = q.base, q.shoulder, q.elbow
    radial = g.l1 * cos(s) + g.l2 * cos(s + e)
    dr_ds = -g.l1 * sin(s) - g.l2 * sin(s + e)
    dz_ds = radial
    dr_de = -g.l2 * sin(s + e)
    dz_de = g.l2 * cos(s + e)

    base_column = (-radial * sin(b), radial * cos(b), 0.0)
    shoulder_column = (dr_ds * cos(b), dr_ds * sin(b), dz_ds)
    elbow_column = (dr_de * cos(b), dr_de * sin(b), dz_de)
    return base_column, shoulder_column, elbow_column


def characterize_workspace(
    geometry: ArmGeometry,
    *,
    base_range_deg: tuple[float, float] = (-90.0, 90.0),
    shoulder_range_deg: tuple[float, float] = (-30.0, 120.0),
    elbow_range_deg: tuple[float, float] = (-135.0, -5.0),
    points_per_joint: int = 17,
    near_singular_relative_threshold: float = 1e-6,
) -> tuple[WorkspaceSummary, JacobianSummary]:
    """Sample the ideal workspace and positional Jacobian over explicit joint limits."""
    if points_per_joint < 2:
        raise ValueError("points_per_joint must be >= 2")
    if not 0.0 < near_singular_relative_threshold < 1.0:
        raise ValueError("near_singular_relative_threshold must be in (0, 1)")

    deg = pi / 180.0
    xs: list[float] = []
    ys: list[float] = []
    zs: list[float] = []
    radii: list[float] = []
    shoulder_distances: list[float] = []
    determinants: list[float] = []

    for base_deg in _linspace(*base_range_deg, points_per_joint):
        for shoulder_deg in _linspace(*shoulder_range_deg, points_per_joint):
            for elbow_deg in _linspace(*elbow_range_deg, points_per_joint):
                q = JointAngles(base_deg * deg, shoulder_deg * deg, elbow_deg * deg)
                x, y, z = forward_kinematics(q, geometry)
                xs.append(x)
                ys.append(y)
                zs.append(z)
                radius = hypot(x, y)
                radii.append(radius)
                shoulder_distances.append(hypot(radius, z - geometry.base_height))
                determinants.append(abs(_det3(position_jacobian(q, geometry))))

    samples = len(xs)
    workspace = WorkspaceSummary(
        samples=samples,
        x_min=min(xs),
        x_max=max(xs),
        y_min=min(ys),
        y_max=max(ys),
        z_min=min(zs),
        z_max=max(zs),
        horizontal_radius_min=min(radii),
        horizontal_radius_max=max(radii),
        shoulder_distance_min=min(shoulder_distances),
        shoulder_distance_max=max(shoulder_distances),
    )

    max_det = max(determinants)
    threshold = max_det * near_singular_relative_threshold
    near_count = sum(value <= threshold for value in determinants)
    jacobian = JacobianSummary(
        samples=samples,
        abs_det_min=min(determinants),
        abs_det_p05=_percentile(determinants, 0.05),
        abs_det_median=median(determinants),
        abs_det_p95=_percentile(determinants, 0.95),
        abs_det_max=max_det,
        near_singular_relative_threshold=near_singular_relative_threshold,
        near_singular_samples=near_count,
        near_singular_fraction=near_count / samples,
    )
    return workspace, jacobian
