"""Numerical validation helpers for the 3-DOF arm.

These routines validate the analytic model and quantify sensitivity of the
ideal kinematic model. They are *not* physical accuracy measurements.
"""
from __future__ import annotations

from dataclasses import dataclass
from math import pi, sqrt

from .kinematics import ArmGeometry, JointAngles, forward_kinematics, inverse_kinematics


@dataclass(frozen=True)
class ValidationSummary:
    samples: int
    ik_failures: int
    max_reconstruction_error: float
    rms_reconstruction_error: float
    mean_reconstruction_error: float


@dataclass(frozen=True)
class SensitivitySummary:
    perturbation_deg: float
    samples: int
    mean_endpoint_shift: float
    rms_endpoint_shift: float
    max_endpoint_shift: float


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def _linspace(start: float, stop: float, count: int) -> list[float]:
    if count < 2:
        raise ValueError("count must be >= 2")
    step = (stop - start) / (count - 1)
    return [start + i * step for i in range(count)]


def validate_fk_ik_grid(
    geometry: ArmGeometry,
    *,
    base_range_deg: tuple[float, float] = (-90.0, 90.0),
    shoulder_range_deg: tuple[float, float] = (-30.0, 120.0),
    elbow_range_deg: tuple[float, float] = (-135.0, -5.0),
    points_per_joint: int = 9,
) -> ValidationSummary:
    """Run FK -> IK -> FK over a deterministic joint-space grid.

    The generated points are known to be reachable because they originate from
    forward kinematics. IK failures or reconstruction error therefore expose
    numerical/model inconsistencies rather than physical performance.
    """
    if points_per_joint < 2:
        raise ValueError("points_per_joint must be >= 2")

    deg = pi / 180.0
    errors: list[float] = []
    failures = 0
    for b in _linspace(*base_range_deg, points_per_joint):
        for s in _linspace(*shoulder_range_deg, points_per_joint):
            for e in _linspace(*elbow_range_deg, points_per_joint):
                original = JointAngles(b * deg, s * deg, e * deg)
                xyz = forward_kinematics(original, geometry)
                try:
                    recovered = inverse_kinematics(*xyz, geometry, elbow_up=False)
                except ValueError:
                    failures += 1
                    continue
                reconstructed = forward_kinematics(recovered, geometry)
                errors.append(_distance(xyz, reconstructed))

    total = points_per_joint**3
    if not errors:
        return ValidationSummary(total, failures, float("inf"), float("inf"), float("inf"))
    squared = sum(x * x for x in errors)
    return ValidationSummary(
        samples=total,
        ik_failures=failures,
        max_reconstruction_error=max(errors),
        rms_reconstruction_error=sqrt(squared / len(errors)),
        mean_reconstruction_error=sum(errors) / len(errors),
    )


def endpoint_angle_sensitivity(
    geometry: ArmGeometry,
    *,
    perturbation_deg: float = 1.0,
    points_per_joint: int = 7,
) -> SensitivitySummary:
    """Estimate ideal endpoint sensitivity to small joint-angle perturbations.

    Each sampled pose is perturbed independently by +/- ``perturbation_deg`` on
    every joint. The result is a geometric sensitivity bound for the ideal
    rigid-link model, not a servo specification or measured positioning error.
    """
    if perturbation_deg <= 0:
        raise ValueError("perturbation_deg must be > 0")
    if points_per_joint < 2:
        raise ValueError("points_per_joint must be >= 2")

    deg = pi / 180.0
    delta = perturbation_deg * deg
    shifts: list[float] = []
    for b in _linspace(-90.0, 90.0, points_per_joint):
        for s in _linspace(-30.0, 120.0, points_per_joint):
            for e in _linspace(-135.0, -5.0, points_per_joint):
                q = JointAngles(b * deg, s * deg, e * deg)
                nominal = forward_kinematics(q, geometry)
                for joint in ("base", "shoulder", "elbow"):
                    for sign in (-1.0, 1.0):
                        perturbed = JointAngles(
                            q.base + (sign * delta if joint == "base" else 0.0),
                            q.shoulder + (sign * delta if joint == "shoulder" else 0.0),
                            q.elbow + (sign * delta if joint == "elbow" else 0.0),
                        )
                        shifts.append(_distance(nominal, forward_kinematics(perturbed, geometry)))

    squared = sum(x * x for x in shifts)
    return SensitivitySummary(
        perturbation_deg=perturbation_deg,
        samples=len(shifts),
        mean_endpoint_shift=sum(shifts) / len(shifts),
        rms_endpoint_shift=sqrt(squared / len(shifts)),
        max_endpoint_shift=max(shifts),
    )
