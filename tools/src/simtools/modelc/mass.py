from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

Matrix = NDArray[np.float64]
Vector = NDArray[np.float64]


def box_inertia(mass_kg: float, size_m: tuple[float, float, float]) -> Matrix:
    """Solid box about its centre, edges a, b, c along local x, y, z."""
    a, b, c = size_m
    return mass_kg / 12.0 * np.diag([b * b + c * c, a * a + c * c, a * a + b * b])


def cylinder_inertia(mass_kg: float, radius_m: float, height_m: float) -> Matrix:
    """Solid cylinder about its centre, axis along local z."""
    lateral = mass_kg * (3.0 * radius_m * radius_m + height_m * height_m) / 12.0
    return np.diag([lateral, lateral, mass_kg * radius_m * radius_m / 2.0])


def sphere_inertia(mass_kg: float, radius_m: float) -> Matrix:
    return np.eye(3) * (2.0 * mass_kg * radius_m * radius_m / 5.0)


def rotation_frd_from_part(roll_rad: float, pitch_rad: float, yaw_rad: float) -> Matrix:
    """Part orientation in FRD: yaw, then pitch, then roll (R = Rz Ry Rx)."""
    cr, sr = np.cos(roll_rad), np.sin(roll_rad)
    cp, sp = np.cos(pitch_rad), np.sin(pitch_rad)
    cy, sy = np.cos(yaw_rad), np.sin(yaw_rad)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


@dataclass(frozen=True)
class Body:
    """One rigid part: mass, centre in FRD from the model origin, local inertia, rotation."""

    mass_kg: float
    position_frd_m: Vector
    inertia_local: Matrix
    rotation_frd_from_local: Matrix


@dataclass(frozen=True)
class MassProperties:
    mass_kg: float
    cg_frd_m: Vector
    inertia_about_cg: Matrix


def combine(bodies: list[Body]) -> MassProperties:
    """Total mass, CG and the inertia tensor about the CG in FRD (parallel axis theorem)."""
    if not bodies:
        raise ValueError("no bodies")
    mass = sum(b.mass_kg for b in bodies)
    if mass <= 0.0:
        raise ValueError("total mass must be positive")
    cg = sum((b.mass_kg * b.position_frd_m for b in bodies), np.zeros(3)) / mass
    inertia = np.zeros((3, 3))
    for b in bodies:
        rotated = b.rotation_frd_from_local @ b.inertia_local @ b.rotation_frd_from_local.T
        d = b.position_frd_m - cg
        inertia += rotated + b.mass_kg * (float(d @ d) * np.eye(3) - np.outer(d, d))
    return MassProperties(mass_kg=mass, cg_frd_m=cg, inertia_about_cg=inertia)
