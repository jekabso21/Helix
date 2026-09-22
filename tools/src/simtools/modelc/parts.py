import math
from dataclasses import dataclass
from typing import Literal

import numpy as np

from simtools.config import units
from simtools.config.schemas import (
    BoxPart,
    CylinderPart,
    DroneConfig,
    LayoutConfig,
    MeshPart,
    Part,
    SpherePart,
)
from simtools.modelc.mass import (
    Body,
    Matrix,
    Vector,
    box_inertia,
    cylinder_inertia,
    rotation_frd_from_part,
    sphere_inertia,
)

Shape = Literal["box", "cylinder", "sphere"]

DEFAULT_COLORS = {"arm": "#404040", "motor": "#202020", "prop": "#9a9a9a", "part": "#7a8a99"}


@dataclass(frozen=True)
class PlacedPart:
    """A primitive in the model-origin frame (FRD, metres); dims are (a, b, c), (r, h) or (r,)."""

    name: str
    shape: Shape
    dims_m: tuple[float, ...]
    mass_kg: float
    position_frd_m: Vector
    rotation_frd_from_part: Matrix
    color: str
    generated: bool

    def body(self) -> Body:
        return Body(
            self.mass_kg, self.position_frd_m, self.inertia_local(), self.rotation_frd_from_part
        )

    def inertia_local(self) -> Matrix:
        if self.shape == "box":
            a, b, c = self.dims_m
            return box_inertia(self.mass_kg, (a, b, c))
        if self.shape == "cylinder":
            return cylinder_inertia(self.mass_kg, self.dims_m[0], self.dims_m[1])
        return sphere_inertia(self.mass_kg, self.dims_m[0])

    def projected_areas(self) -> Vector:
        """Projected area onto the planes normal to the body x, y, z axes."""
        axes = self.rotation_frd_from_part  # columns are the part's local axes in FRD
        out = np.zeros(3)
        for k in range(3):
            d = np.zeros(3)
            d[k] = 1.0
            cosines = np.abs(axes.T @ d)
            if self.shape == "box":
                a, b, c = self.dims_m
                out[k] = cosines[0] * b * c + cosines[1] * a * c + cosines[2] * a * b
            elif self.shape == "cylinder":
                r, h = self.dims_m
                axial = cosines[2]
                out[k] = math.pi * r * r * axial + 2.0 * r * h * math.sqrt(
                    max(0.0, 1.0 - axial * axial)
                )
            else:
                out[k] = math.pi * self.dims_m[0] ** 2
        return out


@dataclass(frozen=True)
class MotorPlacement:
    bf_index: int
    position_frd_m: Vector  # model-origin frame
    axis_frd: Vector
    spin: float  # +1 CW seen from above, -1 CCW


def rotation_aligning_z_to(target: Vector) -> Matrix:
    """Shortest rotation taking local +z onto the unit vector target."""
    z = np.array([0.0, 0.0, 1.0])
    t = target / np.linalg.norm(target)
    c = float(z @ t)
    if c > 1.0 - 1e-12:
        return np.eye(3)
    if c < -1.0 + 1e-12:
        return np.diag([1.0, -1.0, -1.0])
    v = np.cross(z, t)
    vx = np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])
    return np.eye(3) + vx + vx @ vx / (1.0 + c)


def motor_placements(layout: LayoutConfig) -> list[MotorPlacement]:
    """Betaflight quad X: 1 rear right CW, 2 front right CCW, 3 rear left CCW, 4 front left CW."""
    up = np.array([0.0, 0.0, -1.0])
    if layout.type == "custom":
        return [
            MotorPlacement(
                m.bf_index,
                np.array([units.mm_to_m(v) for v in m.pos_mm]),
                np.array(m.axis, dtype=float) / np.linalg.norm(m.axis),
                1.0 if m.spin == "cw" else -1.0,
            )
            for m in sorted(layout.motors, key=lambda m: m.bf_index)
        ]
    if layout.type == "quad_x":
        assert layout.motor_spacing_mm is not None
        half = units.mm_to_m(layout.motor_spacing_mm) / 2.0 / math.sqrt(2.0)
        half_x, half_y = half, half
    else:
        assert layout.length_mm is not None and layout.width_mm is not None
        half_x, half_y = units.mm_to_m(layout.length_mm) / 2.0, units.mm_to_m(layout.width_mm) / 2.0
    corners = [
        (-half_x, half_y, 1.0),
        (half_x, half_y, -1.0),
        (-half_x, -half_y, -1.0),
        (half_x, -half_y, 1.0),
    ]
    flip = -1.0 if layout.props_out else 1.0
    return [
        MotorPlacement(i + 1, np.array([x, y, 0.0]), up, flip * spin)
        for i, (x, y, spin) in enumerate(corners)
    ]


def generated_parts(layout: LayoutConfig, motors: list[MotorPlacement]) -> list[PlacedPart]:
    """Arm, motor body and prop disc per motor; arm top face is in the motor plane."""
    out: list[PlacedPart] = []
    arm, body, prop = layout.arm, layout.motor_body, layout.prop_body
    arm_w, arm_t = units.mm_to_m(arm.width_mm), units.mm_to_m(arm.thickness_mm)
    body_r, body_h = units.mm_to_m(body.diameter_mm) / 2.0, units.mm_to_m(body.height_mm)
    prop_r, prop_t = units.mm_to_m(prop.diameter_mm) / 2.0, units.mm_to_m(prop.thickness_mm)
    for m in motors:
        x, y, z = m.position_frd_m
        length = math.hypot(x, y)
        arm_rot = rotation_frd_from_part(0.0, 0.0, math.atan2(y, x))
        out.append(
            PlacedPart(
                f"arm{m.bf_index}",
                "box",
                (length, arm_w, arm_t),
                units.g_to_kg(arm.mass_g),
                np.array([x / 2.0, y / 2.0, z + arm_t / 2.0]),
                arm_rot,
                arm.color or DEFAULT_COLORS["arm"],
                True,
            )
        )
        thrust_rot = rotation_aligning_z_to(-m.axis_frd)
        out.append(
            PlacedPart(
                f"motor{m.bf_index}",
                "cylinder",
                (body_r, body_h),
                units.g_to_kg(body.mass_g),
                m.position_frd_m + m.axis_frd * (body_h / 2.0),
                thrust_rot,
                body.color or DEFAULT_COLORS["motor"],
                True,
            )
        )
        out.append(
            PlacedPart(
                f"prop{m.bf_index}",
                "cylinder",
                (prop_r, prop_t),
                units.g_to_kg(prop.mass_g),
                m.position_frd_m + m.axis_frd * (body_h + prop_t / 2.0),
                thrust_rot,
                prop.color or DEFAULT_COLORS["prop"],
                True,
            )
        )
    return out


def configured_part(name: str, part: Part) -> PlacedPart:
    rotation = rotation_frd_from_part(*(units.deg_to_rad(v) for v in part.rot_deg))
    position = np.array([units.mm_to_m(v) for v in part.pos_mm])
    color = part.color or DEFAULT_COLORS["part"]
    mass = units.g_to_kg(part.mass_g)
    if isinstance(part, BoxPart):
        dims = tuple(units.mm_to_m(v) for v in part.size_mm)
        return PlacedPart(name, "box", dims, mass, position, rotation, color, False)
    if isinstance(part, CylinderPart):
        dims = (units.mm_to_m(part.diameter_mm) / 2.0, units.mm_to_m(part.height_mm))
        return PlacedPart(name, "cylinder", dims, mass, position, rotation, color, False)
    if isinstance(part, SpherePart):
        return PlacedPart(
            name,
            "sphere",
            (units.mm_to_m(part.diameter_mm) / 2.0,),
            mass,
            position,
            rotation,
            color,
            False,
        )
    assert isinstance(part, MeshPart)
    raise NotImplementedError("mesh parts")


def all_parts(drone: DroneConfig, motors: list[MotorPlacement]) -> list[PlacedPart]:
    return [configured_part(n, p) for n, p in drone.parts.items()] + generated_parts(
        drone.layout, motors
    )
