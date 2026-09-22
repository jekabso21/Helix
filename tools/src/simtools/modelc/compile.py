from dataclasses import dataclass
from typing import Any

import numpy as np

from simtools.config import units
from simtools.config.schemas import DroneConfig, MountConfig
from simtools.modelc.frames import quaternion_wxyz_from_rotation
from simtools.modelc.mass import MassProperties, Matrix, Vector, combine, rotation_frd_from_part
from simtools.modelc.parts import MotorPlacement, PlacedPart, all_parts, motor_placements

STANDARD_GRAVITY_MPS2 = 9.80665
COMPILED_SCHEMA_VERSION = 1


@dataclass(frozen=True)
class Mount:
    name: str
    position_frd_m: Vector  # relative to the CG
    rotation_frd_from_mount: Matrix


@dataclass(frozen=True)
class CompiledDrone:
    """Everything simcore and the app need, FRD, SI, positions relative to the CG."""

    name: str
    config: DroneConfig
    mass: MassProperties
    parts: list[PlacedPart]  # positions still in the model-origin frame
    motors: list[MotorPlacement]  # positions relative to the CG
    imu: Mount
    cameras: list[Mount]
    cda_frd_m2: Vector
    cop_frd_m: Vector
    contact_points_frd_m: list[Vector]

    @property
    def cg_from_origin_frd_m(self) -> Vector:
        return self.mass.cg_frd_m

    def max_thrust_per_motor_n(self) -> float:
        m = self.config.motor
        return m.k_t * units.rpm_to_rad_per_s(m.max_rpm) ** 2

    def static_hover_thrust_n(self) -> list[float]:
        """Minimum-norm motor thrusts that balance weight with zero roll and pitch moment."""
        n = len(self.motors)
        rows = np.zeros((3, n))
        for i, m in enumerate(self.motors):
            rows[0, i] = -m.axis_frd[2]
            moment = np.cross(m.position_frd_m, m.axis_frd)
            rows[1, i], rows[2, i] = moment[0], moment[1]
        rhs = np.array([self.mass.mass_kg * STANDARD_GRAVITY_MPS2, 0.0, 0.0])
        solution = np.linalg.lstsq(rows, rhs, rcond=None)[0]
        return [float(t) for t in solution]

    def hover_command(self) -> float:
        """Motor command whose steady-state thrust per motor balances weight on a level drone."""
        thrust = self.mass.mass_kg * STANDARD_GRAVITY_MPS2 / len(self.motors)
        speed = np.sqrt(thrust / self.config.motor.k_t)
        return float(speed / units.rpm_to_rad_per_s(self.config.motor.max_rpm))


def _mount(name: str, cfg: MountConfig, parts: dict[str, PlacedPart], cg: Vector) -> Mount:
    offset = np.array([units.mm_to_m(v) for v in cfg.offset_mm])
    rotation = rotation_frd_from_part(*(units.deg_to_rad(v) for v in cfg.rot_deg))
    if cfg.part is None:
        return Mount(name, offset - cg, rotation)
    base = parts[cfg.part]
    position = base.position_frd_m + base.rotation_frd_from_part @ offset
    return Mount(name, position - cg, base.rotation_frd_from_part @ rotation)


def _aero(drone: DroneConfig, parts: list[PlacedPart], cg: Vector) -> tuple[Vector, Vector]:
    if drone.aero.override is not None:
        cop = np.array([units.mm_to_m(v) for v in drone.aero.override.cop_mm]) - cg
        return np.array(drone.aero.override.cda_m2, dtype=float), cop
    # sum of part projections per axis (overlap ignored); CoP = area-weighted centroid.
    # Props are left out: a spinning disc is mostly open and rotor drag is modelled by k_h
    bluff = [p for p in parts if not p.name.startswith("prop")]
    areas = np.array([p.projected_areas() for p in bluff])
    weights = areas.mean(axis=1)
    positions = np.array([p.position_frd_m for p in bluff])
    cop = (weights[:, None] * positions).sum(axis=0) / weights.sum() - cg
    return drone.aero.cd * areas.sum(axis=0), cop


def compile_drone(drone: DroneConfig) -> CompiledDrone:
    placements = motor_placements(drone.layout)
    parts = all_parts(drone, placements)
    mass = combine([p.body() for p in parts])
    cg = mass.cg_frd_m
    by_name = {p.name: p for p in parts}
    leg_drop = units.mm_to_m(drone.contact.leg_drop_mm)
    cda, cop = _aero(drone, parts, cg)
    return CompiledDrone(
        name=drone.name,
        config=drone,
        mass=mass,
        parts=parts,
        motors=[
            MotorPlacement(m.bf_index, m.position_frd_m - cg, m.axis_frd, m.spin)
            for m in placements
        ],
        imu=_mount("imu", drone.imu, by_name, cg),
        cameras=[_mount(c.name, c, by_name, cg) for c in drone.cameras],
        cda_frd_m2=cda,
        cop_frd_m=cop,
        contact_points_frd_m=[
            m.position_frd_m + np.array([0.0, 0.0, leg_drop]) - cg for m in placements
        ],
    )


def _list(v: Vector) -> list[float]:
    return [float(x) for x in v]


def to_json(model: CompiledDrone) -> dict[str, Any]:
    motor = model.config.motor
    contact = model.config.contact
    cg = model.cg_from_origin_frd_m
    return {
        "schema_version": COMPILED_SCHEMA_VERSION,
        "name": model.name,
        "mass_kg": model.mass.mass_kg,
        "cg_from_origin_frd_m": _list(cg),
        "inertia_frd_kg_m2": [_list(row) for row in model.mass.inertia_about_cg],
        "motors": [
            {
                "bf_index": m.bf_index,
                "position_frd_m": _list(m.position_frd_m),
                "axis_frd": _list(m.axis_frd),
                "spin": m.spin,
                "rotor_inertia_kg_m2": motor.rotor_inertia_kg_m2,
                "first_order": {
                    "max_speed_radps": units.rpm_to_rad_per_s(motor.max_rpm),
                    "time_constant_s": motor.time_constant_s,
                    "k_t": motor.k_t,
                    "k_q": motor.k_q,
                    "k_h": motor.k_h,
                },
            }
            for m in model.motors
        ],
        "imu": {
            "position_frd_m": _list(model.imu.position_frd_m),
            "q_frd_from_imu": list(
                quaternion_wxyz_from_rotation(model.imu.rotation_frd_from_mount)
            ),
        },
        "cameras": [
            {
                "name": c.name,
                "position_frd_m": _list(c.position_frd_m),
                "q_frd_from_camera": list(quaternion_wxyz_from_rotation(c.rotation_frd_from_mount)),
            }
            for c in model.cameras
        ],
        "aero": {
            "cda_frd_m2": _list(model.cda_frd_m2),
            "cop_frd_m": _list(model.cop_frd_m),
            "k_omega": list(model.config.aero.k_omega),
        },
        "contact": {
            "points_frd_m": [_list(p) for p in model.contact_points_frd_m],
            "stiffness_n_per_m": contact.stiffness_n_per_m,
            "damping_n_s_per_m": contact.damping_n_s_per_m,
            "friction": contact.friction,
            "friction_regularization_mps": contact.friction_regularization_mps,
            "crash_speed_mps": contact.crash_speed_mps,
        },
        "parts": [
            {
                "name": p.name,
                "shape": p.shape,
                "dims_m": list(p.dims_m),
                "mass_kg": p.mass_kg,
                "position_frd_m": _list(p.position_frd_m - cg),
                "q_frd_from_part": list(quaternion_wxyz_from_rotation(p.rotation_frd_from_part)),
                "color": p.color,
                "generated": p.generated,
            }
            for p in model.parts
        ],
    }


def _fmt(v: Vector, digits: int) -> str:
    return ", ".join(f"{x:.{digits}f}" for x in v)


def _mm(v: Vector) -> str:
    return _fmt(v * 1e3, 1)


def report(model: CompiledDrone) -> str:
    cg = model.cg_from_origin_frd_m
    inertia = model.mass.inertia_about_cg
    lines = [
        f"# {model.name}",
        "",
        f"- mass: {model.mass.mass_kg * 1e3:.1f} g",
        f"- CG from model origin (FRD): {_mm(cg)} mm",
        f"- CdA (FRD): {_fmt(model.cda_frd_m2, 4)} m2",
        f"- CoP from CG (FRD): {_mm(model.cop_frd_m)} mm",
        "",
        "## Inertia about the CG (kg m2)",
        "",
        "```",
        *(f"{row[0]: .3e} {row[1]: .3e} {row[2]: .3e}" for row in inertia),
        "```",
        "",
        "## Static hover loads",
        "",
        "| motor | position from CG (mm) | spin | thrust (N) | of max |",
        "|---|---|---|---|---|",
    ]
    max_thrust = model.max_thrust_per_motor_n()
    for m, thrust in zip(model.motors, model.static_hover_thrust_n(), strict=True):
        lines.append(
            f"| {m.bf_index} | {_mm(m.position_frd_m)} | {'CW' if m.spin > 0 else 'CCW'} "
            f"| {thrust:.3f} | {100.0 * thrust / max_thrust:.1f}% |"
        )
    lines += ["", "## Parts", "", "| part | mass (g) | position from CG (mm) |", "|---|---|---|"]
    for p in sorted(model.parts, key=lambda p: -p.mass_kg):
        lines.append(f"| {p.name} | {p.mass_kg * 1e3:.1f} | {_mm(p.position_frd_m - cg)} |")
    return "\n".join(lines) + "\n"
