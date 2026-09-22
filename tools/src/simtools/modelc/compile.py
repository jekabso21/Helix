import math
from dataclasses import dataclass
from itertools import pairwise
from typing import Any

import numpy as np

from simtools.config import units
from simtools.config.schemas import BatteryConfig, DroneConfig, MotorConfig, MountConfig, PropConfig
from simtools.modelc.frames import quaternion_wxyz_from_rotation
from simtools.modelc.mass import MassProperties, Matrix, Vector, combine, rotation_frd_from_part
from simtools.modelc.parts import MotorPlacement, PlacedPart, all_parts, motor_placements

STANDARD_GRAVITY_MPS2 = 9.80665
COMPILED_SCHEMA_VERSION = 2


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

    def bus_voltage_v(self) -> float:
        b = self.config.battery
        return b.cells * _ocv(b)

    def full_throttle_speed_radps(self) -> float:
        """Steady no-inflow speed at full throttle and the fresh-pack voltage."""
        return steady_speed_radps(self.config.motor, self.config.prop, 1.0, self.bus_voltage_v())

    def max_thrust_per_motor_n(self) -> float:
        w = self.full_throttle_speed_radps()
        return self.config.prop.k_t * w * w

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
        return hover_command(self.config.motor, self.config.prop, thrust, self.bus_voltage_v())


def _ocv(b: BatteryConfig) -> float:
    x = b.initial_soc * 10.0
    lower = min(int(x), 9)
    return b.ocv_curve_v[lower] + (b.ocv_curve_v[lower + 1] - b.ocv_curve_v[lower]) * (x - lower)


def kv_radps_per_v(motor: MotorConfig) -> float:
    return units.rpm_to_rad_per_s(motor.kv_rpm_per_v)


def steady_speed_radps(motor: MotorConfig, prop: PropConfig, command: float, bus_v: float) -> float:
    """Speed where motor torque equals prop drag torque (dc), or the first-order target."""
    if motor.model == "first_order":
        assert motor.max_rpm is not None
        return command * units.rpm_to_rad_per_s(motor.max_rpm) * bus_v / motor.reference_voltage_v
    # k_q w^2 + (Kt^2 / R) w - Kt (u V / R - I0) = 0, positive root
    kt = 1.0 / kv_radps_per_v(motor)
    a = prop.k_q
    b = kt * kt / motor.winding_resistance_ohm
    c = -kt * (command * bus_v / motor.winding_resistance_ohm - motor.no_load_current_a)
    if c >= 0.0:
        return 0.0
    return (-b + math.sqrt(b * b - 4.0 * a * c)) / (2.0 * a)


def hover_command(motor: MotorConfig, prop: PropConfig, thrust_n: float, bus_v: float) -> float:
    speed = math.sqrt(thrust_n / prop.k_t)
    if motor.model == "first_order":
        assert motor.max_rpm is not None
        return speed / units.rpm_to_rad_per_s(motor.max_rpm) * motor.reference_voltage_v / bus_v
    kt = 1.0 / kv_radps_per_v(motor)
    current = motor.no_load_current_a + prop.k_q * speed * speed / kt
    return (current * motor.winding_resistance_ohm + kt * speed) / bus_v


def _interp(table: tuple[tuple[float, float], ...], x: float) -> float:
    if x <= table[0][0]:
        return table[0][1]
    for (x0, y0), (x1, y1) in pairwise(table):
        if x <= x1:
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
    return table[-1][1]


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
    prop = model.config.prop
    battery = model.config.battery
    sensors = model.config.sensors
    imu = sensors.imu
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
                "motor": {
                    "model": motor.model,
                    "max_speed_radps": units.rpm_to_rad_per_s(motor.max_rpm or 0.0),
                    "time_constant_s": motor.time_constant_s,
                    "reference_voltage_v": motor.reference_voltage_v,
                    "kv_radps_per_v": kv_radps_per_v(motor),
                    "resistance_ohm": motor.winding_resistance_ohm,
                    "no_load_current_a": motor.no_load_current_a,
                    "brake_current_a": motor.brake_current_a,
                    "rotor_inertia_kg_m2": motor.rotor_inertia_kg_m2,
                    "pole_pairs": motor.poles // 2,
                },
                "prop": {
                    "k_t": prop.k_t,
                    "k_q": prop.k_q,
                    "rho_ref_kg_m3": prop.rho_ref_kg_m3,
                    "radius_m": units.mm_to_m(prop.diameter_mm) / 2.0,
                    "pitch_m": units.mm_to_m(prop.pitch_mm),
                    "inflow_coefficient": prop.inflow_coefficient,
                    "rotor_drag_coefficient": prop.k_h,
                    "blades": prop.blades,
                },
            }
            for m in model.motors
        ],
        "battery": {
            "cells": battery.cells,
            "capacity_ah": battery.capacity_mah * 1e-3,
            "cell_resistance_ohm": battery.cell_resistance_mohm
            * 1e-3
            * _interp(battery.temperature_factor, battery.temperature_c),
            "connector_resistance_ohm": battery.connector_resistance_mohm * 1e-3,
            "avionics_current_a": battery.avionics_current_a,
            "esc_cutoff_v": battery.esc_cutoff_v,
            "initial_soc": battery.initial_soc,
            "rc_resistance_ohm": battery.rc_resistance_mohm * 1e-3,
            "rc_capacitance_f": battery.rc_capacitance_f,
            "ocv_v": list(battery.ocv_curve_v),
        },
        "sensors": {
            "imu": {
                "gyro_noise_density_radps_rthz": units.deg_to_rad(imu.gyro_noise_density_dps_rthz),
                "gyro_bias_walk_radps2_rthz": units.deg_to_rad(imu.gyro_bias_walk_dps2_rthz),
                "gyro_range_radps": units.deg_to_rad(imu.gyro_range_dps),
                "accel_noise_density_mps2_rthz": imu.accel_noise_density_mps2_rthz,
                "accel_bias_walk_mps3_rthz": imu.accel_bias_walk_mps3_rthz,
                "accel_range_mps2": imu.accel_range_g * STANDARD_GRAVITY_MPS2,
                "vibration_imbalance_mps2_per_radps2": imu.vibration_imbalance_mps2_per_radps2,
                "vibration_harmonic2": imu.vibration_harmonic2,
                "vibration_blade_pass": imu.vibration_blade_pass,
                "vibration_gyro_gain_radps_per_mps2": imu.vibration_gyro_gain_radps_per_mps2,
            },
            "baro": {"noise_pa": sensors.baro.noise_pa, "bias_pa": sensors.baro.bias_pa},
        },
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
