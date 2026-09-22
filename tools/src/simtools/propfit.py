import csv
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from simtools.config import units

STANDARD_GRAVITY_MPS2 = 9.80665
REQUIRED = ("throttle", "voltage_v", "rpm", "current_a")


@dataclass(frozen=True)
class ThrustRow:
    throttle: float
    voltage_v: float
    rpm: float
    current_a: float
    thrust_n: float
    torque_nm: float | None
    air_density_kg_m3: float


@dataclass(frozen=True)
class PropFit:
    k_t: float
    k_q: float
    rho_ref_kg_m3: float
    winding_resistance_ohm: float
    no_load_current_a: float
    kv_rpm_per_v: float
    max_thrust_error: float  # largest relative thrust error of the fit over the table
    max_current_error: float


class ThrustTableError(ValueError):
    pass


def read_thrust_table(path: Path) -> list[ThrustRow]:
    """CSV with throttle (0..1), voltage_v, rpm, current_a, thrust_g or thrust_n; optional
    torque_nm and air_density_kg_m3 (default 1.225)."""
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        columns = set(reader.fieldnames or [])
        missing = [c for c in REQUIRED if c not in columns]
        if missing or not ({"thrust_g", "thrust_n"} & columns):
            raise ThrustTableError(f"{path}: missing columns {missing or ['thrust_g or thrust_n']}")
        rows: list[ThrustRow] = []
        for record in reader:
            thrust = (
                float(record["thrust_n"])
                if "thrust_n" in columns and record["thrust_n"] != ""
                else units.g_to_kg(float(record["thrust_g"])) * STANDARD_GRAVITY_MPS2
            )
            torque = record.get("torque_nm")
            density = record.get("air_density_kg_m3")
            rows.append(
                ThrustRow(
                    throttle=float(record["throttle"]),
                    voltage_v=float(record["voltage_v"]),
                    rpm=float(record["rpm"]),
                    current_a=float(record["current_a"]),
                    thrust_n=thrust,
                    torque_nm=float(torque) if torque not in (None, "") else None,
                    air_density_kg_m3=float(density) if density not in (None, "") else 1.225,
                )
            )
    if len(rows) < 3:
        raise ThrustTableError(f"{path}: need at least 3 rows")
    return rows


def _through_origin(x: np.ndarray, y: np.ndarray) -> float:
    return float(x @ y / (x @ x))


def fit_prop(rows: list[ThrustRow], kv_rpm_per_v: float, rho_ref_kg_m3: float = 1.225) -> PropFit:
    """Least squares: T = k_t rho_r w^2; Kt (I - I0) = k_q w^2 (or torque); u V - Ke w = I R."""
    spinning = [r for r in rows if r.rpm > 0.0]
    w = np.array([units.rpm_to_rad_per_s(r.rpm) for r in spinning])
    rho_r = np.array([r.air_density_kg_m3 / rho_ref_kg_m3 for r in spinning])
    thrust = np.array([r.thrust_n for r in spinning])
    current = np.array([r.current_a for r in spinning])
    v_motor = np.array([r.throttle * r.voltage_v for r in spinning])
    kt = 1.0 / units.rpm_to_rad_per_s(kv_rpm_per_v)  # Ke = Kt

    k_t = _through_origin(rho_r * w * w, thrust)
    if all(r.torque_nm is not None for r in spinning):
        torque = np.array([r.torque_nm or 0.0 for r in spinning])
        k_q = _through_origin(rho_r * w * w, torque)
        no_load = float(max(np.mean(current - torque / kt), 0.0))
    else:
        # I = I0 + (k_q / Kt) rho_r w^2
        slope, intercept = np.polyfit(rho_r * w * w, current, 1)
        k_q = float(slope) * kt
        no_load = float(max(intercept, 0.0))
    resistance = _through_origin(current, v_motor - kt * w)

    thrust_error = float(np.max(np.abs(k_t * rho_r * w * w / thrust - 1.0)))
    predicted_current = (v_motor - kt * w) / resistance
    current_error = float(np.max(np.abs(predicted_current / current - 1.0)))
    return PropFit(
        k_t=k_t,
        k_q=k_q,
        rho_ref_kg_m3=rho_ref_kg_m3,
        winding_resistance_ohm=resistance,
        no_load_current_a=no_load,
        kv_rpm_per_v=kv_rpm_per_v,
        max_thrust_error=thrust_error,
        max_current_error=current_error,
    )


def steady_state(
    fit: PropFit, throttle: float, voltage_v: float, rho: float = 1.225
) -> tuple[float, float, float]:
    """(rpm, thrust_n, current_a) of the fitted DC model at a throttle and bus voltage."""
    kt = 1.0 / units.rpm_to_rad_per_s(fit.kv_rpm_per_v)
    rho_r = rho / fit.rho_ref_kg_m3
    a = fit.k_q * rho_r
    b = kt * kt / fit.winding_resistance_ohm
    c = -kt * (throttle * voltage_v / fit.winding_resistance_ohm - fit.no_load_current_a)
    if c >= 0.0:
        return 0.0, 0.0, 0.0
    w = (-b + math.sqrt(b * b - 4.0 * a * c)) / (2.0 * a)
    current = (throttle * voltage_v - kt * w) / fit.winding_resistance_ohm
    return w / units.RPM_TO_RAD_PER_S, fit.k_t * rho_r * w * w, current


def as_yaml(fit: PropFit) -> str:
    return (
        "# fitted by simctl propfit\n"
        f"motor:\n  kv_rpm_per_v: {fit.kv_rpm_per_v:g}\n"
        f"  winding_resistance_ohm: {fit.winding_resistance_ohm:.4g}\n"
        f"  no_load_current_a: {fit.no_load_current_a:.3g}\n"
        f"prop:\n  k_t: {fit.k_t:.4g}\n  k_q: {fit.k_q:.4g}\n"
        f"  rho_ref_kg_m3: {fit.rho_ref_kg_m3:g}\n"
        f"# max fit error: thrust {100 * fit.max_thrust_error:.1f} %, "
        f"current {100 * fit.max_current_error:.1f} %\n"
    )
