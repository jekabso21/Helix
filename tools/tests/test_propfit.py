import math
from pathlib import Path

import pytest

from simtools.config import units
from simtools.propfit import ThrustTableError, as_yaml, fit_prop, read_thrust_table, steady_state

REPO_ROOT = Path(__file__).resolve().parents[2]
SYNTHETIC = REPO_ROOT / "data/reference/generic_2207_1900kv_5x4.3x3_6s_synthetic.csv"

KV, R, I0, K_T, K_Q = 1900.0, 0.18, 1.5, 1.5e-6, 2.0e-8


def model_row(throttle: float, voltage: float, rho: float = 1.225) -> str:
    """Exact steady state of the DC motor + prop model, the table a perfect bench would measure."""
    kt = 1.0 / units.rpm_to_rad_per_s(KV)
    rho_r = rho / 1.225
    a, b, c = K_Q * rho_r, kt * kt / R, -kt * (throttle * voltage / R - I0)
    w = (-b + math.sqrt(b * b - 4 * a * c)) / (2 * a)
    current = (throttle * voltage - kt * w) / R
    thrust_g = K_T * rho_r * w * w / 9.80665 * 1000
    return (
        f"{throttle},{voltage},{w / units.RPM_TO_RAD_PER_S:.1f},{current:.3f},{thrust_g:.1f},{rho}"
    )


def write_table(path: Path, densities: bool = False) -> None:
    lines = ["throttle,voltage_v,rpm,current_a,thrust_g,air_density_kg_m3"]
    for i in range(1, 11):
        lines.append(model_row(i / 10, 25.2, 1.1 if densities and i % 2 else 1.225))
    path.write_text("\n".join(lines) + "\n")


def test_fit_recovers_the_generating_parameters(tmp_path: Path) -> None:
    write_table(tmp_path / "table.csv", densities=True)
    fit = fit_prop(read_thrust_table(tmp_path / "table.csv"), KV)
    assert fit.k_t == pytest.approx(K_T, rel=2e-3)
    assert fit.k_q == pytest.approx(K_Q, rel=2e-2)
    assert fit.winding_resistance_ohm == pytest.approx(R, rel=2e-2)
    assert fit.no_load_current_a == pytest.approx(I0, rel=5e-2)
    assert fit.max_thrust_error < 0.01 and fit.max_current_error < 0.01
    assert "k_t: 1.5e-06" in as_yaml(fit)


def test_fitted_model_matches_the_table_within_five_percent() -> None:
    rows = read_thrust_table(SYNTHETIC)
    fit = fit_prop(rows, KV)
    for row in rows:
        rpm, thrust, current = steady_state(fit, row.throttle, row.voltage_v, row.air_density_kg_m3)
        assert rpm == pytest.approx(row.rpm, rel=0.05)
        assert thrust == pytest.approx(row.thrust_n, rel=0.05)
        assert current == pytest.approx(row.current_a, rel=0.05)


def test_torque_column_is_preferred_and_missing_columns_are_reported(tmp_path: Path) -> None:
    kt = 1.0 / units.rpm_to_rad_per_s(KV)
    lines = ["throttle,voltage_v,rpm,current_a,thrust_n,torque_nm"]
    for rpm in (8000.0, 16000.0, 24000.0):
        w = units.rpm_to_rad_per_s(rpm)
        lines.append(f"0.5,25.2,{rpm},{I0 + K_Q * w * w / kt},{K_T * w * w},{K_Q * w * w}")
    (tmp_path / "t.csv").write_text("\n".join(lines) + "\n")
    fit = fit_prop(read_thrust_table(tmp_path / "t.csv"), KV)
    assert fit.k_q == pytest.approx(K_Q, rel=1e-9)
    assert fit.no_load_current_a == pytest.approx(I0, rel=1e-6)
    (tmp_path / "bad.csv").write_text("throttle,rpm\n0.5,1000\n")
    with pytest.raises(ThrustTableError, match="missing columns"):
        read_thrust_table(tmp_path / "bad.csv")
