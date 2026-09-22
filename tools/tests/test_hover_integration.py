import csv
import json
import math
import os
import subprocess
import time
from pathlib import Path

import pytest

from simtools.sitl import configure_and_start

REPO_ROOT = Path(__file__).resolve().parents[2]
SITL_BINARY = REPO_ROOT / "build/betaflight/betaflight_SITL.elf"
TARGET_HEIGHT_M = 5.0
DURATION_S = 36.0
WINDOW_START_S = 16.0

pytestmark = pytest.mark.integration


def find_simcore() -> Path | None:
    if "FPVSIM_SIMCORE" in os.environ:
        return Path(os.environ["FPVSIM_SIMCORE"])
    for preset in ("release", "ci", "clang", "dev"):
        candidate = REPO_ROOT / "build" / preset / "simcore" / "simcore"
        if candidate.exists():
            return candidate
    return None


def write_configs(run_dir: Path) -> Path:
    motor = {
        "rotor_inertia_kg_m2": 6e-6,
        "first_order": {
            "max_speed_radps": 2500,
            "time_constant_s": 0.03,
            "k_t": 1.5e-6,
            "k_q": 2e-8,
        },
    }
    arm = 0.08
    mounts = [(-arm, arm, 1), (arm, arm, -1), (-arm, -arm, -1), (arm, -arm, 1)]
    drone = {
        "schema_version": 1,
        "mass_kg": 0.5,
        "inertia_frd_kg_m2": [[0.003, 0, 0], [0, 0.003, 0], [0, 0, 0.005]],
        "motors": [
            {
                "bf_index": i + 1,
                "position_frd_m": [x, y, 0.0],
                "axis_frd": [0, 0, -1],
                "spin": s,
                **motor,
            }
            for i, (x, y, s) in enumerate(mounts)
        ],
        "imu": {"position_frd_m": [0, 0, 0]},
        "aero": {"cda_frd_m2": [0.01, 0.01, 0.02], "cop_frd_m": [0, 0, 0], "k_omega": [5e-4] * 3},
        "contact": {
            "points_frd_m": [
                [arm, arm, 0.02],
                [arm, -arm, 0.02],
                [-arm, arm, 0.02],
                [-arm, -arm, 0.02],
            ],
            "stiffness_n_per_m": 3000,
            "damping_n_s_per_m": 30,
            "friction": 0.6,
            "friction_regularization_mps": 0.01,
            "crash_speed_mps": 6.0,
        },
    }
    hover_command = math.sqrt(0.5 * 9.80665 / 4 / 1.5e-6) / 2500
    session = {
        "schema_version": 1,
        "seed": 42,
        "physics_rate_hz": 1000,
        "duration_s": DURATION_S,
        "betaflight": {"host": "127.0.0.1", "ports": {"pwm": 9002, "fdm": 9003, "rc": 9004}},
        "origin": {"lat_rad": math.radians(56.0), "lon_rad": math.radians(24.0), "altitude_m": 0.0},
        "atmosphere": {"ground_temperature_k": 288.15, "ground_pressure_pa": 101325.0},
        "spawn": {"north_m": 0.0, "east_m": 0.0, "height_agl_m": 0.0, "heading_rad": 0.0},
        "input": {
            "source": "altitude_hold",
            "rc_rate_hz": 250,
            "altitude_hold": {
                "target_height_m": TARGET_HEIGHT_M,
                "climb_rate_mps": 1.0,
                "kp_us_per_m": 100.0,
                "ki_us_per_m_s": 20.0,
                "kd_us_per_mps": 80.0,
                "hover_throttle_us": 1000.0 + 1000.0 * hover_command,
                "integral_limit_us": 300.0,
                "arm_delay_s": 6.0,
            },
        },
        "logging": {"rate_hz": 200, "truth_csv": "truth.csv"},
        "drone": "drone.json",
    }
    (run_dir / "drone.json").write_text(json.dumps(drone))
    session_path = run_dir / "session.json"
    session_path.write_text(json.dumps(session))
    return session_path


def tilt_deg(qw: float, qx: float, qy: float, qz: float) -> float:
    # angle between body down and NED down: cos = R[2][2]
    cos_tilt = 1.0 - 2.0 * (qx * qx + qy * qy)
    return math.degrees(math.acos(max(-1.0, min(1.0, cos_tilt))))


@pytest.mark.skipif(not SITL_BINARY.exists(), reason="Betaflight SITL not built")
@pytest.mark.skipif(find_simcore() is None, reason="simcore not built")
def test_altitude_hold_keeps_height_within_half_a_metre(tmp_path: Path) -> None:
    cli_script = tmp_path / "cli.txt"
    cli_script.write_text(
        (REPO_ROOT / "configs/betaflight/baseline_cli.txt").read_text()
        + "\naux 1 1 0 900 2100 0 0\n"
    )
    session_path = write_configs(tmp_path)
    sitl = configure_and_start(SITL_BINARY, tmp_path, cli_script)
    try:
        started = time.monotonic()
        result = subprocess.run(
            [str(find_simcore()), "--session", str(session_path)],
            capture_output=True,
            text=True,
            timeout=DURATION_S + 30.0,
        )
        elapsed = time.monotonic() - started
    finally:
        sitl.terminate()
        sitl.wait(timeout=5.0)
    assert result.returncode == 0, result.stderr
    assert abs(elapsed - DURATION_S) < 2.0, f"realtime pacing off: {elapsed:.1f} s"

    with (tmp_path / "truth.csv").open() as f:
        rows = [r for r in csv.DictReader(f) if float(r["t_s"]) >= WINDOW_START_S]
    assert len(rows) > 0
    heights = [-float(r["down_m"]) for r in rows]
    tilts = [tilt_deg(*(float(r[k]) for k in ("qw", "qx", "qy", "qz"))) for r in rows]
    assert all(r["crashed"] == "0" for r in rows)
    assert max(abs(h - TARGET_HEIGHT_M) for h in heights) <= 0.5, (min(heights), max(heights))
    assert max(tilts) <= 5.0, max(tilts)
