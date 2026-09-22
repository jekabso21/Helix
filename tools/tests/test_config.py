import json
import math
from pathlib import Path

import pytest

from simtools.config import load_yaml, resolve_session, write_run_directory
from simtools.config.resolver import ConfigError, hover_throttle_us
from simtools.config.schemas import DroneConfig

REPO_ROOT = Path(__file__).resolve().parents[2]
SESSION = REPO_ROOT / "configs/sessions/ci_hover.yaml"


def test_ci_hover_session_resolves_to_si_json() -> None:
    resolved = resolve_session(SESSION, REPO_ROOT)
    session = resolved.session
    assert session["physics_rate_hz"] == 1000
    assert session["atmosphere"] == {"ground_temperature_k": 288.15, "ground_pressure_pa": 101325.0}
    assert session["origin"]["lat_rad"] == pytest.approx(math.radians(56.0))
    assert session["input"]["altitude_hold"]["hover_throttle_us"] == pytest.approx(1361.7, abs=0.5)
    assert "aux 1 1 0 900 2100 0 0" in resolved.cli_lines
    assert "aux 0 0 0 1700 2100 0 0" in resolved.cli_lines


def test_quad_x_layout_matches_betaflight_motor_order() -> None:
    drone = resolve_session(SESSION, REPO_ROOT).drone
    motors = drone["motors"]
    arm = 0.226 / 2 / math.sqrt(2)
    assert [m["bf_index"] for m in motors] == [1, 2, 3, 4]
    assert motors[0]["position_frd_m"] == pytest.approx([-arm, arm, 0.0])  # rear right
    assert motors[1]["position_frd_m"] == pytest.approx([arm, arm, 0.0])  # front right
    assert [m["spin"] for m in motors] == [1.0, -1.0, -1.0, 1.0]
    assert motors[0]["first_order"]["max_speed_radps"] == pytest.approx(2500.0, abs=0.1)
    assert drone["mass_kg"] == 0.5
    assert drone["contact"]["points_frd_m"][0][2] == pytest.approx(0.02)


def test_props_out_reverses_all_spins(tmp_path: Path) -> None:
    text = (
        (REPO_ROOT / "configs/drones/reference_5in.yaml")
        .read_text()
        .replace("props_out: false", "props_out: true")
    )
    (tmp_path / "drone.yaml").write_text(text)
    drone = DroneConfig.model_validate(load_yaml(tmp_path / "drone.yaml"))
    assert drone.layout.props_out
    from simtools.config.resolver import resolve_drone

    assert [m["spin"] for m in resolve_drone(drone)["motors"]] == [-1.0, 1.0, 1.0, -1.0]


def test_extends_deep_merges_and_replaces_lists(tmp_path: Path) -> None:
    (tmp_path / "base.yaml").write_text("a: {x: 1, y: 2}\nlist: [1, 2, 3]\n")
    (tmp_path / "child.yaml").write_text("extends: base.yaml\na: {y: 3}\nlist: [9]\n")
    assert load_yaml(tmp_path / "child.yaml") == {"a": {"x": 1, "y": 3}, "list": [9]}


def test_validation_error_names_file_and_field(tmp_path: Path) -> None:
    text = SESSION.read_text().replace("physics_rate_hz: 1000", "physics_rate_hz: 2500")
    bad = tmp_path / "bad.yaml"
    bad.write_text(text)
    with pytest.raises(ConfigError) as info:
        resolve_session(bad, REPO_ROOT)
    assert "bad.yaml" in str(info.value)
    assert "multiple of 1000" in str(info.value)


def test_unknown_field_is_rejected(tmp_path: Path) -> None:
    bad = tmp_path / "bad.yaml"
    bad.write_text(SESSION.read_text() + "typo_field: 1\n")
    with pytest.raises(ConfigError, match="typo_field"):
        resolve_session(bad, REPO_ROOT)


def test_run_directory_layout(tmp_path: Path) -> None:
    resolved = resolve_session(SESSION, REPO_ROOT)
    run_dir = write_run_directory(resolved, tmp_path, REPO_ROOT, ["simctl", "run"])
    assert run_dir.name.endswith("_ci_hover")
    session = json.loads((run_dir / "resolved/session.json").read_text())
    assert session["drone"] == "drone.json"
    assert (run_dir / "resolved/drone.json").exists()
    assert (run_dir / "betaflight/cli.txt").read_text().splitlines()[-1] == "aux 1 1 0 900 2100 0 0"
    meta = json.loads((run_dir / "meta.json").read_text())
    assert meta["session"] == "ci_hover" and meta["seed"] == 42
    for sub in ("logs", "data"):
        assert (run_dir / sub).is_dir()


def test_hover_throttle_is_between_idle_and_full() -> None:
    drone = DroneConfig.model_validate(load_yaml(REPO_ROOT / "configs/drones/reference_5in.yaml"))
    assert 1200.0 < hover_throttle_us(drone) < 1500.0
