import json
import math
from pathlib import Path

import pytest
import yaml
from pydantic import ValidationError

from simtools.config import load_yaml, resolve_session, write_run_directory
from simtools.config.resolver import ConfigError, hover_throttle_us, load_drone
from simtools.config.schemas import CameraConfig

REPO_ROOT = Path(__file__).resolve().parents[2]
SESSION = REPO_ROOT / "configs/sessions/ci_hover.yaml"


def test_ci_hover_session_resolves_to_si_json() -> None:
    resolved = resolve_session(SESSION, REPO_ROOT)
    session = resolved.session
    assert session["physics_rate_hz"] == 1000
    assert session["atmosphere"] == {"ground_temperature_k": 288.15, "ground_pressure_pa": 101325.0}
    assert session["origin"]["lat_rad"] == pytest.approx(math.radians(56.0))
    assert session["input"]["altitude_hold"]["hover_throttle_us"] == pytest.approx(1213.6, abs=1.0)
    assert "aux 1 1 0 900 2100 0 0" in resolved.cli_lines
    assert "aux 0 0 0 1700 2100 0 0" in resolved.cli_lines
    assert "feature ESC_SENSOR" in resolved.cli_lines
    assert "serial 3 1024 115200 57600 0 115200" in resolved.cli_lines
    assert "set motor_poles = 14" in resolved.cli_lines
    assert "set rpm_filter_harmonics = 3" in resolved.cli_lines
    assert session["betaflight"]["ports"]["rpm"] == 9006
    assert session["betaflight"]["esc"] == {
        "enabled": True,
        "request_port": 9005,
        "uart_port": 5764,
    }


def test_quad_x_layout_matches_betaflight_motor_order() -> None:
    drone = resolve_session(SESSION, REPO_ROOT).drone
    motors = drone["motors"]
    arm = 0.226 / 2 / math.sqrt(2)
    cg = drone["cg_from_origin_frd_m"]
    assert [m["bf_index"] for m in motors] == [1, 2, 3, 4]
    assert motors[0]["position_frd_m"] == pytest.approx(
        [-arm - cg[0], arm - cg[1], -cg[2]]
    )  # rear right
    assert motors[1]["position_frd_m"] == pytest.approx(
        [arm - cg[0], arm - cg[1], -cg[2]]
    )  # front right
    assert [m["spin"] for m in motors] == [1.0, -1.0, -1.0, 1.0]
    assert motors[0]["motor"]["model"] == "dc"
    assert motors[0]["motor"]["kv_radps_per_v"] == pytest.approx(1900 * math.tau / 60)
    assert motors[0]["prop"]["rotor_drag_coefficient"] == 6e-5
    assert drone["battery"]["capacity_ah"] == pytest.approx(1.1)
    assert drone["schema_version"] == 2
    assert drone["mass_kg"] == pytest.approx(0.497)
    assert drone["contact"]["points_frd_m"][0][2] == pytest.approx(0.02 - cg[2])


def test_props_out_reverses_all_spins(tmp_path: Path) -> None:
    text = (
        (REPO_ROOT / "configs/drones/reference_5in.yaml")
        .read_text()
        .replace("props_out: false", "props_out: true")
    )
    (tmp_path / "drone.yaml").write_text(text)
    drone = load_drone(tmp_path / "drone.yaml", REPO_ROOT)
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
    assert "aux 1 1 0 900 2100 0 0" in (run_dir / "betaflight/cli.txt").read_text().splitlines()
    meta = json.loads((run_dir / "meta.json").read_text())
    assert meta["session"] == "ci_hover" and meta["seed"] == 42
    for sub in ("logs", "data"):
        assert (run_dir / sub).is_dir()


def test_hover_throttle_is_between_idle_and_full() -> None:
    drone = load_drone(REPO_ROOT / "configs/drones/reference_5in.yaml", REPO_ROOT)
    assert 1150.0 < hover_throttle_us(drone) < 1500.0


def test_gamepad_session_resolves_the_mapping_by_channel_name() -> None:
    resolved = resolve_session(REPO_ROOT / "configs/sessions/dev_gamepad.yaml", REPO_ROOT)
    mapping = resolved.session["input"]["mapping"]
    assert mapping["device_name_contains"] == "Radiomaster Boxer"
    assert mapping["arm_channel"] == "aux1"
    # the profile is the user's and the app rewrites it, so compare against the file itself
    profile = yaml.safe_load((REPO_ROOT / "configs/input/radiomaster_boxer.yaml").read_text())
    for name, source in profile["channels"].items():
        expected = {"inverted": False, "deadband": 0.0, **source}
        assert mapping["channels"][name] == expected
    assert set(mapping["channels"]) == set(profile["channels"])
    assert "altitude_hold" not in resolved.session["input"]
    assert resolved.session["duration_s"] == 0


def test_mapping_rejects_unknown_channel_and_double_source(tmp_path: Path) -> None:
    session = tmp_path / "s.yaml"
    mapping = tmp_path / "m.yaml"
    session.write_text(
        (REPO_ROOT / "configs/sessions/dev_gamepad.yaml")
        .read_text()
        .replace("configs/input/default.yaml", str(mapping))
    )
    mapping.write_text(
        "schema_version: 1\nname: bad\ndevice: {name_contains: x}\n"
        "channels: {gear: {axis: 1}}\narm_channel: aux1\n"
    )
    with pytest.raises(ConfigError, match="unknown channel"):
        resolve_session(session, REPO_ROOT)
    mapping.write_text(
        "schema_version: 1\nname: bad\ndevice: {name_contains: x}\n"
        "channels: {roll: {axis: 1, button: 2}}\narm_channel: aux1\n"
    )
    with pytest.raises(ConfigError, match="exactly one"):
        resolve_session(session, REPO_ROOT)


def test_cameras_resolve_into_the_document_simvideo_reads() -> None:
    resolved = resolve_session(SESSION, REPO_ROOT)
    cameras = resolved.cameras
    assert cameras["schema_version"] == 1
    assert cameras["status_port"] == 7730
    assert len(cameras["cameras"]) == 1
    camera = cameras["cameras"][0]
    assert camera["name"] == "main_fpv"  # the drone mounts a camera by that name
    assert (camera["width"], camera["height"]) == (1280, 720)
    assert camera["pixel_format"] == "rgb8"
    assert camera["sensor_latency_s"] == pytest.approx(0.012)
    assert camera["outputs"][0]["enabled"] is True
    assert "v4l2sink" in camera["outputs"][0]["pipeline"]
    assert camera["optics"]["hfov_rad"] == pytest.approx(math.radians(120.0))
    assert camera["optics"]["rolling_shutter_readout_s"] == pytest.approx(0.008)


def test_a_camera_without_a_mount_on_the_drone_is_rejected(tmp_path: Path) -> None:
    camera = (REPO_ROOT / "configs/cameras/generic_fpv.yaml").read_text()
    (tmp_path / "ghost.yaml").write_text(camera.replace("name: main_fpv", "name: not_mounted"))
    session = load_yaml(SESSION)
    session["cameras"] = [str(tmp_path / "ghost.yaml")]
    (tmp_path / "session.yaml").write_text(yaml.safe_dump(session))
    with pytest.raises(ConfigError, match="no mount on the drone"):
        resolve_session(tmp_path / "session.yaml", REPO_ROOT)


def test_odd_camera_resolutions_are_rejected(tmp_path: Path) -> None:
    camera = (REPO_ROOT / "configs/cameras/generic_fpv.yaml").read_text()
    (tmp_path / "odd.yaml").write_text(camera.replace("[1280, 720]", "[1281, 720]"))
    with pytest.raises(ValidationError, match="even"):
        CameraConfig.model_validate(load_yaml(tmp_path / "odd.yaml"))


def test_the_run_directory_carries_cameras_json(tmp_path: Path) -> None:
    resolved = resolve_session(SESSION, REPO_ROOT)
    run_dir = write_run_directory(resolved, tmp_path, REPO_ROOT, ["pytest"])
    written = json.loads((run_dir / "resolved/cameras.json").read_text())
    assert written["cameras"][0]["name"] == "main_fpv"


def test_video_is_disabled_when_a_session_has_no_camera(tmp_path: Path) -> None:
    session = load_yaml(SESSION)
    session.pop("cameras", None)
    (tmp_path / "no_camera.yaml").write_text(yaml.safe_dump(session))
    resolved = resolve_session(tmp_path / "no_camera.yaml", REPO_ROOT)
    assert resolved.cameras["cameras"] == []
    assert resolved.video_enabled is False  # nothing to publish, so simvideo is not started


def test_video_can_be_switched_off_in_the_session(tmp_path: Path) -> None:
    session = load_yaml(SESSION)
    session["video"] = {"enabled": False}
    (tmp_path / "off.yaml").write_text(yaml.safe_dump(session))
    assert resolve_session(tmp_path / "off.yaml", REPO_ROOT).video_enabled is False


def test_calm_environment_resolves_to_still_air() -> None:
    wind = resolve_session(SESSION, REPO_ROOT).session["wind"]
    assert wind == {
        "mean_speed_mps": 0.0,
        "mean_from_rad": 0.0,
        "profile": {"log_law": False, "roughness_m": 0.1, "reference_height_m": 10.0},
        "turbulence_w20_mps": 0.0,
        "turbulence_intensity": "none",
        "gusts": [],
    }


def test_windy_environment_resolves_to_si(tmp_path: Path) -> None:
    environment = tmp_path / "windy.yaml"
    environment.write_text(
        "schema_version: 1\n"
        "wind:\n"
        "  mean: { speed_mps: 6, from_deg: 270 }\n"
        "  profile: { type: log, z0_m: 0.05, ref_height_m: 10 }\n"
        "  turbulence: { model: dryden, intensity: moderate }\n"
        "  gusts: [{ at_s: 12, duration_s: 2, amplitude_mps: 5, from_deg: 90 }]\n"
    )
    session = tmp_path / "s.yaml"
    session.write_text(
        SESSION.read_text().replace("configs/environments/calm_15c.yaml", str(environment))
    )
    wind = resolve_session(session, REPO_ROOT).session["wind"]
    assert wind["mean_speed_mps"] == 6.0
    assert wind["mean_from_rad"] == pytest.approx(math.radians(270.0))
    assert wind["profile"] == {"log_law": True, "roughness_m": 0.05, "reference_height_m": 10.0}
    # MIL-F-8785C: moderate turbulence is a 30 kt wind at 20 ft
    assert wind["turbulence_w20_mps"] == pytest.approx(30.0 * 1852.0 / 3600.0)
    assert wind["gusts"] == [
        {
            "start_s": 12.0,
            "duration_s": 2.0,
            "amplitude_mps": 5.0,
            "from_rad": pytest.approx(math.radians(90.0)),
        }
    ]


def test_wind_settings_are_checked(tmp_path: Path) -> None:
    from simtools.config.schemas import EnvironmentConfig

    with pytest.raises(ValidationError):
        EnvironmentConfig.model_validate({"schema_version": 1, "wind": {"mean": {"speed_mps": -1}}})
    with pytest.raises(ValidationError):
        EnvironmentConfig.model_validate(
            {"schema_version": 1, "wind": {"turbulence": {"intensity": "hurricane"}}}
        )
    with pytest.raises(ValidationError, match="ref_height_m"):
        EnvironmentConfig.model_validate(
            {
                "schema_version": 1,
                "wind": {"profile": {"type": "log", "z0_m": 2, "ref_height_m": 1}},
            }
        )
    many = [{"at_s": i, "duration_s": 1, "amplitude_mps": 1, "from_deg": 0} for i in range(9)]
    with pytest.raises(ValidationError, match="8"):
        EnvironmentConfig.model_validate({"schema_version": 1, "wind": {"gusts": many}})


def test_a_live_wind_change_is_converted_to_si() -> None:
    from simtools.config.resolver import env_update_message
    from simtools.config.schemas import EnvUpdateConfig

    message = env_update_message(
        EnvUpdateConfig.model_validate(
            {
                "wind": {
                    "mean": {"speed_mps": 8, "from_deg": 180},
                    "turbulence": {"intensity": "light"},
                },
                "gust": {"duration_s": 1.5, "amplitude_mps": 4, "from_deg": 90},
            }
        )
    )
    assert message["wind"]["mean_speed_mps"] == 8.0
    assert message["wind"]["mean_from_rad"] == pytest.approx(math.pi)
    assert message["wind"]["turbulence_w20_mps"] == pytest.approx(15.0 * 1852.0 / 3600.0)
    assert message["gust"] == {
        "duration_s": 1.5,
        "amplitude_mps": 4.0,
        "from_rad": pytest.approx(math.pi / 2.0),
    }
    only_gust = env_update_message(
        EnvUpdateConfig.model_validate({"gust": {"duration_s": 2, "amplitude_mps": 3}})
    )
    assert "wind" not in only_gust
    with pytest.raises(ValidationError, match="nothing"):
        EnvUpdateConfig.model_validate({})


def test_failure_requests_are_converted_to_si() -> None:
    from simtools.config.resolver import failure_message
    from simtools.config.schemas import FailureRequestConfig

    def convert(document: dict) -> dict:
        return failure_message(FailureRequestConfig.model_validate(document))

    assert convert({"type": "motor_out", "motor": 2}) == {
        "type": "motor_out",
        "target": {"motor": 2},
        "params": {},
    }
    degraded = convert({"type": "motor_degraded", "motor": 1, "output_pct": 60, "duration_s": 3})
    assert degraded["params"] == {"output_gain": pytest.approx(0.6)}
    assert degraded["duration_s"] == 3.0
    assert convert({"type": "esc_desync", "motor": 3, "period_ms": 400, "dropout_ms": 50})[
        "params"
    ] == {"period_s": pytest.approx(0.4), "dropout_s": pytest.approx(0.05)}
    assert convert(
        {"type": "prop_damage", "motor": 4, "thrust_loss_pct": 25, "vibration_scale": 10}
    )["params"] == {"thrust_loss": pytest.approx(0.25), "vibration_scale": 10.0}
    assert convert(
        {"type": "battery_high_resistance", "cell_resistance_scale": 3, "connector_add_mohm": 20}
    )["params"] == {"cell_resistance_scale": 3.0, "connector_add_ohm": pytest.approx(0.02)}
    gyro = convert({"type": "imu_bias", "sensor": "gyro", "axis": "x", "step_dps": 30})
    assert gyro["target"] == {"sensor": "gyro", "axis": "x"}
    assert gyro["params"] == {"step": pytest.approx(math.radians(30.0))}
    accel = convert({"type": "imu_bias", "sensor": "accel", "axis": "all", "step_mps2": 2})
    assert accel["params"] == {"step": 2.0}
    assert convert({"type": "imu_saturation", "sensor": "gyro", "range_pct": 25})["params"] == {
        "range_scale": pytest.approx(0.25)
    }
    assert convert({"type": "baro_offset", "offset_hpa": -3})["params"] == {
        "offset_pa": pytest.approx(-300.0)
    }


def test_failure_requests_are_checked() -> None:
    from simtools.config.schemas import FailureRequestConfig

    for bad in (
        {"type": "gremlins"},
        {"type": "motor_out"},
        {"type": "motor_out", "motor": 0},
        {"type": "motor_degraded", "motor": 1, "output_pct": 150},
        {"type": "esc_desync", "motor": 1, "period_ms": 100, "dropout_ms": 200},
        {"type": "imu_bias", "sensor": "gyro", "step_mps2": 1},
        {"type": "imu_bias", "sensor": "accel", "step_dps": 1},
        {"type": "imu_noise", "sensor": "compass", "noise_scale": 2},
        {"type": "baro_stuck", "duration_s": 0},
    ):
        with pytest.raises(ValidationError):
            FailureRequestConfig.model_validate(bad)
