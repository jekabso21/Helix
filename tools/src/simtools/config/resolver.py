import json
import platform
import subprocess
import sys
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import yaml
from pydantic import ValidationError

from simtools.config import units
from simtools.config.schemas import (
    DroneConfig,
    EnvironmentConfig,
    InputMappingConfig,
    SessionConfig,
)
from simtools.modelc.compile import compile_drone, to_json
from simtools.modelc.gltf import export_glb

RESOLVED_SCHEMA_VERSION = 1


class ConfigError(Exception):
    pass


def _deep_merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    merged = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = _deep_merge(merged[key], value)  # type: ignore[arg-type]
        else:
            merged[key] = value
    return merged


def load_yaml(path: Path, depth: int = 0) -> dict[str, Any]:
    """Load a YAML mapping, applying `extends` (relative to the file) recursively."""
    if depth > 8:
        raise ConfigError(f"{path}: extends chain too deep")
    try:
        data = yaml.safe_load(path.read_text())
    except (OSError, yaml.YAMLError) as error:
        raise ConfigError(f"{path}: {error}") from error
    if not isinstance(data, dict):
        raise ConfigError(f"{path}: expected a mapping at the top level")
    document: dict[str, Any] = dict(data)  # type: ignore[arg-type]
    base_ref = document.pop("extends", None)
    if base_ref is None:
        return document
    if not isinstance(base_ref, str):
        raise ConfigError(f"{path}: extends must be a path")
    return _deep_merge(load_yaml(path.parent / base_ref, depth + 1), document)


def _validation_message(path: Path, error: ValidationError) -> str:
    lines = [f"{path}:"]
    for item in error.errors():
        field = ".".join(str(part) for part in item["loc"]) or "(root)"
        lines.append(f"  {field}: {item['msg']}")
    return "\n".join(lines)


def _load_model[T: SessionConfig | DroneConfig | EnvironmentConfig | InputMappingConfig](
    model: type[T], path: Path
) -> T:
    try:
        return model.model_validate(load_yaml(path))
    except ValidationError as error:
        raise ConfigError(_validation_message(path, error)) from error


@dataclass(frozen=True)
class ResolvedSession:
    name: str
    session: dict[str, Any]
    drone: dict[str, Any]
    drone_config: DroneConfig
    cli_lines: list[str]
    betaflight_binary: Path
    ports: dict[str, int]
    logging_root: str

    def session_logging_root(self) -> str:
        return self.logging_root


PART_FILES = ("motor", "prop", "battery")


def load_drone(path: Path, base_dir: Path) -> DroneConfig:
    """Loads a drone YAML; motor, prop and battery may name YAML files relative to base_dir."""
    data = load_yaml(path)
    for key in PART_FILES:
        if isinstance(data.get(key), str):
            data[key] = load_yaml(base_dir / str(data[key]))
    try:
        return DroneConfig.model_validate(data)
    except ValidationError as error:
        raise ConfigError(_validation_message(path, error)) from error


def resolve_drone(drone: DroneConfig) -> dict[str, Any]:
    return to_json(compile_drone(drone))


def hover_throttle_us(drone: DroneConfig) -> float:
    """Stick position whose motor command balances weight if Betaflight passed it through."""
    return 1000.0 + 1000.0 * compile_drone(drone).hover_command()


def resolve_session(session_path: Path, base_dir: Path) -> ResolvedSession:
    """Validate a session and all it references; YAML paths are relative to base_dir."""
    session = _load_model(SessionConfig, session_path)
    drone = load_drone(base_dir / session.drone, base_dir)
    environment = _load_model(EnvironmentConfig, base_dir / session.environment)
    cli_script = base_dir / session.betaflight.cli_script
    if not cli_script.exists():
        raise ConfigError(f"{session_path}: betaflight.cli_script not found: {cli_script}")
    cli_lines = [
        line.strip()
        for line in cli_script.read_text().splitlines()
        if line.strip() and not line.strip().startswith("#")
    ] + list(session.betaflight.cli_extra)
    bf_ports = session.betaflight.ports
    esc_uart_port = bf_ports.uart_base + bf_ports.esc_uart - 1
    if session.betaflight.virtual_esc:
        cli_lines += [
            "feature ESC_SENSOR",
            f"serial {bf_ports.esc_uart - 1} 1024 115200 57600 0 115200",
            "set battery_meter = ESC",
            "set current_meter = ESC",
            f"set motor_poles = {drone.motor.poles}",
        ]

    resolved_input: dict[str, Any] = {
        "source": session.input.source,
        "rc_rate_hz": session.input.rc_rate_hz,
    }
    if session.input.source == "gamepad":
        mapping = _load_model(InputMappingConfig, base_dir / str(session.input.mapping))
        resolved_input["mapping"] = {
            "device_name_contains": mapping.device.name_contains,
            "arm_channel": mapping.arm_channel,
            "channels": {
                name: source.model_dump(exclude_none=True)
                for name, source in mapping.channels.items()
            },
        }
    else:
        hold = session.input.altitude_hold.model_dump()
        hold["hover_throttle_us"] = hover_throttle_us(drone)
        resolved_input["altitude_hold"] = hold
    ports = {
        "pwm": session.betaflight.ports.pwm,
        "fdm": session.betaflight.ports.fdm,
        "rc": session.betaflight.ports.rc,
    }
    resolved: dict[str, Any] = {
        "schema_version": RESOLVED_SCHEMA_VERSION,
        "seed": session.seed,
        "physics_rate_hz": session.physics_rate_hz,
        "duration_s": session.duration_s,
        "betaflight": {
            "host": session.betaflight.host,
            "ports": ports,
            "esc": {
                "enabled": session.betaflight.virtual_esc,
                "request_port": bf_ports.esc_request,
                "uart_port": esc_uart_port,
            },
        },
        "origin": {
            "lat_rad": units.deg_to_rad(session.origin.lat_deg),
            "lon_rad": units.deg_to_rad(session.origin.lon_deg),
            "altitude_m": session.origin.altitude_m,
        },
        "atmosphere": {
            "ground_temperature_k": units.c_to_k(environment.atmosphere.ground_temperature_c),
            "ground_pressure_pa": units.hpa_to_pa(environment.atmosphere.ground_pressure_hpa),
        },
        "spawn": {
            "north_m": session.spawn.north_m,
            "east_m": session.spawn.east_m,
            "height_agl_m": session.spawn.height_agl_m,
            "heading_rad": units.deg_to_rad(session.spawn.heading_deg),
        },
        "input": resolved_input,
        "control_api": {"host": session.control_api.host, "port": session.control_api.port},
        "app": {
            "host": session.app.host,
            "port": session.app.port,
            "state_rate_hz": session.app.state_rate_hz,
        },
        "logging": {"rate_hz": session.logging.rate_hz, "truth_csv": "../data/truth.csv"},
        "drone": "drone.json",
    }
    return ResolvedSession(
        name=session.name,
        session=resolved,
        drone=resolve_drone(drone),
        drone_config=drone,
        cli_lines=cli_lines,
        betaflight_binary=base_dir / session.betaflight.binary,
        ports={"uart_base": session.betaflight.ports.uart_base, **ports},
        logging_root=session.logging.root,
    )


def _git_commit(directory: Path) -> str | None:
    try:
        return subprocess.run(
            ["git", "-C", str(directory), "rev-parse", "HEAD"],
            capture_output=True,
            text=True,
            check=True,
        ).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def write_run_directory(
    resolved: ResolvedSession, run_root: Path, base_dir: Path, argv: list[str]
) -> Path:
    """Create runs/<timestamp>_<name>/ with resolved/, logs/, data/, betaflight/ and meta.json."""
    stamp = datetime.now(UTC).strftime("%Y-%m-%dT%H-%M-%S")
    run_dir = run_root / f"{stamp}_{resolved.name}"
    for sub in ("resolved", "logs", "data", "betaflight"):
        (run_dir / sub).mkdir(parents=True, exist_ok=False)
    (run_dir / "resolved/session.json").write_text(json.dumps(resolved.session, indent=2))
    (run_dir / "resolved/drone.json").write_text(json.dumps(resolved.drone, indent=2))
    (run_dir / "resolved/drone.glb").write_bytes(export_glb(compile_drone(resolved.drone_config)))
    (run_dir / "betaflight/cli.txt").write_text("\n".join(resolved.cli_lines) + "\n")
    meta = {
        "created_utc": stamp,
        "session": resolved.name,
        "seed": resolved.session["seed"],
        "git_commit": _git_commit(base_dir),
        "betaflight_commit": _git_commit(base_dir / "third_party/betaflight"),
        "host": platform.node(),
        "python": sys.version.split()[0],
        "argv": argv,
    }
    (run_dir / "meta.json").write_text(json.dumps(meta, indent=2))
    return run_dir
