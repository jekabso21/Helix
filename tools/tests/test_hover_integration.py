import csv
import math
import socket
import struct
import threading
import time
from datetime import datetime
from pathlib import Path

import pytest
import yaml

from simtools.api import ControlClient
from simtools.config import load_yaml, resolve_session, write_run_directory
from simtools.msp import MspCommand, parse_analog, parse_motor_telemetry
from simtools.proto import MESSAGE_SIZE, decode_render_state
from simtools.simctl.launcher import find_simcore, run_headless
from simtools.sitl import connect_uart, sitl_port_busy

REPO_ROOT = Path(__file__).resolve().parents[2]
SITL_BINARY = REPO_ROOT / "build/betaflight/betaflight_SITL.elf"
TARGET_HEIGHT_M = 5.0
DURATION_S = 36.0
WINDOW_START_S = 18.0

pytestmark = pytest.mark.integration


class ApiObserver(threading.Thread):
    """Exercises the control API and the RenderState stream while the flight runs."""

    def __init__(self) -> None:
        super().__init__(daemon=True)
        self.error: str | None = None
        self.info: dict = {}
        self.state: dict = {}
        self.paused_state: dict = {}
        self.resumed_state: dict = {}
        self.time_advanced_during_pause_ns = -1
        self.telemetry: list[dict] = []
        self.render_states = 0
        self.last_render = None
        self.analog = None
        self.motor_telemetry: list = []

    def run(self) -> None:
        try:
            self._run()
        except Exception as error:
            self.error = f"{type(error).__name__}: {error}"

    def _run(self) -> None:
        udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp.bind(("127.0.0.1", 7710))
        udp.settimeout(0.5)
        client = None
        deadline = time.monotonic() + 20.0
        while client is None and time.monotonic() < deadline:
            try:
                client = ControlClient(timeout_s=3.0)
            except OSError:
                time.sleep(0.2)
        assert client is not None, "control API never came up"
        with client:
            assert client.request("ping") == {"pong": True}
            self.info = client.request("get_info")
            time.sleep(12.0)  # armed and climbing by now
            self.state = client.request("get_state")
            # Pause for one second: sim time must stand still while the FC keeps being fed
            client.request("pause")
            t0 = client.request("get_state")["sim_time_ns"]
            time.sleep(1.0)
            self.paused_state = client.request("get_state")
            self.time_advanced_during_pause_ns = self.paused_state["sim_time_ns"] - t0
            client.request("resume")
            # the snapshot of the step that applied the command can trail the response by a step
            for _ in range(20):
                self.resumed_state = client.request("get_state")
                if not self.resumed_state["paused"]:
                    break
                time.sleep(0.01)
            sub = client.request("subscribe", {"topic": "telemetry", "rate_hz": 5})
            self.telemetry = list(client.events(6.0))
            client.request("unsubscribe", {"subscription_id": sub["subscription_id"]})
            # what Betaflight believes about the battery and motors comes from the virtual ESC
            msp = connect_uart(5763, timeout_s=5.0, check_msp=True)
            try:
                self.analog = parse_analog(msp.request(MspCommand.ANALOG))
                self.motor_telemetry = parse_motor_telemetry(
                    msp.request(MspCommand.MOTOR_TELEMETRY)
                )
            finally:
                msp.close()
            end = time.monotonic() + 3.0
            while time.monotonic() < end:
                try:
                    data = udp.recv(4096)
                except TimeoutError:
                    continue
                if len(data) == MESSAGE_SIZE:
                    self.render_states += 1
                    self.last_render = decode_render_state(data)
        udp.close()


def simcore_wall_seconds(log: Path) -> float:
    """Wall time between simcore's start and finish lines, so SITL setup is not counted."""
    stamps = [
        datetime.strptime(line[1:24], "%Y-%m-%d %H:%M:%S.%f")
        for line in log.read_text().splitlines()
        if line.startswith("[") and ("starting:" in line or "run finished:" in line)
    ]
    assert len(stamps) == 2, log.read_text()
    return (stamps[1] - stamps[0]).total_seconds()


def tilt_deg(qw: float, qx: float, qy: float, qz: float) -> float:
    # angle between body down and NED down: cos = R[2][2]
    cos_tilt = 1.0 - 2.0 * (qx * qx + qy * qy)
    return math.degrees(math.acos(max(-1.0, min(1.0, cos_tilt))))


@pytest.mark.skipif(not SITL_BINARY.exists(), reason="Betaflight SITL not built")
@pytest.mark.skipif(
    not any(
        (REPO_ROOT / "build" / p / "simcore/simcore").exists()
        for p in ("release", "ci", "clang", "dev")
    ),
    reason="simcore not built",
)
def test_altitude_hold_keeps_height_within_half_a_metre(tmp_path: Path) -> None:
    if sitl_port_busy():
        pytest.skip("a Betaflight SITL is already running on TCP 5761")
    resolved = resolve_session(REPO_ROOT / "configs/sessions/ci_hover.yaml", REPO_ROOT)
    assert resolved.session["duration_s"] == DURATION_S
    run_dir = write_run_directory(resolved, tmp_path, REPO_ROOT, ["pytest"])
    observer = ApiObserver()
    observer.start()
    started = time.monotonic()
    result = run_headless(resolved, run_dir, find_simcore(REPO_ROOT))
    elapsed = time.monotonic() - started
    observer.join(timeout=10.0)
    assert result.exit_code == 0, (run_dir / "logs/simcore.log").read_text()
    assert observer.error is None, observer.error
    assert observer.info["api_version"] == 1
    assert observer.state["armed"] is True
    assert observer.state["loop"]["step_time_p99_us"] < 1000.0
    assert 20 <= len(observer.telemetry) <= 40, len(observer.telemetry)
    heights_seen = [e["data"]["flight"]["altitude_agl_m"] for e in observer.telemetry]
    assert max(heights_seen) > 4.0
    assert observer.render_states > 100, observer.render_states
    assert observer.last_render is not None and observer.last_render.motor_count == 4
    assert observer.paused_state["paused"] is True
    assert observer.resumed_state["paused"] is False
    assert observer.time_advanced_during_pause_ns == 0
    last = observer.telemetry[-1]["data"]
    assert observer.analog is not None
    assert abs(observer.analog.voltage_v - last["battery"]["voltage_v"]) < 0.5
    assert 2.0 < observer.analog.current_a < 60.0, observer.analog
    assert len(observer.motor_telemetry) >= 4
    for motor, seen in zip(last["motors"], observer.motor_telemetry[:4], strict=False):
        assert abs(seen.rpm - motor["rpm"]) < 0.15 * motor["rpm"] + 200, (seen, motor)
    assert 20.0 < last["battery"]["voltage_v"] < 25.2 and 0.0 < last["battery"]["soc"] < 1.0
    # pace simcore itself: `elapsed` also covers configuring and rebooting the SITL
    flight_s = simcore_wall_seconds(run_dir / "logs/simcore.log")
    assert abs(flight_s - DURATION_S) < 2.0, f"realtime pacing off: {flight_s:.1f} s"
    assert elapsed >= flight_s

    with (run_dir / "data/truth.csv").open() as f:
        rows = [r for r in csv.DictReader(f) if float(r["t_s"]) >= WINDOW_START_S]
    assert len(rows) > 0
    heights = [-float(r["down_m"]) for r in rows]
    tilts = [tilt_deg(*(float(r[k]) for k in ("qw", "qx", "qy", "qz"))) for r in rows]
    assert all(r["crashed"] == "0" for r in rows)
    assert max(abs(h - TARGET_HEIGHT_M) for h in heights) <= 0.5, (min(heights), max(heights))
    assert max(tilts) <= 5.0, max(tilts)


@pytest.mark.skipif(not SITL_BINARY.exists(), reason="Betaflight SITL not built")
@pytest.mark.skipif(
    not any(
        (REPO_ROOT / "build" / p / "simcore/simcore").exists()
        for p in ("release", "ci", "clang", "dev")
    ),
    reason="simcore not built",
)
def test_hover_motor_outputs_follow_the_moment_balance_after_a_battery_shift(
    tmp_path: Path,
) -> None:
    if sitl_port_busy():
        pytest.skip("a Betaflight SITL is already running on TCP 5761")
    drone = load_yaml(REPO_ROOT / "configs/drones/reference_5in.yaml")
    drone["parts"]["battery"]["pos_mm"][0] += 20.0
    (tmp_path / "drone.yaml").write_text(yaml.safe_dump(drone))
    session = load_yaml(REPO_ROOT / "configs/sessions/ci_hover.yaml")
    session["name"] = "ci_hover_shifted"
    session["drone"] = str(tmp_path / "drone.yaml")
    (tmp_path / "session.yaml").write_text(yaml.safe_dump(session))
    resolved = resolve_session(tmp_path / "session.yaml", REPO_ROOT)
    run_dir = write_run_directory(resolved, tmp_path, REPO_ROOT, ["pytest"])
    result = run_headless(resolved, run_dir, find_simcore(REPO_ROOT))
    assert result.exit_code == 0, (run_dir / "logs/simcore.log").read_text()

    with (run_dir / "data/truth.csv").open() as f:
        rows = [r for r in csv.DictReader(f) if float(r["t_s"]) >= WINDOW_START_S]
    assert rows and all(r["crashed"] == "0" for r in rows)
    mean = {k: sum(float(r[k]) for r in rows) / len(rows) for k in ("m1", "m2", "m3", "m4")}
    front, rear = (mean["m2"] + mean["m4"]) / 2.0, (mean["m1"] + mean["m3"]) / 2.0
    # steady thrust goes with command squared; static balance about the shifted CG:
    # T_front (a - d) = T_rear (a + d) with a the arm half-length and d the forward CG shift
    motors = resolved.drone["motors"]
    a = (motors[1]["position_frd_m"][0] - motors[0]["position_frd_m"][0]) / 2.0
    d = resolved.drone["cg_from_origin_frd_m"][0]
    predicted = (a + d) / (a - d) - 1.0
    measured = (front / rear) ** 2 - 1.0
    assert d > 0.005
    assert measured > 0.0, (front, rear)
    assert abs(measured - predicted) <= 0.1 * predicted, (measured, predicted)


class RpmObserver(threading.Thread):
    """Reads what Betaflight believes the rotors are doing, mid-flight."""

    def __init__(self) -> None:
        super().__init__(daemon=True)
        self.error: str | None = None
        self.debug_rpm: tuple[int, ...] = ()
        self.sim_rpm: list[float] = []

    def run(self) -> None:
        try:
            time.sleep(18.0)  # armed and hovering
            msp = connect_uart(5763, timeout_s=10.0, check_msp=True)
            try:
                self.debug_rpm = struct.unpack_from("<8H", msp.request(MspCommand.DEBUG))
            finally:
                msp.close()
            with ControlClient(timeout_s=5.0) as client:
                self.sim_rpm = [m["rpm"] for m in client.request("get_state")["motors"]]
        except Exception as error:
            self.error = f"{type(error).__name__}: {error}"


@pytest.mark.skipif(not SITL_BINARY.exists(), reason="Betaflight SITL not built")
@pytest.mark.skipif(
    not any(
        (REPO_ROOT / "build" / p / "simcore/simcore").exists()
        for p in ("release", "ci", "clang", "dev")
    ),
    reason="simcore not built",
)
def test_betaflight_sees_the_simulated_rotor_speeds(tmp_path: Path) -> None:
    """The RPM filter is fed by DShot telemetry, which the simulator stands in for over UDP."""
    if sitl_port_busy():
        pytest.skip("a Betaflight SITL is already running on TCP 5761")
    resolved = resolve_session(REPO_ROOT / "configs/sessions/ci_rpm_debug.yaml", REPO_ROOT)
    assert "set rpm_filter_harmonics = 3" in resolved.cli_lines  # the notches are enabled
    assert resolved.session["betaflight"]["ports"]["rpm"] == 9006
    run_dir = write_run_directory(resolved, tmp_path, REPO_ROOT, ["pytest"])
    observer = RpmObserver()
    observer.start()
    result = run_headless(resolved, run_dir, find_simcore(REPO_ROOT))
    observer.join(timeout=10.0)
    assert result.exit_code == 0, (run_dir / "logs/simcore.log").read_text()
    assert observer.error is None, observer.error

    assert len(observer.sim_rpm) == 4
    assert all(rpm > 4000.0 for rpm in observer.sim_rpm), observer.sim_rpm
    # Betaflight fills DEBUG_DSHOT_RPM_TELEMETRY from the packet, as its DShot driver would
    for motor, (seen, simulated) in enumerate(
        zip(observer.debug_rpm[:4], observer.sim_rpm, strict=True)
    ):
        assert abs(seen - simulated) < 0.05 * simulated, (motor, seen, simulated)


class WindChanger(threading.Thread):
    """Raises the wind through the control API partway through the flight."""

    def __init__(self, at_sim_s: float, speed_mps: float) -> None:
        super().__init__(daemon=True)
        self.at_sim_s = at_sim_s
        self.speed_mps = speed_mps
        self.error: str | None = None
        self.applied_at_ns = -1
        self.wind_after: list[float] = []

    def run(self) -> None:
        try:
            client = None
            deadline = time.monotonic() + 30.0
            while client is None and time.monotonic() < deadline:
                try:
                    client = ControlClient(timeout_s=3.0)
                except OSError:
                    time.sleep(0.2)
            assert client is not None, "control API never came up"
            with client:
                while client.request("get_state")["sim_time_ns"] < self.at_sim_s * 1e9:
                    time.sleep(0.05)
                result = client.request(
                    "set_env",
                    {"wind": {"mean_speed_mps": self.speed_mps, "mean_from_rad": 1.5 * math.pi}},
                )
                self.applied_at_ns = result["applied_at_ns"]
                time.sleep(0.5)
                sub = client.request("subscribe", {"topic": "telemetry", "rate_hz": 5})
                events = list(client.events(1.0))
                client.request("unsubscribe", {"subscription_id": sub["subscription_id"]})
                self.wind_after = events[-1]["data"]["environment"]["wind_ned_mps"]
        except Exception as error:
            self.error = f"{type(error).__name__}: {error}"


@pytest.mark.skipif(not SITL_BINARY.exists(), reason="Betaflight SITL not built")
@pytest.mark.skipif(
    not any(
        (REPO_ROOT / "build" / p / "simcore/simcore").exists()
        for p in ("release", "ci", "clang", "dev")
    ),
    reason="simcore not built",
)
def test_a_steady_wind_carries_the_hovering_drone_downwind(tmp_path: Path) -> None:
    """Altitude hold keeps the height but not the position, so the drone drifts with the air
    and settles at the wind speed, where the drag is zero. On the way it overshoots: the drag
    acceleration reads as tilt in Betaflight's accelerometer-aided attitude estimate, so angle
    mode rolls a few degrees downwind (sticks stay centred; seen 3.4 deg at 1 m/s^2)."""
    if sitl_port_busy():
        pytest.skip("a Betaflight SITL is already running on TCP 5761")
    environment = load_yaml(REPO_ROOT / "configs/environments/calm_15c.yaml")
    environment["wind"] = {"mean": {"speed_mps": 4.0, "from_deg": 270.0}}
    (tmp_path / "windy.yaml").write_text(yaml.safe_dump(environment))
    session = load_yaml(REPO_ROOT / "configs/sessions/ci_hover.yaml")
    session["name"] = "ci_hover_windy"
    session["environment"] = str(tmp_path / "windy.yaml")
    (tmp_path / "session.yaml").write_text(yaml.safe_dump(session))
    resolved = resolve_session(tmp_path / "session.yaml", REPO_ROOT)
    run_dir = write_run_directory(resolved, tmp_path, REPO_ROOT, ["pytest"])
    changer = WindChanger(at_sim_s=26.0, speed_mps=8.0)
    changer.start()
    result = run_headless(resolved, run_dir, find_simcore(REPO_ROOT))
    changer.join(timeout=10.0)
    assert result.exit_code == 0, (run_dir / "logs/simcore.log").read_text()
    assert changer.error is None, changer.error
    assert changer.applied_at_ns >= 26e9
    assert changer.wind_after[1] == pytest.approx(8.0, abs=1e-6)  # blowing east

    with (run_dir / "data/truth.csv").open() as f:
        rows = [r for r in csv.DictReader(f) if float(r["t_s"]) >= WINDOW_START_S]
    assert rows and all(r["crashed"] == "0" for r in rows)
    east = [float(r["ve_mps"]) for r in rows]
    north = [float(r["vn_mps"]) for r in rows]
    heights = [-float(r["down_m"]) for r in rows]

    def mean_east(start_s: float, end_s: float) -> float:
        picked = [v for r, v in zip(rows, east, strict=True) if start_s <= float(r["t_s"]) < end_s]
        return sum(picked) / len(picked)

    assert mean_east(24.0, 26.0) == pytest.approx(4.0, abs=0.4)  # settled at the 4 m/s wind
    assert mean_east(34.0, 36.0) == pytest.approx(8.0, abs=1.2)  # carried by the raised wind
    assert max(east) <= 8.0 * 1.25, max(east)  # the overshoot stays bounded
    assert max(abs(v) for v in north) < 0.5, max(abs(v) for v in north)
    assert max(abs(h - TARGET_HEIGHT_M) for h in heights) <= 1.0, (min(heights), max(heights))


class FailureInjector(threading.Thread):
    """Damages a prop, then cuts a motor, then clears everything, through the control API."""

    def __init__(self) -> None:
        super().__init__(daemon=True)
        self.error: str | None = None
        self.prop_id = -1
        self.motor_id = -1
        self.listed: list[dict] = []
        self.telemetry_failures: list[dict] = []

    def run(self) -> None:
        try:
            client = None
            deadline = time.monotonic() + 30.0
            while client is None and time.monotonic() < deadline:
                try:
                    client = ControlClient(timeout_s=3.0)
                except OSError:
                    time.sleep(0.2)
            assert client is not None, "control API never came up"
            with client:

                def wait_until(sim_s: float) -> None:
                    while client.request("get_state")["sim_time_ns"] < sim_s * 1e9:
                        time.sleep(0.05)

                wait_until(20.0)
                self.prop_id = client.request(
                    "inject_failure",
                    {
                        "type": "prop_damage",
                        "target": {"motor": 1},
                        "params": {"thrust_loss": 0.3, "vibration_scale": 10.0},
                        "duration_s": 4.0,
                    },
                )["failure_id"]
                time.sleep(0.5)
                self.listed = client.request("list_failures")["failures"]
                wait_until(26.0)
                self.motor_id = client.request(
                    "inject_failure", {"type": "motor_out", "target": {"motor": 2}}
                )["failure_id"]
                sub = client.request("subscribe", {"topic": "telemetry", "rate_hz": 5})
                events = list(client.events(0.6))
                client.request("unsubscribe", {"subscription_id": sub["subscription_id"]})
                self.telemetry_failures = events[-1]["data"]["sim"]["active_failures"]
                wait_until(30.0)
                client.request("clear_failure", {"all": True})
        except Exception as error:
            self.error = f"{type(error).__name__}: {error}"


@pytest.mark.skipif(not SITL_BINARY.exists(), reason="Betaflight SITL not built")
@pytest.mark.skipif(
    not any(
        (REPO_ROOT / "build" / p / "simcore/simcore").exists()
        for p in ("release", "ci", "clang", "dev")
    ),
    reason="simcore not built",
)
def test_failures_injected_live_change_the_flight_and_are_logged(tmp_path: Path) -> None:
    if sitl_port_busy():
        pytest.skip("a Betaflight SITL is already running on TCP 5761")
    resolved = resolve_session(REPO_ROOT / "configs/sessions/ci_hover.yaml", REPO_ROOT)
    run_dir = write_run_directory(resolved, tmp_path, REPO_ROOT, ["pytest"])
    injector = FailureInjector()
    injector.start()
    result = run_headless(resolved, run_dir, find_simcore(REPO_ROOT))
    injector.join(timeout=10.0)
    assert result.exit_code == 0, (run_dir / "logs/simcore.log").read_text()
    assert injector.error is None, injector.error
    assert injector.prop_id > 0 and injector.motor_id > injector.prop_id
    assert [f["type"] for f in injector.listed] == ["prop_damage"]
    assert injector.listed[0]["in_effect"] is True
    assert injector.listed[0]["end_s"] == pytest.approx(injector.listed[0]["start_s"] + 4.0)
    assert {f["type"] for f in injector.telemetry_failures} == {"motor_out"}  # the prop expired

    with (run_dir / "data/events.csv").open() as f:
        events = list(csv.DictReader(f))
    by_id = {(int(e["failure_id"]), e["event"]): float(e["t_s"]) for e in events}
    prop_start = by_id[(injector.prop_id, "start")]
    assert 20.0 <= prop_start < 21.0
    assert by_id[(injector.prop_id, "end")] == pytest.approx(prop_start + 4.0, abs=0.01)
    motor_start = by_id[(injector.motor_id, "start")]
    assert 26.0 <= motor_start < 27.0
    assert 30.0 <= by_id[(injector.motor_id, "end")] < 31.0

    with (run_dir / "data/truth.csv").open() as f:
        rows = list(csv.DictReader(f))

    def window(start_s: float, end_s: float) -> list[dict]:
        return [r for r in rows if start_s <= float(r["t_s"]) < end_s]

    def mean(rows_: list[dict], key: str) -> float:
        return sum(float(r[key]) for r in rows_) / len(rows_)

    # with 30 % less thrust on motor 1 Betaflight drives it harder than its neighbours
    before, damaged = window(17.0, prop_start), window(prop_start + 1.0, prop_start + 4.0)
    assert (
        mean(damaged, "m1") - mean(damaged, "m4") > mean(before, "m1") - mean(before, "m4") + 0.03
    )
    assert all(abs(-float(r["down_m"]) - TARGET_HEIGHT_M) < 1.5 for r in damaged)
    # on three motors the drone cannot hold its height
    cut = window(motor_start, motor_start + 3.0)
    assert -float(cut[-1]["down_m"]) < TARGET_HEIGHT_M - 1.0 or cut[-1]["crashed"] == "1"
