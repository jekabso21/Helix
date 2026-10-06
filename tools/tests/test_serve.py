import json
import os
import signal
import socket
import threading
import time
from collections.abc import Iterator
from pathlib import Path
from typing import Any

import pytest
import yaml

from simtools.simctl.serve import Server, Supervisor, free_port

REPO_ROOT = Path(__file__).resolve().parents[2]


class LineClient:
    def __init__(self, port: int, timeout_s: float = 5.0) -> None:
        # starting a session configures and reboots the SITL, which takes over ten seconds
        self.sock = socket.create_connection(("127.0.0.1", port), timeout=timeout_s)
        self.file = self.sock.makefile("rb")
        self.next_id = 1

    def request(self, method: str, params: dict[str, Any] | None = None) -> dict[str, Any]:
        request_id = self.next_id
        self.next_id += 1
        self.sock.sendall(
            (
                json.dumps({"id": request_id, "method": method, "params": params or {}}) + "\n"
            ).encode()
        )
        while True:
            message = json.loads(self.file.readline())
            if message.get("id") == request_id:
                return message

    def next_event(self) -> dict[str, Any]:
        while True:
            message = json.loads(self.file.readline())
            if "event" in message:
                return message

    def close(self) -> None:
        self.sock.close()


@pytest.fixture
def server() -> Iterator[Server]:
    server = Server(("127.0.0.1", free_port()), Supervisor(REPO_ROOT))
    thread = threading.Thread(
        target=server.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True
    )
    thread.start()
    yield server
    server.close()
    thread.join(timeout=5.0)


def test_lists_sessions_with_their_input_source(server: Server) -> None:
    client = LineClient(server.server_address[1])
    result = client.request("list_sessions")["result"]["sessions"]
    by_name = {s["name"]: s for s in result}
    assert by_name["ci_hover"]["input"] == "altitude_hold"
    assert by_name["dev_gamepad"]["input"] == "gamepad"
    assert by_name["ci_hover"]["path"] == "configs/sessions/ci_hover.yaml"
    client.close()


def test_validate_reports_errors_and_status_is_idle(server: Server, tmp_path: Path) -> None:
    client = LineClient(server.server_address[1])
    assert (
        client.request("validate", {"path": "configs/sessions/ci_hover.yaml"})["result"]["ok"]
        is True
    )
    bad = client.request("validate", {"path": "configs/sessions/nope.yaml"})["result"]
    assert bad["ok"] is False and bad["errors"]
    status = client.request("status")["result"]
    assert status["state"] == "idle" and status["processes"] == []
    assert client.request("tail_log", {"process": "simcore"})["result"]["lines"] == []
    client.close()


def test_input_mappings_are_listed_and_saved(server: Server, tmp_path: Path) -> None:
    client = LineClient(server.server_address[1])
    listed = client.request("list_input_mappings")["result"]["mappings"]
    names = {m["name"] for m in listed}
    assert {"radiomaster_boxer", "default"} <= names
    boxer = next(m for m in listed if m["name"] == "radiomaster_boxer")
    assert boxer["mapping"]["channels"]["throttle"]["axis"] == 0
    # the default entry names the profile it extends, so a client can save edits into that one
    default = next(m for m in listed if m["name"] == "default")
    assert boxer["profile"] == "radiomaster_boxer"
    assert default["profile"] == "radiomaster_boxer"
    bad = client.request(
        "save_input_mapping", {"name": "x", "mapping": {"channels": {"gear": {"axis": 1}}}}
    )
    assert bad["error"]["code"] == "invalid_state"
    client.close()


def test_input_devices_are_listed_without_a_session(server: Server) -> None:
    client = LineClient(server.server_address[1])
    devices = client.request("list_input_devices")["result"]["devices"]
    assert isinstance(devices, list) and all(isinstance(d, str) for d in devices)
    client.close()


def test_errors_use_the_shared_codes(server: Server) -> None:
    client = LineClient(server.server_address[1])
    assert client.request("fly")["error"]["code"] == "unknown_method"
    assert client.request("validate")["error"]["code"] == "invalid_params"
    assert client.request("subscribe", {"topic": "weather"})["error"]["code"] == "invalid_params"
    client.close()


def test_shutdown_stops_the_server() -> None:
    server = Server(("127.0.0.1", free_port()), Supervisor(REPO_ROOT))
    thread = threading.Thread(
        target=server.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True
    )
    thread.start()
    client = LineClient(server.server_address[1])
    assert client.request("shutdown")["ok"]
    thread.join(timeout=5.0)
    assert not thread.is_alive()
    client.close()


def test_status_subscription_sends_the_current_status_first(server: Server) -> None:
    client = LineClient(server.server_address[1])
    result = client.request("subscribe", {"topic": "status"})["result"]
    event = client.next_event()
    assert event["event"] == "status" and event["data"]["state"] == "idle"
    assert client.request("unsubscribe", {"subscription_id": result["subscription_id"]})["ok"]
    client.close()


def test_drone_preview_without_a_session_needs_a_path(server: Server) -> None:
    client = LineClient(server.server_address[1])
    assert client.request("get_drone")["error"]["code"] == "invalid_state"
    base = client.request("get_drone", {"path": "configs/sessions/ci_hover.yaml"})["result"]
    assert base["mass_kg"] == pytest.approx(0.497)
    names = [p["name"] for p in base["parts"]]
    assert "battery" in names and "motor1" not in names
    moved = client.request(
        "preview_drone",
        {
            "path": "configs/sessions/ci_hover.yaml",
            "overrides": {"parts": {"battery": {"pos_mm": [20.0, 0.0, -46.0]}}},
        },
    )["result"]
    shift = moved["cg_from_origin_frd_m"][0] - base["cg_from_origin_frd_m"][0]
    assert shift == pytest.approx(0.185 * 0.020 / 0.497, abs=1e-9)
    front = [h["fraction"] for h in moved["hover"] if h["bf_index"] in (2, 4)]
    rear = [h["fraction"] for h in moved["hover"] if h["bf_index"] in (1, 3)]
    assert min(front) > max(rear)
    bad = client.request(
        "preview_drone",
        {"path": "configs/sessions/ci_hover.yaml", "overrides": {"parts": {"nope": {}}}},
    )
    assert bad["error"]["code"] == "invalid_params"
    assert client.request("apply_drone", {"overrides": {}})["error"]["code"] == "invalid_state"


@pytest.mark.integration
@pytest.mark.skipif(
    not (REPO_ROOT / "build/betaflight/betaflight_SITL.elf").exists(), reason="SITL not built"
)
def test_session_outlives_the_client_that_started_it() -> None:
    """Drives the real backend process, as the app does; an in-process Server never showed it."""
    import shutil
    import subprocess
    import sys

    from simtools.sitl import sitl_port_busy

    if sitl_port_busy():
        pytest.skip("a Betaflight SITL is already running on TCP 5761")
    port = free_port()
    uv = shutil.which("uv")
    command = (
        [uv, "run", "--project", str(REPO_ROOT / "tools"), "simctl"]
        if uv
        else [sys.executable, "-m", "simtools.simctl.cli"]
    ) + ["serve", "--base-dir", str(REPO_ROOT), "--port", str(port)]
    # its own process group: `uv run` spawns the real server as a child, so a kill must reach both
    backend = subprocess.Popen(
        command, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT, start_new_session=True
    )
    try:
        deadline = time.monotonic() + 10.0
        while True:
            try:
                starter = LineClient(port, timeout_s=90.0)
                break
            except OSError:
                assert time.monotonic() < deadline, "backend did not come up"
                time.sleep(0.2)
        assert starter.request("start", {"path": "configs/sessions/ci_hover.yaml"})["ok"]
        starter.close()  # the handler thread of this connection ends here
        time.sleep(6.0)
        watcher = LineClient(port)
        processes = watcher.request("status")["result"]["processes"]
        osd = watcher.request("get_osd")["result"]  # Betaflight pushes its OSD over DisplayPort
        watcher.request("stop")
        watcher.request("shutdown")
        watcher.close()
        running = [(p["name"], p["state"]) for p in processes]
        assert running[:2] == [("betaflight", "running"), ("simcore", "running")], processes
        # simvideo joins the session when it has been built and the session configures a camera
        assert running[2:] in ([], [("simvideo", "running")]), processes
        assert osd["draws"] > 0 and any(any(c != 0x20 for c in row) for row in osd["codes"]), osd[
            "draws"
        ]
    finally:
        try:
            backend.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            os.killpg(backend.pid, signal.SIGKILL)
            backend.wait(timeout=5.0)


VIDEO_REPORT = {
    "version": 1,
    "camera": "main_fpv",
    "input_fps": 59.9,
    "late_frames": 0,
    "outputs": [
        {
            "index": 0,
            "pipeline": "v4l2sink device=/dev/video10",
            "state": "running",
            "fps": 59.9,
            "bitrate_bps": None,
            "last_error": None,
        },
        {
            "index": 1,
            "pipeline": "udpsink port=5600",
            "state": "error",
            "fps": 0.0,
            "bitrate_bps": None,
            "last_error": "could not link",
        },
    ],
}


def test_video_status_datagrams_reach_subscribers(server: Server) -> None:
    port = free_port()
    supervisor = server.supervisor
    supervisor.watch_video_status(port)
    client = LineClient(server.server_address[1])
    try:
        client.request("subscribe", {"topic": "video"})
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            sender.sendto(json.dumps(VIDEO_REPORT).encode(), ("127.0.0.1", port))
            try:
                event = client.next_event()
            except (TimeoutError, OSError):
                continue
            if event["event"] == "video":
                break
        else:
            pytest.fail("no video event arrived")
        assert event["data"]["camera"] == "main_fpv"
        assert [o["state"] for o in event["data"]["outputs"]] == ["running", "error"]
        assert client.request("get_video")["result"]["cameras"][0]["outputs"][1]["last_error"] == (
            "could not link"
        )
        sender.close()
    finally:
        supervisor.stop()
        client.close()


def test_a_malformed_video_datagram_is_ignored(server: Server) -> None:
    from simtools.simctl.serve import _video_report

    assert _video_report(b"not json") is None
    assert _video_report(json.dumps({"version": 2, "camera": "c", "outputs": []}).encode()) is None
    assert _video_report(json.dumps({"version": 1, "outputs": []}).encode()) is None
    assert _video_report(json.dumps(VIDEO_REPORT).encode())["camera"] == "main_fpv"


def test_subscribe_rejects_an_unknown_topic(server: Server) -> None:
    client = LineClient(server.server_address[1])
    error = client.request("subscribe", {"topic": "nonsense"})["error"]
    assert error["code"] == "invalid_params" and "video" in error["message"]
    client.close()


def test_set_output_enabled_needs_a_running_session(server: Server) -> None:
    client = LineClient(server.server_address[1])
    error = client.request(
        "set_output_enabled", {"camera": "main_fpv", "index": 0, "enabled": False}
    )["error"]
    assert error["code"] == "invalid_state"
    client.close()


def test_set_output_enabled_sends_a_datagram_to_simvideo(server: Server) -> None:
    from simtools.config.resolver import resolve_session

    control = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    control.bind(("127.0.0.1", 0))
    control.settimeout(5.0)
    port = control.getsockname()[1]
    supervisor = server.supervisor
    resolved = resolve_session(REPO_ROOT / "configs/sessions/ci_hover.yaml", REPO_ROOT)
    resolved.cameras["control_port"] = port
    supervisor._resolved = resolved
    client = LineClient(server.server_address[1])
    try:
        result = client.request(
            "set_output_enabled", {"camera": "main_fpv", "index": 1, "enabled": False}
        )["result"]
        assert result["ok"] and result["index"] == 1
        message = json.loads(control.recv(4096))
        assert message == {"version": 1, "camera": "main_fpv", "index": 1, "enabled": False}
        unknown = client.request(
            "set_output_enabled", {"camera": "nope", "index": 0, "enabled": True}
        )["error"]
        assert unknown["code"] == "invalid_state" and "unknown camera" in unknown["message"]
    finally:
        supervisor._resolved = None
        control.close()
        client.close()


def test_a_saved_mapping_keeps_whole_number_indices(tmp_path: Path) -> None:
    # a client that read the mapping back from JSON sends 4.0 for axis 4
    mapping = {
        "device_name_contains": "Boxer",
        "channels": {"aux2": {"axis": 4.0, "inverted": True, "deadband": 0.0}},
    }
    Supervisor(tmp_path).save_input_mapping("pad", mapping, False)
    saved = yaml.safe_load((tmp_path / "configs/input/pad.yaml").read_text())
    assert saved["channels"]["aux2"] == {"axis": 4, "inverted": True, "deadband": 0.0}
    assert isinstance(saved["channels"]["aux2"]["axis"], int)
