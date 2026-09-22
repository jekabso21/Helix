import json
import socket
import threading
from collections.abc import Iterator
from pathlib import Path
from typing import Any

import pytest

from simtools.simctl.serve import Server, Supervisor, free_port

REPO_ROOT = Path(__file__).resolve().parents[2]


class LineClient:
    def __init__(self, port: int) -> None:
        self.sock = socket.create_connection(("127.0.0.1", port), timeout=5.0)
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


def test_errors_use_the_shared_codes(server: Server) -> None:
    client = LineClient(server.server_address[1])
    assert client.request("fly")["error"]["code"] == "unknown_method"
    assert client.request("validate")["error"]["code"] == "invalid_params"
    assert client.request("subscribe", {"topic": "weather"})["error"]["code"] == "invalid_params"
    client.close()


def test_status_subscription_sends_the_current_status_first(server: Server) -> None:
    client = LineClient(server.server_address[1])
    result = client.request("subscribe", {"topic": "status"})["result"]
    event = client.next_event()
    assert event["event"] == "status" and event["data"]["state"] == "idle"
    assert client.request("unsubscribe", {"subscription_id": result["subscription_id"]})["ok"]
    client.close()
