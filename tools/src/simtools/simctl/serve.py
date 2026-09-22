import json
import queue
import socket
import socketserver
import sys
import threading
import time
from pathlib import Path
from typing import Any, cast

from simtools.config.resolver import ConfigError, resolve_session, write_run_directory
from simtools.msp import MspCommand, MspError, parse_status_ex
from simtools.simctl.launcher import LaunchError, ProcessSet, find_simcore, start_processes
from simtools.sitl import HOST, connect_uart

API_VERSION = 1
PORT_UART2 = 5762
FC_POLL_S = 0.2

Json = dict[str, Any]


class Supervisor:
    """Owns at most one running session; every method is safe to call from any thread."""

    def __init__(self, base_dir: Path) -> None:
        self.base_dir = base_dir
        self._lock = threading.Lock()
        self._processes: ProcessSet | None = None
        self._run_dir: Path | None = None
        self._state = "idle"
        self._last_error: str | None = None
        self._subscribers: list[tuple[str, queue.Queue[Json]]] = []
        self._fc_thread: threading.Thread | None = None
        self._fc_stop = threading.Event()

    # queries

    def list_sessions(self) -> list[Json]:
        sessions: list[Json] = []
        for path in sorted((self.base_dir / "configs/sessions").glob("*.yaml")):
            entry: Json = {"path": str(path.relative_to(self.base_dir)), "name": path.stem}
            try:
                resolved = resolve_session(path, self.base_dir)
                entry["name"] = resolved.name
                entry["input"] = resolved.session["input"]["source"]
                entry["duration_s"] = resolved.session["duration_s"]
            except ConfigError as error:
                entry["error"] = str(error)
            sessions.append(entry)
        return sessions

    def validate(self, path: str) -> Json:
        try:
            resolved = resolve_session(self.base_dir / path, self.base_dir)
        except ConfigError as error:
            return {"ok": False, "errors": str(error).splitlines()}
        return {"ok": True, "name": resolved.name}

    def status(self) -> Json:
        with self._lock:
            processes = [] if self._processes is None else self._processes.poll()
            return {
                "state": self._state,
                "run_dir": None if self._run_dir is None else str(self._run_dir),
                "last_error": self._last_error,
                "processes": [
                    {
                        "name": p.name,
                        "pid": p.pid,
                        "state": p.state,
                        "exit_code": p.exit_code,
                        "uptime_s": round(time.monotonic() - p.started_at, 1),
                    }
                    for p in processes
                ],
            }

    def tail_log(self, process: str, lines: int) -> list[str]:
        with self._lock:
            run_dir = self._run_dir
        if run_dir is None:
            return []
        candidates = {
            "simcore": run_dir / "logs/simcore.log",
            "betaflight": run_dir / "betaflight/sitl_run.log",
            "betaflight_configure": run_dir / "betaflight/sitl_configure.log",
        }
        path = candidates.get(process)
        if path is None or not path.exists():
            path = run_dir / "logs" / f"{process}.log"
        if not path.exists():
            return []
        return path.read_text(errors="replace").splitlines()[-max(1, lines) :]

    # commands

    def start(self, path: str, argv: list[str]) -> Json:
        with self._lock:
            if self._processes is not None and self._processes.alive():
                raise LaunchError("a session is already running; stop it first")
            self._state = "starting"
            self._last_error = None
        try:
            resolved = resolve_session(self.base_dir / path, self.base_dir)
            run_root = self.base_dir / resolved.logging_root
            run_dir = write_run_directory(resolved, run_root, self.base_dir, argv)
            simcore = find_simcore(self.base_dir)
            with self._lock:
                self._run_dir = run_dir
            self._publish("status", self.status())
            processes = start_processes(resolved, run_dir, simcore)
        except (ConfigError, LaunchError, OSError) as error:
            with self._lock:
                self._state = "idle"
                self._last_error = str(error)
            self._publish("status", self.status())
            raise LaunchError(str(error)) from error
        with self._lock:
            self._processes = processes
            self._state = "running"
        self._start_fc_poller()
        self._publish("status", self.status())
        return {"run_dir": str(run_dir), "processes": self.status()["processes"]}

    def stop(self) -> Json:
        self._stop_fc_poller()
        with self._lock:
            processes = self._processes
            self._processes = None
            self._state = "idle"
        exit_code = None if processes is None else processes.stop()
        self._publish("status", self.status())
        return {"ok": True, "simcore_exit_code": exit_code}

    def shutdown(self) -> None:
        self.stop()

    # subscriptions

    def subscribe(self, topic: str) -> queue.Queue[Json]:
        events: queue.Queue[Json] = queue.Queue()
        with self._lock:
            self._subscribers.append((topic, events))
        if topic == "status":
            events.put({"event": "status", "data": self.status()})
        return events

    def unsubscribe(self, events: queue.Queue[Json]) -> None:
        with self._lock:
            self._subscribers = [(t, q) for t, q in self._subscribers if q is not events]

    def _publish(self, topic: str, data: Json) -> None:
        with self._lock:
            targets = [q for t, q in self._subscribers if t == topic]
        for events in targets:
            events.put({"event": topic, "data": data})

    # flight controller status over MSP on UART2

    def _start_fc_poller(self) -> None:
        self._fc_stop.clear()
        self._fc_thread = threading.Thread(target=self._poll_fc, daemon=True)
        self._fc_thread.start()

    def _stop_fc_poller(self) -> None:
        self._fc_stop.set()
        if self._fc_thread is not None:
            self._fc_thread.join(timeout=3.0)
            self._fc_thread = None

    def _poll_fc(self) -> None:
        try:
            msp = connect_uart(PORT_UART2, 10.0, check_msp=True)
        except (TimeoutError, OSError):
            return
        with msp:
            while not self._fc_stop.is_set():
                try:
                    status = parse_status_ex(msp.request(MspCommand.STATUS_EX))
                except (MspError, OSError):
                    return
                self._publish(
                    "fc",
                    {
                        "armed": status.armed,
                        "arming_disable_flags": status.arming_disable_names,
                        "flight_mode_flags": status.flight_mode_flags,
                        "pid_cycle_time_us": status.pid_cycle_time_us,
                    },
                )
                self._fc_stop.wait(FC_POLL_S)


def split_lines(data: bytes) -> tuple[list[bytes], bytes]:
    """Complete non-empty lines and the unterminated rest."""
    parts = data.split(b"\n")
    return [line for line in parts[:-1] if line.strip()], parts[-1]


class Handler(socketserver.StreamRequestHandler):
    """One JSON-lines client; subscriptions stream events on the same connection."""

    def handle(self) -> None:
        server = cast("Server", self.server)
        supervisor = server.supervisor
        subscriptions: dict[int, queue.Queue[Json]] = {}
        next_id = 1
        self.request.settimeout(0.05)
        buffer: bytes = b""
        try:
            while not server.stopping:
                for events in subscriptions.values():
                    while True:
                        try:
                            event = events.get_nowait()
                        except queue.Empty:
                            break
                        self._send(event)
                try:
                    data = cast(bytes, self.request.recv(65536))
                except TimeoutError:
                    continue
                if not data:
                    break
                lines, buffer = split_lines(buffer + data)
                for line in lines:
                    response, sub = self._dispatch(supervisor, line, subscriptions, next_id)
                    if sub is not None:
                        subscriptions[next_id] = sub
                        next_id += 1
                    self._send(response)
        except (OSError, ConnectionError):
            pass
        finally:
            for events in subscriptions.values():
                supervisor.unsubscribe(events)

    def _send(self, message: Json) -> None:
        self.request.sendall((json.dumps(message) + "\n").encode())

    def _dispatch(
        self,
        supervisor: Supervisor,
        line: bytes,
        subscriptions: dict[int, queue.Queue[Json]],
        next_id: int,
    ) -> tuple[Json, queue.Queue[Json] | None]:
        try:
            request: Json = json.loads(line)
        except json.JSONDecodeError:
            return {
                "id": None,
                "ok": False,
                "error": {"code": "invalid_json", "message": "not JSON"},
            }, None
        request_id = request.get("id")
        method = request.get("method")
        params: Json = request.get("params") or {}
        try:
            if method == "ping":
                return self._ok(request_id, {"pong": True}), None
            if method == "get_info":
                return self._ok(
                    request_id, {"api_version": API_VERSION, "base_dir": str(supervisor.base_dir)}
                ), None
            if method == "list_sessions":
                return self._ok(request_id, {"sessions": supervisor.list_sessions()}), None
            if method == "validate":
                return self._ok(request_id, supervisor.validate(str(params["path"]))), None
            if method == "start":
                return self._ok(request_id, supervisor.start(str(params["path"]), sys.argv)), None
            if method == "stop":
                return self._ok(request_id, supervisor.stop()), None
            if method == "status":
                return self._ok(request_id, supervisor.status()), None
            if method == "tail_log":
                lines = supervisor.tail_log(
                    str(params.get("process", "simcore")), int(params.get("lines", 50))
                )
                return self._ok(request_id, {"lines": lines}), None
            if method == "subscribe":
                topic = str(params.get("topic", ""))
                if topic not in ("status", "fc"):
                    return self._error(
                        request_id, "invalid_params", "topic must be status or fc"
                    ), None
                return self._ok(request_id, {"subscription_id": next_id}), supervisor.subscribe(
                    topic
                )
            if method == "unsubscribe":
                events = subscriptions.pop(int(params["subscription_id"]), None)
                if events is None:
                    return self._error(request_id, "invalid_params", "unknown subscription"), None
                supervisor.unsubscribe(events)
                return self._ok(request_id, {"ok": True}), None
            return self._error(request_id, "unknown_method", str(method)), None
        except KeyError as error:
            return self._error(request_id, "invalid_params", f"missing {error}"), None
        except LaunchError as error:
            return self._error(request_id, "invalid_state", str(error)), None

    @staticmethod
    def _ok(request_id: Any, result: Json) -> Json:
        return {"id": request_id, "ok": True, "result": result}

    @staticmethod
    def _error(request_id: Any, code: str, message: str) -> Json:
        return {"id": request_id, "ok": False, "error": {"code": code, "message": message}}


class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, address: tuple[str, int], supervisor: Supervisor) -> None:
        super().__init__(address, Handler)
        self.supervisor = supervisor
        self.stopping = False

    def close(self) -> None:
        self.stopping = True
        self.shutdown()
        self.server_close()
        self.supervisor.shutdown()


def serve_forever(base_dir: Path, host: str = HOST, port: int = 7740) -> None:
    server = Server((host, port), Supervisor(base_dir))
    print(f"simctl serve listening on {host}:{port}, base directory {base_dir}", flush=True)
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.close()


def free_port() -> int:
    with socket.socket() as probe:
        probe.bind((HOST, 0))
        return int(probe.getsockname()[1])
