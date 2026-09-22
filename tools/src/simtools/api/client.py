import json
import socket
import time
from collections.abc import Iterator
from types import TracebackType
from typing import Any, Self


class ApiError(Exception):
    def __init__(self, code: str, message: str) -> None:
        super().__init__(f"{code}: {message}")
        self.code = code


class ControlClient:
    """Blocking JSON-lines client for the simcore control API."""

    def __init__(self, host: str = "127.0.0.1", port: int = 7700, timeout_s: float = 2.0) -> None:
        self._socket = socket.create_connection((host, port), timeout=timeout_s)
        self._socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._timeout_s = timeout_s
        self._buffer = b""
        self._next_id = 1
        self._events: list[dict[str, Any]] = []

    def __enter__(self) -> Self:
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
        self.close()

    def close(self) -> None:
        self._socket.close()

    def request(self, method: str, params: dict[str, Any] | None = None) -> Any:
        request_id = self._next_id
        self._next_id += 1
        line = json.dumps({"id": request_id, "method": method, "params": params or {}}) + "\n"
        self._socket.sendall(line.encode())
        deadline = time.monotonic() + self._timeout_s
        while True:
            message = self._read_message(deadline)
            if "event" in message:
                self._events.append(message)
                continue
            if message.get("id") != request_id:
                continue
            if message.get("ok"):
                return message["result"]
            error = message.get("error", {})
            raise ApiError(error.get("code", "unknown"), error.get("message", ""))

    def events(self, duration_s: float) -> Iterator[dict[str, Any]]:
        """Yield events received within duration_s, including any queued during requests."""
        deadline = time.monotonic() + duration_s
        while self._events:
            yield self._events.pop(0)
        while time.monotonic() < deadline:
            try:
                message = self._read_message(deadline)
            except TimeoutError:
                return
            if "event" in message:
                yield message

    def _read_message(self, deadline: float) -> dict[str, Any]:
        while b"\n" not in self._buffer:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("no message within the timeout")
            self._socket.settimeout(remaining)
            data = self._socket.recv(65536)
            if not data:
                raise ConnectionError("control API closed the connection")
            self._buffer += data
        line, self._buffer = self._buffer.split(b"\n", 1)
        return json.loads(line)
