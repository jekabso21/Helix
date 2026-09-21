import socket
import time
from types import TracebackType
from typing import Self

from simtools.msp.codec import MspDecoder, MspFrame, encode_request


class MspError(Exception):
    pass


class MspClient:
    """One TCP connection to one SITL UART; a UART accepts a single client at a time."""

    def __init__(self, host: str = "127.0.0.1", port: int = 5763, timeout_s: float = 1.0) -> None:
        self._timeout_s = timeout_s
        self._decoder = MspDecoder()
        self._pending: list[MspFrame] = []
        self._socket = socket.create_connection((host, port), timeout=timeout_s)
        self._socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def __enter__(self) -> Self:
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        self.close()

    def close(self) -> None:
        self._socket.close()

    def request(self, command: int, payload: bytes = b"") -> bytes:
        self._socket.sendall(encode_request(command, payload))
        deadline = time.monotonic() + self._timeout_s
        while True:
            for index, frame in enumerate(self._pending):
                if frame.command == command:
                    del self._pending[index]
                    if frame.is_error:
                        raise MspError(f"MSP command {command} rejected by the flight controller")
                    return frame.payload
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise MspError(f"MSP command {command} timed out after {self._timeout_s} s")
            self._socket.settimeout(remaining)
            try:
                data = self._socket.recv(4096)
            except TimeoutError:
                continue
            if not data:
                raise MspError("connection closed by the flight controller")
            self._pending += self._decoder.feed(data)

    def send_raw(self, data: bytes) -> None:
        self._socket.sendall(data)

    def read_raw(self, duration_s: float) -> bytes:
        chunks: list[bytes] = []
        deadline = time.monotonic() + duration_s
        while (remaining := deadline - time.monotonic()) > 0:
            self._socket.settimeout(remaining)
            try:
                data = self._socket.recv(4096)
            except TimeoutError:
                break
            except ConnectionError:
                break
            if not data:
                break
            chunks.append(data)
        return b"".join(chunks)
