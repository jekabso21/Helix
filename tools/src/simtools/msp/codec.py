from dataclasses import dataclass
from enum import IntEnum
from functools import reduce

# Frame: $M, direction (< request, > response, ! error), size, command, payload, XOR checksum
_PREAMBLE = b"$M"
_MAX_PAYLOAD = 255


# IDs from src/main/msp/msp_protocol.h at Betaflight tag 2026.6.2
class MspCommand(IntEnum):
    API_VERSION = 1
    STATUS = 101
    RAW_IMU = 102
    MOTOR = 104
    RC = 105
    ATTITUDE = 108
    ALTITUDE = 109
    ANALOG = 110
    STATUS_EX = 150
    SET_RAW_RC = 200


@dataclass(frozen=True)
class MspFrame:
    command: int
    payload: bytes
    is_error: bool


def _checksum(size: int, command: int, payload: bytes) -> int:
    return reduce(lambda acc, byte: acc ^ byte, payload, size ^ command)


def encode_request(command: int, payload: bytes = b"") -> bytes:
    if len(payload) > _MAX_PAYLOAD:
        raise ValueError(f"MSP v1 payload too long: {len(payload)} > {_MAX_PAYLOAD}")
    if not 0 <= command <= 255:
        raise ValueError(f"MSP v1 command out of range: {command}")
    size = len(payload)
    return (
        _PREAMBLE
        + b"<"
        + bytes([size, command])
        + payload
        + bytes([_checksum(size, command, payload)])
    )


class MspDecoder:
    """Incremental response decoder; skips non-frame bytes and counts bad checksums."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.checksum_errors = 0

    def feed(self, data: bytes) -> list[MspFrame]:
        self._buffer += data
        frames: list[MspFrame] = []
        while True:
            start = self._buffer.find(_PREAMBLE)
            if start < 0:
                # Keep a trailing "$": the preamble may be split across reads
                del self._buffer[: max(0, len(self._buffer) - 1)]
                return frames
            del self._buffer[:start]
            if len(self._buffer) < 5:
                return frames
            direction = self._buffer[2]
            if direction not in (ord(">"), ord("!")):
                del self._buffer[:2]
                continue
            size = self._buffer[3]
            frame_length = 5 + size + 1
            if len(self._buffer) < frame_length:
                return frames
            command = self._buffer[4]
            payload = bytes(self._buffer[5 : 5 + size])
            if self._buffer[5 + size] == _checksum(size, command, payload):
                frames.append(MspFrame(command, payload, is_error=direction == ord("!")))
                del self._buffer[:frame_length]
            else:
                self.checksum_errors += 1
                del self._buffer[:2]
