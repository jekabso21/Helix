import struct
from dataclasses import dataclass

KISS_FRAME_SIZE = 10
ERPM_PER_LSB = 100.0


@dataclass(frozen=True)
class KissTelemetry:
    temperature_c: float
    voltage_v: float
    current_a: float
    consumption_mah: float
    erpm: float


def crc8(data: bytes) -> int:
    """CRC8 with polynomial 0x07, as in src/main/sensors/esc_sensor.c at Betaflight tag 2026.6.2."""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def _clamp(value: float, maximum: int) -> int:
    return max(0, min(maximum, round(value)))


def encode_kiss_frame(telemetry: KissTelemetry) -> bytes:
    """10 bytes, big-endian: temperature C, 0.01 V, 0.01 A, mAh, eRPM / 100, CRC8."""
    body = struct.pack(
        ">BHHHH",
        _clamp(telemetry.temperature_c, 0xFF),
        _clamp(telemetry.voltage_v * 100.0, 0xFFFF),
        _clamp(telemetry.current_a * 100.0, 0xFFFF),
        _clamp(telemetry.consumption_mah, 0xFFFF),
        _clamp(telemetry.erpm / ERPM_PER_LSB, 0xFFFF),
    )
    return body + bytes([crc8(body)])
