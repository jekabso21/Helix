import struct

import pytest

from simtools.msp import (
    MspCommand,
    MspDecoder,
    encode_request,
    parse_analog,
    parse_attitude,
    parse_motor,
    parse_motor_telemetry,
    parse_raw_imu,
    parse_rc,
    parse_status_ex,
)


def _response(command: int, payload: bytes, direction: bytes = b">") -> bytes:
    checksum = len(payload) ^ command
    for byte in payload:
        checksum ^= byte
    return b"$M" + direction + bytes([len(payload), command]) + payload + bytes([checksum])


def test_request_without_payload_matches_golden_bytes() -> None:
    assert encode_request(MspCommand.API_VERSION) == bytes.fromhex("244d3c000101")
    assert encode_request(MspCommand.STATUS) == bytes.fromhex("244d3c006565")


def test_request_checksum_covers_size_command_and_payload() -> None:
    frame = encode_request(MspCommand.SET_RAW_RC, struct.pack("<2H", 1500, 1000))
    # checksum 0xfe = 0x04 ^ 0xc8 ^ 0xdc ^ 0x05 ^ 0xe8 ^ 0x03
    assert frame == bytes.fromhex("244d3c04c8dc05e803fe")


def test_request_rejects_payload_longer_than_255_bytes() -> None:
    with pytest.raises(ValueError, match="too long"):
        encode_request(MspCommand.SET_RAW_RC, bytes(256))


def test_decoder_returns_frame_split_across_reads() -> None:
    data = _response(MspCommand.ATTITUDE, b"\x01\x02\x03\x04\x05\x06")
    decoder = MspDecoder()
    assert decoder.feed(data[:4]) == []
    frames = decoder.feed(data[4:])
    assert [(f.command, f.payload, f.is_error) for f in frames] == [
        (MspCommand.ATTITUDE, b"\x01\x02\x03\x04\x05\x06", False)
    ]


def test_decoder_skips_text_and_handles_preamble_split_across_reads() -> None:
    decoder = MspDecoder()
    assert decoder.feed(b"\r\n# cli text $") == []
    frames = decoder.feed(_response(MspCommand.MOTOR, b"\xe8\x03")[1:])
    assert [f.payload for f in frames] == [b"\xe8\x03"]


def test_decoder_drops_bad_checksum_and_recovers() -> None:
    bad = bytearray(_response(MspCommand.RC, b"\x00\x01"))
    bad[-1] ^= 0xFF
    decoder = MspDecoder()
    frames = decoder.feed(bytes(bad) + _response(MspCommand.RC, b"\x02\x03"))
    assert [f.payload for f in frames] == [b"\x02\x03"]
    assert decoder.checksum_errors == 1


def test_decoder_marks_error_responses() -> None:
    frames = MspDecoder().feed(_response(MspCommand.SET_RAW_RC, b"", direction=b"!"))
    assert frames[0].is_error


def test_status_ex_reports_armed_and_arming_disable_names() -> None:
    # One extra mode-flag byte shifts the arming flags: RXLOSS is bit 2, THROTTLE bit 7
    payload = struct.pack("<HHHIBHBBB", 125, 0, 0x23, 0x1, 0, 7, 4, 0, 1) + b"\x00"
    payload += struct.pack("<BI", 30, (1 << 2) | (1 << 7)) + b"\x00\x00\x00\x04"
    status = parse_status_ex(payload)
    assert status.armed
    assert status.pid_cycle_time_us == 125
    assert status.arming_disable_names == ["RXLOSS", "THROTTLE"]


def test_attitude_is_signed_with_roll_and_pitch_in_decidegrees() -> None:
    attitude = parse_attitude(struct.pack("<3h", 205, -153, 270))
    assert (attitude.roll_deg, attitude.pitch_deg, attitude.yaw_deg) == (20.5, -15.3, 270.0)


def test_raw_imu_splits_into_acc_gyro_mag() -> None:
    imu = parse_raw_imu(struct.pack("<9h", 1, 2, 3, -4, -5, -6, 7, 8, 9))
    assert imu.acc_counts == (1, 2, 3)
    assert imu.gyro_dps == (-4, -5, -6)
    assert imu.mag_counts == (7, 8, 9)


def test_analog_scales_voltage_and_current() -> None:
    analog = parse_analog(struct.pack("<BHHhH", 235, 492, 0, 3000, 2350))
    assert (analog.voltage_v, analog.current_a, analog.consumed_mah) == (23.5, 30.0, 492)


def test_motor_telemetry_reads_one_record_per_motor() -> None:
    payload = bytes([2]) + struct.pack("<IHBHHH", 20000, 0, 41, 2350, 750, 123)
    payload += struct.pack("<IHBHHH", 21000, 0, 42, 2340, 760, 124)
    motors = parse_motor_telemetry(payload)
    assert [m.rpm for m in motors] == [20000, 21000]
    assert (motors[1].temperature_c, motors[1].voltage_v, motors[1].current_a) == (42, 23.4, 7.6)
    assert motors[0].consumption_mah == 123


def test_rc_payload_parses_as_uint16_list() -> None:
    assert parse_rc(struct.pack("<5H", 1500, 1500, 1500, 1000, 2000)) == [
        1500,
        1500,
        1500,
        1000,
        2000,
    ]


def test_motor_payload_parses_as_uint16_list() -> None:
    assert parse_motor(struct.pack("<4H", 1000, 1100, 1200, 0)) == [1000, 1100, 1200, 0]
