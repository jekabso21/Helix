from simtools.esc import KISS_FRAME_SIZE, KissTelemetry, crc8, encode_kiss_frame


def test_crc8_matches_known_values_for_polynomial_0x07() -> None:
    assert crc8(b"") == 0x00
    assert crc8(b"\x01") == 0x07
    assert crc8(b"123456789") == 0xF4


def test_frame_packs_fields_big_endian_with_scaling() -> None:
    frame = encode_kiss_frame(
        KissTelemetry(
            temperature_c=41, voltage_v=23.5, current_a=7.5, consumption_mah=123, erpm=140_000
        )
    )
    assert len(frame) == KISS_FRAME_SIZE
    assert frame[:9] == bytes.fromhex("29 092e 02ee 007b 0578")
    assert frame[9] == crc8(frame[:9])


def test_frame_clamps_out_of_range_values() -> None:
    frame = encode_kiss_frame(
        KissTelemetry(
            temperature_c=-5, voltage_v=1000.0, current_a=-1.0, consumption_mah=1e9, erpm=1e9
        )
    )
    assert frame[:9] == bytes.fromhex("00 ffff 0000 ffff ffff")
