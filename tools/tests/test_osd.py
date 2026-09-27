import pytest

from simtools.msp.codec import MspDecoder, _checksum
from simtools.osd import (
    DP_CLEAR_SCREEN,
    DP_DRAW_SCREEN,
    DP_HEARTBEAT,
    DP_WRITE_STRING,
    MSP_DISPLAYPORT,
    OsdGrid,
    osd_pos,
)


def displayport_frame(payload: bytes) -> bytes:
    """What Betaflight pushes: an MSP v1 frame in the reply direction with command 182."""
    return (
        b"$M>"
        + bytes([len(payload), MSP_DISPLAYPORT])
        + payload
        + bytes([_checksum(len(payload), MSP_DISPLAYPORT, payload)])
    )


def test_osd_pos_matches_the_betaflight_macro() -> None:
    # OSD_POS(x, y) = (x & 31) | ((y & 31) << 5) | ((x >> 5) << 10), profile 1 flag is bit 11
    assert osd_pos(12, 1) == 12 | (1 << 5) | 0x0800
    assert osd_pos(40, 3) == 8 | (3 << 5) | (1 << 10) | 0x0800
    assert osd_pos(0, 0, profile=2) == 0x1000
    with pytest.raises(ValueError):
        osd_pos(64, 0)


def test_write_clear_and_draw_drive_the_grid() -> None:
    grid = OsdGrid(cols=30, rows=16)
    assert not grid.apply(bytes([DP_HEARTBEAT]))
    assert not grid.apply(bytes([DP_WRITE_STRING, 2, 5, 0]) + b"16.8V")
    assert not grid.apply(bytes([DP_WRITE_STRING, 15, 28, 0x40]) + b"ABCDEF")  # clipped at the edge
    assert grid.apply(bytes([DP_DRAW_SCREEN]))
    assert grid.draws == 1 and grid.frames == 4
    assert grid.text_rows()[2] == "     16.8V"
    assert grid.text_rows()[15] == " " * 28 + "AB"
    assert grid.attrs[15][29] == 0x40 and grid.attrs[2][5] == 0
    assert not grid.apply(bytes([DP_WRITE_STRING, 40, 0, 0]) + b"off canvas")
    grid.apply(bytes([DP_CLEAR_SCREEN]))
    assert all(row == "" for row in grid.text_rows())


def test_frames_decode_through_the_msp_decoder() -> None:
    decoder = MspDecoder()
    stream = displayport_frame(bytes([DP_CLEAR_SCREEN])) + displayport_frame(
        bytes([DP_WRITE_STRING, 0, 1, 0]) + bytes([0x90]) + b"25.1"
    )
    grid = OsdGrid()
    for frame in decoder.feed(stream[:7]) + decoder.feed(stream[7:]):
        assert frame.command == MSP_DISPLAYPORT
        grid.apply(frame.payload)
    assert grid.codes[0][1] == 0x90  # SYM_BATT_FULL survives as a font code
    assert grid.text_rows()[0] == " ?25.1"
    assert grid.to_json()["cols"] == 53
