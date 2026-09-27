import struct

import pytest

from simtools.latency import (
    BURN_IN_CELL_PX,
    BURN_IN_CELLS,
    BURN_IN_MODULUS,
    LatencyReport,
    TimestampReport,
    burn_in_pattern,
    consumer_command,
    decode_burn_in,
    format_report,
    format_timestamp_report,
)
from simtools.proto.frame_ring import (
    HEADER_SIZE,
    MAGIC,
    SLOT_HEADER_SIZE,
    FrameRingError,
    FrameRingReader,
    ring_path,
)

WIDTH = BURN_IN_CELLS * BURN_IN_CELL_PX + 32
HEIGHT = BURN_IN_CELL_PX * 2


def grey_frame(width: int = WIDTH, height: int = HEIGHT) -> bytearray:
    return bytearray(b"\x40" * (width * height))


def stamp(frame: bytearray, frame_index: int, width: int = WIDTH) -> bytearray:
    """The painter the publisher runs in C++, written out again so the decoder can be tested."""
    pattern = burn_in_pattern(frame_index)
    for cell, bit in enumerate(pattern):
        value = 0xFF if bit == "1" else 0x00
        for row in range(BURN_IN_CELL_PX):
            start = row * width + cell * BURN_IN_CELL_PX
            frame[start : start + BURN_IN_CELL_PX] = bytes([value]) * BURN_IN_CELL_PX
    return frame


def test_pattern_is_the_marker_then_sixteen_bits():
    assert burn_in_pattern(0) == "10" + "0" * 16
    assert burn_in_pattern(1) == "10" + "0" * 15 + "1"
    assert burn_in_pattern(12345) == "10" + "0011000000111001"
    assert burn_in_pattern(BURN_IN_MODULUS + 7) == burn_in_pattern(7)


@pytest.mark.parametrize("index", [0, 1, 2, 12345, BURN_IN_MODULUS - 1])
def test_decode_round_trips(index):
    assert decode_burn_in(bytes(stamp(grey_frame(), index)), WIDTH, HEIGHT) == index


def test_decode_rejects_a_frame_without_the_marker():
    assert decode_burn_in(bytes(grey_frame()), WIDTH, HEIGHT) is None


def test_decode_rejects_a_frame_too_small_for_the_counter():
    narrow = BURN_IN_CELLS * BURN_IN_CELL_PX - 16
    frame = bytes(stamp(grey_frame(narrow, HEIGHT), 5, narrow))
    assert decode_burn_in(frame, narrow, HEIGHT) is None


def test_consumer_command_asks_for_grey_frames_at_the_camera_size():
    command = consumer_command("v4l2src device=/dev/video10", 1280, 720)
    assert command[:2] == ["gst-launch-1.0", "-q"]
    joined = " ".join(command)
    assert "v4l2src device=/dev/video10 !" in joined
    assert "format=GRAY8,width=1280,height=720" in joined
    assert joined.endswith("fdsink fd=1 sync=false")


def build_ring(path, frames, width=WIDTH, height=HEIGHT, slots=3):
    """Writes a ring the way the publisher does, so the reader is tested against the real layout."""
    stride = width * 3
    slot_size = SLOT_HEADER_SIZE + stride * height + 8
    slot_size += (-slot_size) % 64
    blob = bytearray(HEADER_SIZE + slot_size * slots)
    struct.pack_into(
        "<IHHIIIIQQd", blob, 0, MAGIC, 1, 2, width, height, stride, slots, slot_size, 0, 60.0
    )
    for seq, (sim_ns, index, fill) in enumerate(frames, start=1):
        offset = HEADER_SIZE + slot_size * ((seq - 1) % slots)
        struct.pack_into("<QqQ3d4d", blob, offset, seq, sim_ns, index, 1.0, 2.0, -3.0, 1.0, 0, 0, 0)
        blob[offset + SLOT_HEADER_SIZE : offset + SLOT_HEADER_SIZE + stride * height] = bytes(
            [fill]
        ) * (stride * height)
        struct.pack_into("<Q", blob, offset + slot_size - 8, seq)
        struct.pack_into("<Q", blob, 32, seq)
    path.write_bytes(blob)
    return path


def test_reader_returns_the_newest_frame_and_then_nothing_new(tmp_path):
    path = build_ring(tmp_path / "ring", [(1000, 1, 0x11), (2000, 2, 0x22)])
    with FrameRingReader(str(path)) as reader:
        assert (reader.width, reader.height, reader.fps_nominal) == (WIDTH, HEIGHT, 60.0)
        frame = reader.read_latest()
        assert frame is not None
        assert (frame.seq, frame.frame_index, frame.sim_time_ns) == (2, 2, 2000)
        assert frame.camera_position_ned == (1.0, 2.0, -3.0)
        assert set(frame.pixels) == {0x22}
        assert reader.read_latest(frame.seq) is None


def test_reader_follows_a_publisher_that_restarted(tmp_path):
    path = build_ring(tmp_path / "ring", [(1000, 1, 0x11)])
    with FrameRingReader(str(path)) as reader:
        frame = reader.read_latest(after_seq=500)
        assert frame is not None and frame.seq == 1


def test_reader_rejects_a_file_that_is_not_a_ring(tmp_path):
    path = tmp_path / "junk"
    path.write_bytes(b"\x00" * 256)
    with pytest.raises(FrameRingError):
        FrameRingReader(str(path))


def test_ring_path_maps_a_camera_name_to_shared_memory():
    assert str(ring_path("main_fpv")) == "/dev/shm/fpvsim.main_fpv"
    assert str(ring_path("/dev/shm/other")) == "/dev/shm/other"


def test_report_percentiles_and_text():
    report = LatencyReport(
        camera="main_fpv",
        consumer="v4l2src",
        width=1280,
        height=720,
        fps_nominal=60.0,
        captured=4,
        undecoded=1,
        matched=3,
        latencies_ms=[10.0, 20.0, 30.0],
        sim_periods_ms=[16.67, 16.67, 23.33],
        skipped_indices=2,
    )
    assert report.p50_ms == 20.0
    assert report.max_ms == 30.0
    assert report.sim_period_error_ms == pytest.approx(6.66, abs=0.01)
    text = format_report(report)
    assert "p50 20.0 ms" in text
    assert "never showed: 2" in text
    assert "worst deviation from 16.67 ms is 6.66 ms" in text


def test_timestamp_report_allows_a_frame_up_to_a_period_behind():
    good = TimestampReport(
        camera="main_fpv", fps_nominal=60.0, behind_ms=[-5.0, 8.0, 15.0], api_window_ms=[1.0, 2.0]
    )
    assert good.frame_period_ms == pytest.approx(16.667, abs=0.01)
    assert good.max_ms == 15.0
    assert good.within_one_frame
    late = TimestampReport(
        camera="main_fpv", fps_nominal=60.0, behind_ms=[60.0], api_window_ms=[1.0]
    )
    assert not late.within_one_frame
    assert "OUTSIDE" in format_timestamp_report(late)
