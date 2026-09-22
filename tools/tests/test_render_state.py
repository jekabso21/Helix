from pathlib import Path

from simtools.proto import MESSAGE_SIZE, RenderState, decode_render_state, encode_render_state

GOLDEN = Path(__file__).resolve().parents[2] / "tests/golden/proto/render_state_v1.bin"


def sample() -> RenderState:
    # Keep in sync with common/tests/render_state_test.cpp
    return RenderState(
        seq=7,
        sim_time_ns=1_234_567_890,
        position_ned=(1.5, -2.25, -10.0),
        q_ned_from_frd=(0.8, 0.1, -0.2, 0.3),
        velocity_ned=(3.0, 4.0, -0.5),
        angular_rate_frd=(0.1, -0.2, 0.3),
        armed=True,
        crashed=False,
        motor_count=4,
        motor_rpm=(20000.0, 21000.0, 22000.0, 23000.0, 0.0, 0.0, 0.0, 0.0),
        sun_dir_ned=(0.5, 0.5, -0.75),
        sun_intensity=0.875,
        fog_density=0.0,
        precip_type=2,
        precip_intensity=0.25,
        wind_ned=(5.0, -1.0, 0.0),
        video_fault_flags=5,
    )


def test_sample_matches_golden_bytes() -> None:
    encoded = encode_render_state(sample())
    assert len(encoded) == MESSAGE_SIZE == 208
    assert encoded == GOLDEN.read_bytes()


def test_round_trip_and_rejections() -> None:
    encoded = encode_render_state(sample())
    assert decode_render_state(encoded) == sample()
    assert decode_render_state(encoded[:-1]) is None
    assert decode_render_state(b"\x00" * MESSAGE_SIZE) is None
