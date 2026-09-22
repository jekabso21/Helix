import struct
from dataclasses import dataclass

MAGIC = 0x53565046
PROTOCOL_VERSION = 1
MSG_TYPE_RENDER_STATE = 1
HEADER_STRUCT = struct.Struct("<IHHII")
PAYLOAD_STRUCT = struct.Struct("<q3d4d3d3d4B8f3f2f4Bf3fI")
MESSAGE_SIZE = HEADER_STRUCT.size + PAYLOAD_STRUCT.size


@dataclass(frozen=True)
class RenderState:
    seq: int
    sim_time_ns: int
    position_ned: tuple[float, float, float]
    q_ned_from_frd: tuple[float, float, float, float]
    velocity_ned: tuple[float, float, float]
    angular_rate_frd: tuple[float, float, float]
    armed: bool
    crashed: bool
    motor_count: int
    motor_rpm: tuple[float, ...]
    sun_dir_ned: tuple[float, float, float]
    sun_intensity: float
    fog_density: float
    precip_type: int
    precip_intensity: float
    wind_ned: tuple[float, float, float]
    video_fault_flags: int


def encode_render_state(s: RenderState) -> bytes:
    header = HEADER_STRUCT.pack(
        MAGIC, PROTOCOL_VERSION, MSG_TYPE_RENDER_STATE, PAYLOAD_STRUCT.size, s.seq
    )
    payload = PAYLOAD_STRUCT.pack(
        s.sim_time_ns,
        *s.position_ned,
        *s.q_ned_from_frd,
        *s.velocity_ned,
        *s.angular_rate_frd,
        int(s.armed),
        int(s.crashed),
        s.motor_count,
        0,
        *s.motor_rpm,
        *s.sun_dir_ned,
        s.sun_intensity,
        s.fog_density,
        s.precip_type,
        0,
        0,
        0,
        s.precip_intensity,
        *s.wind_ned,
        s.video_fault_flags,
    )
    return header + payload


def decode_render_state(data: bytes) -> RenderState | None:
    if len(data) != MESSAGE_SIZE:
        return None
    magic, version, msg_type, payload_size, seq = HEADER_STRUCT.unpack_from(data, 0)
    if (magic, version, msg_type, payload_size) != (
        MAGIC,
        PROTOCOL_VERSION,
        MSG_TYPE_RENDER_STATE,
        PAYLOAD_STRUCT.size,
    ):
        return None
    v = PAYLOAD_STRUCT.unpack_from(data, HEADER_STRUCT.size)
    return RenderState(
        seq=seq,
        sim_time_ns=v[0],
        position_ned=(v[1], v[2], v[3]),
        q_ned_from_frd=(v[4], v[5], v[6], v[7]),
        velocity_ned=(v[8], v[9], v[10]),
        angular_rate_frd=(v[11], v[12], v[13]),
        armed=bool(v[14]),
        crashed=bool(v[15]),
        motor_count=v[16],
        motor_rpm=tuple(v[18:26]),
        sun_dir_ned=(v[26], v[27], v[28]),
        sun_intensity=v[29],
        fog_density=v[30],
        precip_type=v[31],
        precip_intensity=v[35],
        wind_ned=(v[36], v[37], v[38]),
        video_fault_flags=v[39],
    )
