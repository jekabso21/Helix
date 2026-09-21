# Payload layouts from src/main/msp/msp.c at Betaflight tag 2026.6.2, little-endian

import struct
from dataclasses import dataclass

# Bit order of armingDisableFlags_e in src/main/fc/runtime_config.c at tag 2026.6.2
ARMING_DISABLE_FLAG_NAMES: tuple[str, ...] = (
    "NOGYRO",
    "FAILSAFE",
    "RXLOSS",
    "NOT_DISARMED",
    "BOXFAILSAFE",
    "RUNAWAY",
    "CRASH",
    "THROTTLE",
    "ANGLE",
    "BOOTGRACE",
    "NOPREARM",
    "LOAD",
    "CALIB",
    "CLI",
    "CMS",
    "BST",
    "MSP",
    "PARALYZE",
    "GPS",
    "RESCUE_SW",
    "DSHOT_TELEM",
    "REBOOT_REQD",
    "DSHOT_BBANG",
    "NO_ACC_CAL",
    "MOTOR_PROTO",
    "FLIP_SWITCH",
    "ALT_HOLD_SW",
    "POS_HOLD_SW",
    "AUTOPILOT_SW",
    "ARM_SWITCH",
)

_BOX_ARM_BIT = 0  # BOXARM is the first box in the packed flight mode flags


@dataclass(frozen=True)
class MspStatus:
    pid_cycle_time_us: int
    sensors_mask: int
    flight_mode_flags: int
    system_load_percent: int
    arming_disable_flags: int

    @property
    def armed(self) -> bool:
        return bool(self.flight_mode_flags & (1 << _BOX_ARM_BIT))

    @property
    def arming_disable_names(self) -> list[str]:
        names: list[str] = []
        for bit in range(32):
            if self.arming_disable_flags & (1 << bit):
                known = bit < len(ARMING_DISABLE_FLAG_NAMES)
                names.append(ARMING_DISABLE_FLAG_NAMES[bit] if known else f"BIT{bit}")
        return names


def parse_status_ex(payload: bytes) -> MspStatus:
    head = struct.Struct("<HHHIBHBBB")
    cycle_us, _i2c_errors, sensors, mode_flags, _pid_profile, load, _, _, extra_bytes = (
        head.unpack_from(payload, 0)
    )
    offset = head.size + extra_bytes
    _flag_count, arming_disable_flags = struct.unpack_from("<BI", payload, offset)
    return MspStatus(
        pid_cycle_time_us=cycle_us,
        sensors_mask=sensors,
        flight_mode_flags=mode_flags,
        system_load_percent=load,
        arming_disable_flags=arming_disable_flags,
    )


@dataclass(frozen=True)
class MspAttitude:
    roll_deg: float
    pitch_deg: float
    yaw_deg: float


@dataclass(frozen=True)
class MspRawImu:
    acc_counts: tuple[int, int, int]
    gyro_dps: tuple[int, int, int]
    mag_counts: tuple[int, int, int]


def parse_attitude(payload: bytes) -> MspAttitude:
    roll_decideg, pitch_decideg, yaw_deg = struct.unpack_from("<3h", payload, 0)
    return MspAttitude(roll_decideg / 10.0, pitch_decideg / 10.0, float(yaw_deg))


def parse_raw_imu(payload: bytes) -> MspRawImu:
    values = struct.unpack_from("<9h", payload, 0)
    return MspRawImu(values[0:3], values[3:6], values[6:9])


def parse_rc(payload: bytes) -> list[int]:
    """Channels in µs in Betaflight's internal order: roll, pitch, yaw, throttle, AUX1, ..."""
    count = len(payload) // 2
    return list(struct.unpack_from(f"<{count}H", payload, 0))


def parse_motor(payload: bytes) -> list[int]:
    """PWM-style motor outputs; unused motors read 0."""
    count = len(payload) // 2
    return list(struct.unpack_from(f"<{count}H", payload, 0))
