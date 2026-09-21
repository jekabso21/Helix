import argparse
import math
import sys
import tempfile
import time
from pathlib import Path

from spin_motors import (
    PORT_UART3,
    REPO_ROOT,
    STANDARD_GRAVITY_MPS2,
    StateSender,
    apply_cli_config,
    connect_uart,
    start_sitl,
)

from simtools.msp import (
    MspClient,
    MspCommand,
    parse_attitude,
    parse_motor,
    parse_raw_imu,
    parse_status_ex,
)

AXES = ("roll", "pitch", "yaw")
RAMP_RATE_DEG_S = 20.0
RAMP_ANGLE_DEG = 20.0
HOLD_S = 4.0
PULSE_RATE_DEG_S = 60.0
PULSE_S = 0.4
G = STANDARD_GRAVITY_MPS2

Signs = tuple[float, float, float]


def true_state(axis: str, angle_rad: float, rate_radps: float):
    """Physical FRD gyro and specific force, and q_ned_from_frd, for a rotation about one axis."""
    half = angle_rad / 2.0
    sin_a, cos_a = math.sin(angle_rad), math.cos(angle_rad)
    if axis == "roll":  # right side down
        return (
            (rate_radps, 0.0, 0.0),
            (0.0, -G * sin_a, -G * cos_a),
            (math.cos(half), math.sin(half), 0.0, 0.0),
        )
    if axis == "pitch":  # nose up
        return (
            (0.0, rate_radps, 0.0),
            (G * sin_a, 0.0, -G * cos_a),
            (math.cos(half), 0.0, math.sin(half), 0.0),
        )
    return (
        (0.0, 0.0, rate_radps),
        (0.0, 0.0, -G),
        (math.cos(half), 0.0, 0.0, math.sin(half)),
    )  # nose right


class Rig:
    def __init__(self, sender: StateSender, msp: MspClient, gyro: Signs, acc: Signs, quat: Signs):
        self.sender = sender
        self.msp = msp
        self.gyro_signs = gyro
        self.acc_signs = acc
        self.quat_signs = quat

    def apply(self, axis: str, angle_deg: float, rate_deg_s: float) -> None:
        gyro, acc, quat = true_state(axis, math.radians(angle_deg), math.radians(rate_deg_s))
        self.sender.gyro = tuple(s * v for s, v in zip(self.gyro_signs, gyro, strict=True))
        self.sender.acc = tuple(s * v for s, v in zip(self.acc_signs, acc, strict=True))
        self.sender.quat = (
            quat[0],
            *(s * v for s, v in zip(self.quat_signs, quat[1:], strict=True)),
        )

    def ramp(self, axis: str, start_deg: float, end_deg: float) -> None:
        duration = abs(end_deg - start_deg) / RAMP_RATE_DEG_S
        rate = math.copysign(RAMP_RATE_DEG_S, end_deg - start_deg)
        begin = time.perf_counter()
        while (elapsed := time.perf_counter() - begin) < duration:
            self.apply(axis, start_deg + rate * elapsed, rate)
            time.sleep(0.002)
        self.apply(axis, end_deg, 0.0)

    def attitude(self) -> tuple[float, float, float]:
        a = parse_attitude(self.msp.request(MspCommand.ATTITUDE))
        return a.roll_deg, a.pitch_deg, a.yaw_deg

    def mean_motors(self, duration_s: float) -> list[float]:
        samples: list[list[int]] = []
        end = time.perf_counter() + duration_s
        while time.perf_counter() < end:
            samples.append(parse_motor(self.msp.request(MspCommand.MOTOR))[:4])
            time.sleep(0.01)
        return [sum(column) / len(samples) for column in zip(*samples, strict=True)]


def attitude_trials(rig: Rig) -> None:
    print("\nATTITUDE: true rotation of +20 deg (roll right, pitch nose up, yaw nose right)")
    print("axis   before (r,p,y)   end of ramp   after hold   gyro dps mid-ramp   mag after hold")
    for axis in AXES:
        before = rig.attitude()
        begin = time.perf_counter()
        rig_ramp_mid: tuple[int, int, int] | None = None
        duration = RAMP_ANGLE_DEG / RAMP_RATE_DEG_S
        while (elapsed := time.perf_counter() - begin) < duration:
            rig.apply(axis, RAMP_RATE_DEG_S * elapsed, RAMP_RATE_DEG_S)
            if rig_ramp_mid is None and elapsed > duration / 2:
                rig_ramp_mid = parse_raw_imu(rig.msp.request(MspCommand.RAW_IMU)).gyro_dps
            time.sleep(0.002)
        rig.apply(axis, RAMP_ANGLE_DEG, 0.0)
        ramp_end = rig.attitude()
        time.sleep(HOLD_S)
        held = rig.attitude()
        mag = parse_raw_imu(rig.msp.request(MspCommand.RAW_IMU)).mag_counts
        fmt = lambda v: f"({v[0]:6.1f},{v[1]:6.1f},{v[2]:6.1f})"  # noqa: E731
        print(f"{axis:5}  {fmt(before)}  {fmt(ramp_end)}  {fmt(held)}  {rig_ramp_mid}  mag={mag}")
        rig.ramp(axis, RAMP_ANGLE_DEG, 0.0)
        time.sleep(3.0)


def motor_trials(rig: Rig) -> None:
    print("\nMOTORS: armed, level, true rate disturbance of +60 deg/s; change of MSP_MOTOR 1..4")
    for axis in AXES:
        rig.apply(axis, 0.0, 0.0)
        # Re-arm for every pulse: disarming clears the PID I-term left by the previous pulse
        rig.sender.channels[2] = 1000
        rig.sender.channels[4] = 1000
        time.sleep(1.0)
        rig.sender.channels[4] = 2000
        time.sleep(1.0)
        rig.sender.channels[2] = 1500
        time.sleep(1.0)
        armed = parse_status_ex(rig.msp.request(MspCommand.STATUS_EX)).armed
        baseline = rig.mean_motors(0.3)
        rig.apply(axis, 0.0, PULSE_RATE_DEG_S)
        pulsed = rig.mean_motors(PULSE_S)
        rig.apply(axis, 0.0, 0.0)
        delta = [round(p - b) for p, b in zip(pulsed, baseline, strict=True)]
        rising = [index + 1 for index, value in enumerate(delta) if value > 0]
        base = [round(b) for b in baseline]
        print(f"{axis:5}  armed={armed} baseline={base} delta={delta}  speeding up: {rising}")
    rig.sender.channels[2] = 1000
    rig.sender.channels[4] = 1000
    time.sleep(0.5)


def parse_signs(text: str) -> Signs:
    values = tuple(float(part) for part in text.split(","))
    if len(values) != 3 or any(abs(v) != 1.0 for v in values):
        raise argparse.ArgumentTypeError("expected three comma-separated values of 1 or -1")
    return values  # type: ignore[return-value]


def main() -> int:
    parser = argparse.ArgumentParser(description="Find the axis and sign conventions of SITL")
    parser.add_argument("--gyro-signs", type=parse_signs, default=(1.0, 1.0, 1.0))
    parser.add_argument("--acc-signs", type=parse_signs, default=(-1.0, 1.0, 1.0))
    parser.add_argument("--quat-signs", type=parse_signs, default=(1.0, -1.0, -1.0))
    parser.add_argument("--skip-motors", action="store_true")
    parser.add_argument(
        "--binary", type=Path, default=REPO_ROOT / "build/betaflight/betaflight_SITL.elf"
    )
    parser.add_argument(
        "--cli-script", type=Path, default=REPO_ROOT / "configs/betaflight/baseline_cli.txt"
    )
    args = parser.parse_args()
    print(f"signs: gyro={args.gyro_signs} acc={args.acc_signs} quat={args.quat_signs}")

    workdir = Path(tempfile.mkdtemp(prefix="fpvsim_conventions_"))
    sitl = start_sitl(args.binary, workdir, "sitl_configure.log")
    apply_cli_config(args.cli_script)
    sitl.wait(timeout=5.0)
    sitl = start_sitl(args.binary, workdir, "sitl_run.log")
    sender = StateSender()
    try:
        with connect_uart(PORT_UART3, 5.0, check_msp=True) as msp:
            # Stream only after SITL answers MSP: a packet arriving during its boot crashes it
            sender.start()
            rig = Rig(sender, msp, args.gyro_signs, args.acc_signs, args.quat_signs)
            deadline = time.monotonic() + 20.0
            while time.monotonic() < deadline:
                flags = parse_status_ex(msp.request(MspCommand.STATUS_EX)).arming_disable_names
                if flags in ([], ["ARM_SWITCH"]):
                    break
                time.sleep(0.5)
            time.sleep(2.0)
            attitude_trials(rig)
            if not args.skip_motors:
                motor_trials(rig)
    finally:
        sender.stop()
        sitl.terminate()
        sitl.wait(timeout=3.0)
        for path in workdir.iterdir():
            path.unlink()
        workdir.rmdir()
    return 0


if __name__ == "__main__":
    sys.exit(main())
