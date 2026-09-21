import argparse
import math
import socket
import struct
import sys
import tempfile
import time
from pathlib import Path

from spin_motors import (
    FDM_RATE_HZ,
    FDM_STRUCT,
    HOST,
    ORIGIN_LON_LAT_ALT,
    PORT_FDM,
    PORT_PWM,
    PORT_RC,
    PORT_UART3,
    RC_STRUCT,
    REPO_ROOT,
    SEA_LEVEL_PRESSURE_PA,
    SERVO_STRUCT,
    STANDARD_GRAVITY_MPS2,
    StateSender,
    apply_cli_config,
    connect_uart,
    start_sitl,
)

from simtools.msp import MspCommand, parse_status_ex

RC_EVERY_STEPS = 20
RESPONSE_TIMEOUT_S = 0.05


def scripted_gyro(step: int) -> tuple[float, float, float]:
    t = step / FDM_RATE_HZ
    return (
        0.6 * math.sin(2.0 * math.pi * 5.0 * t),
        0.4 * math.sin(2.0 * math.pi * 3.0 * t + 1.0),
        0.3 * math.sin(2.0 * math.pi * 2.0 * t + 2.0),
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Measure SITL lockstep throughput and repeatability"
    )
    parser.add_argument(
        "--binary", type=Path, default=REPO_ROOT / "build/betaflight/betaflight_SITL.elf"
    )
    parser.add_argument(
        "--cli-script", type=Path, default=REPO_ROOT / "configs/betaflight/baseline_cli.txt"
    )
    parser.add_argument("--steps", type=int, default=5000)
    parser.add_argument("--step-hz", type=float, default=0.0, help="0 = as fast as possible")
    parser.add_argument("--out", type=Path, required=True, help="file for the motor sequence")
    args = parser.parse_args()

    workdir = Path(tempfile.mkdtemp(prefix="fpvsim_lockstep_"))
    sitl = start_sitl(args.binary, workdir, "sitl_configure.log")
    apply_cli_config(args.cli_script)
    sitl.wait(timeout=5.0)

    servo_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    servo_sock.bind((HOST, PORT_PWM))
    sitl = start_sitl(args.binary, workdir, "sitl_run.log")
    sender = StateSender()
    try:
        with connect_uart(PORT_UART3, 5.0, check_msp=True) as msp:
            sender.start()
            deadline = time.monotonic() + 20.0
            while time.monotonic() < deadline:
                flags = parse_status_ex(msp.request(MspCommand.STATUS_EX)).arming_disable_names
                if flags in ([], ["ARM_SWITCH"]):
                    break
                time.sleep(0.5)
            sender.channels[4] = 2000
            time.sleep(1.0)
            sender.channels[2] = 1500
            time.sleep(1.0)
            status = parse_status_ex(msp.request(MspCommand.STATUS_EX))
            print(f"armed={status.armed} pid_cycle_time_us={status.pid_cycle_time_us}")
            channels = list(sender.channels)
            sender.stop()
            first_step = sender.steps_sent

            out_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            servo_sock.setblocking(False)
            try:
                while True:
                    servo_sock.recv(256)
            except BlockingIOError:
                pass
            servo_sock.settimeout(RESPONSE_TIMEOUT_S)

            timeouts = 0
            extra_packets = 0
            records = bytearray()
            begin = time.perf_counter()
            for index in range(args.steps):
                step = first_step + index
                t = step / FDM_RATE_HZ
                if index % RC_EVERY_STEPS == 0:
                    out_sock.sendto(RC_STRUCT.pack(t, *channels), (HOST, PORT_RC))
                fdm = FDM_STRUCT.pack(
                    t,
                    *scripted_gyro(index),
                    0.0, 0.0, -STANDARD_GRAVITY_MPS2,
                    1.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    *ORIGIN_LON_LAT_ALT,
                    SEA_LEVEL_PRESSURE_PA,
                )  # fmt: skip
                out_sock.sendto(fdm, (HOST, PORT_FDM))
                try:
                    data = servo_sock.recv(256)
                except TimeoutError:
                    timeouts += 1
                    data = struct.pack("<4f", *([float("nan")] * 4))
                if len(data) != SERVO_STRUCT.size:
                    extra_packets += 1
                    continue
                records += data
                if args.step_hz > 0:
                    wait = begin + (index + 1) / args.step_hz - time.perf_counter()
                    if wait > 0:
                        time.sleep(wait)
            elapsed = time.perf_counter() - begin
            args.out.write_bytes(bytes(records))
            rate = args.steps / elapsed
            print(
                f"steps={args.steps} elapsed={elapsed:.2f}s rate={rate:.0f} steps/s "
                f"({rate / FDM_RATE_HZ:.1f}x realtime at {FDM_RATE_HZ} Hz) "
                f"timeouts={timeouts} other_packets={extra_packets}"
            )
    finally:
        sender.stop()
        sitl.terminate()
        sitl.wait(timeout=3.0)
        servo_sock.close()
        for path in workdir.iterdir():
            path.unlink()
        workdir.rmdir()
    return 0


if __name__ == "__main__":
    sys.exit(main())
