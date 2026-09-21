import argparse
import socket
import sys
import tempfile
import time
from pathlib import Path

from spin_motors import (
    HOST,
    PORT_UART3,
    REPO_ROOT,
    StateSender,
    apply_cli_config,
    connect_uart,
    start_sitl,
)

from simtools.esc import KissTelemetry, encode_kiss_frame
from simtools.msp import MspCommand, parse_analog, parse_motor_telemetry

PORT_ESC_REQUEST = 9005  # Betaflight -> virtual ESC, one byte: motor index
PORT_UART4 = 5764  # ESC sensor UART
POLE_PAIRS = 7

ESC_CLI_LINES = """
feature ESC_SENSOR
serial 3 1024 115200 57600 0 115200
set battery_meter = ESC
set current_meter = ESC
"""


def motor_telemetry(motor_index: int) -> KissTelemetry:
    mechanical_rpm = 20_000 + 1_000 * motor_index
    return KissTelemetry(
        temperature_c=40 + motor_index,
        voltage_v=23.5,
        current_a=7.5 + 0.5 * motor_index,
        consumption_mah=100 + 10 * motor_index,
        erpm=mechanical_rpm * POLE_PAIRS,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Feed Betaflight SITL from a virtual ESC")
    parser.add_argument(
        "--binary", type=Path, default=REPO_ROOT / "build/betaflight/betaflight_SITL.elf"
    )
    parser.add_argument(
        "--cli-script", type=Path, default=REPO_ROOT / "configs/betaflight/baseline_cli.txt"
    )
    parser.add_argument("--duration", type=float, default=9.0)
    args = parser.parse_args()

    workdir = Path(tempfile.mkdtemp(prefix="fpvsim_virtual_esc_"))
    cli_script = workdir / "cli.txt"
    cli_script.write_text(args.cli_script.read_text() + ESC_CLI_LINES)

    requests = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    requests.bind((HOST, PORT_ESC_REQUEST))
    requests.settimeout(0.05)

    sitl = start_sitl(args.binary, workdir, "sitl_configure.log")
    print(apply_cli_config(cli_script).strip().splitlines()[-1])
    sitl.wait(timeout=5.0)
    sitl = start_sitl(args.binary, workdir, "sitl_run.log")
    sender = StateSender()
    try:
        with connect_uart(PORT_UART3, 5.0, check_msp=True) as msp:
            esc_uart = socket.create_connection((HOST, PORT_UART4), timeout=2.0)
            sender.start()
            counts = [0, 0, 0, 0]
            first_request_s: float | None = None
            start = time.monotonic()
            while time.monotonic() - start < args.duration:
                try:
                    datagram = requests.recv(16)
                except TimeoutError:
                    continue
                motor_index = datagram[0]
                if first_request_s is None:
                    first_request_s = time.monotonic() - start
                if motor_index < len(counts):
                    counts[motor_index] += 1
                    esc_uart.sendall(encode_kiss_frame(motor_telemetry(motor_index)))

            elapsed = time.monotonic() - start - (first_request_s or 0.0)
            print(f"first request after {first_request_s:.1f} s; requests per motor: {counts}")
            print(f"request rate: {sum(counts) / elapsed:.0f} Hz total")
            analog = parse_analog(msp.request(MspCommand.ANALOG))
            print(f"MSP_ANALOG: {analog}")
            for index, motor in enumerate(
                parse_motor_telemetry(msp.request(MspCommand.MOTOR_TELEMETRY))
            ):
                print(f"motor {index + 1}: {motor}   sent: {motor_telemetry(index)}")
            esc_uart.close()
    finally:
        sender.stop()
        sitl.terminate()
        sitl.wait(timeout=3.0)
        requests.close()
        for path in workdir.iterdir():
            path.unlink()
        workdir.rmdir()
    return 0


if __name__ == "__main__":
    sys.exit(main())
