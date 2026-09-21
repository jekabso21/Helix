import argparse
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

from simtools.msp import MspClient, MspCommand, MspError, parse_motor, parse_status_ex

REPO_ROOT = Path(__file__).resolve().parents[2]

HOST = "127.0.0.1"
PORT_PWM = 9002  # Betaflight -> sim, servo_packet
PORT_FDM = 9003  # sim -> Betaflight, fdm_packet
PORT_RC = 9004  # sim -> Betaflight, rc_packet
PORT_UART1 = 5761  # CLI
PORT_UART3 = 5763  # MSP for tests and tools, enabled by the baseline CLI config

FDM_STRUCT = struct.Struct("<d3d3d4d3d3dd")  # 144 bytes
RC_STRUCT = struct.Struct("<d16H")  # 40 bytes
SERVO_STRUCT = struct.Struct("<4f")  # 16 bytes

STANDARD_GRAVITY_MPS2 = 9.80665
SEA_LEVEL_PRESSURE_PA = 101325.0
# Virtual GPS: longitude and latitude in degrees, altitude in metres
ORIGIN_LON_LAT_ALT = (24.0, 56.0, 0.0)

FDM_RATE_HZ = 1000
RC_RATE_HZ = 50


def connect_uart(port: int, timeout_s: float, check_msp: bool) -> MspClient:
    """Retry until the UART accepts us; never probe first, SITL frees a UART slot late."""
    deadline = time.monotonic() + timeout_s
    while True:
        try:
            client = MspClient(HOST, port)
            try:
                if check_msp:
                    client.request(MspCommand.API_VERSION)
                return client
            except (MspError, OSError):
                client.close()
                raise
        except (MspError, OSError) as error:
            if time.monotonic() > deadline:
                raise TimeoutError(f"UART on TCP {port} not usable after {timeout_s} s") from error
            time.sleep(0.2)


def start_sitl(binary: Path, workdir: Path, log_name: str) -> subprocess.Popen[bytes]:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        if probe.connect_ex((HOST, PORT_UART1)) == 0:
            raise SystemExit(
                f"error: another SITL already listens on TCP {PORT_UART1}; stop it first"
            )
    log = (workdir / log_name).open("wb")
    # setpriv makes the kernel kill SITL when this script dies, however it ends
    command = ["setpriv", "--pdeathsig", "KILL", str(binary.resolve())]
    return subprocess.Popen(command, cwd=workdir, stdout=log, stderr=subprocess.STDOUT)


def apply_cli_config(cli_script: Path) -> str:
    lines = [
        line.strip()
        for line in cli_script.read_text().splitlines()
        if line.strip() and not line.strip().startswith("#")
    ]
    with connect_uart(PORT_UART1, 5.0, check_msp=False) as uart:
        uart.send_raw(b"#")
        output = uart.read_raw(0.5)
        for line in lines:
            uart.send_raw(line.encode() + b"\r\n")
            output += uart.read_raw(0.2)
        uart.send_raw(b"save\r\n")
        output += uart.read_raw(2.0)
    return output.decode(errors="replace")


class StateSender(threading.Thread):
    def __init__(self) -> None:
        super().__init__(daemon=True)
        self.channels = [1500, 1500, 1000, 1500] + [1000] * 12  # A, E, T, R, AUX1...
        self.gyro = (0.0, 0.0, 0.0)
        self.acc = (0.0, 0.0, -STANDARD_GRAVITY_MPS2)  # specific force at rest, FRD: points up
        self.quat = (1.0, 0.0, 0.0, 0.0)  # w x y z
        self._stop_event = threading.Event()
        self.steps_sent = 0

    def stop(self) -> None:
        self._stop_event.set()
        if self.is_alive():
            self.join()

    def run(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        start = time.perf_counter()
        step = 0
        rc_every = FDM_RATE_HZ // RC_RATE_HZ
        while not self._stop_event.is_set():
            t = step / FDM_RATE_HZ
            fdm = FDM_STRUCT.pack(
                t,
                *self.gyro,
                *self.acc,
                *self.quat,
                0.0, 0.0, 0.0,  # velocity
                *ORIGIN_LON_LAT_ALT,
                SEA_LEVEL_PRESSURE_PA,
            )  # fmt: skip
            sock.sendto(fdm, (HOST, PORT_FDM))
            if step % rc_every == 0:
                sock.sendto(RC_STRUCT.pack(t, *self.channels), (HOST, PORT_RC))
            step += 1
            self.steps_sent = step
            delay = start + step / FDM_RATE_HZ - time.perf_counter()
            if delay > 0:
                time.sleep(delay)


def read_latest_servo_packet(sock: socket.socket) -> tuple[float, ...] | None:
    latest = None
    while True:
        try:
            data = sock.recv(256)
        except BlockingIOError:
            return latest
        if len(data) == SERVO_STRUCT.size:
            latest = SERVO_STRUCT.unpack(data)


def main() -> int:
    parser = argparse.ArgumentParser(description="Arm Betaflight SITL and read its motor outputs")
    parser.add_argument(
        "--binary", type=Path, default=REPO_ROOT / "build/betaflight/betaflight_SITL.elf"
    )
    parser.add_argument(
        "--cli-script", type=Path, default=REPO_ROOT / "configs/betaflight/baseline_cli.txt"
    )
    parser.add_argument("--throttle-us", type=int, default=1300)
    parser.add_argument("--keep", action="store_true", help="keep the scratch directory")
    args = parser.parse_args()

    if not args.binary.exists():
        print(f"error: {args.binary} not found. Run ./scripts/bf_build.sh", file=sys.stderr)
        return 2

    workdir = Path(tempfile.mkdtemp(prefix="fpvsim_spin_motors_"))
    print(f"scratch directory: {workdir}")

    print("1. configuring a fresh eeprom.bin through the CLI")
    sitl = start_sitl(args.binary, workdir, "sitl_configure.log")
    try:
        cli_output = apply_cli_config(args.cli_script)
        exit_code = sitl.wait(timeout=5.0)
        print(f"   SITL exited after save with code {exit_code}")
    finally:
        if sitl.poll() is None:
            sitl.kill()
    if "###ERROR" in cli_output or "Invalid" in cli_output:
        print(cli_output)
        print("error: the CLI rejected a command", file=sys.stderr)
        return 1

    print("2. restarting SITL and streaming FDM + RC")
    servo_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    servo_sock.bind((HOST, PORT_PWM))
    servo_sock.setblocking(False)
    sitl = start_sitl(args.binary, workdir, "sitl_run.log")
    sender = StateSender()
    success = False
    try:
        with connect_uart(PORT_UART3, 5.0, check_msp=True) as msp:
            # Stream only after SITL answers MSP: a packet arriving during its boot crashes it
            sender.start()
            print("3. waiting for arming disable flags to clear (arm switch low)")
            deadline = time.monotonic() + 20.0
            while time.monotonic() < deadline:
                status = parse_status_ex(msp.request(MspCommand.STATUS_EX))
                blockers = [n for n in status.arming_disable_names if n != "ARM_SWITCH"]
                print(f"   t={time.monotonic() - deadline + 20.0:5.1f}s flags={blockers}")
                if not blockers:
                    break
                time.sleep(1.0)

            print("4. arm switch high, then throttle up")
            sender.channels[4] = 2000
            time.sleep(1.0)
            sender.channels[2] = args.throttle_us
            for _ in range(5):
                time.sleep(0.5)
                status = parse_status_ex(msp.request(MspCommand.STATUS_EX))
                motors_msp = parse_motor(msp.request(MspCommand.MOTOR))[:4]
                servo = read_latest_servo_packet(servo_sock)
                servo_text = "none" if servo is None else " ".join(f"{v:.3f}" for v in servo)
                print(
                    f"   armed={status.armed} flags={status.arming_disable_names} "
                    f"udp_motor_speed=[{servo_text}] msp_motor={motors_msp}"
                )
                if status.armed and servo is not None and all(v > 0.0 for v in servo):
                    success = True

            sender.channels[2] = 1000
            sender.channels[4] = 1000
            time.sleep(0.5)
    except (MspError, TimeoutError, OSError) as error:
        print(f"error: {error}", file=sys.stderr)
    finally:
        sender.stop()
        sitl.terminate()
        try:
            sitl.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            sitl.kill()
        servo_sock.close()

    print("result:", "armed, motors spinning" if success else "FAILED (see logs in scratch dir)")
    if not args.keep and success:
        for path in workdir.iterdir():
            path.unlink()
        workdir.rmdir()
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
