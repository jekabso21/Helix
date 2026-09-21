import argparse
import array
import fcntl
import os
import struct
import sys
import tempfile
import time
from pathlib import Path
from typing import Any

import yaml
from spin_motors import (
    PORT_UART3,
    REPO_ROOT,
    StateSender,
    apply_cli_config,
    connect_uart,
    start_sitl,
)

from simtools.msp import MspCommand, parse_motor, parse_rc, parse_status_ex

JSIOCGNAME_128 = 0x80806A13
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
AXIS_FULL_SCALE = 32767.0

# rc_packet order: A, E, T, R, AUX1, ...
CHANNEL_ORDER = ["roll", "pitch", "throttle", "yaw"] + [f"aux{i}" for i in range(1, 13)]


def find_device(name_contains: str) -> tuple[int, str]:
    names: list[str] = []
    for path in sorted(Path("/dev/input").glob("js*")):
        fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        buffer = array.array("B", [0] * 128)
        fcntl.ioctl(fd, JSIOCGNAME_128, buffer)
        name = buffer.tobytes().rstrip(b"\0").decode(errors="replace")
        if name_contains.lower() in name.lower():
            return fd, name
        names.append(name)
        os.close(fd)
    raise LookupError(f"no joystick name contains {name_contains!r}; connected: {names}")


class Joystick:
    def __init__(self, fd: int) -> None:
        self._fd = fd
        self.axes: dict[int, int] = {}
        self.buttons: dict[int, int] = {}

    def poll(self) -> None:
        while True:
            try:
                data = os.read(self._fd, 8 * 64)
            except BlockingIOError:
                return
            for offset in range(0, len(data) - 7, 8):
                _time_ms, value, kind, number = struct.unpack_from("<IhBB", data, offset)
                if kind & JS_EVENT_AXIS:
                    self.axes[number] = value
                elif kind & JS_EVENT_BUTTON:
                    self.buttons[number] = value


def channel_us(source: dict[str, Any], joystick: Joystick) -> int:
    if "axis" in source:
        value = joystick.axes.get(source["axis"], 0) / AXIS_FULL_SCALE
        deadband = float(source.get("deadband", 0.0))
        if abs(value) < deadband:
            value = 0.0
        us = 1500.0 + 500.0 * value
    else:
        us = 2000.0 if joystick.buttons.get(source["button"], 0) else 1000.0
    if source.get("inverted", False):
        us = 3000.0 - us
    return round(min(2000.0, max(1000.0, us)))


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Drive Betaflight SITL from a USB radio through an input mapping"
    )
    parser.add_argument(
        "--mapping", type=Path, default=REPO_ROOT / "configs/input/radiomaster_boxer.yaml"
    )
    parser.add_argument(
        "--binary", type=Path, default=REPO_ROOT / "build/betaflight/betaflight_SITL.elf"
    )
    parser.add_argument(
        "--cli-script", type=Path, default=REPO_ROOT / "configs/betaflight/baseline_cli.txt"
    )
    parser.add_argument("--duration", type=float, default=60.0, help="seconds to run")
    args = parser.parse_args()

    mapping = yaml.safe_load(args.mapping.read_text())
    fd, device_name = find_device(mapping["device"]["name_contains"])
    joystick = Joystick(fd)
    print(f"input device: {device_name}")

    workdir = Path(tempfile.mkdtemp(prefix="fpvsim_radio_"))
    sitl = start_sitl(args.binary, workdir, "sitl_configure.log")
    apply_cli_config(args.cli_script)
    sitl.wait(timeout=5.0)

    sitl = start_sitl(args.binary, workdir, "sitl_run.log")
    sender = StateSender()
    try:
        sender.start()
        with connect_uart(PORT_UART3, 5.0, check_msp=True) as msp:
            print("sent = A E T R AUX1 AUX2 | fc_rc = roll pitch yaw throttle AUX1 AUX2")
            end = time.monotonic() + args.duration
            next_print = 0.0
            while time.monotonic() < end:
                joystick.poll()
                for index, name in enumerate(CHANNEL_ORDER):
                    if name in mapping["channels"]:
                        sender.channels[index] = channel_us(mapping["channels"][name], joystick)
                    else:
                        sender.channels[index] = 1500 if index < 4 else 1000
                if time.monotonic() >= next_print:
                    next_print = time.monotonic() + 0.5
                    status = parse_status_ex(msp.request(MspCommand.STATUS_EX))
                    fc_rc = parse_rc(msp.request(MspCommand.RC))[:6]
                    motors = parse_motor(msp.request(MspCommand.MOTOR))[:4]
                    print(
                        f"sent={sender.channels[:6]} fc_rc={fc_rc} armed={status.armed} "
                        f"flags={status.arming_disable_names} motors={motors}",
                        flush=True,
                    )
                time.sleep(0.004)
    finally:
        sender.stop()
        sitl.terminate()
        sitl.wait(timeout=3.0)
        os.close(fd)
        for path in workdir.iterdir():
            path.unlink()
        workdir.rmdir()
    return 0


if __name__ == "__main__":
    sys.exit(main())
