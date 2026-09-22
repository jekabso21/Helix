import socket
import subprocess
import time
from pathlib import Path

from simtools.msp import MspClient, MspCommand, MspError

HOST = "127.0.0.1"
PORT_PWM = 9002
PORT_FDM = 9003
PORT_RC = 9004
PORT_UART1 = 5761
PORT_UART3 = 5763


def start_sitl(binary: Path, workdir: Path, log_name: str) -> subprocess.Popen[bytes]:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        if probe.connect_ex((HOST, PORT_UART1)) == 0:
            raise SystemExit(
                f"error: another SITL already listens on TCP {PORT_UART1}; stop it first"
            )
    log = (workdir / log_name).open("wb")
    # setpriv makes the kernel kill SITL when the parent dies, however it ends
    command = ["setpriv", "--pdeathsig", "KILL", str(binary.resolve())]
    return subprocess.Popen(command, cwd=workdir, stdout=log, stderr=subprocess.STDOUT)


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


def configure_and_start(binary: Path, workdir: Path, cli_script: Path) -> subprocess.Popen[bytes]:
    """Fresh eeprom: apply the CLI script (SITL exits on save), start again, wait for MSP."""
    sitl = start_sitl(binary, workdir, "sitl_configure.log")
    try:
        output = apply_cli_config(cli_script)
        sitl.wait(timeout=5.0)
    finally:
        if sitl.poll() is None:
            sitl.kill()
    if "###ERROR" in output or "Invalid" in output:
        raise RuntimeError("the Betaflight CLI rejected a command:\n" + output)
    sitl = start_sitl(binary, workdir, "sitl_run.log")
    connect_uart(PORT_UART3, 5.0, check_msp=True).close()
    return sitl
