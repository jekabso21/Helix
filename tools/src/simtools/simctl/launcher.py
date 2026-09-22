import os
import signal
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path

from simtools.config.resolver import ResolvedSession
from simtools.sitl import SitlPortBusyError, configure_and_start

SIMCORE_PRESETS = ("release", "ci", "clang", "dev")


class LaunchError(Exception):
    pass


def find_simcore(base_dir: Path) -> Path:
    override = os.environ.get("FPVSIM_SIMCORE")
    if override:
        return Path(override)
    for preset in SIMCORE_PRESETS:
        candidate = base_dir / "build" / preset / "simcore" / "simcore"
        if candidate.exists():
            return candidate
    raise LaunchError("simcore binary not found; build it with cmake --preset release")


def _terminate(process: subprocess.Popen[bytes], timeout_s: float = 5.0) -> None:
    if process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()


@dataclass
class RunResult:
    exit_code: int
    run_dir: Path


def run_headless(resolved: ResolvedSession, run_dir: Path, simcore: Path) -> RunResult:
    """Start Betaflight SITL, wait until it answers MSP, run simcore to completion, stop both."""
    if not resolved.betaflight_binary.exists():
        raise LaunchError(f"Betaflight SITL binary not found: {resolved.betaflight_binary}")
    bf_dir = run_dir / "betaflight"
    try:
        sitl = configure_and_start(resolved.betaflight_binary, bf_dir, bf_dir / "cli.txt")
    except SitlPortBusyError as error:
        raise LaunchError(str(error)) from error
    session_json = run_dir / "resolved/session.json"
    core: subprocess.Popen[bytes] | None = None
    exit_code = 1
    with (run_dir / "logs/simcore.log").open("wb") as simcore_log:
        try:
            core = subprocess.Popen(
                ["setpriv", "--pdeathsig", "KILL", str(simcore), "--session", str(session_json)],
                stdout=simcore_log,
                stderr=subprocess.STDOUT,
            )
            while True:
                code = core.poll()
                if code is not None:
                    exit_code = code
                    break
                if sitl.poll() is not None:
                    raise LaunchError(
                        f"Betaflight SITL exited with code {sitl.returncode}, "
                        f"see {bf_dir}/sitl_run.log"
                    )
                time.sleep(0.1)
        except KeyboardInterrupt:
            exit_code = 130
        finally:
            if core is not None:
                _terminate(core)
            _terminate(sitl)
            for name in ("sitl_configure.log", "sitl_run.log"):
                source = bf_dir / name
                if source.exists():
                    source.replace(run_dir / "logs" / name.replace("sitl_", "betaflight_"))
    return RunResult(exit_code=exit_code, run_dir=run_dir)


def _raise_interrupt(_signum: int, _frame: object) -> None:
    raise KeyboardInterrupt


def install_signal_handlers() -> None:
    signal.signal(signal.SIGTERM, _raise_interrupt)
