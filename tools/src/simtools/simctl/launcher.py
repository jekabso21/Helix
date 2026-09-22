import os
import signal
import subprocess
import time
from dataclasses import dataclass, field
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
class ProcessInfo:
    name: str
    pid: int | None
    state: str  # starting, running, exited, failed
    exit_code: int | None
    started_at: float
    log: Path


@dataclass
class ProcessSet:
    """Betaflight SITL and simcore for one run; both die with the parent (setpriv pdeathsig)."""

    run_dir: Path
    sitl: subprocess.Popen[bytes]
    core: subprocess.Popen[bytes]
    started_at: float = field(default_factory=time.monotonic)

    def poll(self) -> list[ProcessInfo]:
        infos: list[ProcessInfo] = []
        for name, process, log in (
            ("betaflight", self.sitl, self.run_dir / "logs/betaflight_run.log"),
            ("simcore", self.core, self.run_dir / "logs/simcore.log"),
        ):
            code = process.poll()
            if code is None:
                state = "running"
            elif code == 0:
                state = "exited"
            else:
                state = "failed"
            infos.append(ProcessInfo(name, process.pid, state, code, self.started_at, log))
        return infos

    def alive(self) -> bool:
        return self.core.poll() is None and self.sitl.poll() is None

    def stop(self) -> int:
        """Stops simcore then SITL; returns simcore's exit code."""
        _terminate(self.core)
        _terminate(self.sitl)
        self._finish_logs()
        return self.core.returncode if self.core.returncode is not None else 1

    def _finish_logs(self) -> None:
        bf_dir = self.run_dir / "betaflight"
        for name in ("sitl_configure.log", "sitl_run.log"):
            source = bf_dir / name
            if source.exists():
                source.replace(self.run_dir / "logs" / name.replace("sitl_", "betaflight_"))


def start_processes(resolved: ResolvedSession, run_dir: Path, simcore: Path) -> ProcessSet:
    """Configure and start SITL, wait until it answers MSP, then start simcore."""
    if not resolved.betaflight_binary.exists():
        raise LaunchError(f"Betaflight SITL binary not found: {resolved.betaflight_binary}")
    bf_dir = run_dir / "betaflight"
    try:
        sitl = configure_and_start(resolved.betaflight_binary, bf_dir, bf_dir / "cli.txt")
    except SitlPortBusyError as error:
        raise LaunchError(str(error)) from error
    session_json = run_dir / "resolved/session.json"
    with (run_dir / "logs/simcore.log").open("wb") as simcore_log:
        core = subprocess.Popen(
            ["setpriv", "--pdeathsig", "KILL", str(simcore), "--session", str(session_json)],
            stdout=simcore_log,
            stderr=subprocess.STDOUT,
        )
    return ProcessSet(run_dir=run_dir, sitl=sitl, core=core)


@dataclass
class RunResult:
    exit_code: int
    run_dir: Path


def run_headless(resolved: ResolvedSession, run_dir: Path, simcore: Path) -> RunResult:
    """Run to completion in the foreground; Ctrl+C stops everything."""
    processes = start_processes(resolved, run_dir, simcore)
    exit_code = 1
    try:
        while True:
            code = processes.core.poll()
            if code is not None:
                exit_code = code
                break
            if processes.sitl.poll() is not None:
                raise LaunchError(
                    f"Betaflight SITL exited with code {processes.sitl.returncode}, "
                    f"see {run_dir}/betaflight/sitl_run.log"
                )
            time.sleep(0.1)
    except KeyboardInterrupt:
        exit_code = 130
    finally:
        processes.stop()
    return RunResult(exit_code=exit_code, run_dir=run_dir)


def _raise_interrupt(_signum: int, _frame: object) -> None:
    raise KeyboardInterrupt


def install_signal_handlers() -> None:
    signal.signal(signal.SIGTERM, _raise_interrupt)
