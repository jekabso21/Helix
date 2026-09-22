import csv
import math
import time
from pathlib import Path

import pytest

from simtools.config import resolve_session, write_run_directory
from simtools.simctl.launcher import find_simcore, run_headless
from simtools.sitl import sitl_port_busy

REPO_ROOT = Path(__file__).resolve().parents[2]
SITL_BINARY = REPO_ROOT / "build/betaflight/betaflight_SITL.elf"
TARGET_HEIGHT_M = 5.0
DURATION_S = 36.0
WINDOW_START_S = 16.0

pytestmark = pytest.mark.integration


def tilt_deg(qw: float, qx: float, qy: float, qz: float) -> float:
    # angle between body down and NED down: cos = R[2][2]
    cos_tilt = 1.0 - 2.0 * (qx * qx + qy * qy)
    return math.degrees(math.acos(max(-1.0, min(1.0, cos_tilt))))


@pytest.mark.skipif(not SITL_BINARY.exists(), reason="Betaflight SITL not built")
@pytest.mark.skipif(
    not any(
        (REPO_ROOT / "build" / p / "simcore/simcore").exists()
        for p in ("release", "ci", "clang", "dev")
    ),
    reason="simcore not built",
)
def test_altitude_hold_keeps_height_within_half_a_metre(tmp_path: Path) -> None:
    if sitl_port_busy():
        pytest.skip("a Betaflight SITL is already running on TCP 5761")
    resolved = resolve_session(REPO_ROOT / "configs/sessions/ci_hover.yaml", REPO_ROOT)
    assert resolved.session["duration_s"] == DURATION_S
    run_dir = write_run_directory(resolved, tmp_path, REPO_ROOT, ["pytest"])
    started = time.monotonic()
    result = run_headless(resolved, run_dir, find_simcore(REPO_ROOT))
    elapsed = time.monotonic() - started
    assert result.exit_code == 0, (run_dir / "logs/simcore.log").read_text()
    assert abs(elapsed - DURATION_S) < 8.0, f"realtime pacing off: {elapsed:.1f} s"

    with (run_dir / "data/truth.csv").open() as f:
        rows = [r for r in csv.DictReader(f) if float(r["t_s"]) >= WINDOW_START_S]
    assert len(rows) > 0
    heights = [-float(r["down_m"]) for r in rows]
    tilts = [tilt_deg(*(float(r[k]) for k in ("qw", "qx", "qy", "qz"))) for r in rows]
    assert all(r["crashed"] == "0" for r in rows)
    assert max(abs(h - TARGET_HEIGHT_M) for h in heights) <= 0.5, (min(heights), max(heights))
    assert max(tilts) <= 5.0, max(tilts)
