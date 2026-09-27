import re
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
PATCHES = REPO_ROOT / "patches/betaflight"
MIRROR = REPO_ROOT / "simcore/src/bridge/betaflight/protocol_2026_6_2.hpp"


def added_lines(patch: Path) -> str:
    return "\n".join(
        line[1:]
        for line in patch.read_text().splitlines()
        if line.startswith("+") and line != "+++"
    )


def test_every_patch_is_a_unified_diff_against_the_sitl_platform() -> None:
    patches = sorted(PATCHES.glob("*.patch"))
    assert [p.name[:4] for p in patches] == ["0001", "0002", "0003"], patches
    for patch in patches:
        text = patch.read_text()
        assert text.startswith("diff --git "), patch.name
        assert "\n--- a/" in text and "\n+++ b/" in text, patch.name


def test_rpm_packet_in_the_patch_matches_the_simcore_mirror() -> None:
    """The SITL struct is ours, so nothing upstream stops the two sides drifting apart."""
    added = added_lines(PATCHES / "0003-sitl-rpm-filter.patch")
    struct = re.search(r"typedef struct \{(.*?)\} rpm_packet;", added, re.S)
    assert struct is not None, "the patch no longer defines rpm_packet"
    body = struct.group(1)
    assert re.search(r"double\s+timestamp;", body), body
    assert re.search(r"float\s+motor_frequency_hz\[4\];", body), body
    assert body.index("timestamp") < body.index("motor_frequency_hz"), "field order changed"

    mirror = MIRROR.read_text()
    assert (
        "struct RpmPacket {\n  double timestamp;\n  std::array<float, kMotorSpeedCount>" in mirror
    )
    assert "static_assert(sizeof(RpmPacket) == 24);" in mirror
    assert "static_assert(offsetof(RpmPacket, motor_frequency_hz) == 8);" in mirror


@pytest.mark.parametrize(
    "patch, needle",
    [
        ("0002-sitl-osd-displayport.patch", "#define MAX_MSP_PORT_COUNT 5"),
        ("0002-sitl-osd-displayport.patch", "#define USE_OSD"),
        ("0003-sitl-rpm-filter.patch", "#define USE_RPM_FILTER"),
        ("0003-sitl-rpm-filter.patch", "#define PORT_RPM"),
    ],
)
def test_patches_keep_the_features_the_simulator_depends_on(patch: str, needle: str) -> None:
    assert needle in added_lines(PATCHES / patch), f"{patch} no longer adds {needle}"
