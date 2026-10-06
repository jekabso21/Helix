import fcntl
from pathlib import Path

from simtools.proto.frame_ring import SHM_PREFIX, remove_stale_rings


def test_only_rings_nobody_holds_are_removed(tmp_path: Path) -> None:
    held = tmp_path / f"{SHM_PREFIX}main_fpv"
    stale = tmp_path / f"{SHM_PREFIX}rear_fpv"
    other = tmp_path / "someone_else"
    for path in (held, stale, other):
        path.write_bytes(b"\0" * 128)

    with held.open("rb") as publisher:
        fcntl.flock(publisher, fcntl.LOCK_SH)
        removed = remove_stale_rings(tmp_path)

    assert removed == [stale]
    assert held.exists()
    assert not stale.exists()
    assert other.exists()


def test_a_missing_directory_is_not_an_error(tmp_path: Path) -> None:
    assert remove_stale_rings(tmp_path / "nothing") == []
