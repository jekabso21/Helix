import fcntl
import mmap
import struct
from dataclasses import dataclass
from pathlib import Path

MAGIC = 0x46565046
LAYOUT_VERSION = 1
HEADER_SIZE = 128
SLOT_HEADER_SIZE = 128
SLOT_TRAILER_SIZE = 8
HEADER_STRUCT = struct.Struct("<IHHIIIIQQd")
SLOT_META_STRUCT = struct.Struct("<QqQ3d4d")
PIXEL_FORMAT_RGBA8 = 1
PIXEL_FORMAT_RGB8 = 2
BYTES_PER_PIXEL = {PIXEL_FORMAT_RGBA8: 4, PIXEL_FORMAT_RGB8: 3}
SHM_DIR = Path("/dev/shm")
SHM_PREFIX = "fpvsim."


class FrameRingError(Exception):
    pass


@dataclass(frozen=True)
class RingFrame:
    seq: int
    sim_time_ns: int
    frame_index: int
    camera_position_ned: tuple[float, float, float]
    q_ned_from_camera: tuple[float, float, float, float]
    pixels: bytes


def ring_path(camera: str) -> Path:
    """A bare camera name is the shared-memory object the publisher creates for it."""
    if "/" in camera:
        return Path(camera)
    return SHM_DIR / f"{SHM_PREFIX}{camera}"


def remove_stale_rings(shm_dir: Path = SHM_DIR) -> list[Path]:
    """Deletes rings no publisher holds any more, which is what a killed app leaves behind."""
    removed: list[Path] = []
    for path in sorted(shm_dir.glob(f"{SHM_PREFIX}*")):
        try:
            with path.open("rb") as ring:
                # a live publisher holds a shared lock for as long as it runs
                fcntl.flock(ring, fcntl.LOCK_EX | fcntl.LOCK_NB)
                path.unlink()
        except OSError:
            continue
        removed.append(path)
    return removed


class FrameRingReader:
    """Reads the newest complete frame out of a publisher's ring (docs/INTERFACES.md section 4)."""

    def __init__(self, camera: str) -> None:
        self.path = ring_path(camera)
        try:
            self._file = self.path.open("rb")
        except OSError as error:
            raise FrameRingError(f"no frame ring at {self.path}: {error}") from error
        self._map = mmap.mmap(self._file.fileno(), 0, prot=mmap.PROT_READ)
        (
            magic,
            layout_version,
            self.pixel_format,
            self.width,
            self.height,
            self.stride_bytes,
            self.slot_count,
            self.slot_size_bytes,
            _latest,
            self.fps_nominal,
        ) = HEADER_STRUCT.unpack_from(self._map, 0)
        if magic != MAGIC or layout_version != LAYOUT_VERSION:
            self.close()
            raise FrameRingError(
                f"{self.path}: not a frame ring (magic {magic:#x} v{layout_version})"
            )
        if self.pixel_format not in BYTES_PER_PIXEL:
            self.close()
            raise FrameRingError(f"{self.path}: unknown pixel format {self.pixel_format}")

    @property
    def frame_bytes(self) -> int:
        return self.stride_bytes * self.height

    def latest_seq(self) -> int:
        return struct.unpack_from("<Q", self._map, 32)[0]

    def read_latest(self, after_seq: int = 0, retries: int = 4) -> RingFrame | None:
        """The newest frame unless it is the one after_seq names; None when there is nothing new.

        A sequence below after_seq means the publisher restarted and began counting again.
        """
        for _ in range(retries + 1):
            seq = self.latest_seq()
            if seq == 0 or seq == after_seq:
                return None
            offset = HEADER_SIZE + self.slot_size_bytes * ((seq - 1) % self.slot_count)
            values = SLOT_META_STRUCT.unpack_from(self._map, offset)
            pixels = bytes(
                self._map[offset + SLOT_HEADER_SIZE : offset + SLOT_HEADER_SIZE + self.frame_bytes]
            )
            end = struct.unpack_from("<Q", self._map, offset + self.slot_size_bytes - 8)[0]
            if values[0] == seq and end == seq:
                return RingFrame(
                    seq=seq,
                    sim_time_ns=values[1],
                    frame_index=values[2],
                    camera_position_ned=values[3:6],
                    q_ned_from_camera=values[6:10],
                    pixels=pixels,
                )
        return None

    def close(self) -> None:
        if getattr(self, "_map", None) is not None:
            self._map.close()
            self._map = None
        if getattr(self, "_file", None) is not None:
            self._file.close()
            self._file = None

    def __enter__(self) -> "FrameRingReader":
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()
