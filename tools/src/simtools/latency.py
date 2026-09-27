import select
import subprocess
import threading
import time
from dataclasses import dataclass, field

from simtools.api import ApiError, ControlClient
from simtools.proto.frame_ring import FrameRingError, FrameRingReader

BURN_IN_CELL_PX = 16
BURN_IN_BITS = 16
BURN_IN_CELLS = BURN_IN_BITS + 2
BURN_IN_MODULUS = 1 << BURN_IN_BITS


class LatencyError(Exception):
    pass


def burn_in_pattern(frame_index: int) -> str:
    """Marker cell, guard cell, then the frame index in binary, most significant bit first."""
    bits = format(frame_index % BURN_IN_MODULUS, f"0{BURN_IN_BITS}b")
    return f"10{bits}"


def decode_burn_in(
    frame: bytes, width: int, height: int, cell_px: int = BURN_IN_CELL_PX
) -> int | None:
    """Reads the counter back out of a greyscale frame, or None when the marker is missing."""
    if height < cell_px or width < BURN_IN_CELLS * cell_px or len(frame) < width * height:
        return None
    row = (cell_px // 2) * width
    centres = [frame[row + cell * cell_px + cell_px // 2] for cell in range(BURN_IN_CELLS)]
    bits = "".join("1" if value >= 128 else "0" for value in centres)
    if bits[:2] != "10":
        return None
    return int(bits[2:], 2)


@dataclass(frozen=True)
class CapturedFrame:
    monotonic: float
    index: int | None


def consumer_command(consumer: str, width: int, height: int) -> list[str]:
    """Wraps a consumer fragment so frames arrive on stdout as GRAY8 at the camera resolution."""
    tail = (
        f"videoconvert ! video/x-raw,format=GRAY8,width={width},height={height} ! "
        "fdsink fd=1 sync=false"
    )
    return ["gst-launch-1.0", "-q", *f"{consumer} ! {tail}".split()]


def capture_frames(
    consumer: str, width: int, height: int, frames: int, timeout_s: float
) -> list[CapturedFrame]:
    """Runs the consumer pipeline and timestamps every complete frame it hands over."""
    frame_bytes = width * height
    command = consumer_command(consumer, width, height)
    try:
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    except FileNotFoundError as error:
        raise LatencyError("gst-launch-1.0 not found; install the GStreamer tools") from error
    captured: list[CapturedFrame] = []
    deadline = time.monotonic() + timeout_s
    assert process.stdout is not None
    fd = process.stdout.fileno()
    pending = bytearray()
    try:
        while len(captured) < frames and time.monotonic() < deadline:
            ready, _, _ = select.select(
                [fd], [], [], min(0.2, max(0.0, deadline - time.monotonic()))
            )
            if not ready:
                if process.poll() is not None:
                    break
                continue
            chunk = process.stdout.read1(frame_bytes)
            if not chunk:
                break
            pending += chunk
            while len(pending) >= frame_bytes and len(captured) < frames:
                frame = bytes(pending[:frame_bytes])
                del pending[:frame_bytes]
                captured.append(
                    CapturedFrame(time.monotonic(), decode_burn_in(frame, width, height))
                )
    finally:
        process.terminate()
        try:
            stderr = process.communicate(timeout=5)[1]
        except subprocess.TimeoutExpired:
            process.kill()
            stderr = process.communicate()[1]
    if not captured:
        message = stderr.decode(errors="replace").strip().splitlines()
        raise LatencyError(
            "the consumer delivered no frames: " + (message[-1] if message else "no output")
        )
    return captured


class _RingWatcher(threading.Thread):
    """Notes when each published frame appeared in the ring, keyed by its burn-in counter."""

    def __init__(self, camera: str, poll_s: float) -> None:
        super().__init__(daemon=True)
        self.camera = camera
        self.poll_s = poll_s
        self.published: dict[int, tuple[int, float, int]] = {}
        self.indices: list[int] = []
        self.sim_times_ns: list[int] = []
        self.reader: FrameRingReader | None = None
        self._stop_requested = threading.Event()

    def open(self) -> FrameRingReader:
        self.reader = FrameRingReader(self.camera)
        return self.reader

    def run(self) -> None:
        assert self.reader is not None
        seq = 0
        while not self._stop_requested.is_set():
            frame = self.reader.read_latest(seq)
            if frame is None:
                time.sleep(self.poll_s)
                continue
            seq = frame.seq
            now = time.monotonic()
            self.published[frame.frame_index % BURN_IN_MODULUS] = (
                frame.frame_index,
                now,
                frame.sim_time_ns,
            )
            self.indices.append(frame.frame_index)
            self.sim_times_ns.append(frame.sim_time_ns)

    def stop(self) -> None:
        self._stop_requested.set()
        self.join(timeout=2.0)
        if self.reader is not None:
            self.reader.close()


@dataclass
class LatencyReport:
    camera: str
    consumer: str
    width: int
    height: int
    fps_nominal: float
    captured: int
    undecoded: int
    matched: int
    latencies_ms: list[float] = field(default_factory=list)
    sim_periods_ms: list[float] = field(default_factory=list)
    skipped_indices: int = 0

    @property
    def p50_ms(self) -> float:
        return _percentile(self.latencies_ms, 0.5)

    @property
    def p95_ms(self) -> float:
        return _percentile(self.latencies_ms, 0.95)

    @property
    def max_ms(self) -> float:
        return max(self.latencies_ms) if self.latencies_ms else float("nan")

    @property
    def sim_period_error_ms(self) -> float:
        """Worst deviation of the sim time between published frames from the nominal period."""
        if not self.sim_periods_ms or self.fps_nominal <= 0.0:
            return float("nan")
        nominal = 1e3 / self.fps_nominal
        return max(abs(period - nominal) for period in self.sim_periods_ms)


def _percentile(values: list[float], q: float) -> float:
    if not values:
        return float("nan")
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(q * len(ordered)))]


def measure_latency(
    camera: str,
    consumer: str,
    frames: int = 120,
    timeout_s: float = 30.0,
    poll_s: float = 0.002,
) -> LatencyReport:
    """Times how long a published frame takes to reach a consumer of one of its outputs."""
    watcher = _RingWatcher(camera, poll_s)
    try:
        reader = watcher.open()
    except FrameRingError as error:
        raise LatencyError(str(error)) from error
    width, height, fps = reader.width, reader.height, reader.fps_nominal
    watcher.start()
    try:
        captured = capture_frames(consumer, width, height, frames, timeout_s)
    finally:
        watcher.stop()
    report = LatencyReport(
        camera=camera,
        consumer=consumer,
        width=width,
        height=height,
        fps_nominal=fps,
        captured=len(captured),
        undecoded=sum(1 for frame in captured if frame.index is None),
        matched=0,
    )
    report.sim_periods_ms = [
        (later - earlier) * 1e-6
        for earlier, later in zip(watcher.sim_times_ns, watcher.sim_times_ns[1:], strict=False)
    ]
    seen: set[int] = set()
    previous_index: int | None = None
    for frame in captured:
        if frame.index is None:
            continue
        entry = watcher.published.get(frame.index)
        if entry is None or frame.index in seen:
            continue
        seen.add(frame.index)
        index, published_at, _sim_time_ns = entry
        report.matched += 1
        report.latencies_ms.append((frame.monotonic - published_at) * 1e3)
        if previous_index is not None and index > previous_index:
            report.skipped_indices += index - previous_index - 1
        previous_index = index
    return report


def format_report(report: LatencyReport) -> str:
    lines = [
        f"camera {report.camera} {report.width}x{report.height} at {report.fps_nominal:.1f} fps",
        f"consumer: {report.consumer}",
        f"frames captured {report.captured}, counter unreadable {report.undecoded}, "
        f"matched to a published frame {report.matched}",
    ]
    if report.matched:
        lines.append(
            f"publish to consumer latency: p50 {report.p50_ms:.1f} ms, "
            f"p95 {report.p95_ms:.1f} ms, max {report.max_ms:.1f} ms"
        )
        lines.append(f"published frames the consumer never showed: {report.skipped_indices}")
    if report.sim_periods_ms:
        lines.append(
            f"sim time between published frames: {len(report.sim_periods_ms) + 1} frames, "
            f"worst deviation from {1e3 / report.fps_nominal:.2f} ms "
            f"is {report.sim_period_error_ms:.2f} ms"
        )
    else:
        lines.append(
            "no captured frame matched a published one; is the burn-in counter enabled "
            "for this camera?"
        )
    return "\n".join(lines) + "\n"


@dataclass
class TimestampReport:
    """How far the sim time on a published frame sits behind the state the backend reports.

    A frame is always a little older than the live state: it was rendered from a state that had
    already been sent. Each sample brackets the frame read between two state requests, so the
    request round trip is measured instead of being charged to the frame.
    """

    camera: str
    fps_nominal: float
    behind_ms: list[float] = field(default_factory=list)
    api_window_ms: list[float] = field(default_factory=list)

    @property
    def p50_ms(self) -> float:
        return _percentile(self.behind_ms, 0.5)

    @property
    def max_ms(self) -> float:
        return max(self.behind_ms, default=float("nan"))

    @property
    def max_api_window_ms(self) -> float:
        return max(self.api_window_ms, default=float("nan"))

    @property
    def frame_period_ms(self) -> float:
        return 1e3 / self.fps_nominal if self.fps_nominal > 0.0 else float("nan")

    @property
    def budget_ms(self) -> float:
        return self.frame_period_ms + self.max_api_window_ms

    @property
    def within_one_frame(self) -> bool:
        """Never ahead of the state, and at most a period plus the round trip behind it."""
        return bool(self.behind_ms) and -1e-6 <= self.max_ms <= self.budget_ms


def check_frame_timestamps(
    camera: str, port: int = 7700, samples: int = 60, host: str = "127.0.0.1"
) -> TimestampReport:
    """Reads the newest frame and the live state together, so their sim times can be compared."""
    try:
        reader = FrameRingReader(camera)
    except FrameRingError as error:
        raise LatencyError(str(error)) from error
    report = TimestampReport(camera=camera, fps_nominal=reader.fps_nominal)
    try:
        with ControlClient(host=host, port=port) as client:
            seq = 0
            before_ns = int(client.request("get_state")["sim_time_ns"])
            deadline = time.monotonic() + samples / max(reader.fps_nominal, 1.0) + 10.0
            while len(report.behind_ms) < samples and time.monotonic() < deadline:
                frame = reader.read_latest(seq)
                if frame is None:
                    time.sleep(0.002)
                    continue
                seq = frame.seq
                after_ns = int(client.request("get_state")["sim_time_ns"])
                report.behind_ms.append((before_ns - frame.sim_time_ns) * 1e-6)
                report.api_window_ms.append((after_ns - before_ns) * 1e-6)
                before_ns = after_ns
    except (ApiError, OSError) as error:
        raise LatencyError(f"control API on {host}:{port}: {error}") from error
    finally:
        reader.close()
    if not report.behind_ms:
        raise LatencyError("no frames were published while the timestamps were being checked")
    return report


def format_timestamp_report(report: TimestampReport) -> str:
    verdict = "within" if report.within_one_frame else "OUTSIDE"
    return (
        f"camera {report.camera} at {report.fps_nominal:.1f} fps, "
        f"{len(report.behind_ms)} frames\n"
        f"published frames sit behind the live state by p50 {report.p50_ms:.2f} ms, "
        f"max {report.max_ms:.2f} ms\n"
        f"{verdict} one frame period plus the state request round trip "
        f"({report.frame_period_ms:.2f} + {report.max_api_window_ms:.2f} ms)\n"
    )
