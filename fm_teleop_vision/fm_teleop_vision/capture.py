# Vendored from vision-based-joint-tracking (pose_pipeline/capture.py).
# Change vs upstream: _backend_flag() is platform-aware so a device index works inside
# the Linux container (V4L2), not only on macOS (AVFoundation). See _backend_flag.
"""Phone / webcam frame source with reconnect + stall handling.

`CameraSource.frames()` is an infinite generator yielding `(frame_bgr, wall_clock_ts)`.
All reconnect/backoff/stall logic is hidden inside it, so the caller stays clean:

    for frame, ts in source.frames():
        ...

On Mac/OrbStack there is no USB/camera passthrough into the container, so use an
http/rtsp URL (a phone IP-webcam app on the OrbStack network). On Linux with
compose.linux.yaml (/dev passthrough), an integer /dev/video* index works directly.
"""

from __future__ import annotations

import logging
import sys
import threading
import time
from dataclasses import dataclass

import cv2

log = logging.getLogger(__name__)


@dataclass
class CaptureStatus:
    connected: bool
    consecutive_failures: int
    time_since_good_frame_s: float
    total_good_frames: int
    total_reconnects: int


class CameraSource:
    def __init__(
        self,
        source,
        *,
        backend: str = "auto",
        max_retries: int = 0,
        backoff_initial_s: float = 0.5,
        backoff_max_s: float = 10.0,
        backoff_factor: float = 2.0,
        max_consecutive_failed_reads: int = 30,
        max_time_since_good_frame_s: float = 3.0,
    ):
        self.source = source
        self.backend = backend
        self._max_retries = max_retries
        self._backoff_initial = backoff_initial_s
        self._backoff_max = backoff_max_s
        self._backoff_factor = backoff_factor
        self._max_failed_reads = max_consecutive_failed_reads
        self._max_since_good = max_time_since_good_frame_s

        self._cap = None
        self._consecutive_failures = 0
        self._total_good_frames = 0
        self._total_reconnects = 0
        self._last_good_t = None

        # Latest-frame handoff: a reader thread drains the stream and keeps only the most
        # recent frame, so a slow consumer (MediaPipe) never falls behind a growing capture
        # buffer (which showed up as tens of seconds of accumulating video latency).
        self._latest = None
        self._latest_seq = 0
        self._reader = None
        self._cond = threading.Condition()

    @property
    def resolved_source(self):
        """Return an int device index or a URL string."""
        if isinstance(self.source, int):
            return self.source
        s = str(self.source)
        try:
            return int(s)
        except ValueError:
            return s

    def _backend_flag(self) -> int:
        src = self.resolved_source
        if self.backend == "ffmpeg":
            return cv2.CAP_FFMPEG
        if self.backend == "avfoundation":
            return cv2.CAP_AVFOUNDATION
        if self.backend == "v4l2":
            return cv2.CAP_V4L2
        # auto: a URL stream always goes through FFMPEG; a device index uses the
        # platform-native backend — AVFoundation on macOS, V4L2 on Linux. The upstream
        # product ran only on macOS and always returned AVFoundation here, which fails
        # for /dev/video* inside the Linux container; hence the platform split.
        if not isinstance(src, int):
            return cv2.CAP_FFMPEG
        return cv2.CAP_AVFOUNDATION if sys.platform == "darwin" else cv2.CAP_V4L2

    def open(self) -> bool:
        self.release()
        src = self.resolved_source
        flag = self._backend_flag()
        log.info("Opening capture source=%r (backend flag=%d)", src, flag)
        self._cap = cv2.VideoCapture(src, flag)
        # Keep the backend buffer tiny; combined with the reader thread this bounds latency
        # to ~one frame instead of letting a slow consumer accumulate a growing backlog.
        try:
            self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass
        if self._cap.isOpened():
            self._last_good_t = time.time()
            self._consecutive_failures = 0
            return True
        return False

    def _reconnect(self) -> None:
        self._total_reconnects += 1
        backoff = self._backoff_initial
        attempt = 0
        while True:
            attempt += 1
            if self._max_retries and attempt > self._max_retries:
                raise RuntimeError(
                    f"Capture reconnect failed after {attempt - 1} attempts "
                    f"(source={self.resolved_source!r})"
                )
            log.warning("Reconnecting (attempt %d), sleeping %.1fs ...", attempt, backoff)
            time.sleep(backoff)
            if self.open():
                log.info("Capture reconnected after %d attempt(s)", attempt)
                return
            backoff = min(backoff * self._backoff_factor, self._backoff_max)

    def frames(self):
        """Yield (frame_bgr, wall_clock_ts) forever — always the LATEST frame.

        A background reader thread continuously drains the stream (fast MJPEG decode) and
        keeps only the most recent frame; this generator hands the consumer that newest
        frame and drops everything in between, so a slow consumer (e.g. MediaPipe) can never
        lag behind a growing capture buffer. Reconnect/stall handling lives in the reader.
        """
        if self._cap is None or not self._cap.isOpened():
            if not self.open():
                log.warning("Initial capture open failed; entering reconnect loop")
                self._reconnect()
        self._start_reader()
        last_seq = 0
        while True:
            with self._cond:
                self._cond.wait_for(
                    lambda: self._latest_seq != last_seq,
                    timeout=self._max_since_good + 1.0,
                )
                if self._latest_seq == last_seq:
                    continue  # timed out waiting; the reader handles any stall/reconnect
                frame, ts = self._latest
                last_seq = self._latest_seq
            yield frame, ts

    def _start_reader(self):
        if self._reader is not None:
            return
        self._reader = threading.Thread(
            target=self._reader_loop, name="camera_reader", daemon=True
        )
        self._reader.start()

    def _reader_loop(self):
        """Drain the stream forever, publishing only the newest frame (drops the backlog)."""
        while True:
            ok, frame = self._cap.read()
            now = time.time()
            if ok and frame is not None:
                self._consecutive_failures = 0
                self._last_good_t = now
                self._total_good_frames += 1
                with self._cond:
                    self._latest = (frame, now)
                    self._latest_seq += 1
                    self._cond.notify_all()
                continue

            # Failed read.
            self._consecutive_failures += 1
            since_good = (now - self._last_good_t) if self._last_good_t else float("inf")
            stalled = (
                self._consecutive_failures >= self._max_failed_reads
                or since_good >= self._max_since_good
            )
            if stalled:
                log.warning(
                    "Stream stalled (failures=%d, since_good=%.1fs); reconnecting",
                    self._consecutive_failures,
                    since_good,
                )
                self._reconnect()
            else:
                time.sleep(0.01)

    def status(self) -> CaptureStatus:
        now = time.time()
        return CaptureStatus(
            connected=self._cap is not None and self._cap.isOpened(),
            consecutive_failures=self._consecutive_failures,
            time_since_good_frame_s=(now - self._last_good_t) if self._last_good_t else float("inf"),
            total_good_frames=self._total_good_frames,
            total_reconnects=self._total_reconnects,
        )

    def release(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None
