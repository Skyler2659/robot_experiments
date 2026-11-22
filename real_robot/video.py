"""Record named RGB cameras on one monotonic timeline during a physical episode."""

import math
import os
import re
import select
import subprocess
import tempfile
import threading
import time
from pathlib import Path

import imageio_ffmpeg
import numpy as np


class _MP4:
    """One raw RGB pipe to FFmpeg. Limit the time a stalled pipe can block shutdown."""

    def __init__(self, path, image, fps):
        self.shape = image.shape
        h, w, _ = self.shape
        self.log = tempfile.TemporaryFile()  # noqa: SIM115 - lives as long as the encoder
        # 4:2:0 needs even dimensions. Keep the exact source dimensions for odd crops.
        pixel_format = "yuv420p" if h % 2 == 0 and w % 2 == 0 else "yuv444p"
        argv = [
            imageio_ffmpeg.get_ffmpeg_exe(),
            "-hide_banner",
            "-loglevel",
            "error",
            "-nostdin",
            "-n",
            "-f",
            "rawvideo",
            "-pixel_format",
            "rgb24",
            "-video_size",
            f"{w}x{h}",
            "-framerate",
            str(fps),
            "-i",
            "pipe:0",
            "-an",
            "-c:v",
            "libx264",
            "-threads",
            "1",
            "-filter_threads",
            "1",
            "-preset",
            "veryfast",
            "-crf",
            "20",
            "-pix_fmt",
            pixel_format,
            "-bf",
            "0",
            "-g",
            str(max(1, round(fps))),
            "-movflags",
            "+frag_keyframe+empty_moov+default_base_moof",
            str(path),
        ]
        try:
            self.process = subprocess.Popen(
                argv,
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=self.log,
                bufsize=0,
                start_new_session=True,
            )
        except BaseException:
            self.log.close()
            raise
        try:
            os.set_blocking(self.process.stdin.fileno(), False)
        except BaseException:
            self.process.kill()
            self.close()
            raise

    def write(self, frame):
        if frame.shape != self.shape:
            raise ValueError(f"Video image shape changed from {self.shape} to {frame.shape}")
        data = memoryview(frame).cast("B")
        fd = self.process.stdin.fileno()
        deadline = time.monotonic() + 5
        while data:
            timeout = deadline - time.monotonic()
            if timeout <= 0 or not select.select([], [fd], [], timeout)[1]:
                raise TimeoutError("FFmpeg stopped accepting RGB frames")
            try:
                count = os.write(fd, data)
            except BlockingIOError:
                continue
            data = data[count:]

    def close(self):
        try:
            self.process.stdin.close()
            try:
                code = self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
                raise
            if code:
                self.log.seek(0, os.SEEK_END)
                self.log.seek(max(0, self.log.tell() - 2000))
                raise RuntimeError(
                    f"FFmpeg exited with {code}: {self.log.read().decode(errors='replace')}"
                )
        finally:
            self.log.close()


class VideoRecorder:
    """Sample an independent camera source and publish all MP4s together on close."""

    def __init__(self, output, camera_names, read_frames, *, fps=15):
        self.names = tuple(camera_names)
        if (
            not self.names
            or len(set(self.names)) != len(self.names)
            or any(not re.fullmatch(r"[A-Za-z0-9_-]+", n) for n in self.names)
        ):
            raise ValueError("Expected unique video camera names")
        self.fps = float(fps)
        if not math.isfinite(self.fps) or self.fps <= 0:
            raise ValueError("Video fps must be finite and positive")
        self.frames_written = 0
        self.read_frames = read_frames
        self.final = Path(output) / "videos"
        self.partial = Path(output) / "videos.partial"
        self._writers = {}
        self._stop = threading.Event()
        self._writing = threading.Lock()
        self._error = None
        self._thread = None
        self._closed = False

    def _read(self):
        views = self.read_frames()
        if set(views) != set(self.names):
            raise ValueError(f"Video camera bundle must contain {self.names}, got {tuple(views)}")
        frames = {}
        for name in self.names:
            frame = np.asarray(views[name])
            if frame.dtype != np.uint8 or frame.ndim != 3 or frame.shape[2] != 3 or not frame.size:
                raise ValueError(f"{name}: expected a nonempty HWC uint8 RGB frame")
            # Keep an owned previous sample when the driver reuses its camera buffers.
            frames[name] = frame.copy(order="C")
        return frames

    def _write(self, frames):
        for name, writer in self._writers.items():
            writer.write(frames[name])
        self.frames_written += 1

    def _finish_encoders(self):
        for writer in self._writers.values():
            try:
                writer.close()
            except (OSError, RuntimeError, ValueError, subprocess.SubprocessError) as exc:
                if self._error is None:
                    self._error = exc
        self._writers.clear()

    def start(self):
        if self._thread is not None or self._closed:
            raise RuntimeError("Recorder cannot be started twice")
        if self.final.exists():
            raise FileExistsError(self.final)
        self.partial.mkdir(parents=True, exist_ok=False)
        try:
            self._last = self._read()
            self._start = time.monotonic()
            for name in self.names:
                self._writers[name] = _MP4(self.partial / f"{name}.mp4", self._last[name], self.fps)
            self._write(self._last)
            self._thread = threading.Thread(target=self._run, name="piper-video", daemon=True)
            self._thread.start()
        except BaseException:
            self._closed = True
            self._finish_encoders()
            raise

    def _run(self):
        try:
            while True:
                next_tick = self._start + self.frames_written / self.fps
                if self._stop.wait(max(0, next_tick - time.monotonic())):
                    break
                frames = self._read()
                sampled = time.monotonic()
                with self._writing:
                    if self._stop.is_set():
                        break
                    # A delayed read cannot supply past frames. Extend the previous
                    # sample up to this tick, keeping elapsed time and camera PTS aligned.
                    due = int((sampled - self._start) * self.fps)
                    while self.frames_written < due and not self._stop.is_set():
                        self._write(self._last)
                    if self._stop.is_set():
                        break
                    self._write(frames)
                    self._last = frames
        except BaseException as exc:  # noqa: BLE001 - relay worker failures to the episode
            self._error = exc

    def check(self):
        if self._error is not None:
            raise RuntimeError(
                f"Continuous camera recording failed: {self._error}"
            ) from self._error

    def close(self):
        if self._closed:
            self.check()
            return
        if self._thread is None:
            raise RuntimeError("Recorder has not started")
        end = time.monotonic()
        self._closed = True
        self._stop.set()
        self._thread.join(timeout=6)
        if self._thread.is_alive():
            self._error = TimeoutError("Video source did not stop within six seconds")
        # A tardy camera call will see the stop event before touching an encoder.
        with self._writing:
            try:
                if self._error is None:
                    due = math.ceil((end - self._start) * self.fps)
                    while self.frames_written < due:
                        self._write(self._last)
            except (OSError, ValueError, RuntimeError) as exc:
                self._error = exc
            finally:
                self._finish_encoders()
        self.check()
        self.partial.rename(self.final)
