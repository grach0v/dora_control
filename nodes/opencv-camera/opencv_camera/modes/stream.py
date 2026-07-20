"""`stream` mode — publish each captured frame as it lands (self-paced at FPS).

The only mode the opencv-camera node runs. It OWNS the background `CameraReader`:
`__init__` constructs it, `start()` starts the acquisition thread and sends the
startup `node_state`, `handle(event)` dispatches each input through an explicit
handler table (raising on an unrecognized id — a wiring bug for a producer),
and `close()` stops the reader.

There is no `tick`: the wire rate IS the configured FPS (the reader caps a file
source at it; a real camera self-paces). The dora loop polls (`node.next(timeout=…)`
in main.py) and calls `maybe_publish()`, which publishes the reader's latest frame
exactly once per capture (dedup by capture timestamp) on `<name>_image`. When the
capture ends (file source exhausted / device gone) it sends a `node_state` line and
stops the loop.

Inputs:  program_state.
Outputs: <name>_image, <name>_node_state.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Callable

import cv2
import pyarrow as pa
from dora import Node

from opencv_camera.node_config import CameraConfig

logger = logging.getLogger("opencv-camera")


def open_capture(cfg: CameraConfig) -> cv2.VideoCapture:
    source = int(cfg.camera_index) if cfg.camera_index.isdigit() else cfg.camera_index
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        raise RuntimeError(f"{cfg.camera_name}: failed to open capture {cfg.camera_index!r}")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.height)
    return cap


def encode(frame_bgr, cfg: CameraConfig):
    """Return a flat uint8 array of the encoded frame."""
    if cfg.encoding == "jpeg":
        ok, buf = cv2.imencode(".jpg", frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, cfg.jpeg_quality])
        if not ok:
            raise RuntimeError(f"{cfg.camera_name}: JPEG encode failed")
        return buf.ravel()
    if cfg.encoding == "rgb8":
        return cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB).ravel()
    raise ValueError(f"{cfg.camera_name}: unsupported encoding {cfg.encoding!r}")


class CameraReader:
    """Background acquisition: keeps the latest (frame, capture_ts), capped at FPS.

    A real camera self-paces (``cap.read`` blocks at the device rate); the FPS cap
    just stops a file source from being consumed faster than real time. The dora
    loop publishes each new frame, so it never blocks on a slow read.
    """

    def __init__(self, cfg: CameraConfig):
        self._cfg = cfg
        self._cap = open_capture(cfg)
        self._period = 1.0 / max(1, cfg.fps)
        self._lock = threading.Lock()
        self._latest: tuple | None = None  # (frame_bgr, capture_ts)
        self._ended = False
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="camera-read", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        self._cap.release()

    def latest(self) -> tuple | None:
        with self._lock:
            return self._latest

    @property
    def ended(self) -> bool:
        return self._ended

    def _run(self) -> None:
        while not self._stop.is_set():
            start = time.monotonic()
            ok, frame = self._cap.read()
            if not ok:  # device error or end of a file source
                self._ended = True
                return
            with self._lock:
                self._latest = (frame, time.time())  # wall-clock capture time
            self._stop.wait(max(0.0, self._period - (time.monotonic() - start)))


class StreamMode:
    def __init__(self, cfg: CameraConfig, node: Node):
        self.cfg = cfg
        self.node = node
        self.reader = CameraReader(cfg)
        self.status = ""
        self._last_ts: float | None = None  # capture ts of the last published frame
        self._handlers: dict[str, Callable] = {
            "program_state": self._on_program_state,
        }

    def start(self) -> None:
        self.reader.start()
        logger.info("%s: capture open (%s)", self.cfg.camera_name, self.cfg.encoding)
        self._emit_status("ready")

    def _emit_status(self, text: str) -> None:
        self.status = text
        self.node.send_output(f"{self.cfg.camera_name}_node_state", pa.array([text]))

    def handle(self, event) -> bool:
        return bool(self._handlers[event["id"]](event))  # KeyError on an unwired input id = loud

    def close(self) -> None:
        self.reader.stop()

    def _on_program_state(self, event) -> bool:
        return event["value"][0].as_py() == "disconnect"


    def maybe_publish(self) -> bool:
        """Publish the latest captured frame once per capture. Returns True to stop."""
        sample = self.reader.latest()
        if sample is not None:
            frame, ts = sample
            if ts != self._last_ts:
                self._last_ts = ts
                height, width = frame.shape[:2]
                self.node.send_output(
                    f"{self.cfg.camera_name}_image",
                    pa.array(encode(frame, self.cfg)),
                    metadata={
                        "encoding": self.cfg.encoding,
                        "width": int(width),
                        "height": int(height),
                        "timestamp": ts,
                    },
                )
        if self.reader.ended:  # file source exhausted / device gone
            self._emit_status(f"{self.cfg.camera_name}: capture ended")
            return True
        return False
