"""`stream` mode — publish each captured frameset as it lands (self-paced at the sensor FPS).

The only mode the realsense-camera node runs. It OWNS the background frame reader:
`__init__` constructs it (via the injected `reader_factory`, which keeps the
`pyrealsense2`-bound `FrameReader` and its native device plumbing in `main.py` —
this module imports no `pyrealsense2`, so it stays importable on hosts without the
wheel), `start()` starts the acquisition thread and sends the startup `node_state`,
`handle(event)` dispatches each input through an explicit handler table (raising on
an unrecognized id — a wiring bug for a producer), and `close()` stops the reader.

There is no `tick`: the wire rate IS the sensor FPS. The dora loop polls
(`node.next(timeout=…)` in main.py) and calls `maybe_publish()`, which publishes the
reader's latest frameset exactly once per capture (dedup by capture timestamp) —
color on `<name>_image` and, when depth is enabled and present, `<name>_depth`.

Inputs:  program_state.
Outputs: <name>_image, <name>_depth (when enabled), node_state.
"""

from __future__ import annotations

import logging
from typing import Callable

import cv2
import pyarrow as pa
from dora import Node

from realsense_camera.node_config import CameraConfig

logger = logging.getLogger("realsense-camera")


def encode_color(frame_rgb, cfg: CameraConfig):
    """Return a flat uint8 array of the encoded color frame ((H, W, 3) RGB)."""
    if cfg.encoding == "jpeg":
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        ok, buf = cv2.imencode(".jpg", frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, cfg.jpeg_quality])
        if not ok:
            raise RuntimeError(f"{cfg.camera_name}: JPEG encode failed")
        return buf.ravel()
    if cfg.encoding == "rgb8":
        return frame_rgb.ravel()
    raise ValueError(f"{cfg.camera_name}: unsupported encoding {cfg.encoding!r}")


class StreamMode:
    def __init__(self, cfg: CameraConfig, node: Node, reader_factory: Callable):
        self.cfg = cfg
        self.node = node
        # Owns the reader; the factory (main.py's FrameReader) keeps pyrealsense2
        # out of this module so it imports cleanly without the native wheel.
        self.reader = reader_factory(cfg)
        self.status = ""
        self._last_ts: float | None = None  # capture ts of the last published frameset
        self._handlers: dict[str, Callable] = {
            "program_state": self._on_program_state,
        }

    def start(self) -> None:
        self.reader.start()
        logger.info("%s: pipeline open (%s, depth=%s)",
                    self.cfg.camera_name, self.cfg.encoding, self.cfg.enable_depth)
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
        """Publish the latest captured frameset once per capture. Returns True to stop."""
        sample = self.reader.latest()
        if sample is None:  # no frame captured yet
            return False
        color, depth, ts = sample
        if ts == self._last_ts:  # already published this capture
            return False
        self._last_ts = ts
        height, width = color.shape[:2]
        self.node.send_output(
            f"{self.cfg.camera_name}_image",
            pa.array(encode_color(color, self.cfg)),
            metadata={"encoding": self.cfg.encoding, "width": int(width), "height": int(height), "timestamp": ts},
        )
        if depth is not None:
            d_height, d_width = depth.shape[:2]
            self.node.send_output(
                f"{self.cfg.camera_name}_depth",
                pa.array(depth.ravel()),
                metadata={"width": int(d_width), "height": int(d_height), "timestamp": ts},
            )
        return False
