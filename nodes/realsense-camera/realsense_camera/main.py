"""Intel RealSense camera node.

Opens a RealSense device (selected by serial) via pyrealsense2. Acquisition runs
on a **background thread** that polls the pipeline and keeps the latest color
(and optional depth) frameset with its capture timestamp; the dora loop polls
(`node.next(timeout=…)`) and publishes each captured frameset exactly once, so
the wire rate IS the configured sensor FPS — no tick, no timer-phase latency.
The capture `timestamp` travels in metadata so a consumer can align streams
(see docs/message_formats.md).

The color payload is a flat uint8 Arrow array (raw RGB for "rgb8", or JPEG bytes
for "jpeg"); depth (when enabled) is a flat uint16 array in millimetres. Shape /
encoding / timestamp travel in metadata, so any-language consumer can read it.

This file owns the dora skeleton and the device plumbing (the `pyrealsense2`-bound
background `FrameReader`, hardware reset/recovery); it builds the mode (which owns
the reader, constructed via the injected `FrameReader`), runs the event loop, and
always tears the mode down. What each input *means* lives in a mode
(modes/stream.py) selected by ``MODE``.

Inputs:
  program_state    manager/program_state; stops the node on `disconnect`
Outputs:
  <name>_image     uint8[N] + metadata {encoding, width, height, timestamp}
  <name>_depth     uint16[width*height] + metadata {width, height, timestamp}  (when enabled)
  <name>_node_state  edge-triggered state token (see docs/message_formats.md)
"""

from __future__ import annotations

import sys
import threading
import time

import numpy as np
import pyrealsense2 as rs
from dora import Node

from realsense_camera.modes import MODES
from realsense_camera.node_config import CameraConfig, load_config


def open_pipeline(cfg: CameraConfig) -> rs.pipeline:
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(cfg.serial)
    config.enable_stream(rs.stream.color, cfg.width, cfg.height, rs.format.rgb8, cfg.fps)
    if cfg.enable_depth:
        config.enable_stream(rs.stream.depth, cfg.width, cfg.height, rs.format.z16, cfg.fps)
    pipeline.start(config)
    return pipeline


def hardware_reset(serial: str) -> None:
    """Power-cycle the device. D405s sharing a USB controller routinely wedge
    after an unclean close (opened but never deliver a frame again)."""
    for device in rs.context().devices:
        if device.get_info(rs.camera_info.serial_number) == serial:
            device.hardware_reset()
            return
    raise RuntimeError(f"no RealSense device with serial {serial}")


class FrameReader:
    """Background acquisition: keeps the latest (color, depth, capture_ts).

    `pipeline.try_wait_for_frames` is a bounded poll, so the thread stays
    responsive to stop() even if the device goes silent. Incomplete framesets are
    skipped rather than stored as zero-size payloads.
    """

    def __init__(self, cfg: CameraConfig):
        self._cfg = cfg
        self._lock = threading.Lock()
        self._latest: tuple | None = None  # (color_rgb, depth_or_None, capture_ts)
        self._stop = threading.Event()
        self._pipeline: rs.pipeline | None = None
        # Always reset before the first connect: D405s left wedged from a prior
        # run open fine but never deliver a frame, so we power-cycle up front.
        self._reset_and_open("startup")
        self._thread = threading.Thread(target=self._run, name="realsense-read", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        if self._pipeline is not None:
            self._pipeline.stop()

    def latest(self) -> tuple | None:
        with self._lock:
            return self._latest

    def _reset_and_open(self, reason: str) -> None:
        """Hardware-reset the device, then (re)open the pipeline. Reset-first
        because a wedged D405 opens but never delivers frames; a clean
        power-cycle fixes it. Retries until frames can flow or stop() is set."""
        while not self._stop.is_set():
            print(f"{self._cfg.camera_name}: {reason}, hardware-resetting device", flush=True)
            hardware_reset(self._cfg.serial)
            self._stop.wait(self._cfg.reset_settle_s)  # device re-enumerates after a reset
            try:
                self._pipeline = open_pipeline(self._cfg)
                return
            except RuntimeError as err:  # device not back yet — reset again
                print(f"{self._cfg.camera_name}: reopen failed ({err}), retrying", flush=True)

    def _recover(self) -> None:
        """Reset + reopen a device that stopped delivering frames."""
        self._pipeline.stop()
        self._reset_and_open(f"no frames for {self._cfg.watchdog_s}s")

    def _run(self) -> None:
        last_frame = time.monotonic()
        while not self._stop.is_set():
            ok, frames = self._pipeline.try_wait_for_frames(timeout_ms=200)
            if not ok:
                if time.monotonic() - last_frame > self._cfg.watchdog_s:
                    self._recover()
                    last_frame = time.monotonic()
                continue
            last_frame = time.monotonic()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            color = np.asanyarray(color_frame.get_data())
            depth = None
            if self._cfg.enable_depth:
                depth_frame = frames.get_depth_frame()
                if not depth_frame:
                    continue
                depth = np.asanyarray(depth_frame.get_data())
            with self._lock:
                self._latest = (color, depth, time.time())  # wall-clock capture time


def run(node: Node, mode) -> None:
    # Self-paced producer: poll for events with a short timeout and publish each
    # newly captured frameset in between — the sensor FPS paces the wire.
    mode.start()
    try:
        while True:
            event = node.next(timeout=0.005)
            if event is not None:
                if event["type"] == "STOP":
                    break
                if event["type"] == "INPUT" and mode.handle(event):  # True -> program_state stop
                    break
            if mode.maybe_publish():
                break
    finally:
        mode.close()


def main() -> int:
    cfg = load_config()
    node = Node()
    # The mode owns the reader; we inject FrameReader so modes/stream.py stays
    # free of pyrealsense2 (importable without the native wheel).
    mode = MODES[cfg.mode](cfg, node, FrameReader)  # KeyError on a bad MODE = loud
    run(node, mode)
    return 0


if __name__ == "__main__":
    sys.exit(main())
