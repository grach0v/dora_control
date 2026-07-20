"""`record` mode — operator-controlled multi-camera episode recording.

The only mode. An explicit idle↔recording state machine driven by the ``control``
input: ``start`` begins an episode, ``finish`` saves it and goes idle, ``task=<text>``
sets the task label. The FIRST camera in ``cameras`` paces recording (one dataset
frame per pacing frame), pairing it with the other cameras' latest frame and each
state/action input's sample nearest the pacing timestamp (within ``sync_tolerance``).

`main.py` owns the dora skeleton and opens the ``LeRobotDataset`` (injected here so
the tests can pass a fake); this module owns the per-input logic and the recording
state (frames/episodes/buffers/task). Each input is dispatched through an explicitly
built handler table; an input id the node was not configured for *raises* rather
than being silently dropped. The pure pacing/sync helpers live here too; nothing
here imports main, so there is no cycle.

Inputs:  the cameras / state_inputs / action_inputs named in config, plus
         `control` and `program_state` (disconnect -> True).
Outputs: node_state.
"""

from __future__ import annotations

import logging
from collections import deque
from functools import partial
from typing import Callable

import cv2
import numpy as np
import pyarrow as pa
from dora import Node

from lerobot_node.node_config import RecorderConfig

logger = logging.getLogger("lerobot")

NODE_ID = "lerobot"


def image_key(camera: str) -> str:
    return f"observation.images.{camera}"


def decode_image(event, cfg: RecorderConfig) -> np.ndarray:
    """Decode a flat Arrow image payload into an (H, W, 3) uint8 RGB frame."""
    md = event["metadata"]
    encoding = md["encoding"]
    buf = event["value"].to_numpy(zero_copy_only=False)
    if encoding == "jpeg":
        bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if bgr is None:
            raise ValueError(f"{NODE_ID}: failed to decode jpeg frame")
        frame = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    elif encoding == "rgb8":
        width, height = int(md["width"]), int(md["height"])
        frame = buf.reshape(height, width, 3)
    else:
        raise ValueError(f"{NODE_ID}: unsupported image encoding {encoding!r}")
    height, width = frame.shape[:2]
    if (height, width) != (cfg.image_height, cfg.image_width):
        raise ValueError(
            f"{NODE_ID}: image {width}x{height} != configured "
            f"{cfg.image_width}x{cfg.image_height}"
        )
    return frame


def event_timestamp(event) -> float:
    return float(event["metadata"]["timestamp"])


def to_float32(event) -> np.ndarray:
    return event["value"].to_numpy(zero_copy_only=False).astype(np.float32)


def nearest(buf, t: float):
    """The (ts, value) in `buf` whose ts is closest to `t` (None if empty)."""
    return min(buf, key=lambda item: abs(item[0] - t)) if buf else None


def concat_nearest(bufs: dict, input_ids: list[str], t: float, tol: float):
    """Concatenate each input's sample nearest `t`, in order. None if any input
    has nothing buffered or its nearest sample is outside the tolerance."""
    parts = []
    for input_id in input_ids:
        got = nearest(bufs[input_id], t)
        if got is None or abs(got[0] - t) > tol:
            return None
        parts.append(got[1])
    return np.concatenate(parts) if parts else np.empty(0, dtype=np.float32)


class RecordMode:
    """Idle↔recording state machine. Buffers state/action by input id, assembles
    one dataset frame per pacing-camera frame when every stream lines up."""

    def __init__(self, cfg: RecorderConfig, node: Node, dataset):
        self.cfg = cfg
        self.node = node
        self.dataset = dataset
        self.pacing = cfg.cameras[0]
        self.other_cams = cfg.cameras[1:]
        self.latest_img: dict[str, tuple[float, np.ndarray]] = {}  # non-pacing camera -> (ts, frame)
        self.state_bufs = {sid: deque(maxlen=cfg.sync_buffer) for sid in cfg.state_inputs}
        self.action_bufs = {aid: deque(maxlen=cfg.sync_buffer) for aid in cfg.action_inputs}
        self.task = cfg.task
        self.recording = False
        self.frames = 0
        self.skipped = 0
        self.episodes = 0

        self.status = ""
        self._handlers: dict[str, Callable] = {
            "program_state": self._on_program_state,
            "control": self._on_control,
        }
        for sid in cfg.state_inputs:
            self._handlers[sid] = partial(self._on_state, sid)
        for aid in cfg.action_inputs:
            self._handlers[aid] = partial(self._on_action, aid)
        for cam in cfg.cameras:
            self._handlers[cam] = partial(self._on_camera, cam)

    def start(self) -> None:
        logger.info("%s: ready -> %s (idle until 'start')", NODE_ID, self.cfg.repo_root)
        self._emit_status("ready")

    def _emit_status(self, text: str) -> None:
        self.status = text
        self.node.send_output("node_state", pa.array([text]))

    def handle(self, event) -> bool:
        return bool(self._handlers[event["id"]](event))  # KeyError on an unwired input id = loud

    def close(self) -> None:
        """Save an in-progress episode (>0 frames) on shutdown."""
        self._save_if_any()

    def _save_if_any(self) -> None:
        """Save an in-progress episode (>0 frames) — on `finish` and on shutdown."""
        if self.frames > 0:
            self.dataset.save_episode()
            self.episodes += 1
            self._emit_status(
                f"{NODE_ID}: saved episode {self.episodes - 1} ({self.frames} frames, {self.skipped} skipped)"
            )

    def _on_program_state(self, event) -> bool:
        return event["value"][0].as_py() == "disconnect"


    def _on_control(self, event) -> None:
        cmd = event["value"][0].as_py()
        if cmd == "start":
            self._save_if_any()  # a double `start` closes the open episode instead of
            # leaking its buffered frames into the next save_episode
            self.recording, self.frames, self.skipped = True, 0, 0
            self.latest_img.clear()
            self._emit_status(f"{NODE_ID}: recording (task={self.task!r})")
        elif cmd == "finish":
            self._save_if_any()
            self.recording, self.frames, self.skipped = False, 0, 0
        elif cmd.startswith("task="):
            self.task = cmd[len("task="):]

    def _on_state(self, sid: str, event) -> None:
        self.state_bufs[sid].append((event_timestamp(event), to_float32(event)))

    def _on_action(self, aid: str, event) -> None:
        self.action_bufs[aid].append((event_timestamp(event), to_float32(event)))

    def _on_camera(self, cam: str, event) -> None:
        if not self.recording:
            return
        img = decode_image(event, self.cfg)
        img_ts = event_timestamp(event)
        if cam != self.pacing:
            self.latest_img[cam] = (img_ts, img)
            return
        # Pacing frame: assemble one dataset frame if every stream lines up.
        images = {image_key(self.pacing): img}
        for other in self.other_cams:
            got = self.latest_img.get(other)
            if got is None or abs(got[0] - img_ts) > self.cfg.sync_tolerance:
                images = None
                break
            images[image_key(other)] = got[1]
        state = concat_nearest(self.state_bufs, self.cfg.state_inputs, img_ts, self.cfg.sync_tolerance)
        action = concat_nearest(self.action_bufs, self.cfg.action_inputs, img_ts, self.cfg.sync_tolerance)
        if images is None or state is None or action is None:
            self.skipped += 1
            return
        if len(state) != self.cfg.state_dim or len(action) != self.cfg.action_dim:
            raise ValueError(
                f"{NODE_ID}: state/action dims {len(state)}/{len(action)} != "
                f"configured {self.cfg.state_dim}/{self.cfg.action_dim}"
            )
        self.dataset.add_frame({**images, self.cfg.state_key: state, self.cfg.action_key: action, "task": self.task})
        self.frames += 1
