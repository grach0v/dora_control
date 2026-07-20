"""LeRobot node (record mode) — multi-camera, operator-controlled episodes.

Records N camera streams + a concatenated state vector + a concatenated action
vector into a ``LeRobotDataset``. Recording is gated by a ``control`` input
(from the web/record node): ``start`` begins an episode, ``finish`` saves it and
goes idle, ``task=<text>`` sets the task label. Multiple episodes are recorded in
one process (each ``finish`` appends an episode).

The FIRST camera in ``cameras`` paces recording: one frame is added per pacing
frame, pairing it with the other cameras' latest frame and each state/action
input's sample captured *nearest the pacing timestamp* (within ``sync_tolerance``;
otherwise the frame is skipped as a gap). Alignment is by capture ``timestamp``
(metadata), not arrival order.

Wire payloads are plain Arrow (see docs/message_formats.md):
  <camera>   uint8[N] + metadata {encoding:"rgb8"|"jpeg", width, height, timestamp}
  <state/action input>  float64[K] + metadata {timestamp}
  control    utf8[1]  "start" | "finish" | "task=<text>"
  program_state / status  utf8[1]

This file owns only the dora skeleton: open the dataset, build the mode selected by
``MODE``, run the event loop, and always tear the mode down (flush an in-progress
episode). What each input *means* lives in a mode (modes/record.py).

Inputs:  the cameras / state_inputs / action_inputs named in config, plus
         `control` and `program_state` (saves any in-progress episode and stops
         on `disconnect`).
Outputs: node_state.
"""

from __future__ import annotations

import sys

from dora import Node
from lerobot.datasets.lerobot_dataset import LeRobotDataset

# `image_key` is re-exported (the tests import it from here); the per-input state
# machine lives in modes/record.py.
from lerobot_node.modes import MODES
from lerobot_node.modes.record import NODE_ID, image_key  # noqa: F401
from lerobot_node.node_config import RecorderConfig, load_config

__all__ = ["build_features", "image_key", "main", "open_dataset", "run"]


def build_features(cfg: RecorderConfig) -> dict:
    """LeRobotDataset feature spec: one video per camera + state + action."""
    features = {
        image_key(cam): {
            "dtype": "video",
            "shape": (cfg.image_height, cfg.image_width, 3),
            "names": ["height", "width", "channels"],
        }
        for cam in cfg.cameras
    }
    features[cfg.state_key] = {
        "dtype": "float32",
        "shape": (cfg.state_dim,),
        "names": [f"state_{i}" for i in range(cfg.state_dim)],
    }
    features[cfg.action_key] = {
        "dtype": "float32",
        "shape": (cfg.action_dim,),
        "names": [f"action_{i}" for i in range(cfg.action_dim)],
    }
    return features


def open_dataset(cfg):
    """Create a fresh dataset, or load an existing one to APPEND to — so a new
    recording session accumulates episodes instead of failing on an existing root
    (LeRobotDataset.create refuses a non-empty root)."""
    if (cfg.repo_root / "meta" / "info.json").exists():
        return LeRobotDataset(cfg.repo_id, root=cfg.repo_root, streaming_encoding=cfg.streaming_encoding)
    return LeRobotDataset.create(
        repo_id=cfg.repo_id,
        fps=cfg.fps,
        features=build_features(cfg),
        root=cfg.repo_root,
        use_videos=True,
        image_writer_threads=cfg.image_writer_threads,
        streaming_encoding=cfg.streaming_encoding,
    )


def run(node: Node, cfg: RecorderConfig, dataset) -> None:
    """Record until shutdown, then ALWAYS flush an in-progress episode on any exit
    path (dora STOP, program_state stop, or error) via the mode's `close()`."""
    mode = MODES[cfg.mode](cfg, node, dataset)  # KeyError on a bad MODE = loud
    mode.start()
    try:
        for event in node:
            if event["type"] == "STOP":
                break
            if event["type"] != "INPUT":
                continue
            if mode.handle(event):  # True -> program_state stop
                break
    finally:
        mode.close()


def main() -> int:
    cfg = load_config()
    dataset = open_dataset(cfg)
    node = Node()
    run(node, cfg, dataset)
    return 0


if __name__ == "__main__":
    sys.exit(main())
