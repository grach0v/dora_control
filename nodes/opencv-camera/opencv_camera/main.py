"""OpenCV camera node.

Acquisition runs on a **background thread** that reads frames as the device
delivers them (capped at `FPS`) and keeps only the latest frame with its capture
timestamp. The dora loop polls (`node.next(timeout=…)`) and publishes each
captured frame exactly once, so the wire rate IS the configured FPS — no tick,
no timer-phase latency. The capture `timestamp` travels in metadata so a
consumer can align streams (see docs/message_formats.md).

This file owns only the dora skeleton: build the mode (which owns the background
`CameraReader`), run the event loop, and always tear the mode down. What each
input *means* lives in a mode (modes/stream.py) selected by ``MODE``.

Inputs:
  program_state      manager/program_state; stops the node on `disconnect`
Outputs:
  <name>_image       uint8[N] + metadata {encoding, width, height, timestamp}
  <name>_node_state  edge-triggered state token (see docs/message_formats.md)
"""

from __future__ import annotations

import sys

from dora import Node

from opencv_camera.modes import MODES
from opencv_camera.node_config import load_config


def run(node: Node, mode) -> None:
    # Self-paced producer: poll for events with a short timeout and publish each
    # newly captured frame in between — the configured FPS paces the wire.
    mode.start()
    try:
        while True:
            event = node.next(timeout=0.005)
            if event is not None:
                if event["type"] == "STOP":
                    break
                if event["type"] == "INPUT" and mode.handle(event):  # True -> program_state stop
                    break
            if mode.maybe_publish():  # True -> capture ended
                break
    finally:
        mode.close()


def main() -> int:
    cfg = load_config()
    node = Node()
    mode = MODES[cfg.mode](cfg, node)  # KeyError on a bad MODE = loud
    run(node, mode)
    return 0


if __name__ == "__main__":
    sys.exit(main())
