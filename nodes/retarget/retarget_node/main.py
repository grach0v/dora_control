"""Retarget node — dora skeleton.

Event-driven inputs (leader TCP/gripper, follower measured pose — cache latest), tick-driven
output (the follower's cartesian `command` bundle). The behaviour lives in the `follow` mode;
``MODE`` selects it (only `follow` today).
"""

from __future__ import annotations

import logging
import sys

from dora import Node

from retarget_node.modes import MODES
from retarget_node.node_config import load_config


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    cfg = load_config()
    node = Node()
    mode = MODES[cfg.mode](cfg, node)  # KeyError on a bad MODE = loud
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
    return 0


if __name__ == "__main__":
    sys.exit(main())
