"""Pinocchio IK + safety node — dora skeleton.

A transform: it owns no hardware and no background thread. The event loop hands each
input to the mode, which solves + gates + publishes immediately (see modes/control.py).
``MODE`` selects the state machine (only ``control`` today).
"""

from __future__ import annotations

import logging
import sys

from dora import Node

from pinocchio_node.modes import MODES
from pinocchio_node.node_config import load_config


def run(node: Node, mode) -> None:
    """Event loop, with the mode torn down on every exit path (STOP / program_state / error)."""
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
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    cfg = load_config()
    node = Node()
    mode = MODES[cfg.mode](cfg, node)  # KeyError on a bad MODE = loud
    run(node, mode)
    return 0


if __name__ == "__main__":
    sys.exit(main())
