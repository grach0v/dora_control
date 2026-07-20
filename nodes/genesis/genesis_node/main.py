"""Genesis sim node — dora skeleton.

A tick-driven producer: physics + rendering live on a background worker thread (see
sim.py); the loop drains commands, and PUBLISHES the latest state + cameras only when a
`tick` arrives. ``MODE`` selects the state machine (only ``sim`` today).
"""

from __future__ import annotations

import logging
import sys

from dora import Node

from genesis_node.modes import MODES
from genesis_node.node_config import load_config


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    cfg = load_config()
    node = Node()
    mode = MODES[cfg.mode](cfg, node)  # KeyError on a bad MODE = loud
    mode.start()
    try:
        while True:
            event = node.next(timeout=0.005)
            if event is not None:
                if event["type"] == "STOP":
                    break
                if event["type"] == "INPUT":
                    if mode.handle(event):  # True -> program_state stop
                        break
            mode.step()
            mode.maybe_publish()
    finally:
        mode.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
