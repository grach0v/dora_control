"""Sync node — dora skeleton.

Event-driven inputs (cache latest), tick-driven output (emit the concatenated bundle). The
behaviour lives in the `collect` mode. ``MODE`` selects it (only `collect` today).
"""

from __future__ import annotations

import logging
import sys

from dora import Node

from sync_node.modes import MODES
from sync_node.node_config import load_config


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
