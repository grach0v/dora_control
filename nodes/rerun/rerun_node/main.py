"""Rerun node — record robot streams to .rrd, or visualize them live.

One node, two modes selected by ``MODE`` (see modes/record.py and
modes/visualize.py). This file owns only the dora skeleton: load config, build the
mode, run the event loop, and always tear the mode down. Each mode is a small
state machine that owns its own background resources (video encoders for record; a
viewer sink + logging worker thread for visualize), so this loop is mode-agnostic.

Messages are plain Arrow (see docs/message_formats.md). Per mode I/O is documented
in the mode files.
"""

from __future__ import annotations

import sys

from dora import Node

from rerun_node.modes import MODES
from rerun_node.node_config import load_config


def main() -> int:
    cfg = load_config()
    node = Node()
    mode = MODES[cfg.mode](cfg.active, node, cfg.app_id)  # KeyError on a bad MODE = loud
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
