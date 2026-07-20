"""Trossen real-robot node — one arm of the stationary Trossen AI kit.

Drives a single WXAI arm over Ethernet via the trossen-arm SDK and mirrors
the sim nodes' per-arm stream interface so sim and real are swappable. Bimanual
hardware runs two of these nodes (NAME=left, NAME=right). Driver calls are
millisecond-scale RPCs, so everything runs on the dora loop — no background thread.

This file owns only the dora skeleton: connect the arm, run the event loop, and
always tear the mode down (which homes/releases the arm). What each input *means*
lives in a mode (modes/follower.py, modes/leader.py) selected by ``MODE``.
"""

from __future__ import annotations

import logging
import sys

from dora import Node

# Re-exported for the tests, which import these from `trossen_robot.main`.
from trossen_robot.conversions import cartesian_to_pose7, pose7_to_cartesian  # noqa: F401
from trossen_robot.driver import make_driver
from trossen_robot.modes import MODES
from trossen_robot.node_config import load_config


def run(node: Node, mode) -> None:
    """Drive the arm until shutdown, then ALWAYS tear the mode down (home/release)
    on any exit path — dora STOP, program_state stop, or error — unless the mode already
    disconnected (operator Disconnect), so the arm is never left energized or homed
    twice. The loop reacts to STOP/program_state promptly (driver RPCs are ms-scale), so
    dora never needs to escalate to SIGTERM and the finally always reaches close()."""
    mode.start()
    try:
        for event in node:
            if event["type"] == "STOP":
                break
            if event["type"] != "INPUT":
                continue
            if mode.handle(event):  # True -> program_state stop / operator disconnect
                break
    finally:
        mode.close()


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    cfg = load_config()
    driver = make_driver(cfg.mode, cfg.ip)
    node = Node()
    mode = MODES[cfg.mode](cfg.active, node, driver, cfg.name)  # KeyError on a bad MODE = loud
    run(node, mode)
    return 0


if __name__ == "__main__":
    sys.exit(main())
