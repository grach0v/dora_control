"""Manager node — owns the program lifecycle as an explicit state machine.

Every other node emits its logical `node_state` (edge-triggered tokens, see
docs/message_formats.md); the manager reports each event to its program state
machine, which advances itself and broadcasts `program_state` on every state
entry. No heartbeat / liveness — dora detects node death. The machine lives in
modes/; `MODE` selects it.

Inputs:  one input per producer, wired to its `node_state` (plus e.g. the
         controller's `robot_command`).
Outputs: program_state (e.g. boot / homing / teleop / disconnect).
"""

from __future__ import annotations

import logging
import sys

from dora import Node

from manager.modes import MODES
from manager.node_config import load_config


def run(node, machine) -> None:
    """Report each input to the machine: node A said token B. Nothing else."""
    for event in node:
        if event["type"] == "STOP":
            return
        if event["type"] != "INPUT":
            continue
        machine.observe(event["id"], event["value"][0].as_py())
        if machine.is_terminated:
            return


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    cfg = load_config()
    node = Node()
    machine = MODES[cfg.mode](list(cfg.producers), node)  # KeyError on a bad MODE = loud
    run(node, machine)
    return 0


if __name__ == "__main__":
    sys.exit(main())
