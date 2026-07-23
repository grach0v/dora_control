"""Headless motion driver for remote-teleop QoS testing (test-only).

Like scripted_driver.py it stands in for the web-controller, but instead of
hardcoded poses it ANCHORS to pinocchio's measured poses (exactly like the real
web-controller's manual mode) — safe on any cell: waits for the teleop stage and
one measured pose per arm, then oscillates each arm's z around its anchor and
sweeps the grippers.

Env: AMPLITUDE (m, default 0.03), PERIOD (s, default 2.5).

Inputs:  tick, program_state, left_tcp_pose, right_tcp_pose
Outputs: command, episode_control, robot_command, node_state
"""

from __future__ import annotations

import math
import os
import time

import pyarrow as pa
from dora import Node

GRIPPER_OPEN = 0.044
GRIPPER_CLOSED = 0.0


def main() -> None:
    amplitude = float(os.environ.get("AMPLITUDE", "0.03"))
    period = float(os.environ.get("PERIOD", "2.5"))

    node = Node()
    anchors: dict[str, list[float]] = {}
    stage = "boot"
    t0 = None
    for event in node:
        if event["type"] == "STOP":
            break
        if event["type"] != "INPUT":
            continue
        if event["id"] == "program_state":
            stage = event["value"][0].as_py()
            if stage == "disconnect":
                break
            continue
        if event["id"] in ("left_tcp_pose", "right_tcp_pose"):
            side = event["id"].split("_")[0]
            if side not in anchors:
                anchors[side] = event["value"].to_pylist()
            continue
        if event["id"] != "tick":
            continue
        if stage != "teleop" or len(anchors) < 2:
            continue

        now = time.monotonic()
        if t0 is None:
            t0 = now
        phase = math.sin(2.0 * math.pi * (now - t0) / period)
        dz = amplitude * phase
        grip = 0.5 * (GRIPPER_OPEN + GRIPPER_CLOSED) + 0.5 * (GRIPPER_OPEN - GRIPPER_CLOSED) * phase
        left = [*anchors["left"][:2], anchors["left"][2] + dz, *anchors["left"][3:]]
        right = [*anchors["right"][:2], anchors["right"][2] + dz, *anchors["right"][3:]]
        command = [*left, grip, *right, grip]
        node.send_output("command", pa.array(command, type=pa.float64()),
                         metadata={"timestamp": time.time()})


if __name__ == "__main__":
    main()
