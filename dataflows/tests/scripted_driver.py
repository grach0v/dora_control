"""Headless test driver — stands in for the (browser-driven) web-controller.

Emits the whole-robot `command` bundle (the descriptor's `cartesian` command layout:
left pose7 | left_gripper 1 | right pose7 | right_gripper 1) plus one episode `start`, then
on every tick gently oscillates the arms' z + the grippers so the recorded data varies. Same
node id + relevant outputs as web-controller's manual mode, so the rest of the flow is unchanged.

Run via the web-controller venv's python (it has dora + pyarrow).
"""

from __future__ import annotations

import math
import time

import pyarrow as pa
from dora import Node

# Reachable resting TCP poses [x,y,z, qx,qy,qz,qw] (xyzw), matching the stationary home.
HOMES = {
    "left": [-0.02, 0.195, 0.31, 0.0, 0.0, 0.7071, -0.7071],
    "right": [-0.02, -0.195, 0.31, 0.0, 0.0, 0.7071, 0.7071],
}
AMPLITUDE = 0.04   # metres, z sweep
PERIOD = 2.0       # seconds
GRIPPER_OPEN = 0.044
GRIPPER_CLOSED = 0.0


def main() -> None:
    node = Node()
    t0 = time.monotonic()
    started = False
    for event in node:
        if event["type"] == "STOP":
            break
        if event["type"] != "INPUT":
            continue
        if event["id"] == "program_state":
            if event["value"][0].as_py() == "disconnect":
                break
            continue
        if event["id"] != "tick":
            continue

        now = time.monotonic()
        md = {"timestamp": time.time()}
        if not started:
            node.send_output("episode_control", pa.array(["task=headless smoke"]), metadata=md)
            node.send_output("episode_control", pa.array(["start"]), metadata=md)
            started = True

        phase = math.sin(2.0 * math.pi * (now - t0) / PERIOD)
        dz = AMPLITUDE * phase
        grip = 0.5 * (GRIPPER_OPEN + GRIPPER_CLOSED) + 0.5 * (GRIPPER_OPEN - GRIPPER_CLOSED) * phase
        left = [*HOMES["left"][:2], HOMES["left"][2] + dz, *HOMES["left"][3:]]
        right = [*HOMES["right"][:2], HOMES["right"][2] + dz, *HOMES["right"][3:]]
        # cartesian command layout order: left pose7 | left_gripper 1 | right pose7 | right_gripper 1
        command = [*left, grip, *right, grip]
        node.send_output("command", pa.array(command, type=pa.float64()), metadata=md)


if __name__ == "__main__":
    main()
