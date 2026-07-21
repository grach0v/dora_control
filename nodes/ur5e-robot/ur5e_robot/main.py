"""UR5e real-robot node — one arm of the bimanual UR5e workstation.

Drives a single UR5e over RTDE (`ur_rtde`) with a Robotiq 2F-85, mirroring the sim
nodes' per-arm stream interface so sim and real are swappable. Bimanual hardware runs
two of these nodes (NAME=left, NAME=right). Joint control only: pinocchio owns IK +
collision safety and sends per-arm joint targets; this node streams them via servoJ.

This file owns the dora skeleton + the `ur_rtde`-bound driver: connect the arm, build
the mode (injecting the driver, so the mode stays SDK-free), run the event loop, and
always tear the mode down (which stops/releases the arm). The loop lives in loop.py,
which imports no `ur_rtde` — the mode/loop stay importable on hosts without the SDK.

Inputs:  <name>_joint_target, <name>_gripper_joint_target, robot_command, tick,
  program_state.
Outputs: <name>_tcp_pose, <name>_joint_state, <name>_gripper_state, <name>_node_state.
"""

from __future__ import annotations

import logging
import sys

from dora import Node

from ur5e_robot.driver import make_driver
from ur5e_robot.loop import run
from ur5e_robot.modes import MODES
from ur5e_robot.node_config import load_config


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    cfg = load_config()
    driver = make_driver(cfg.ip, cfg.active)
    node = Node()
    mode = MODES[cfg.mode](cfg.active, node, driver, cfg.name)  # KeyError on a bad MODE = loud
    run(node, mode)
    return 0


if __name__ == "__main__":
    sys.exit(main())
