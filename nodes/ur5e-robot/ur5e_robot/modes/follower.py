"""`follower` mode — drive the UR5e to streamed JOINT setpoints.

The node's only mode. Joint control only: pinocchio owns IK + collision safety and
emits ``<name>_joint_target`` (6 arm joints), which this mode streams to the arm via
the driver's ``servo_j`` (UR ``servoJ``). The gripper part target
(``<name>_gripper_joint_target``, the 2F-85 driver-joint opening in radians) drives
the Robotiq gripper. There is NO Cartesian / ``servoL`` / ``tcp_target`` path — the
node never converts a pose into a command.

On each `tick` it reads and publishes the arm's measured state (the TCP pose is read
back only as feedback for the web UI / Rerun). A target whose largest joint jump
exceeds ``max_joint_jump`` from the current measured joints is rejected (a last-resort
backstop; pinocchio already bounds the step). Nothing is commanded until the first
state read. On shutdown (STOP / program_state stop / operator Disconnect) the arm is stopped
smoothly in place and released (the UR controller holds the pose — no surprise motion).

Inputs:  <name>_joint_target, <name>_gripper_joint_target, tick,
  robot_command, program_state.
Outputs: <name>_tcp_pose, <name>_joint_state, <name>_gripper_state, <name>_node_state.

This module imports no `ur_rtde`: the driver is injected (see main.py), so it stays
importable on hosts without the native SDK (the unit tests run there).
"""

from __future__ import annotations

import logging
import time
from typing import Callable

import numpy as np
import pyarrow as pa
from dora import Node
from pydantic import BaseModel

from ur5e_robot.conversions import tcp_to_pose7

logger = logging.getLogger("ur5e-robot")

class FollowerConfig(BaseModel):
    # Reject a joint target whose largest joint jump exceeds this (rad) from the
    # current measured joints — a backstop (so a connect/stream glitch can't make the arm
    # leap); pinocchio is the real IK + safety owner.
    max_joint_jump: float = 0.8
    # Whether THIS node drives the Robotiq gripper (over the UR controller's URCap socket).
    # Set false when the gripper is wired elsewhere (e.g. directly to the PC) or absent: the
    # arm then connects + runs normally, gripper commands are ignored, and gripper_state is
    # reported as a constant (open) so the state bundle keeps its shape.
    with_gripper: bool = True
    # servoJ execution params (used by the driver): control period, lookahead, gain.
    servo_time: float = 0.033        # s; ~the dora tick period
    servo_lookahead: float = 0.1     # s; servoJ smoothing horizon (0.03..0.2)
    servo_gain: float = 300.0        # servoJ proportional gain (100..2000)
    # Robotiq 2F-85 params (used by the driver): move speed/force (0..255) and the
    # driver-joint range (rad) that maps to the gripper's 0..255 position.
    gripper_speed: int = 255
    gripper_force: int = 100
    gripper_max_rad: float = 0.8     # descriptor `closed`; 0 rad = open, this = closed


class FollowerMode:
    def __init__(self, cfg: FollowerConfig, node: Node, driver, name: str):
        self.cfg = cfg
        self.node = node
        self.driver = driver
        self.name = name
        self.status = ""
        self.last_joints: list[float] | None = None  # measured each tick (jump guard)
        self._disconnected = False  # operator Disconnect already stopped+released the arm
        self._handlers: dict[str, Callable] = {
            "program_state": self._on_program_state,
            "robot_command": self._on_command,
            "tick": self._on_tick,
            f"{name}_joint_target": self._on_joint_target,
            f"{name}_gripper_joint_target": self._on_gripper_target,
        }

    def start(self) -> None:
        logger.info("%s: follower connected", self.name)
        self._emit_status("ready")

    def handle(self, event) -> bool:
        return bool(self._handlers[event["id"]](event))  # KeyError on an unwired input id = loud

    def close(self) -> None:
        if not self._disconnected:  # never release twice
            self.driver.stop_and_release()

    def _emit_status(self, text: str) -> None:
        self.status = text
        self.node.send_output(f"{self.name}_node_state", pa.array([text]))

    def _on_program_state(self, event) -> bool:
        return event["value"][0].as_py() == "disconnect"

    def _on_command(self, event) -> bool:
        if event["value"][0].as_py() == "disconnect":
            self.driver.stop_and_release()
            self._disconnected = True
            return True  # exit -> manager sees us gone -> stops the rest
        return False


    def _on_tick(self, event) -> None:
        md = {"timestamp": time.time()}
        joints = list(self.driver.get_actual_q())          # 6 arm joints (rad)
        tcp = list(self.driver.get_actual_tcp_pose())      # [x,y,z, rx,ry,rz]
        gripper = float(self.driver.get_gripper_position())  # driver-joint opening (rad)
        self.last_joints = joints
        self.node.send_output(f"{self.name}_tcp_pose", pa.array(tcp_to_pose7(tcp)), metadata=md)
        self.node.send_output(f"{self.name}_joint_state", pa.array(joints), metadata=md)
        self.node.send_output(f"{self.name}_gripper_state", pa.array([gripper]), metadata=md)

    def _on_joint_target(self, event) -> None:
        joints = event["value"].to_numpy()  # 6 arm joints (rad)
        if self.last_joints is None:
            return  # no state read yet — never command blind
        jump = float(np.max(np.abs(np.subtract(joints, self.last_joints))))
        if jump > self.cfg.max_joint_jump:
            msg = f"{self.name}: rejected joint target ({jump:.3f} rad jump)"
            logger.warning(msg)
            self._emit_status(msg)
            return
        self.driver.servo_j(joints.tolist())

    def _on_gripper_target(self, event) -> None:
        opening = float(event["value"][0].as_py())  # 2F-85 driver-joint opening (rad)
        self.driver.gripper_move(opening)
