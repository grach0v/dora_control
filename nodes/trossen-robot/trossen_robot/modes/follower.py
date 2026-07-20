"""`follower` mode — drive the arm to streamed setpoints.

The default mode. On each `tick` it reads and publishes the arm's state; setpoints stream
to the arm as they arrive, and nothing is commanded until the first state read. Two command
spaces (``CONTROL_SPACE``):

- ``cartesian`` (default): consume ``<name>_tcp_target`` and let the arm firmware do
  Cartesian IK; a target farther than ``max_pos_jump`` from the current TCP is rejected.
- ``joint``: consume ``<name>_joint_target`` (from the pinocchio node) and command joints
  directly via ``set_all_positions``; a target whose largest joint jump exceeds
  ``max_joint_jump`` is rejected. In this mode Pinocchio owns IK + collision safety; this
  per-joint guard is only a last-resort backstop.

On `command` = disconnect (or any shutdown) the arm is folded to its sleep pose and released.

Inputs:  <name>_tcp_target | <name>_joint_target, <name>_gripper_target, tick,
  robot_command, program_state.
Outputs: <name>_tcp_pose, <name>_joint_state, <name>_gripper_state, <name>_node_state.
"""

from __future__ import annotations

import logging
import math
import time
from typing import Callable, Literal

import numpy as np
import pyarrow as pa
import trossen_arm
from dora import Node
from pydantic import BaseModel, Field

from trossen_robot.conversions import cartesian_to_pose7, pose7_to_cartesian
from trossen_robot.driver import home_and_release

logger = logging.getLogger("trossen-robot")

# Folded-but-raised pose, then all-zero sleep: moving STAGED -> SLEEP under torque
# folds the arm down rather than letting it drop (6 arm joints + gripper).
DEFAULT_STAGED = [0.0, math.pi / 3, math.pi / 6, math.pi / 5, 0.0, 0.0, 0.0]
DEFAULT_SLEEP = [0.0] * 7


class FollowerConfig(BaseModel):
    # Horizon (s) for streamed setpoints: 0 = immediate, <=0.2 linear, else quintic.
    goal_time: float = 0.0
    # Command space. `cartesian` (default): consume <name>_tcp_target and let the arm
    # firmware do Cartesian IK. `joint`: consume <name>_joint_target (from pinocchio) and
    # command joints directly — Pinocchio is then the sole owner of IK + collision safety,
    # so the firmware's Cartesian guard no longer applies (the per-joint guard below is a
    # cheap last-resort backstop). See the pinocchio node.
    control_space: Literal["cartesian", "joint"] = "cartesian"
    # Reject a TCP target farther than this (metres) from the arm's current TCP (cartesian).
    max_pos_jump: float = 0.10
    # Reject a joint target whose largest joint jump exceeds this (rad) from the current
    # measured joints (joint mode backstop).
    max_joint_jump: float = 0.5
    # Graceful-disconnect poses (6 arm joints + gripper).
    staged_pose: list[float] = Field(default_factory=lambda: list(DEFAULT_STAGED))
    sleep_pose: list[float] = Field(default_factory=lambda: list(DEFAULT_SLEEP))


class FollowerMode:
    def __init__(self, cfg: FollowerConfig, node: Node, driver, name: str):
        self.cfg = cfg
        self.node = node
        self.driver = driver
        self.name = name
        self.status = ""
        # State read from the arm every tick — used by the jump guards / joint assembly.
        self.last_cart: list[float] | None = None
        self.last_joints: list[float] | None = None
        self.last_gripper: float | None = None
        self._disconnected = False  # operator Disconnect already homed the arm
        self._handlers: dict[str, Callable] = {
            "program_state": self._on_program_state,
            "robot_command": self._on_command,   # operator control (disconnect)
            "tick": self._on_tick,
        }
        # One command path, chosen by control_space: Cartesian (firmware IK) or joint (from
        # pinocchio). The gripper arrives as this arm's gripper PART target in joint mode.
        if cfg.control_space == "joint":
            self._handlers[f"{name}_joint_target"] = self._on_joint_target
            self._handlers[f"{name}_gripper_joint_target"] = self._on_gripper_target
        else:
            self._handlers[f"{name}_tcp_target"] = self._on_tcp_target
            self._handlers[f"{name}_gripper_target"] = self._on_gripper_target

    def start(self) -> None:
        logger.info("%s: follower connected", self.name)
        self._emit_status("ready")

    def _emit_status(self, text: str) -> None:
        self.status = text
        self.node.send_output(f"{self.name}_node_state", pa.array([text]))

    def handle(self, event) -> bool:
        return bool(self._handlers[event["id"]](event))  # KeyError on an unwired input id = loud

    def close(self) -> None:
        if not self._disconnected:  # never home twice
            home_and_release(self.name, self.driver, self.cfg.staged_pose, self.cfg.sleep_pose)

    def _on_program_state(self, event) -> bool:
        return event["value"][0].as_py() == "disconnect"

    def _on_command(self, event) -> bool:
        if event["value"][0].as_py() == "disconnect":
            home_and_release(self.name, self.driver, self.cfg.staged_pose, self.cfg.sleep_pose)
            self._disconnected = True
            return True  # exit -> manager sees us gone -> stops the rest
        return False


    def _on_tick(self, event) -> None:
        md = {"timestamp": time.time()}
        cart = list(self.driver.get_cartesian_positions())
        joints = list(self.driver.get_all_positions())  # 6 arm joints (rad) + gripper (m)
        self.last_cart = cart
        self.last_joints = joints[:6]
        if self.last_gripper is None:  # seed the commanded gripper from the real opening
            self.last_gripper = joints[6]
        self.node.send_output(f"{self.name}_tcp_pose", pa.array(cartesian_to_pose7(cart)), metadata=md)
        self.node.send_output(f"{self.name}_joint_state", pa.array(joints[:6]), metadata=md)
        self.node.send_output(f"{self.name}_gripper_state", pa.array([joints[6]]), metadata=md)

    def _on_joint_target(self, event) -> None:
        joints = event["value"].to_numpy()  # n arm joints (rad)
        if self.last_joints is None:
            return  # no state read yet — never command blind
        jump = float(np.max(np.abs(np.subtract(joints, self.last_joints))))
        if jump > self.cfg.max_joint_jump:
            msg = f"{self.name}: rejected joint target ({jump:.3f} rad jump)"
            logger.warning(msg)
            self._emit_status(msg)
            return
        grip = self.last_gripper if self.last_gripper is not None else 0.0
        self.driver.set_all_positions([*joints.tolist(), grip], self.cfg.goal_time, False)

    def _on_tcp_target(self, event) -> None:
        cart = pose7_to_cartesian(event["value"].to_numpy())
        if self.last_cart is None:
            return  # no state read yet — never command blind
        jump = float(np.linalg.norm(np.subtract(cart[:3], self.last_cart[:3])))
        if jump > self.cfg.max_pos_jump:
            msg = f"{self.name}: rejected target {jump:.3f}m from current TCP"
            logger.warning(msg)
            self._emit_status(msg)
            return
        self.driver.set_cartesian_positions(
            cart, trossen_arm.InterpolationSpace.cartesian, self.cfg.goal_time, False
        )

    def _on_gripper_target(self, event) -> None:
        opening = float(event["value"][0].as_py())
        self.last_gripper = opening
        # In joint mode the gripper rides along with the next set_all_positions.
        if self.cfg.control_space == "cartesian":
            self.driver.set_gripper_position(opening, self.cfg.goal_time, False)
