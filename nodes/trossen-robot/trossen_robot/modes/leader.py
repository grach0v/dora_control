"""`leader` mode — a backdrivable teleop source with a staged startup pose.

On connect the arm is DRIVEN to a configurable ``staged_pose`` under position control,
then switched to backdrivable external-effort with zero commanded efforts (= pure gravity
compensation, so it hangs weightless in the operator's hand) — mirroring the
lerobot_trossen leader's ``configure()``. On each `tick` the hand-moved pose is read and
published as **state** (`<name>_tcp_pose`, `<name>_joint_state`, `<name>_gripper_state`)
— identical to a follower or the sim. It has no target handler — a target arriving at a
leader is a wiring bug and raises.

Shutdown (STOP / program_state stop / operator Disconnect) mirrors startup: back to position
control, fold staged → sleep under torque, then release — so the arm is parked, never
dropped.

Inputs:  tick, robot_command, program_state.
Outputs: <name>_tcp_pose, <name>_joint_state, <name>_gripper_state, <name>_node_state.
"""

from __future__ import annotations

import logging
import time
from typing import Callable

import pyarrow as pa
import trossen_arm
from dora import Node
from pydantic import BaseModel, Field

from trossen_robot.conversions import cartesian_to_pose7
from trossen_robot.driver import home_and_release
from trossen_robot.modes.follower import DEFAULT_SLEEP, DEFAULT_STAGED

logger = logging.getLogger("trossen-robot")


class LeaderConfig(BaseModel):
    # Startup pose (6 arm joints rad + gripper m): the arm is driven here on connect,
    # before going backdrivable. Env: LEADER__STAGED_POSE='[0,1.047,0.524,0.628,0,0,0]'.
    staged_pose: list[float] = Field(default_factory=lambda: list(DEFAULT_STAGED))
    # Disconnect path: staged -> sleep under torque, then release (same as the follower).
    sleep_pose: list[float] = Field(default_factory=lambda: list(DEFAULT_SLEEP))
    # Seconds per staged/sleep move.
    goal_time: float = 2.0


class LeaderMode:
    def __init__(self, cfg: LeaderConfig, node: Node, driver, name: str):
        self.cfg = cfg
        self.node = node
        self.driver = driver
        self.name = name
        self.status = ""
        self._released = False
        self._handlers: dict[str, Callable] = {
            "program_state": self._on_program_state,
            "robot_command": self._on_command,   # operator control (disconnect)
            "tick": self._on_tick,
        }
        # No *_target handler: a target at a leader is a wiring bug -> raises.

    def start(self) -> None:
        # Stage under position control (the driver connects in position mode), then go
        # backdrivable with zero external efforts = gravity compensation.
        self.driver.set_all_positions(self.cfg.staged_pose, self.cfg.goal_time, True)
        self.driver.set_all_modes(trossen_arm.Mode.external_effort)
        self.driver.set_all_external_efforts([0.0] * len(self.cfg.staged_pose), 0.0, True)
        logger.info("%s: leader staged, backdrivable", self.name)
        self._emit_status("ready")

    def _emit_status(self, text: str) -> None:
        self.status = text
        self.node.send_output(f"{self.name}_node_state", pa.array([text]))

    def handle(self, event) -> bool:
        return bool(self._handlers[event["id"]](event))  # KeyError on an unwired input id = loud

    def close(self) -> None:
        if not self._released:
            self._park()

    def _park(self) -> None:
        """Back under torque, fold staged -> sleep, release (never drop the arm)."""
        self.driver.set_all_modes(trossen_arm.Mode.position)
        home_and_release(self.name, self.driver, self.cfg.staged_pose, self.cfg.sleep_pose)
        self._released = True

    def _on_program_state(self, event) -> bool:
        return event["value"][0].as_py() == "disconnect"

    def _on_command(self, event) -> bool:
        if event["value"][0].as_py() == "disconnect":
            self._park()
            return True
        return False


    def _on_tick(self, event) -> None:
        md = {"timestamp": time.time()}
        cart = list(self.driver.get_cartesian_positions())
        joints = list(self.driver.get_all_positions())  # 6 arm joints (rad) + gripper (m)
        self.node.send_output(f"{self.name}_tcp_pose", pa.array(cartesian_to_pose7(cart)), metadata=md)
        self.node.send_output(f"{self.name}_joint_state", pa.array(joints[:6]), metadata=md)
        self.node.send_output(f"{self.name}_gripper_state", pa.array([joints[6]]), metadata=md)
