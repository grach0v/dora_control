"""`follow` mode — retarget a LEADER robot's hand-guided TCP motion onto a FOLLOWER robot.

The leader (e.g. two backdrivable Trossen WXAI arms) and the follower (e.g. two UR5e) are
different robots in different places with different sizes, so absolute TCP mapping is
meaningless. The mapping is DELTA-based: per arm, once both the leader's TCP pose and the
follower's measured ee pose (pinocchio's model-frame FK) have arrived, the pair is ENGAGED —
the leader pose L0 and follower pose F0 are latched — and from then on the leader's
displacement is mapped into the follower workspace:

    p_target = p_F0 + scale · A (p_L − p_L0)
    R_target = (A · R_L R_L0⁻¹ · A⁻¹) · R_F0

where A (`align_rpy`) is the fixed rotation expressing leader-base-frame directions in the
follower world frame. Translation is scaled (the robots differ in size); rotation deltas
apply 1:1. Gripper openings map linearly from the leader's configured range onto the follower
part's descriptor open/closed range.

The wire stays the ordinary ABSOLUTE `command` bundle of the follower's cartesian layout —
retargeting is relative, the message is idempotent-absolute, so a dropped or duplicated packet
never accumulates error. The commanded target's per-tick motion is clamped
(max_pos_step/max_rot_step): a leader jump (bumped arm, torque-off flop) becomes a bounded
glide, on top of pinocchio's joint rate-limit + collision/plane gate downstream. Until EVERY
arm part is engaged nothing is emitted — never command blind.

Inputs:  tick, program_state, `<arm>_tcp_pose` + `<arm>_gripper_state` (leader state),
  `<arm>_measured_pose` (follower FK — the engage anchor).
Outputs: command (float64 bundle), status.
"""

from __future__ import annotations

import logging
import time
from functools import partial
from pathlib import Path
from typing import Callable

import numpy as np
import pyarrow as pa
import yaml
from dora import Node
from scipy.spatial.transform import Rotation

from retarget_node.node_config import RetargetConfig

logger = logging.getLogger("retarget")


def load_descriptor(scene_path: str) -> dict:
    return yaml.safe_load(Path(scene_path).resolve().read_text())


class Engagement:
    """The latched reference pair of one arm: leader L0 (its base frame) and follower F0
    (follower world frame), plus the clamped target the node is currently commanding."""

    def __init__(self, leader0: np.ndarray, follower0: np.ndarray):
        self.p_l0 = leader0[:3].copy()
        self.r_l0 = Rotation.from_quat(leader0[3:7])
        self.p_f0 = follower0[:3].copy()
        self.r_f0 = Rotation.from_quat(follower0[3:7])
        self.p_target = self.p_f0.copy()
        self.r_target = self.r_f0


class FollowMode:
    def __init__(self, cfg: RetargetConfig, node: Node):
        self.cfg = cfg
        self.node = node
        desc = load_descriptor(cfg.scene)
        self.layout = desc["command_layouts"][cfg.command_layout]
        self.arm_parts = [e["part"] for e in self.layout if e["space"] == "cartesian"]
        self.gripper_parts = [e["part"] for e in self.layout if e["space"] != "cartesian"]
        # A gripper part rides its arm's leader gripper stream: part `left_gripper` consumes
        # input `left_gripper_state`. The follower's value range comes from the descriptor.
        self.gripper_bounds = {
            p: (float(desc["parts"][p]["open"]), float(desc["parts"][p]["closed"]))
            for p in self.gripper_parts
        }
        self.align = Rotation.from_euler("xyz", cfg.align_rpy)

        self.leader_pose: dict[str, np.ndarray | None] = {p: None for p in self.arm_parts}
        self.follower_pose: dict[str, np.ndarray | None] = {p: None for p in self.arm_parts}
        self.engaged: dict[str, Engagement | None] = {p: None for p in self.arm_parts}
        self.gripper: dict[str, float | None] = {p: None for p in self.gripper_parts}
        self.status = ""
        # Operating stage from the manager's program_state: retargeting is ACTIVE only in
        # `teleop`. Any other stage — including the window before the manager's first
        # broadcast, and `homing` (pinocchio ramping the followers home) — drops the
        # engage anchors, so re-engagement always latches against the settled pose,
        # never mid-motion and never before the program is up.
        self.active_stages = ("teleop",)
        self.stage = "start"

        self._handlers: dict[str, Callable] = {
            "tick": self._on_tick,
            "program_state": self._on_program_state,
        }
        for part in self.arm_parts:
            self._handlers[f"{part}_tcp_pose"] = partial(self._on_leader_pose, part)
            self._handlers[f"{part}_measured_pose"] = partial(self._on_follower_pose, part)
        for part in self.gripper_parts:
            arm = part.removesuffix("_gripper")
            self._handlers[f"{arm}_gripper_state"] = partial(self._on_leader_gripper, part)

    def start(self) -> None:
        self._set_status("ready")

    def handle(self, event) -> bool:
        return bool(self._handlers[event["id"]](event))  # KeyError on an unwired input id = loud

    def close(self) -> None:
        pass

    # --- program_state / status ------------------------------------------------------------

    def _on_program_state(self, event) -> bool:
        value = event["value"][0].as_py()
        if value == "disconnect":
            return True
        if value != self.stage:
            self.stage = value
            if value not in self.active_stages:
                self.engaged = {p: None for p in self.arm_parts}  # re-anchor on return
        return False


    def _set_status(self, text: str) -> None:
        if text != self.status:
            self.status = text
            self.node.send_output("node_state", pa.array([text]))

    # --- inputs (cache latest) ----------------------------------------------------------

    def _on_leader_pose(self, part: str, event) -> None:
        self.leader_pose[part] = event["value"].to_numpy().astype(float)

    def _on_follower_pose(self, part: str, event) -> None:
        self.follower_pose[part] = event["value"].to_numpy().astype(float)

    def _on_leader_gripper(self, part: str, event) -> None:
        lo, lc = self.cfg.leader_gripper_open, self.cfg.leader_gripper_closed
        fo, fc = self.gripper_bounds[part]
        openness = (float(event["value"][0].as_py()) - lc) / (lo - lc)  # 1 = open, 0 = closed
        value = fc + max(0.0, min(1.0, openness)) * (fo - fc)
        self.gripper[part] = value

    # --- tick: engage, retarget, clamp, emit ---------------------------------------------

    def _on_tick(self, event) -> None:
        if self.stage not in self.active_stages:
            self._set_status("waiting")
            return
        for part in self.arm_parts:
            if self.engaged[part] is None and self.leader_pose[part] is not None \
                    and self.follower_pose[part] is not None:
                self.engaged[part] = Engagement(self.leader_pose[part], self.follower_pose[part])
        pending = [p for p in self.arm_parts if self.engaged[p] is None]
        pending += [p for p in self.gripper_parts if self.gripper[p] is None]
        if pending:
            logger.info("retarget: waiting for %s", ", ".join(pending))
            self._set_status("waiting")
            return  # never command blind

        out: list[float] = []
        for entry in self.layout:
            part = entry["part"]
            if entry["space"] == "cartesian":
                out.extend(self._arm_target(part))
            else:
                out.append(self.gripper[part])
        self.node.send_output("command", pa.array(out, type=pa.float64()),
                              metadata={"timestamp": time.time()})
        self._set_status("engaged")

    def _arm_target(self, part: str) -> list[float]:
        """The clamped absolute pose7 target for one arm: map the leader delta through
        (align, scale) onto the engage anchor, then step the running target toward it by at
        most max_pos_step / max_rot_step."""
        eng = self.engaged[part]
        pose = self.leader_pose[part]
        p_l, r_l = pose[:3], Rotation.from_quat(pose[3:7])

        p_desired = eng.p_f0 + self.cfg.scale * self.align.apply(p_l - eng.p_l0)
        delta_r = self.align * (r_l * eng.r_l0.inv()) * self.align.inv()
        r_desired = delta_r * eng.r_f0

        dp = p_desired - eng.p_target
        dist = float(np.linalg.norm(dp))
        if dist > self.cfg.max_pos_step:
            dp *= self.cfg.max_pos_step / dist
        eng.p_target = eng.p_target + dp

        rotvec = (r_desired * eng.r_target.inv()).as_rotvec()
        angle = float(np.linalg.norm(rotvec))
        if angle > self.cfg.max_rot_step:
            rotvec *= self.cfg.max_rot_step / angle
        eng.r_target = Rotation.from_rotvec(rotvec) * eng.r_target

        return [*eng.p_target, *eng.r_target.as_quat()]
