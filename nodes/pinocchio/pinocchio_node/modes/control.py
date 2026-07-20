"""`control` mode — turn a whole-robot command bundle into SAFE per-part joint targets.

One descriptor-defined `command` vector carries every part's setpoint (Cartesian pose, joint
positions, gripper opening, base twist ...), each tagged by (space, quantity) in the layout.
On each `command` the node steps ALL parts at once into one combined candidate config, gates
that combined config (cross-group self-collision + plane/table constraint + joint limits), and
— if safe — emits every part's `<part>_joint_target` together; else it HOLDs (re-emits the last
safe targets) and says why. The `state` bundle (measured joints) seeds the working config and
is the config the gate protects. Solving the whole robot from one message is the synchronisation:
the cross-group collision check sees all parts' new poses at once, and outputs leave together.

Inputs:  command, state, program_state.
Outputs (per part): <part>_joint_target; <arm-part>_solution_pose (the safe solved EE pose,
  pose7 xyzw, posted back so the controller can re-anchor its target); plus status.
"""

from __future__ import annotations

import logging
import time
from typing import Callable

import numpy as np
import pyarrow as pa
from dora import Node

from pinocchio_node.model import RobotModel, quat_xyzw_to_matrix
from pinocchio_node.node_config import PinocchioConfig

logger = logging.getLogger("pinocchio")


class ControlMode:
    def __init__(self, cfg: PinocchioConfig, node: Node):
        self.cfg = cfg
        self.node = node
        self.model = RobotModel(cfg.scene)
        self.cmd_layout = self.model.desc["command_layouts"][cfg.command_layout]
        self.state_layout = self.model.desc["state_layout"]
        self.parts = [e["part"] for e in self.cmd_layout]

        self.q = self.model.q_home.copy()
        self.last_safe = {p: self.model.part_q(self.q, p) for p in self.parts}
        self._ready = False
        self.status = ""
        # Operating stage from the manager's program_state. In `homing` the command bundle is
        # ignored and each state message ramps the robot toward the descriptor home
        # instead (status `homed` on arrival — the manager advances the stage on that).
        self.stage = "start"
        # The homing ramp's COMMANDED config: seeded from the measured state on the first
        # homing step, then stepped toward home. Ramping from the measured state instead
        # ratchets: a part whose measured position lags its actuator (the gripper) would
        # have each re-anchored target chase the sag away from home.
        self._homing_q: np.ndarray | None = None
        self._handlers: dict[str, Callable] = {
            "command": self._on_command,
            "state": self._on_state,
            "program_state": self._on_program_state,
        }

    def start(self) -> None:
        logger.info("pinocchio: ready (%s: %s)", self.cfg.command_layout, ", ".join(self.parts))
        self._emit_status("ready")

    def handle(self, event) -> bool:
        return bool(self._handlers[event["id"]](event))  # KeyError on an unwired input id = loud

    def close(self) -> None:
        pass

    # --- program_state / liveness --------------------------------------------------------

    def _on_program_state(self, event) -> bool:
        value = event["value"][0].as_py()
        if value == "disconnect":
            return True
        if value != self.stage:
            self.stage = value
            self._homing_q = None  # (re)seed the ramp from the measured state on entry
            if value == "homing":
                self._set_status("homing")
        return False


    def _emit_status(self, text: str) -> None:
        self.status = text
        self.node.send_output("node_state", pa.array([text]))

    def _set_status(self, text: str) -> None:
        """Edge-triggered status: emit only when the token changes (no per-command spam)."""
        if text != self.status:
            self._emit_status(text)

    # --- bundle helpers --------------------------------------------------------------

    @staticmethod
    def _segments(layout, vec: np.ndarray):
        """Yield (entry, slice-of-vec) for each part in a descriptor layout."""
        off = 0
        for entry in layout:
            dim = entry["dim"]
            yield entry, vec[off:off + dim]
            off += dim

    # --- state feedback --------------------------------------------------------------

    def _on_state(self, event) -> None:
        vec = event["value"].to_numpy()
        for entry, seg in self._segments(self.state_layout, vec):
            self.model.set_part_q(self.q, entry["part"], seg)
        if not self._ready:
            self.last_safe = {p: self.model.part_q(self.q, p) for p in self.parts}
            self._ready = True
        # Publish the MEASURED ee pose (model-frame FK of the joint state). The controller seeds
        # and anchors its Cartesian target from this, so the target lives in the SAME frame
        # pinocchio solves IK in. Seeding from the robot's raw getActualTCPPose instead puts the
        # target in the UR base frame — a different frame than the model ee — so on startup IK
        # would lunge the arm to reconcile that offset even with no operator input.
        md = {"timestamp": time.time()}
        for part, pose7 in self.model.ee_poses7(self.q).items():
            self.node.send_output(f"{part}_measured_pose", pa.array([float(v) for v in pose7]), metadata=md)
        if self.stage == "homing":
            self._homing_step()

    # --- homing stage: slow ramp to the descriptor home ------------------------------

    def _homing_step(self) -> None:
        """One slow step of the COMMANDED config toward the descriptor home, through the
        same collision gate as normal control. Driven by `state` messages (~the robot
        rate), so the ramp speed is homing_max_step * state rate. `homed` is judged on
        the MEASURED state, so the manager only advances once the robot actually arrived."""
        if self._homing_q is None:
            # Travel guard BEFORE the first step: a home far from the measured pose means
            # the descriptor doesn't match the real cell — refuse loudly, move nothing.
            # (Wording must not contain "homed" — the manager advances on that substring.)
            for part in self.parts:
                travel = np.abs(self.model.part_q(self.model.q_home, part) - self.model.part_q(self.q, part))
                worst = float(np.max(travel))
                if worst > self.cfg.homing_max_travel:
                    self._set_status(
                        f"homing refused: {part} needs {worst:.2f} rad > max "
                        f"{self.cfg.homing_max_travel} — pre-pose the arm or fix the descriptor home")
                    return
            self._homing_q = self.q.copy()  # seed the ramp where the robot measurably is
        candidate_q = self._homing_q.copy()
        arrived = True
        for part in self.parts:
            cur = self.model.part_q(self._homing_q, part)
            home = self.model.part_q(self.model.q_home, part)
            measured = self.model.part_q(self.q, part)
            if float(np.max(np.abs(home - measured))) > self.cfg.homing_tol:
                arrived = False
            step = np.clip(home - cur, -self.cfg.homing_max_step, self.cfg.homing_max_step)
            self.model.set_part_q(candidate_q, part, self.model.clamp_to_limits(part, cur + step))

        if self.cfg.collision_check:
            ok, dmin = self.model.collision_ok(candidate_q)
            if not ok:
                self._hold(f"homing: self-collision {dmin:.3f}m < {self.model.collision_margin}m")
                return

        self._homing_q = candidate_q
        for part in self.parts:
            cand = self.model.part_q(candidate_q, part)
            self.last_safe[part] = cand
            self._publish(part, cand)
        self._set_status("homed" if arrived else "homing")

    # --- command: synchronized whole-robot step --------------------------------------

    def _on_command(self, event) -> None:
        if not self._ready:
            return  # never command before we know where the robot is
        if self.stage != "teleop":
            return  # commands drive the robot only in the teleop stage (homing owns it
            # while ramping; boot must never move a robot before the program is up)
        vec = event["value"].to_numpy()
        candidate_q = self.q.copy()
        goal_q = self.q.copy()  # the converged IK goal (pre rate-limit) — posted as the solution
        proposed: dict[str, np.ndarray] = {}
        for entry, seg in self._segments(self.cmd_layout, vec):
            part = entry["part"]
            cur = self.model.part_q(self.q, part)
            goal = self._goal(part, entry["space"], entry["quantity"], seg)
            self.model.set_part_q(goal_q, part, goal)
            max_step = self.model.parts[part].max_step or self.cfg.max_joint_step
            step = np.clip(goal - cur, -max_step, max_step)  # rate-limit motion toward the stable goal
            cand = self.model.clamp_to_limits(part, cur + step)
            proposed[part] = cand
            self.model.set_part_q(candidate_q, part, cand)

        if self.cfg.collision_check:
            ok, dmin = self.model.collision_ok(candidate_q)
            if not ok:
                self._hold(f"self-collision {dmin:.3f}m < {self.model.collision_margin}m")
                return
            for part in proposed:
                pok, coord = self.model.plane_ok(candidate_q, part)
                if not pok:
                    self._hold(f"{part} below plane ({coord:.3f})")
                    return

        # Gate passed: post the converged IK GOAL pose (= the target when reachable, the
        # closest-reachable pose when not) so the controller can re-anchor its target. The
        # goal (not the rate-limited candidate) is the stable signal — it equals the target
        # mid-motion too, so a reachable move never looks "unreachable". Posted even when
        # nothing moved, so a target pushed further past reach keeps getting pulled back.
        self._publish_solution(goal_q)

        if np.allclose(candidate_q, self.q, atol=1e-6):
            self._set_status("tracking")  # also announces recovery from a HOLD
            return  # already at the commanded config — no new joint targets to send

        self.q = candidate_q
        for part, cand in proposed.items():
            self.last_safe[part] = cand
            self._publish(part, cand)
        self._set_status("tracking")

    def _goal(self, part, space, quantity, seg) -> np.ndarray:
        """Stable goal joint vector for one part (per space/quantity). For a Cartesian part
        this is the converged IK solution (a steady pose even when the target is unreachable);
        for a joint part it is the commanded joints. The caller rate-limits the actual motion
        toward this goal by the part's max_step."""
        if quantity == "velocity":
            # TODO: velocity control — cartesian: J⁺·twist; joint: integrate. One branch each.
            raise NotImplementedError("velocity quantity not implemented yet")
        if quantity != "position":
            raise ValueError(f"unknown quantity {quantity!r}")
        if space == "cartesian":
            goal, _ = self.model.ik_solve(
                self.q, part, seg[:3], quat_xyzw_to_matrix(seg[3:7]),
                damping=self.cfg.ik_damping, error_damping=self.cfg.ik_error_damping,
                max_iters=self.cfg.ik_max_iters, tol=self.cfg.ik_tol,
            )
            return goal
        if space == "joint":
            return np.asarray(seg, dtype=float)
        raise ValueError(f"unknown space {space!r}")

    def _hold(self, reason: str) -> None:
        for part in self.parts:
            self._publish(part, self.last_safe[part])  # keep every part at its last safe pose
        self._publish_solution(self.q)  # held EE pose, so the controller's target snaps back too
        # Edge-triggered: announce entering HELD once (the numeric part of the reason varies
        # per command, which would otherwise re-emit at the command rate). Recovery is
        # announced by the next `tracking` edge.
        if not self.status.startswith("HELD"):
            logger.warning("pinocchio: HELD (%s)", reason)
            self._emit_status(f"HELD ({reason})")

    def _publish_solution(self, q: np.ndarray) -> None:
        """Post each arm part's solved EE pose (`<part>_solution_pose`, pose7 xyzw) so the
        web-controller can re-anchor its target to what was actually reachable."""
        md = {"timestamp": time.time()}
        for part, pose7 in self.model.ee_poses7(q).items():
            self.node.send_output(f"{part}_solution_pose", pa.array([float(v) for v in pose7]), metadata=md)

    def _publish(self, part: str, joints: np.ndarray) -> None:
        self.node.send_output(
            f"{part}_joint_target",
            pa.array([float(v) for v in joints]),
            metadata={"timestamp": time.time()},
        )
