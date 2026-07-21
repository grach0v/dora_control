"""`manual` mode — operator drives the robot from the web control surface.

Builds the whole-robot **`command` bundle** (the descriptor's `cartesian` command layout:
each arm part a Cartesian pose, each gripper part an opening) and republishes it on each tick
(coalescing browser nudges to the tick rate) — but ONLY while the program is in the `teleop`
stage; any stage change resets the target anchors so a stage that moved the robot (homing)
never leaves a stale target behind. `main.py` owns the NiceGUI/FastAPI surface; this module
owns the per-input logic + the thread-shared `State`. Feedback `<arm>_tcp_pose` shows the
real value and seeds the target so there is no startup jump.

Inputs:  tick, program_state, `<arm-part>_tcp_pose` (measured, seeds + shows current)
  and `<arm-part>_solution_pose` (pinocchio's safe solved pose, re-anchors the target).
Outputs: command (float64 bundle), episode_control (utf8), robot_command (utf8: disconnect),
  node_state.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from functools import partial
from pathlib import Path
from typing import Callable

import pyarrow as pa
import yaml
from dora import Node
from scipy.spatial.transform import Rotation

from web_controller.node_config import WebControllerConfig

logger = logging.getLogger("web-controller")

NODE_ID = "web-controller"
FIELDS = ("x", "y", "z", "roll", "pitch", "yaw")  # an arm part's 6-DOF target, in this order
# roll/pitch/yaw nudges rotate about these fixed WORLD axes (applied to the target quaternion,
# so they stay independent at any orientation — no gimbal lock, unlike editing Euler angles).
_ROT_AXIS = {"roll": (1.0, 0.0, 0.0), "pitch": (0.0, 1.0, 0.0), "yaw": (0.0, 0.0, 1.0)}


def quat_to_rpy(qx, qy, qz, qw) -> list[float]:
    return Rotation.from_quat([qx, qy, qz, qw]).as_euler("xyz").tolist()


def pose_to_target(pose7) -> list[float]:
    x, y, z, qx, qy, qz, qw = (float(v) for v in pose7)
    return [x, y, z, *quat_to_rpy(qx, qy, qz, qw)]


def load_descriptor(scene_path: str) -> dict:
    return yaml.safe_load(Path(scene_path).resolve().read_text())


# Re-anchor the target only when the commanded pose was missed by more than this (i.e. it was
# unreachable) — well above pinocchio's IK convergence tolerance so a reached target never snaps.
REANCHOR_POS_TOL = 0.01   # metres
REANCHOR_ROT_TOL = 0.05   # radians (~3°)


class State:
    """Shared between the HTTP thread and the dora loop (guarded by `lock`). Builds the
    `command` bundle from the descriptor's command layout: cartesian parts = arms (6-DOF
    pose target + ee feedback), other parts = grippers (a scalar opening)."""

    def __init__(self, cfg: WebControllerConfig):
        self.cfg = cfg
        self.lock = threading.Lock()
        # `episode` mode has no scene/layout — it only drives episode_control + disconnect.
        desc = load_descriptor(cfg.scene) if cfg.scene else None
        self.layout = desc["command_layouts"][cfg.command_layout] if desc else []
        self.arm_parts = [e["part"] for e in self.layout if e["space"] == "cartesian"]
        self.gripper_parts = [e["part"] for e in self.layout if e["space"] != "cartesian"]
        self.current: dict[str, list[float] | None] = {p: None for p in self.arm_parts}  # measured pose7
        # Target stored as pose7 [x,y,z, qx,qy,qz,qw] (a QUATERNION, not Euler) so orientation
        # nudges are gimbal-free; displayed as roll/pitch/yaw only for readout.
        self.target: dict[str, list[float] | None] = {p: None for p in self.arm_parts}
        # Last pose7 we COMMANDED per arm part — compared with pinocchio's solved pose to detect
        # an unreachable target and snap back to it (see on_solution).
        self.last_commanded: dict[str, list[float]] = {}
        # Gripper bounds / nudge step per part, from the descriptor (robot-agnostic:
        # Trossen open>closed in metres, UR open<closed in radians — both handled order-free).
        # The commanded opening starts at open (= the cell home keyframe's gripper pose).
        self.gripper: dict[str, float] = {}
        self.gripper_bounds: dict[str, tuple[float, float]] = {}
        self.gripper_step: dict[str, float] = {}
        for p in self.gripper_parts:
            gp = desc["parts"][p]
            lo, hi = sorted((float(gp["open"]), float(gp["closed"])))
            self.gripper_bounds[p] = (lo, hi)
            self.gripper_step[p] = (hi - lo) * cfg.gripper_step_frac
            self.gripper[p] = float(gp["open"])
        # Active drag-slider anchors: (arm part, field) -> the target pose7 captured when the
        # slider was grabbed. While a slider is held, its deflection in [-1, 1] sets the target
        # ABSOLUTELY as `anchor + deflection * span` (so the operator drags the target to a
        # location and the arm chases it); on release the anchor is dropped and the target holds.
        self.drag_anchor: dict[tuple[str, str], list[float]] = {}
        self.task = ""
        self.recording = False
        self.pending: list[str] = []
        self.disconnect_requested = False

    def set_task(self, text: str) -> None:
        with self.lock:
            self.task = text
            self.pending.append(f"task={text}")

    def start_episode(self) -> None:
        with self.lock:
            self.recording = True
            self.pending.append(f"task={self.task}")
            self.pending.append("start")

    def finish_episode(self) -> None:
        with self.lock:
            self.recording = False
            self.pending.append("finish")

    def request_disconnect(self) -> None:
        with self.lock:
            self.disconnect_requested = True

    def drain_pending(self) -> list[str]:
        with self.lock:
            msgs, self.pending = self.pending, []
            return msgs

    def on_feedback(self, part: str, pose7: list[float]) -> None:
        with self.lock:
            self.current[part] = pose7
            if self.target[part] is None:  # seed target from first real pose -> no jump
                self.target[part] = list(pose7)

    def reset_targets(self) -> None:
        """Drop every target anchor so the next feedback reseeds it from the measured
        pose. Called on each program-state change: a stage like `homing` moves the
        robot underneath us, and a target latched before it would command the arm
        straight back to its pre-homing pose on the next stage."""
        with self.lock:
            self.target = {p: None for p in self.arm_parts}
            self.last_commanded.clear()
            self.drag_anchor.clear()

    def _seed_target_if_needed(self, part: str) -> bool:
        """Ensure `target[part]` exists (seed from the latest measured pose). Returns False
        if we don't yet know where the arm is (no feedback) — nothing to move. Lock held."""
        if self.target[part] is None:
            if self.current[part] is None:
                return False
            self.target[part] = list(self.current[part])
        return True

    def _offset_target(self, part: str, field: str, base: list[float], delta: float) -> None:
        """Set `target[part] = base` displaced by `delta` in `field`'s NATIVE units (m for
        x/y/z, rad for rpy). Orientation displaces `base`'s quaternion by a rotation about a
        fixed WORLD axis, so it stays gimbal-free and independent of orientation. Lock held."""
        t = list(base)  # copy so a drag anchor is never mutated
        if field in ("x", "y", "z"):
            t[("x", "y", "z").index(field)] += delta
        else:
            axis = _ROT_AXIS[field]
            drot = Rotation.from_rotvec([a * delta for a in axis])
            t[3:7] = (drot * Rotation.from_quat(base[3:7])).as_quat().tolist()
        self.target[part] = t

    def _step(self, field: str) -> float:
        return self.cfg.pos_step if field in ("x", "y", "z") else self.cfg.rot_step

    def _span(self, field: str) -> float:
        return self.cfg.drag_span_pos if field in ("x", "y", "z") else self.cfg.drag_span_rot

    def nudge(self, part: str, field: str, direction: int) -> None:
        """One discrete step (a +/- button): displace the CURRENT target by ±one step."""
        with self.lock:
            if self._seed_target_if_needed(part):
                self._offset_target(part, field, self.target[part], direction * self._step(field))

    # --- drag slider: absolute target location within a grab ---------------------------
    # A field's slider maps its deflection to a target LOCATION, so the operator drags the
    # target and the arm chases it (a responsiveness test). On grab we anchor to the current
    # target; deflection d in [-1, 1] sets `target = anchor + d * span`. On release the anchor
    # is dropped (target holds) and the slider recentres — that recentre event is ignored
    # because no anchor is active.

    def begin_drag(self, part: str, field: str) -> None:
        with self.lock:
            if self._seed_target_if_needed(part):
                self.drag_anchor[(part, field)] = list(self.target[part])

    def drag_to(self, part: str, field: str, value: float) -> None:
        with self.lock:
            anchor = self.drag_anchor.get((part, field))
            if anchor is None:
                return  # not an active drag (e.g. the recentre event after release)
            self._offset_target(part, field, anchor, max(-1.0, min(1.0, float(value))) * self._span(field))

    def end_drag(self, part: str, field: str) -> None:
        with self.lock:
            self.drag_anchor.pop((part, field), None)

    def nudge_gripper(self, part: str, direction: int) -> None:
        lo, hi = self.gripper_bounds[part]
        with self.lock:
            g = self.gripper[part] + direction * self.gripper_step[part]
            self.gripper[part] = max(lo, min(hi, g))

    def on_solution(self, part: str, pose7: list[float]) -> None:
        """Re-anchor the target to pinocchio's solved (closest-reachable) pose, but ONLY when
        what we commanded couldn't be reached — judged by the pose error between the solved and
        commanded poses (position distance + quaternion angle, so there's no Euler/gimbal
        ambiguity). Reachable -> leave the target (and the operator's nudges) alone; unreachable
        -> snap to the achievable pose. The snap is absolute, so nothing accumulates."""
        with self.lock:
            commanded = self.last_commanded.get(part)  # pose7
            if self.target[part] is None or commanded is None:
                return
            pos_err = math.dist(pose7[:3], commanded[:3])
            ang_err = (Rotation.from_quat(pose7[3:7]).inv() * Rotation.from_quat(commanded[3:7])).magnitude()
            if pos_err > REANCHOR_POS_TOL or ang_err > REANCHOR_ROT_TOL:
                self.target[part] = list(pose7)  # snap to achievable (no accumulation)

    def build_command(self) -> list[float] | None:
        """The whole-robot command vector in the descriptor layout order, or None until
        every arm part has been seeded by feedback (never command blind)."""
        with self.lock:
            out: list[float] = []
            for e in self.layout:
                part = e["part"]
                if e["space"] == "cartesian":
                    if self.target[part] is None:
                        return None
                    out.extend(self.target[part])  # already pose7 [x,y,z, qx,qy,qz,qw]
                    self.last_commanded[part] = list(self.target[part])  # what we're sending now
                else:
                    out.append(self.gripper[part])
            return out

    def snapshot(self) -> dict:
        with self.lock:
            return {
                "arm_parts": self.arm_parts,
                "gripper_parts": self.gripper_parts,
                "fields": list(FIELDS),
                "current": {p: (pose_to_target(self.current[p]) if self.current[p] else None) for p in self.arm_parts},
                "target": {p: (pose_to_target(self.target[p]) if self.target[p] else None) for p in self.arm_parts},
                "gripper": {p: self.gripper[p] for p in self.gripper_parts},
                "task": self.task,
                "recording": self.recording,
            }


class ManualMode:
    def __init__(self, cfg: WebControllerConfig, node: Node, state: State):
        self.cfg = cfg
        self.node = node
        self.state = state
        self.disconnecting = False
        self.status = ""
        # Operating stage from the manager's program_state: motion commands are emitted
        # only in `teleop`. Any stage change resets the target anchors (see reset_targets).
        self.stage = "start"
        self._handlers: dict[str, Callable] = {
            "program_state": self._on_program_state,
            "tick": self._on_tick,
        }
        for part in state.arm_parts:
            self._handlers[f"{part}_tcp_pose"] = partial(self._on_feedback, part)
            self._handlers[f"{part}_solution_pose"] = partial(self._on_solution, part)

    def start(self) -> None:
        logger.info("%s: open http://%s:%s", NODE_ID, self.cfg.host, self.cfg.port)
        self._emit_status("ready")

    def _emit_status(self, text: str) -> None:
        self.status = text
        self.node.send_output("node_state", pa.array([text]))

    def handle(self, event) -> bool:
        return bool(self._handlers[event["id"]](event))  # KeyError on an unwired input id = loud

    def close(self) -> None:
        pass

    def _on_program_state(self, event) -> bool:
        value = event["value"][0].as_py()
        if value == "disconnect":
            return True
        if value != self.stage:
            self.stage = value
            self.state.reset_targets()  # a stage (e.g. homing) moved the robot: reseed
        return False


    def _on_feedback(self, part: str, event) -> None:
        self.state.on_feedback(part, [float(v) for v in event["value"].to_numpy()])

    def _on_solution(self, part: str, event) -> None:
        self.state.on_solution(part, [float(v) for v in event["value"].to_numpy()])

    def _on_tick(self, event) -> None:
        md = {"timestamp": time.time()}
        for msg in self.state.drain_pending():
            self.node.send_output("episode_control", pa.array([msg]), metadata=md)
        if not self.disconnecting and self.state.disconnect_requested:
            self.disconnecting = True
            self.node.send_output("robot_command", pa.array(["disconnect"]), metadata=md)
        if not self.disconnecting and self.stage == "teleop":  # motion only once homing is done
            cmd = self.state.build_command()
            if cmd is not None:
                self.node.send_output("command", pa.array(cmd, type=pa.float64()), metadata=md)
