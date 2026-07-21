"""Whole-robot kinematics + collision, built from a scene descriptor (within-node only).

Generic: it knows *parts* (each a set of joints + an OPTIONAL ee frame — arm, gripper, base,
leg...) and *constraints* (typed: plane, self_collision, ...). Nothing is baked to "arms" or
"table". Self-collision is grouped by MJCF frame-name prefix (so an arm and its gripper share
a group), decoupled from the command parts.

Not a shared library: this lives inside the pinocchio node only (CLAUDE.md).
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pinocchio as pin
import yaml
from scipy.spatial.transform import Rotation

_AXIS = {"x": 0, "y": 1, "z": 2}


def quat_xyzw_to_matrix(quat_xyzw: np.ndarray) -> np.ndarray:
    """Wire quaternion [qx,qy,qz,qw] -> 3x3 rotation matrix."""
    return Rotation.from_quat(np.asarray(quat_xyzw, dtype=float)).as_matrix()


@dataclass
class Part:
    name: str
    type: str
    q_idx: np.ndarray          # config indices of this part's joints
    v_idx: np.ndarray          # velocity/dof indices
    ee_frame_id: int | None    # None => joint-only part (no IK)
    lower: np.ndarray
    upper: np.ndarray
    max_step: float | None = None   # max joint motion per control step (units per the part)

    @property
    def dof(self) -> int:
        return len(self.q_idx)


class RobotModel:
    def __init__(self, scene_path: str):
        path = Path(scene_path).resolve()
        self.desc = yaml.safe_load(path.read_text())
        scene_root = path.parent.parent

        model_file = scene_root / self.desc["model"]
        self.model = pin.buildModelFromMJCF(str(model_file))
        self.data = self.model.createData()
        self.geom = pin.buildGeomFromMJCF(self.model, str(model_file), pin.GeometryType.COLLISION)

        # The cell home pose comes from the MJCF `home` keyframe (pinocchio parses
        # keyframes into referenceConfigurations) — the model is the source of truth.
        self.q_home = np.array(self.model.referenceConfigurations["home"])

        self.parts: dict[str, Part] = {}
        for name, p in self.desc["parts"].items():
            q_idx, v_idx = [], []
            for jn in p["joints"]:
                joint = self.model.joints[self.model.getJointId(jn)]
                q_idx.append(joint.idx_q)
                v_idx.append(joint.idx_v)
            q_idx, v_idx = np.array(q_idx), np.array(v_idx)
            ee = p.get("ee_frame")
            self.parts[name] = Part(
                name=name, type=p.get("type", ""), q_idx=q_idx, v_idx=v_idx,
                ee_frame_id=(self.model.getFrameId(ee) if ee else None),
                lower=self.model.lowerPositionLimit[q_idx].copy(),
                upper=self.model.upperPositionLimit[q_idx].copy(),
                max_step=p.get("max_step"),
            )

        self._parse_constraints()
        self._setup_collision_pairs()
        self.geom_data = self.geom.createData()

    # --- constraints -----------------------------------------------------------------

    def _parse_constraints(self) -> None:
        self.plane_axis: int | None = None
        self.plane_min = 0.0
        self.collision_groups: list[str] = []
        self.collision_margin = 0.0
        for c in self.desc.get("constraints", []):
            if c["type"] == "plane":
                self.plane_axis = _AXIS[c["axis"]]
                self.plane_min = float(c["min"])
            elif c["type"] == "self_collision":
                self.collision_groups = list(c["groups"])
                self.collision_margin = float(c.get("margin", 0.0))
            else:
                raise ValueError(f"unknown constraint type {c['type']!r}")

    def _group_of_geom(self, gi: int) -> str | None:
        fname = self.model.frames[self.geom.geometryObjects[gi].parentFrame].name
        for g in self.collision_groups:
            if fname.startswith(g):
                return g
        return None

    def _setup_collision_pairs(self) -> None:
        by_group: dict[str, list[int]] = {g: [] for g in self.collision_groups}
        for gi in range(self.geom.ngeoms):
            g = self._group_of_geom(gi)
            if g is not None:
                by_group[g].append(gi)
        for i in range(len(self.collision_groups)):
            for j in range(i + 1, len(self.collision_groups)):
                for gi in by_group[self.collision_groups[i]]:
                    for gj in by_group[self.collision_groups[j]]:
                        self.geom.addCollisionPair(pin.CollisionPair(gi, gj))

    # --- queries ---------------------------------------------------------------------

    def ee_poses7(self, q: np.ndarray) -> dict[str, list[float]]:
        """{part: [x,y,z, qx,qy,qz,qw]} for every part with an ee frame, at config q.
        Used to post the solved EE pose back to the controller (joint-only parts have none)."""
        pin.framesForwardKinematics(self.model, self.data, q)
        out: dict[str, list[float]] = {}
        for name, p in self.parts.items():
            if p.ee_frame_id is None:
                continue
            oMf = self.data.oMf[p.ee_frame_id]
            out[name] = [*oMf.translation.tolist(), *Rotation.from_matrix(oMf.rotation).as_quat().tolist()]
        return out

    def part_q(self, q: np.ndarray, part: str) -> np.ndarray:
        return q[self.parts[part].q_idx].copy()

    def set_part_q(self, q: np.ndarray, part: str, joints: np.ndarray) -> None:
        q[self.parts[part].q_idx] = joints

    def clamp_to_limits(self, part: str, joints: np.ndarray) -> np.ndarray:
        p = self.parts[part]
        return np.clip(joints, p.lower, p.upper)

    def collision_ok(self, q: np.ndarray) -> tuple[bool, float]:
        """(safe?, min cross-group distance). Safe if no monitored pair is within margin."""
        if len(self.geom.collisionPairs) == 0:
            return True, float("inf")
        pin.computeDistances(self.model, self.data, self.geom, self.geom_data, q)
        dmin = min(dr.min_distance for dr in self.geom_data.distanceResults)
        return dmin >= self.collision_margin, dmin

    def plane_ok(self, q: np.ndarray, part: str) -> tuple[bool, float]:
        """(safe?, ee coord on the plane axis). True if no plane, or part has no ee."""
        p = self.parts[part]
        if self.plane_axis is None or p.ee_frame_id is None:
            return True, float("inf")
        pin.framesForwardKinematics(self.model, self.data, q)
        coord = float(self.data.oMf[p.ee_frame_id].translation[self.plane_axis])
        return coord >= self.plane_min - 1e-4, coord

    # --- inverse kinematics ----------------------------------------------------------

    def _project_target(self, target_pos: np.ndarray) -> np.ndarray:
        out = np.asarray(target_pos, dtype=float).copy()
        if self.plane_axis is not None:
            out[self.plane_axis] = max(out[self.plane_axis], self.plane_min)
        return out

    def ik_step(self, q, part, target_pos, target_R, *, damping, error_damping=0.0):
        """One error-damped-least-squares (full 6-DOF) IK step for `part` (must have an ee).
        Returns (dq for the part's joints, error norm).

        Damping is `λ² = damping² + (error_damping·‖e‖)²` (error-damped least squares): the
        extra term grows with the task-space error, so a far/unreachable/near-singular target
        is heavily damped (small, smooth steps — no pseudo-inverse blow-up or wobble) while a
        close target keeps the small base damping for accuracy. This is robot-agnostic (the
        knob is in task units), unlike a fixed manipulability threshold."""
        p = self.parts[part]
        oMdes = pin.SE3(target_R, self._project_target(target_pos))
        pin.framesForwardKinematics(self.model, self.data, q)
        err = pin.log6(oMdes.actInv(self.data.oMf[p.ee_frame_id])).vector
        jac = pin.computeFrameJacobian(self.model, self.data, q, p.ee_frame_id, pin.ReferenceFrame.LOCAL)
        j = jac[:, p.v_idx]
        jjt = j @ j.T
        e_norm = float(np.linalg.norm(err))
        lam2 = damping**2 + (error_damping * e_norm) ** 2
        dq = -j.T @ np.linalg.solve(jjt + lam2 * np.eye(jjt.shape[0]), err)
        return dq, e_norm

    def ik_solve(self, q, part, target_pos, target_R, *, damping, error_damping, max_iters, tol):
        """Iterate `ik_step` to convergence for `part`, returning (goal joints, final error).

        Unlike a single step, this lands on a STABLE configuration every call: a reachable
        target converges to it; an unreachable one settles on the (error-damped) least-squares
        closest pose where dq -> 0. The caller then rate-limits the real motion toward this
        goal, so a held unreachable target produces a steady pose instead of a wobble. Joints
        are clamped to their limits each iteration so the goal is always feasible."""
        p = self.parts[part]
        qw = q.copy()
        err = float("inf")
        for _ in range(max_iters):
            dq, err = self.ik_step(qw, part, target_pos, target_R, damping=damping, error_damping=error_damping)
            if err < tol:
                break
            qw[p.q_idx] = self.clamp_to_limits(part, qw[p.q_idx] + dq)
        return qw[p.q_idx].copy(), err
