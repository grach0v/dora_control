"""Optional 3D robot view for the rerun node (within-node only; CLAUDE.md).

When ``VISUALIZE__SCENE`` is set, the node loads the descriptor's ``model`` once
(MuJoCo) and logs it the idiomatic rerun way (see docs "Transforms & Coordinate
Frames" / the URDF example):

  * entity paths MIRROR the kinematic tree (``robot/table/left_base/left_shoulder_link/…``);
  * each geom's mesh + its body-local pose are logged ONCE as static children of the body;
  * on each joint ``state`` message it runs FK and logs, per body, that body's transform
    RELATIVE TO ITS PARENT — and only for bodies whose relative transform actually changed.

Why this shape matters: rerun's viewer cost is per-entity-update, not per-byte, and rerun
COMPOSES transforms down the entity tree. Logging every geom's WORLD pose every frame is
~M updates/frame (M≈84 here) and floods the viewer so the 3D view lags seconds behind.
Logging parent-relative transforms means moving one joint re-logs ONE transform (its child
body); the whole downstream chain moves for free via composition, and unchanged bodies are
skipped entirely (rerun holds the last value). Jogging one arm ≈ a handful of updates/frame,
idle ≈ zero — same picture, orders of magnitude fewer per-frame log calls, full 30 fps.

Driven entirely by the joint ``state`` stream (FK only, no IK), so it works the same in sim
and on the real robot. Only mesh/box geoms are logged (planes/props are skipped) — a
robot+workstation view.
"""

from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np
import rerun as rr
import yaml

# MuJoCo's default-visible geom groups (0–2). Group 3 is collision geometry (hidden by
# default) — skip it. Covers the robot's visual meshes (group 2) AND the bench/risers
# (group 0), so the whole workstation shows, not just the arms.
VISIBLE_GROUPS = (0, 1, 2)


def _tf(pos: np.ndarray, mat: np.ndarray) -> rr.Transform3D:
    return rr.Transform3D(translation=pos.copy(), mat3x3=mat.reshape(3, 3).copy())


class RobotScene:
    def __init__(self, scene_path: str, entity_prefix: str = "robot"):
        path = Path(scene_path).resolve()
        desc = yaml.safe_load(path.read_text())
        root = path.parent.parent
        model_file = str(root / desc["model"])
        self.model = mujoco.MjModel.from_xml_path(model_file)
        self.data = mujoco.MjData(self.model)
        self.prefix = entity_prefix
        mujoco.mj_kinematics(self.model, self.data)

        # state-vector layout -> qpos addresses, per part, in state_layout order.
        self.state_layout = desc["state_layout"]
        self._qadr: list[list[int]] = []
        for entry in self.state_layout:
            joints = desc["parts"][entry["part"]]["joints"]
            self._qadr.append([self.model.jnt_qposadr[self._jid(jn)] for jn in joints])

        # Entity path per body, mirroring the kinematic tree (worldbody -> the "robot" root).
        self._entity: dict[int, str] = {0: self.prefix}
        for b in range(1, self.model.nbody):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, b) or f"body{b}"
            self._entity[b] = f"{self._entity[self.model.body_parentid[b]]}/{name}"

        # Visible geoms: mesh + body-local pose, both STATIC, under the geom's body entity.
        self._geom_meshes: list[tuple[str, object]] = []
        self._geom_locals: list[tuple[str, rr.Transform3D]] = []
        geom_bodies: set[int] = set()
        for g in range(self.model.ngeom):
            if self.model.geom_group[g] not in VISIBLE_GROUPS:
                continue
            arch = self._static_archetype(g)
            if arch is None:  # non-mesh/box primitive (none in these models)
                continue
            b = int(self.model.geom_bodyid[g])
            ge = f"{self._entity[b]}/g{g}"
            self._geom_meshes.append((ge, arch))
            self._geom_locals.append((ge, self._local(g, b)))
            geom_bodies.add(b)

        # Model cameras (MJCF mode="fixed" mounts — the model describes the physical cell,
        # so the poses hold for sim and real alike). Each camera is an entity under ITS BODY
        # with its constant body-local pose, so wrist cameras ride the kinematic chain via
        # rerun's transform composition. The rr.Pinhole is logged by visualize.py when the
        # first image arrives (fovy from the model here, resolution from the stream's
        # width/height metadata — a real camera keeps its real aspect).
        self.cameras: dict[str, str] = {}
        self._cam_fovy: dict[str, float] = {}
        for c in range(self.model.ncam):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_CAMERA, c) or f"cam{c}"
            b = int(self.model.cam_bodyid[c])
            mat = np.zeros(9)
            mujoco.mju_quat2Mat(mat, self.model.cam_quat[c])
            self.cameras[name] = f"{self._entity[b]}/{name}"
            self._cam_fovy[name] = float(self.model.cam_fovy[c])
            self._geom_locals.append((self.cameras[name], _tf(self.model.cam_pos[c], mat)))
            geom_bodies.add(b)  # the camera's body chain needs per-frame transforms too

        # Bodies that need a per-frame relative transform = every ancestor of a visible geom
        # (up to, but excluding, worldbody). A fixed body's relative transform is constant, so
        # change-detection logs it once; a jointed body's changes only when its joint moves.
        needed: set[int] = set()
        for b in geom_bodies:
            while b != 0:
                needed.add(b)
                b = int(self.model.body_parentid[b])
        self._bodies = sorted(needed)
        self._last: dict[int, tuple[np.ndarray, np.ndarray]] = {}
        print(f"[robot3d] LOADED MJCF={model_file} ngeom={self.model.ngeom} "
              f"geoms_shown={len(self._geom_meshes)} tree_bodies={len(self._bodies)} "
              f"(per-frame = only bodies whose PARENT-RELATIVE transform changed)", flush=True)

    def _jid(self, name: str) -> int:
        i = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if i < 0:
            raise ValueError(f"joint {name!r} not in model")
        return i

    # --- static geometry -------------------------------------------------------------

    def _color(self, g: int) -> np.ndarray:
        matid = self.model.geom_matid[g]
        rgba = self.model.mat_rgba[matid] if matid >= 0 else self.model.geom_rgba[g]
        return (np.clip(rgba[:3], 0.0, 1.0) * 255).astype(np.uint8)

    def _static_archetype(self, g: int):
        gtype = self.model.geom_type[g]
        color = self._color(g)
        if gtype == mujoco.mjtGeom.mjGEOM_MESH:
            mid = self.model.geom_dataid[g]
            va, vn = self.model.mesh_vertadr[mid], self.model.mesh_vertnum[mid]
            fa, fn = self.model.mesh_faceadr[mid], self.model.mesh_facenum[mid]
            return rr.Mesh3D(
                vertex_positions=self.model.mesh_vert[va:va + vn].copy(),
                triangle_indices=self.model.mesh_face[fa:fa + fn].copy(),
                albedo_factor=color,
            )
        if gtype == mujoco.mjtGeom.mjGEOM_BOX:
            return rr.Boxes3D(half_sizes=[self.model.geom_size[g].copy()], colors=[color])
        return None

    def _local(self, g: int, b: int) -> rr.Transform3D:
        """Geom pose RELATIVE TO ITS BODY (constant; measured at the rest config so it exactly
        reproduces geom world once composed with the body chain)."""
        bR = self.data.xmat[b].reshape(3, 3)
        gR = self.data.geom_xmat[g].reshape(3, 3)
        return _tf(bR.T @ (self.data.geom_xpos[g] - self.data.xpos[b]), bR.T @ gR)

    def _rel(self, b: int) -> tuple[np.ndarray, np.ndarray]:
        """Body pose RELATIVE TO ITS PARENT body (world^-1 composition). Constant for a fixed
        body; changes only when the joint between b and its parent moves."""
        p = int(self.model.body_parentid[b])
        pR = self.data.xmat[p].reshape(3, 3)
        return pR.T @ (self.data.xpos[b] - self.data.xpos[p]), pR.T @ self.data.xmat[b].reshape(3, 3)

    # --- logging surface (called by visualize.py) ------------------------------------

    def camera_pinhole(self, name: str, width: int, height: int) -> tuple[str, rr.Pinhole]:
        """Frustum for a model camera: fovy from the MJCF, resolution from the actual image
        stream. MuJoCo cameras look along -Z with +Y up (OpenGL convention) -> RUB."""
        focal = 0.5 * height / np.tan(0.5 * np.radians(self._cam_fovy[name]))
        return self.cameras[name], rr.Pinhole(
            resolution=[width, height],
            focal_length=[focal, focal],
            camera_xyz=rr.ViewCoordinates.RUB,
            image_plane_distance=0.08,
        )

    def static_logs(self) -> list[tuple[str, object]]:
        """(entity, archetype) logged ONCE, static: each geom's mesh."""
        return self._geom_meshes

    def static_transforms(self) -> list[tuple[str, rr.Transform3D]]:
        """(entity, Transform3D) logged ONCE, static: each geom's / camera's body-local pose."""
        return self._geom_locals

    def transforms(self, state_vec: np.ndarray) -> dict[str, rr.Transform3D]:
        """Set qpos from the joint state, run FK, and return {body entity: parent-relative
        Transform3D} for the bodies whose relative pose CHANGED since the last frame (all of
        them on the first call). One update per moved joint; rerun composes onto the children."""
        off = 0
        for adrs, entry in zip(self._qadr, self.state_layout):
            dim = entry["dim"]
            for adr, v in zip(adrs, state_vec[off:off + dim]):
                self.data.qpos[adr] = v
            off += dim
        mujoco.mj_kinematics(self.model, self.data)
        out: dict[str, rr.Transform3D] = {}
        for b in self._bodies:
            t, R = self._rel(b)
            last = self._last.get(b)
            if last is None or not (np.allclose(t, last[0], atol=1e-6)
                                    and np.allclose(R, last[1], atol=1e-6)):
                out[self._entity[b]] = _tf(t, R)
                self._last[b] = (t.copy(), R.copy())
        return out
