"""Genesis world wrapper (within-node only).

Genesis' renderer needs the process MAIN thread (a worker thread crashes with an NSException
on macOS), so the whole sim (step + render) runs inline on the dora loop. Generic over the
scene descriptor's **parts**: each part maps to Genesis dof indices (+ optional ee link/offset).
Consumes per-part joint targets, emits the whole-robot `state` bundle (per `state_layout`) +
per-part ee poses. Genesis does no IK (pinocchio owns it). Not a shared library (CLAUDE.md).
"""

from __future__ import annotations

import logging
from pathlib import Path

import cv2
import mujoco
import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from genesis_node.node_config import GenesisConfig

logger = logging.getLogger("genesis-node")


def load_descriptor(scene_path: str) -> dict:
    path = Path(scene_path).resolve()
    desc = yaml.safe_load(path.read_text())
    desc["_root"] = path.parent.parent
    return desc


def encode(frame_rgb: np.ndarray, cfg: GenesisConfig) -> np.ndarray:
    if cfg.encoding == "jpeg":
        bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, cfg.jpeg_quality])
        if not ok:
            raise RuntimeError("JPEG encode failed")
        return buf.ravel()
    if cfg.encoding == "rgb8":
        return np.ascontiguousarray(frame_rgb).ravel()
    raise ValueError(f"unsupported encoding {cfg.encoding!r}")


def _quat_wxyz_to_R(q_wxyz):
    w, x, y, z = q_wxyz
    return Rotation.from_quat([x, y, z, w]).as_matrix()


def _np(x):
    return np.asarray(x.cpu().numpy() if hasattr(x, "cpu") else x, dtype=float)


class GenesisWorld:
    def __init__(self, cfg: GenesisConfig, descriptor: dict):
        self.cfg = cfg
        self.desc = descriptor
        self.timestep = 0.01  # physics dt (s); the mode paces substeps to wall-clock with it

    def build(self) -> None:
        import genesis as gs

        backend = {"cpu": gs.cpu, "gpu": gs.gpu, "metal": gs.metal, "": gs.gpu}[self.cfg.backend]
        gs.init(backend=backend, logging_level="warning")
        # Say what genesis actually resolved (BACKEND="" asks for gpu; taichi can still fall
        # back) — the first thing to check when the sim seems to run on the wrong device.
        logger.info("genesis: requested backend %r -> resolved %s, device %r",
                    self.cfg.backend or "gpu", gs.backend, gs.device)
        directional = gs.options.vis.DirectionalLight
        vis = gs.options.VisOptions(
            ambient_light=(0.6, 0.6, 0.6), shadow=False,
            lights=[directional(dir=(0.0, 0.0, -1.0), color=(1.0, 1.0, 1.0), intensity=6.0),
                    directional(dir=(-1.0, -1.0, -1.0), color=(1.0, 1.0, 1.0), intensity=4.0),
                    directional(dir=(1.0, 1.0, -1.0), color=(1.0, 1.0, 1.0), intensity=4.0)],
        )
        self.scene = gs.Scene(show_viewer=not self.cfg.headless,
                              sim_options=gs.options.SimOptions(dt=self.timestep), vis_options=vis)
        self.robot = self.scene.add_entity(gs.morphs.MJCF(file=str(self.desc["_root"] / self.desc["model"])))
        self.scene.add_entity(gs.morphs.Plane())
        self._cams = self._add_cameras()
        self.scene.build()
        self._index()
        self._init_pose()
        for spec in self._cams.values():  # resolve moving cameras' genesis links (post-build)
            spec["link"] = self.robot.get_link(spec["link_name"]) if spec["link_name"] else None

    def _add_cameras(self) -> dict:
        """Cameras come from the MJCF <camera> elements — the model is the single source of
        truth (same names mujoco-sim renders and rerun's 3D view frustums use); the scene
        descriptor carries no camera list. mujoco (already in this venv via genesis) parses
        them exactly (quats, defaults, fovy). A camera on a MOVING body follows that body's
        genesis link each render; one on a static body (table mounts) is baked at its rest
        world pose. MuJoCo cameras look along -Z with +Y up."""
        m = mujoco.MjModel.from_xml_path(str(self.desc["_root"] / self.desc["model"]))
        d = mujoco.MjData(m)
        mujoco.mj_forward(m, d)
        # Also read the cell home while the model is loaded: the MJCF `home` keyframe is
        # the source of truth (no home in the descriptor). {joint name: home value}.
        k = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "home")
        if k < 0:
            raise ValueError(f"no `home` keyframe in {self.desc['model']!r}")
        self._home = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j):
                      float(m.key_qpos[k][m.jnt_qposadr[j]]) for j in range(m.njnt)}
        available = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_CAMERA, c) for c in range(m.ncam)]
        cams = {}
        for name in self.cfg.cameras:
            cid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, name)
            if cid < 0:
                raise ValueError(f"camera {name!r} not in the MJCF (has: {available})")
            body = int(m.cam_bodyid[cid])
            movable, b = False, body
            while b != 0:  # any joint between the camera's body and the world?
                if m.body_jntnum[b] > 0:
                    movable = True
                    break
                b = int(m.body_parentid[b])
            local_R = np.zeros(9)
            mujoco.mju_quat2Mat(local_R, m.cam_quat[cid])
            cams[name] = {
                "cam": self.scene.add_camera(res=(self.cfg.width, self.cfg.height),
                                             fov=float(m.cam_fovy[cid]), GUI=False),
                # moving: body-local pose + the genesis link to follow; static: rest world pose
                "link_name": mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, body) if movable else None,
                "pos": m.cam_pos[cid].copy() if movable else d.cam_xpos[cid].copy(),
                "R": local_R.reshape(3, 3).copy() if movable else d.cam_xmat[cid].reshape(3, 3).copy(),
            }
        return cams

    def _dof(self, name):
        return self.robot.get_joint(name).dofs_idx_local[0]

    def _index(self) -> None:
        joint_names = [j.name for j in self.robot.joints]
        self.parts: dict[str, dict] = {}
        for name, p in self.desc["parts"].items():
            state_dofs = [self._dof(jn) for jn in p["joints"]]
            if p.get("type") == "gripper":   # genesis sets dofs directly: drive ALL carriages
                ctrl_joints = [jn for jn in joint_names
                               if jn.startswith(p["prefix"]) and "carriage" in jn]
            else:
                ctrl_joints = list(p["joints"])
            ee = p.get("ee_link")
            self.parts[name] = {
                "state_dofs": state_dofs,
                "ctrl_dofs": [self._dof(jn) for jn in ctrl_joints],
                "ctrl_joints": ctrl_joints,
                "ee_link": self.robot.get_link(ee) if ee else None,
                "ee_offset": np.asarray(p.get("ee_offset", [0, 0, 0]), dtype=float),
            }

    def _init_pose(self) -> None:
        """Gains per part + start every part at the cell home (MJCF `home` keyframe)."""
        for name, part in self.parts.items():
            ctrl = part["ctrl_dofs"]
            kp = 1000.0 if self.desc["parts"][name].get("type") == "gripper" else 200.0
            self.robot.set_dofs_kp(np.full(len(ctrl), kp), ctrl)
            self.robot.set_dofs_kv(np.full(len(ctrl), kp / 20.0), ctrl)
            pos = np.array([self._home[jn] for jn in part["ctrl_joints"]])
            self.robot.set_dofs_position(pos, ctrl)
            self.robot.control_dofs_position(pos, ctrl)

    def apply(self, joint_targets: dict) -> None:
        for name, vals in joint_targets.items():
            if vals is None:
                continue
            ctrl = self.parts[name]["ctrl_dofs"]
            vals = np.asarray(vals, dtype=float)
            pos = vals if len(vals) == len(ctrl) else np.full(len(ctrl), vals[0])
            self.robot.control_dofs_position(pos, ctrl)

    def step(self, n: int) -> None:
        for _ in range(n):
            self.scene.step()

    def state_bundle(self) -> np.ndarray:
        out = []
        for e in self.desc["state_layout"]:
            dofs = self.parts[e["part"]]["state_dofs"]
            out.append(_np(self.robot.get_dofs_position(dofs))[: e["dim"]])
        return np.concatenate(out)

    def tcp_poses(self) -> dict:
        poses = {}
        for name, part in self.parts.items():
            link = part["ee_link"]
            if link is None:
                continue
            p = _np(link.get_pos())
            q_wxyz = _np(link.get_quat())
            ee_pos = p + _quat_wxyz_to_R(q_wxyz) @ part["ee_offset"]
            w, x, y, z = q_wxyz
            poses[name] = [*ee_pos.tolist(), float(x), float(y), float(z), float(w)]
        return poses

    def render(self) -> dict:
        frames = {}
        for name, spec in self._cams.items():
            if spec["link"] is not None:  # body-mounted: compose body-local pose onto the link
                base = _np(spec["link"].get_pos())
                link_R = _quat_wxyz_to_R(_np(spec["link"].get_quat()))
                pos, R = base + link_R @ spec["pos"], link_R @ spec["R"]
            else:                          # static mount: rest world pose
                pos, R = spec["pos"], spec["R"]
            # MuJoCo camera frame: -Z view direction, +Y up.
            spec["cam"].set_pose(pos=pos, lookat=pos - R[:, 2], up=R[:, 1])
            frames[name] = encode(np.asarray(spec["cam"].render()[0]), self.cfg)
        return frames

    def close(self) -> None:
        pass
