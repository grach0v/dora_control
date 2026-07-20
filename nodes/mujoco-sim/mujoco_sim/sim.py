"""MuJoCo world + background camera renderer (within-node only).

Generic over the scene descriptor's **parts**: each part maps to MuJoCo actuators (arm
joints, or a gripper actuator) + optional ee site. Consumes per-part joint targets, emits the
whole-robot `state` bundle (per the descriptor `state_layout`) + per-part ee poses. MuJoCo
does no IK (pinocchio owns it). Camera rendering runs on a background thread (MuJoCo is happy
off the main thread). Not a shared library: lives in the mujoco-sim node only (CLAUDE.md).
"""

from __future__ import annotations

import threading
import time
from pathlib import Path

import cv2
import mujoco
import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from mujoco_sim.node_config import MujocoSimConfig


def load_descriptor(scene_path: str) -> dict:
    path = Path(scene_path).resolve()
    desc = yaml.safe_load(path.read_text())
    desc["_root"] = path.parent.parent
    return desc


def encode(frame_rgb: np.ndarray, cfg: MujocoSimConfig) -> np.ndarray:
    if cfg.encoding == "jpeg":
        bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, cfg.jpeg_quality])
        if not ok:
            raise RuntimeError("JPEG encode failed")
        return buf.ravel()
    if cfg.encoding == "rgb8":
        return np.ascontiguousarray(frame_rgb).ravel()
    raise ValueError(f"unsupported encoding {cfg.encoding!r}")


class CameraRenderer:
    """Background thread keeping the latest encoded frame per camera (see the MuJoCo node README)."""

    def __init__(self, model, data, data_lock, cfg: MujocoSimConfig):
        self._model, self._data, self._data_lock, self._cfg = model, data, data_lock, cfg
        self._frames: dict[str, np.ndarray] = {}
        self._frame_ts = 0.0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="mujoco-render", daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2.0)

    def latest(self):
        with self._lock:
            return dict(self._frames), self._frame_ts

    def _run(self):
        renderer = mujoco.Renderer(self._model, height=self._cfg.height, width=self._cfg.width)
        period = 1.0 / 30.0
        try:
            while not self._stop.is_set():
                start = time.monotonic()
                rendered = {}
                for cam in self._cfg.cameras:
                    with self._data_lock:
                        renderer.update_scene(self._data, camera=cam)
                    rendered[cam] = encode(renderer.render(), self._cfg)
                with self._lock:
                    self._frames, self._frame_ts = rendered, time.time()
                self._stop.wait(max(0.0, period - (time.monotonic() - start)))
        finally:
            renderer.close()


class MujocoWorld:
    def __init__(self, cfg: MujocoSimConfig, descriptor: dict):
        self.cfg = cfg
        self.desc = descriptor
        self.data_lock = threading.Lock()
        self._renderer: CameraRenderer | None = None

    def _id(self, objtype, name):
        i = mujoco.mj_name2id(self.model, objtype, name)
        if i < 0:
            raise ValueError(f"{name!r} not found ({objtype})")
        return i

    def build(self) -> None:
        scene_file = str(self.desc["_root"] / self.desc["model"])
        self.model = mujoco.MjModel.from_xml_path(scene_file)
        self.data = mujoco.MjData(self.model)

        # Per-part index maps: qpos to read state, actuators to drive, optional ee site.
        self.parts: dict[str, dict] = {}
        for name, p in self.desc["parts"].items():
            qpos = [self.model.jnt_qposadr[self._id(mujoco.mjtObj.mjOBJ_JOINT, jn)] for jn in p["joints"]]
            if "actuator" in p:   # gripper-style: one actuator drives the opening
                act = [self._id(mujoco.mjtObj.mjOBJ_ACTUATOR, p["actuator"])]
            else:                 # arm-style: one position actuator per joint (same name)
                act = [self._id(mujoco.mjtObj.mjOBJ_ACTUATOR, jn) for jn in p["joints"]]
            ee = p.get("ee_frame")
            self.parts[name] = {
                "qpos": qpos, "act": act,
                "ee_site": self._id(mujoco.mjtObj.mjOBJ_SITE, ee) if ee else None,
            }
            home = p.get("home")
            if home is not None:
                for adr, v in zip(qpos, home):
                    self.data.qpos[adr] = v
                for a, v in zip(act, home):
                    self.data.ctrl[a] = v
        mujoco.mj_forward(self.model, self.data)
        self.state_layout = self.desc["state_layout"]
        self._renderer = CameraRenderer(self.model, self.data, self.data_lock, self.cfg)
        self._renderer.start()

    def apply(self, joint_targets: dict) -> None:
        with self.data_lock:
            for name, vals in joint_targets.items():
                if vals is None:
                    continue
                for a, v in zip(self.parts[name]["act"], np.asarray(vals, dtype=float)):
                    self.data.ctrl[a] = v

    def step(self, n: int) -> None:
        with self.data_lock:
            for _ in range(n):
                mujoco.mj_step(self.model, self.data)

    def state_bundle(self) -> np.ndarray:
        """Whole-robot measured-joint vector, in `state_layout` order."""
        with self.data_lock:
            return np.concatenate([
                np.array([self.data.qpos[adr] for adr in self.parts[e["part"]]["qpos"]])
                for e in self.state_layout
            ])

    def tcp_poses(self) -> dict[str, list[float]]:
        """Measured ee pose7 [x,y,z,qx,qy,qz,qw] for every part that has an ee."""
        poses = {}
        with self.data_lock:
            for name, p in self.parts.items():
                if p["ee_site"] is None:
                    continue
                pos = self.data.site_xpos[p["ee_site"]].copy()
                quat_xyzw = Rotation.from_matrix(self.data.site_xmat[p["ee_site"]].reshape(3, 3)).as_quat()
                poses[name] = [*pos.tolist(), *quat_xyzw.tolist()]
        return poses

    def render(self) -> dict:
        frames, _ = self._renderer.latest()
        return frames

    def close(self) -> None:
        if self._renderer is not None:
            self._renderer.stop()
