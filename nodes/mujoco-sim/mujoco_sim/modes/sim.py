"""`sim` mode — MuJoCo physics + on-board cameras, driven by per-part joint targets.

Consumes `<part>_joint_target` for every part (arm joints, gripper opening, ...), and on each
`tick` applies them, steps physics, and publishes the whole-robot `state` bundle (per the
descriptor `state_layout`) + per-part `<part>_tcp_pose` (for parts with an ee) + cameras.
MuJoCo does NO IK — pinocchio owns it.

Inputs:  <part>_joint_target (per part), tick, program_state.
Outputs: state (bundle), <part>_tcp_pose (per ee part), <cam> images, status.
"""

from __future__ import annotations

import logging
import time
from typing import Callable

import pyarrow as pa
from dora import Node

from mujoco_sim.node_config import MujocoSimConfig
from mujoco_sim.sim import MujocoWorld, load_descriptor

logger = logging.getLogger("mujoco-sim")


class SimMode:
    def __init__(self, cfg: MujocoSimConfig, node: Node, world=None):
        self.cfg = cfg
        self.node = node
        self.desc = load_descriptor(cfg.scene)
        self.world = world if world is not None else MujocoWorld(cfg, self.desc)
        self.parts = list(self.desc["parts"].keys())
        self.img_md = {"encoding": cfg.encoding, "width": cfg.width, "height": cfg.height}
        self.joint_target: dict[str, object] = {p: None for p in self.parts}
        self._publish_now = False
        self._last_wall: float | None = None  # wall-clock of the last physics advance (auto-pace)
        self.status = ""
        self._handlers: dict[str, Callable] = {
            "tick": self._on_tick,
            "program_state": self._on_program_state,
        }
        for part in self.parts:
            self._handlers[f"{part}_joint_target"] = self._make_joint_handler(part)

    def start(self) -> None:
        self.world.build()
        self._timestep = float(self.world.model.opt.timestep)
        self._last_wall = time.monotonic()
        logger.info("mujoco-sim: ready (%s)", ", ".join(self.parts))
        self._emit_status("ready")

    def _substeps(self) -> int:
        """Physics steps to advance this tick = real time elapsed since the last advance /
        the model timestep — so the sim tracks wall-clock (~realtime) at any tick rate, with
        no manual `steps_per_tick`. Capped (max_substeps) so a slow machine degrades to
        slower-than-realtime rather than spiralling trying to catch up."""
        now = time.monotonic()
        n = round((now - self._last_wall) / self._timestep)
        self._last_wall = now
        return max(1, min(n, self.cfg.max_substeps))

    def handle(self, event) -> bool:
        return bool(self._handlers[event["id"]](event))  # KeyError on an unwired input id = loud

    def maybe_publish(self) -> None:
        if not self._publish_now:
            return
        self._publish_now = False
        self.world.apply(self.joint_target)
        self.world.step(self._substeps())

        state = self.world.state_bundle()
        ts = time.time()
        self.node.send_output("state", pa.array([float(v) for v in state]), metadata={"timestamp": ts})
        for part, pose7 in self.world.tcp_poses().items():
            self.node.send_output(f"{part}_tcp_pose", pa.array([float(v) for v in pose7]), metadata={"timestamp": ts})
        cam_md = {**self.img_md, "timestamp": time.time()}
        for cam, frame in self.world.render().items():
            self.node.send_output(cam, pa.array(frame), metadata=cam_md)

    def close(self) -> None:
        self.world.close()

    def _on_program_state(self, event) -> bool:
        return event["value"][0].as_py() == "disconnect"


    def _on_tick(self, event) -> None:
        self._publish_now = True

    def _emit_status(self, text: str) -> None:
        self.status = text
        self.node.send_output("node_state", pa.array([text]))

    def _make_joint_handler(self, part: str) -> Callable:
        def handler(event) -> None:
            self.joint_target[part] = event["value"].to_numpy()
        return handler
