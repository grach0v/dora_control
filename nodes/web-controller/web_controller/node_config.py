"""Config for the web-controller node — pydantic-settings from the environment."""

from __future__ import annotations

from typing import Literal

from pydantic_settings import BaseSettings


class WebControllerConfig(BaseSettings):
    # Which state machine the node runs (env MODE). `manual` = full teleop (builds + emits the
    # whole-robot `command` bundle + episode/task); `episode` = episode/task + disconnect only,
    # no motion (for leader/policy flows where motion comes from elsewhere).
    mode: Literal["manual", "episode"] = "manual"
    host: str = "127.0.0.1"          # HTTP server bind
    port: int = 8000
    # `manual` mode only: the scene descriptor + which command layout to build (the arms/grippers
    # to control come from that layout). Unused by `episode` mode, so they stay optional.
    scene: str | None = None
    command_layout: str = "cartesian"
    # Nudge step per +/- click: m for x/y/z, rad for roll/pitch/yaw.
    pos_step: float = 0.01
    rot_step: float = 0.05
    # Drag-slider span at full deflection (±): how far a slider grab can move the target from
    # where it was grabbed — m for x/y/z, rad for roll/pitch/yaw. Bigger = larger sweeps per
    # grab (coarser); the slider re-anchors to the current target on each new grab.
    drag_span_pos: float = 0.15
    drag_span_rot: float = 0.5
    # Gripper nudge granularity as a FRACTION of the descriptor's open..closed range per click.
    # The gripper's units, bounds and home all come from the descriptor (per gripper part), so
    # this is robot-agnostic — Trossen (metres) and UR/2F-85 (driver-joint radians) both work.
    gripper_step_frac: float = 0.05


def load_config() -> WebControllerConfig:
    return WebControllerConfig()
