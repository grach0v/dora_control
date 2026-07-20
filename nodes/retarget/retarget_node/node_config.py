"""Config for the retarget node — pydantic-settings from the environment.

The FOLLOWER robot's geometry (which parts exist, the cartesian command layout, the gripper
ranges) all come from the follower's scene descriptor named by ``SCENE`` — same file the
follower-side pinocchio reads, so the emitted `command` bundle matches its layout by
construction. The leader side needs no descriptor: leaders publish their TCP in their own
base frame and the mapping is delta-based, so only the leader gripper's value range and the
leader→follower frame alignment are configuration.
"""

from __future__ import annotations

from typing import Annotated, Literal

from pydantic import field_validator
from pydantic_settings import BaseSettings, NoDecode


class RetargetConfig(BaseSettings):
    mode: Literal["follow"] = "follow"
    scene: str                          # required — the FOLLOWER's scene descriptor
    command_layout: str = "cartesian"   # which descriptor command_layouts.<name> to emit

    # Translation scale leader→follower (the robots differ in size: a small leader sweep
    # should cover the follower workspace). Rotation deltas are always applied 1:1.
    scale: float = 1.0
    # Fixed rotation A (roll,pitch,yaw rad, env csv) expressing LEADER-base-frame directions
    # in the FOLLOWER world frame: "push the leader forward" must mean "forward" on the
    # follower. Both frames are z-up, so in practice this is a yaw. Applied to translation
    # deltas and conjugating rotation deltas.
    align_rpy: Annotated[list[float], NoDecode] = [0.0, 0.0, 0.0]

    # The leader gripper's published value range (Trossen WXAI: metres, open 0.044 → closed
    # 0.0). The follower range comes from the descriptor's gripper parts (open/closed), so
    # the opening is mapped linearly between the two, direction-safe.
    leader_gripper_open: float = 0.044
    leader_gripper_closed: float = 0.0

    # Per-tick clamp on how fast the commanded target may MOVE (m/tick, rad/tick). A leader
    # jump (bumped arm, torque-off flop) becomes a bounded glide instead of a teleport —
    # on top of pinocchio's own joint rate-limit + collision gate downstream.
    max_pos_step: float = 0.03
    max_rot_step: float = 0.15

    @field_validator("align_rpy", mode="before")
    @classmethod
    def _split_csv(cls, v: object) -> object:
        if isinstance(v, str):
            return [float(s.strip()) for s in v.split(",") if s.strip()]
        return v


def load_config() -> RetargetConfig:
    return RetargetConfig()
