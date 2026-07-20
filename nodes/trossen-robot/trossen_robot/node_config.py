"""Config for the Trossen real-robot node — pydantic-settings from the environment.

The node drives ONE thing (a single arm); bimanual hardware is two of these nodes
in the dataflow (NAME=left, NAME=right). `name` is the output prefix and status
label; mode-specific params live in a nested per-mode sub-config selected by
``MODE`` (``follower`` / ``leader``).
"""

from __future__ import annotations

from typing import Literal

from pydantic import Field
from pydantic_settings import BaseSettings, SettingsConfigDict

from trossen_robot.modes.follower import FollowerConfig
from trossen_robot.modes.leader import LeaderConfig


class TrossenRobotConfig(BaseSettings):
    model_config = SettingsConfigDict(env_nested_delimiter="__")

    # The controlled thing's id: output prefix (`<name>_tcp_pose`, ...) and status
    # label. One node = one arm, so e.g. NAME=left and NAME=right for a bimanual rig.
    name: str = "left"
    # The arm controller's IP.
    ip: str = "192.168.1.5"
    # Which state machine the node runs (env MODE). follower drives the arm to
    # setpoints; leader is backdrivable and publishes its hand-moved pose.
    mode: Literal["follower", "leader"] = "follower"
    follower: FollowerConfig = Field(default_factory=FollowerConfig)
    leader: LeaderConfig = Field(default_factory=LeaderConfig)

    @property
    def active(self) -> FollowerConfig | LeaderConfig:
        return {"follower": self.follower, "leader": self.leader}[self.mode]  # explicit, no getattr


def load_config() -> TrossenRobotConfig:
    return TrossenRobotConfig()
