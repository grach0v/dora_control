"""Config for the UR5e real-robot node — pydantic-settings from the environment.

The node drives ONE thing (a single UR5e + its Robotiq 2F-85); the bimanual cell is
two of these nodes in the dataflow (NAME=left, NAME=right). `name` is the output
prefix and status label. Only `follower` mode exists (joint control); its params live
in a nested sub-config.
"""

from __future__ import annotations

from typing import Literal

from pydantic import Field
from pydantic_settings import BaseSettings, SettingsConfigDict

from ur5e_robot.modes.follower import FollowerConfig


class UR5eRobotConfig(BaseSettings):
    model_config = SettingsConfigDict(env_nested_delimiter="__")

    # The controlled thing's id: output prefix (`<name>_tcp_pose`, ...) and status
    # label. One node = one arm, so NAME=left and NAME=right for the bimanual cell.
    name: str = "left"
    # The UR controller's IP (RTDE + the Robotiq URCap socket both live here).
    ip: str = "192.168.1.102"
    # Only joint control (env MODE); kept for symmetry with the other robot nodes.
    mode: Literal["follower"] = "follower"
    follower: FollowerConfig = Field(default_factory=FollowerConfig)

    @property
    def active(self) -> FollowerConfig:
        return {"follower": self.follower}[self.mode]  # explicit, no getattr


def load_config() -> UR5eRobotConfig:
    return UR5eRobotConfig()
