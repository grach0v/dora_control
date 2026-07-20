"""Config for the Genesis sim node — pydantic-settings from the environment.

Generic: the robot (parts, joints, ee, state-vector layout, objects, cameras) all come from
the scene descriptor named by ``SCENE``. Consumes per-part `<part>_joint_target`, emits the
whole-robot `state` bundle + per-part `<part>_tcp_pose` + cameras.
"""

from __future__ import annotations

from typing import Annotated, Literal

from pydantic import field_validator
from pydantic_settings import BaseSettings, NoDecode


class GenesisConfig(BaseSettings):
    # `sim` today; `twin`/`force_preview` are reserved (MODES raises if selected).
    mode: Literal["sim", "twin", "force_preview"] = "sim"
    scene: str                               # required — path to the scene descriptor
    cameras: Annotated[list[str], NoDecode] = ["cam_high", "cam_low", "cam_left_wrist", "cam_right_wrist"]
    width: int = 640
    height: int = 480
    encoding: str = "rgb8"                   # rgb8 or jpeg
    jpeg_quality: int = 90
    # The sim auto-paces to wall-clock: each tick it advances however many physics steps of
    # real time have actually elapsed (so it's ~realtime at ANY tick rate — no manual tuning).
    # This caps the steps per tick so a slow machine degrades to slower-than-realtime instead
    # of spiralling (it never tries to "catch up" more than this many steps in one tick).
    max_substeps: int = 40
    headless: bool = True                    # macOS: offscreen render only (no viewer)
    backend: str = ""                        # "cpu" | "gpu" | "metal"; empty = auto (gpu)

    @field_validator("cameras", mode="before")
    @classmethod
    def _split_csv(cls, v: object) -> object:
        if isinstance(v, str):
            return [s.strip() for s in v.split(",") if s.strip()]
        return v


def load_config() -> GenesisConfig:
    return GenesisConfig()
