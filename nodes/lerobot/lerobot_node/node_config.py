"""Config for the lerobot node — pydantic-settings from the environment.

Records N camera streams + a concatenated state vector + a concatenated action
vector into a ``LeRobotDataset``, one episode at a time under operator control.
"""

from __future__ import annotations

from pathlib import Path
from typing import Annotated, Literal

from pydantic import field_validator
from pydantic_settings import BaseSettings, NoDecode


class RecorderConfig(BaseSettings):
    # Which state machine the node runs (env MODE). Single-mode: always `record`.
    mode: Literal["record"] = "record"
    # LeRobotDataset repo id ("user/dataset") and on-disk root. Both required.
    repo_id: str
    repo_root: Path
    fps: int = 30
    task: str = "record"                  # fallback task until a `task=` control arrives

    # Camera image inputs (CSV); each id <c> becomes feature observation.images.<c>.
    # The FIRST camera paces recording (one frame added per its frame).
    cameras: Annotated[list[str], NoDecode]
    image_width: int = 640
    image_height: int = 480

    # observation.state / action are concatenations of these input ids, in order.
    state_inputs: Annotated[list[str], NoDecode]
    action_inputs: Annotated[list[str], NoDecode]
    state_key: str = "observation.state"
    action_key: str = "action"
    state_dim: int                        # total concatenated length (validated at runtime)
    action_dim: int

    # Background threads (per camera) that write frames to disk, so add_frame
    # stays non-blocking and the dora event loop keeps servicing the daemon.
    image_writer_threads: int = 4
    # Encode video online (as frames arrive) instead of one blocking pass at save.
    streaming_encoding: bool = True

    # Timestamp alignment: per-stream ring-buffer depth, and the max time gap
    # (seconds) allowed between the pacing image and the streams paired with it.
    sync_buffer: int = 100
    sync_tolerance: float = 0.1

    @field_validator("cameras", "state_inputs", "action_inputs", mode="before")
    @classmethod
    def _split_csv(cls, v: object) -> object:
        if isinstance(v, str):
            return [s.strip() for s in v.split(",") if s.strip()]
        return v

    @field_validator("cameras", mode="after")
    @classmethod
    def _at_least_one_camera(cls, v: list[str]) -> list[str]:
        # Recording is paced by the first camera — with none, no frame is ever added.
        if not v:
            raise ValueError("CAMERAS must name at least one camera (the first one paces recording)")
        return v

    @field_validator("repo_root", mode="after")
    @classmethod
    def _expand(cls, v: Path) -> Path:
        return v.expanduser()


def load_config() -> RecorderConfig:
    return RecorderConfig()
