"""Config for the RealSense camera node — pydantic-settings from the environment."""

from __future__ import annotations

from typing import Literal

from pydantic_settings import BaseSettings


class CameraConfig(BaseSettings):
    # Identifies the stream (status label). Required.
    camera_name: str
    # Which state machine the node runs (env MODE). Only `stream` today.
    mode: Literal["stream"] = "stream"
    # RealSense device serial number (required) — selects which attached device
    # to open when more than one is present. Dataflow YAMLs may reference it as
    # `SERIAL: "${STATIC1}"`: dora expands `${VAR}` in YAML env values when
    # `dora start`/`dora build` parses the file, so run those with the robot's
    # `.env` loaded (`uv run --env-file .env dora start …`).
    serial: str
    width: int = 640
    height: int = 480
    fps: int = 30
    # "rgb8" (raw RGB matrix) or "jpeg" (compressed bytes).
    encoding: str = "rgb8"
    jpeg_quality: int = 90
    # When true, also start the depth stream and publish a `depth` output.
    enable_depth: bool = False
    # Hardware-reset + reopen the device when no frame arrives for this long.
    watchdog_s: float = 5.0
    # Seconds to wait after a hardware reset for the device to re-enumerate
    # before (re)opening the pipeline.
    reset_settle_s: float = 3.0


def load_config() -> CameraConfig:
    return CameraConfig()
