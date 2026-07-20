"""Config for the OpenCV camera node — pydantic-settings from the environment.

Dora passes a node's config via the dataflow ``env:`` block; pydantic-settings
reads the matching env vars (case-insensitive) into the typed fields below.
"""

from __future__ import annotations

from typing import Literal

from pydantic_settings import BaseSettings


class CameraConfig(BaseSettings):
    # Identifies the stream (status label). Required.
    camera_name: str
    # Which state machine the node runs (env MODE). Only `stream` today.
    mode: Literal["stream"] = "stream"
    # OpenCV device index ("0") or a file/stream path. A pure-digit value is
    # opened as a device index.
    camera_index: str = "0"
    width: int = 640
    height: int = 480
    fps: int = 30
    # "rgb8" (raw RGB matrix) or "jpeg" (compressed bytes).
    encoding: str = "rgb8"
    jpeg_quality: int = 90


def load_config() -> CameraConfig:
    return CameraConfig()
