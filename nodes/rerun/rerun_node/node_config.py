"""Config for the rerun node — pydantic-settings from the environment.

Shared params (just `app_id`) stay top-level; everything mode-specific lives in a
nested per-mode sub-config selected by ``MODE``. With ``env_nested_delimiter``,
a dataflow sets e.g. ``MODE=record`` + ``RECORD__FPS=30`` + ``RECORD__IMAGE_MODE=h264``,
or ``MODE=visualize`` + ``VISUALIZE__SINK=spawn``.
"""

from __future__ import annotations

from typing import Literal

from pydantic import Field
from pydantic_settings import BaseSettings, SettingsConfigDict

from rerun_node.modes.record import RecordConfig
from rerun_node.modes.visualize import VisualizeConfig


class RerunConfig(BaseSettings):
    model_config = SettingsConfigDict(env_nested_delimiter="__")

    # Rerun application id (shared by both modes).
    app_id: str = "dora"
    # Which state machine to run (env MODE).
    mode: Literal["record", "visualize"] = "visualize"
    # Per-mode params; only the active one is used (see `active`).
    record: RecordConfig = Field(default_factory=RecordConfig)
    visualize: VisualizeConfig = Field(default_factory=VisualizeConfig)

    @property
    def active(self) -> RecordConfig | VisualizeConfig:
        return {"record": self.record, "visualize": self.visualize}[self.mode]  # explicit, no getattr


def load_config() -> RerunConfig:
    return RerunConfig()
