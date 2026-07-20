"""Config for the manager node — pydantic-settings read from the environment."""

from __future__ import annotations

from typing import Annotated

from pydantic import field_validator
from pydantic_settings import BaseSettings, NoDecode


class ManagerConfig(BaseSettings):
    # Which program state machine to run (env MODE); see modes/ for the registry.
    # Required — each dataflow picks its own program lifecycle. No setup-specific default.
    mode: str | None = None
    # Node `state` input ids that must each have reported at least once before the
    # program leaves BOOT (comma-separated env PRODUCERS). Empty = don't wait.
    producers: Annotated[list[str], NoDecode] = []

    @field_validator("producers", mode="before")
    @classmethod
    def _split_csv(cls, v: object) -> object:
        if isinstance(v, str):
            return [p.strip() for p in v.split(",") if p.strip()]
        return v


def load_config() -> ManagerConfig:
    return ManagerConfig()
