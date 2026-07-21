"""Config for the sync node — pydantic-settings from the environment."""

from __future__ import annotations

from typing import Annotated, Literal

from pydantic import field_validator
from pydantic_settings import BaseSettings, NoDecode


class SyncConfig(BaseSettings):
    mode: Literal["collect"] = "collect"
    # Ordered input ids to collect (env INPUTS, comma-separated). The bundle is their latest
    # values concatenated IN THIS ORDER — the consumer's descriptor layout must match it.
    inputs: Annotated[list[str], NoDecode]
    output: str = "bundle"            # output id of the concatenated bundle
    # Warn (log) if the SPREAD between the newest and oldest component timestamps exceeds
    # this (s) at emit time — a producer lagging its peers shows as the spread. None = no
    # warning. Set it in the dataflow to ~a few frame periods (e.g. 3 / FPS).
    max_stale: float | None = None

    @field_validator("inputs", mode="before")
    @classmethod
    def _split_csv(cls, v: object) -> object:
        if isinstance(v, str):
            return [s.strip() for s in v.split(",") if s.strip()]
        return v


def load_config() -> SyncConfig:
    return SyncConfig()
