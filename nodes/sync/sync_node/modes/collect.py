"""`collect` mode — latest-of-each aggregator, emitting the moment a cycle completes.

Inputs are event-driven: each arrival updates that id's latest `(values, timestamp)` and
marks it fresh. As soon as EVERY configured input has a fresh (not-yet-bundled) sample, one
concatenated bundle is emitted immediately (in the configured INPUTS order) — not on the next
tick, so the bundle never trails the producers by a timer phase. The bundle's `timestamp` is
the OLDEST component timestamp (honest for downstream alignment). Before every input has been
seen once we don't emit a partial bundle.

The `tick` is only a watchdog: if any input's latest is older than `MAX_STALE`, we
`log.warning` it (a producer stalled / fell behind — a phase skew shows as the lagging
input's age); if inputs are still missing entirely we report `waiting`. Misalignment is
surfaced (not silent) but kept OUT of `node_state`, which stays a simple liveness value
(`waiting` / `synced`).

Inputs:  <each configured input>, tick (watchdog), program_state.
Outputs: <output> (concatenated bundle), node_state.
"""

from __future__ import annotations

import logging
import time
from typing import Callable

import numpy as np
import pyarrow as pa
from dora import Node

from sync_node.node_config import SyncConfig

logger = logging.getLogger("sync")


class CollectMode:
    def __init__(self, cfg: SyncConfig, node: Node, now_fn: Callable[[], float] = time.time):
        self.cfg = cfg
        self.node = node
        self._now = now_fn
        self.latest: dict[str, tuple[np.ndarray, float]] = {}  # id -> (values, timestamp)
        self._fresh: set[str] = set()  # ids updated since the last emitted bundle
        self.status = ""
        self._last_warning: str | None = None
        self._handlers: dict[str, Callable] = {
            "tick": self._on_tick,
            "program_state": self._on_program_state,
        }
        for input_id in cfg.inputs:
            self._handlers[input_id] = self._make_input_handler(input_id)

    def start(self) -> None:
        self._set_status("ready")

    def handle(self, event) -> bool:
        return bool(self._handlers[event["id"]](event))  # KeyError on an unwired input id = loud

    def close(self) -> None:
        pass

    def _on_program_state(self, event) -> bool:
        return event["value"][0].as_py() == "disconnect"


    def _set_status(self, text: str) -> None:
        if text != self.status:
            self.status = text
            self.node.send_output("node_state", pa.array([text]))

    def _warn(self, text: str | None) -> None:
        """Log a warning independently of status, only when it changes (no per-tick spam)."""
        if text != self._last_warning:
            if text is not None:
                logger.warning("sync: %s", text)
            self._last_warning = text

    def _make_input_handler(self, input_id: str) -> Callable:
        def handler(event) -> None:
            ts = (event.get("metadata") or {}).get("timestamp")
            self.latest[input_id] = (event["value"].to_numpy(), float(ts) if ts is not None else self._now())
            self._fresh.add(input_id)
            if self._fresh.issuperset(self.cfg.inputs):
                self._emit()
        return handler

    def _emit(self) -> None:
        """One complete cycle: every input has a fresh sample — bundle and send now."""
        self._fresh.clear()
        bundle = np.concatenate([np.asarray(self.latest[i][0], dtype=float) for i in self.cfg.inputs])
        oldest = min(self.latest[i][1] for i in self.cfg.inputs)
        self.node.send_output(self.cfg.output, pa.array([float(v) for v in bundle]),
                              metadata={"timestamp": oldest})
        self._set_status("synced")

    def _on_tick(self, event) -> None:
        """Watchdog only — emission happens on input completion, not here."""
        missing = [i for i in self.cfg.inputs if i not in self.latest]
        if missing:
            self._set_status("waiting")
            self._warn(f"waiting for {','.join(missing)}")
            return

        if self.cfg.max_stale is not None:
            now = self._now()
            stale = [f"{i} {(now - self.latest[i][1]) * 1000:.0f}ms"
                     for i in self.cfg.inputs if now - self.latest[i][1] > self.cfg.max_stale]
            self._warn("stale inputs: " + ", ".join(stale) if stale else None)
