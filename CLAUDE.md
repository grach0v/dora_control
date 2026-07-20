# CLAUDE.md

Concise rules for working in this repo (they override default behavior).
Structure and how to run are in [README.md](README.md); **how to build a node is in
[docs/node_development.md](docs/node_development.md)** — read it before adding or
changing a node.

## Hard rules

- **Minimal changes; stop and ask.** Keep every change as small as the request
  allows — no doc/README expansion, renames, or restructuring beyond what was
  asked. If the task turns out to need a workaround or anything clever
  (symlinks, layout tricks, patched deps), STOP and present the problem +
  options for review instead of shipping the trick.

- **No shared cross-node Python library.** Nodes are language-agnostic (any could
  be rewritten in Rust/C++). Inter-node messages are **plain pyarrow arrays** with
  the layout in `docs/message_formats.md` — no shared codec / `make_pose` helpers,
  no cross-node imports. Every data message carries a `timestamp` in metadata.
- **No Python shortcuts.** No `hasattr`/`getattr` probing, no bug-hiding
  `try/except`, no lazy/conditional imports inside functions. Top-level imports,
  let errors raise, reach for a real library over hand-rolled code.
- **Each node is self-contained:** its own `uv` project under `nodes/<name>/`, its
  own `node_config.py` (a **pydantic-settings** `BaseSettings` read from env), and
  an inline `program_state == "stop"` check in the loop.
- **Sampling nodes stay lightweight — never block the loop.** A node that samples
  a periodic source does the heavy/blocking work on a background thread; the dora
  event loop must never block. Sim nodes publish the latest value when their `tick`
  fires; camera nodes self-pace — they publish each captured frame as it lands, so
  the wire rate is the configured sensor FPS (no tick). (Event-driven nodes just
  react to their inputs — this rule is about the sampling sources.)
- **Setpoint inputs use drop-oldest** (`queue_size: 1`, `queue_policy: drop_oldest`)
  so a consumer tracks the latest command, not stale ones; recorders keep all
  frames (default queue). Almost everything is the **Topic (pub/sub)** dora pattern.
- **Skip hardware that can't build/test on this Mac** (macOS arm64, no robot) —
  e.g. `realsense-camera` (no `pyrealsense2` wheel), the real Trossen SDK. Prefer
  the MuJoCo sim / OpenCV paths; leave hardware nodes stubbed with a TODO.
- **One node = one controllable thing** (one arm/camera), named via config and
  composed in the dataflow — not an `arms` list. The sim nodes (`mujoco-sim` /
  `genesis`) and `pinocchio` are the exception (whole scene / whole robot in one model).
- **No heartbeat / no liveness polling.** dora already detects node death (it's in
  the logs) — don't reinvent it. Nodes have no `status_tick`; `status` is
  EDGE-TRIGGERED, emitted only when a node's logical state changes (connected,
  homed, button pressed). The manager is an event-driven state machine (in its
  `mode`, not config) that consumes those state-change events and emits the next
  program state on `program_state`.
- **No SIGTERM/SIGINT handlers.** Tear down in `close()` on the STOP / finally
  path; don't catch signals.
- **Don't shadow an SDK** when wrapping a same-named PyPI package: the package dir
  (`rerun_node`, `lerobot_node`) and the `[project.scripts]` name (`rerun-node`,
  `lerobot-node`) must both differ from the SDK's, or imports/binaries collide.

## Don't hide this warning

`Discarding event … due to queue size limit` is a **real diagnostic**, not noise:
a producer is outpacing a consumer's drop-oldest queue. Rate-limit the producer to
~the consumer's rate (the tick / default 30 Hz); don't silence it.

## Run / test

`uv run dora …` from the repo root. dora is pinned to a **1.0.0-rc1** commit;
build the CLI once with `./scripts/build-dora.sh` (it installs `dora` into
`.venv/bin`, re-run after `uv sync`) and each node pins `dora-rs` to the same
commit via `[tool.uv.sources]`. Tests are deliberately minimal: unit tests only
where there's real pure logic (`nodes/pinocchio`, `nodes/retarget`), plus one
end-to-end smoke (`cd dataflows && uv run --project ../nodes/lerobot pytest
tests -q`) — don't add mock-driver unit tests to other nodes. Full details in
[README.md](README.md). Repeated blocks (arm pairs, camera rig) are dora modules
in `dataflows/modules/`; the `dataflows/{nodes,assets,out}` symlinks exist ONLY
because dora rejects module node paths outside the dataflow's directory —
TODO: remove them when dora lifts that restriction.