# Writing a node

Every node in this repo follows one shape. This is the contract; read it before
adding or changing a node. The hard rules it builds on live in
[CLAUDE.md](../CLAUDE.md); message layouts live in
[message_formats.md](message_formats.md).

## What a node is

A node is a self-contained `uv` project under `nodes/<name>/` that exchanges
**plain Apache Arrow messages** with other nodes — no shared Python library, no
cross-node imports, so any node could be rewritten in another language. It is
**state + a `mode`**: a config-selected state machine. `main.py` is a
mode-agnostic dora skeleton; the behaviour lives in `modes/`.

```
nodes/<name>/
  <pkg>/__init__.py        # short docstring; a NODE_ID constant if handy
  <pkg>/main.py            # dora skeleton: load_config → Node() → MODES[mode] → loop
  <pkg>/node_config.py     # pydantic-settings BaseSettings + load_config()
  <pkg>/modes/__init__.py  # MODES = {"<mode>": <Mode class>}
  <pkg>/modes/<mode>.py    # one mode: its *Config + a class start()/handle()/close()
  <pkg>/inputs.py          # UnknownInput exception (if the node validates input ids)
  <pkg>/debug.py           # maybe_start_debugger (copy verbatim; pick a fresh port)
  pyproject.toml           # name, [project.scripts], packages
  tests/                   # unit tests + one test_dataflow.yml + fake_*.py producers
```

## The skeleton (`main.py`)

```python
def main() -> int:
    maybe_start_debugger("<name>", <port>)   # first line; see the port table in README
    cfg = load_config()
    node = Node()
    mode = MODES[cfg.mode](cfg.active, node, ...)   # KeyError on a bad MODE = loud
    mode.start()
    try:
        for event in node:
            if event["type"] == "STOP":
                break
            if event["type"] != "INPUT":
                continue
            if mode.handle(event):   # True -> program_state stop / done
                break
    finally:
        mode.close()                 # always tear down (flush, release hardware, …)
    return 0
```

Producers that must keep moving between events (sim physics, etc.) may poll with
`node.next(timeout=…)` and do work between events, but the load → loop → `finally:
close()` shape is the same.

## Config (`node_config.py`)

A pydantic-settings `BaseSettings` read from the dataflow's `env:` block, with a
`load_config()` factory. A `mode: Literal[...]` field selects the state machine.
For multi-mode nodes, put mode-specific params in a nested sub-config and expose an
`active` property:

```python
class NodeConfig(BaseSettings):
    model_config = SettingsConfigDict(env_nested_delimiter="__")
    mode: Literal["record", "visualize"] = "visualize"
    record: RecordConfig = Field(default_factory=RecordConfig)
    visualize: VisualizeConfig = Field(default_factory=VisualizeConfig)

    @property
    def active(self):
        return {"record": self.record, "visualize": self.visualize}[self.mode]
```

A dataflow then sets `MODE=record` + `RECORD__FPS=30` (the `__` is the nesting
delimiter). Parse comma-separated env lists with `Annotated[list[str], NoDecode]`
+ a `@field_validator(..., mode="before")` that splits on `,` (NoDecode stops
pydantic JSON-parsing the value first).

## A mode (`modes/<mode>.py`)

A mode is the transition function. It is its `*Config` plus a class with three
methods:

- `start()` — set up resources, emit the first `node_state` (`ready`).
- `handle(event) -> bool` — dispatch on `event["id"]` via a handler table; return
  `True` to stop the loop. An input id the node was not configured for should
  **raise `UnknownInput`**, not be silently dropped (the manager is the one
  exception — it warns and continues, being the supervisor).
- `close()` — tear down (flush buffers, release hardware). Runs on every exit path
  via the skeleton's `finally`.

```python
self._handlers = {"program_state": self._on_program_state, "tick": self._on_tick, ...}
def handle(self, event):
    handler = self._handlers.get(event["id"])
    if handler is None:
        raise UnknownInput(f"<mode>: no handler for {event['id']!r}")
    return bool(handler(event))
```

## node_state & program_state

- **Program state:** the `manager` emits `program_state` — with the `teleop` mode
  machine that is `boot` / `homing` / `teleop` / `disconnect`. Every node breaks its
  loop on `program_state == "disconnect"` (and on the dora `STOP` event); nodes
  that move the robot gate motion on the `teleop` stage.
- **node_state is EDGE-TRIGGERED — no heartbeat, no liveness polling.** dora already
  detects node death (it's in the logs). A node emits `node_state` (a state token,
  see docs/message_formats.md) only when its logical state changes: `ready` once on
  startup, then edges like `homing`/`homed`. Keep a `self.status` and a small
  emit-on-change helper; never re-emit an unchanged state on a timer.
- **No signal handlers.** Don't catch SIGTERM/SIGINT. Process STOP promptly so the
  `finally: close()` runs within dora's stop grace; hardware teardown belongs in
  `close()`, reached on the STOP / program_state-disconnect / error paths.

## One node controls one thing

A node drives a single controllable thing — one arm (follower or leader), one
camera, one mobile base — identified by a configurable `name` used as its output
prefix (`<name>_tcp_pose`). Compose by **naming**: bimanual hardware is two
`trossen-robot` nodes (`NAME=left`, `NAME=right`) wired in the dataflow, not one
node with an `arms` list. (The sim nodes `mujoco-sim` / `genesis` and the `pinocchio`
node are the deliberate exception: they load the whole scene / whole robot — both arms +
all cameras — in one model, reading the shared scene descriptor.)

## Producers, queues, threads

- Producers are **lightweight**: do heavy/blocking work (capture, rendering) on a
  background thread and never block the dora loop. Sim nodes publish the latest
  sample on a `tick`; camera nodes self-pace instead — they publish each captured
  frame as it lands, so the wire rate IS the configured sensor FPS (poll with
  `node.next(timeout=…)`, publish between events).
- **Setpoint inputs use drop-oldest** (`queue_size: 1`, `queue_policy:
  drop_oldest`) so a consumer tracks the latest command; recorders keep all frames
  (default queue). `Discarding event … due to queue size limit` is a real
  diagnostic — rate-limit the producer, don't silence it.

## Naming — avoid SDK collisions

Two collisions we have actually hit; check both when a node wraps a PyPI SDK of
the same name (e.g. `rerun`, `lerobot`):

- **Package name** must not shadow the SDK: name the package `rerun_node` /
  `lerobot_node`, never `rerun` / `lerobot`, or `import rerun` resolves to your own
  package.
- **Console-script name** (`[project.scripts]`) must not collide with a script the
  SDK installs into the same venv: name it `rerun-node` / `lerobot-node`. If both
  define `rerun`, whichever installs last wins and the dataflow silently launches
  the wrong binary.

## Tests

Two layers, deliberately thin:

- **Fast unit tests** only for nodes with genuinely non-trivial pure logic
  (currently `pinocchio` IK/safety and `retarget` mapping math): plain pytest on
  the math, no dora subprocess, no mock-driver theater. Run with
  `cd nodes/<name> && uv run pytest -q`.
- **One end-to-end smoke** for the whole graph:
  `dataflows/tests/test_smoke.py` runs the local UR5e sim dataflow headless
  (a scripted driver replaces the web UI, sim cameras re-enabled) and checks a
  recorded episode.

Don't add per-node `dora run` integration tests that spin up a daemon and
regex-scrape a logger's stdout — they're slow and brittle and duplicate the unit
tests or the e2e. Test logic in unit tests; trust the e2e for wiring.
