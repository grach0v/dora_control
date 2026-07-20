# manager

Owns the **program lifecycle** for a dataflow as an explicit state machine. It is
the one node that sees the whole system: it watches each producer's liveness,
decides when the program is healthy enough to `start`, relays operating-stage
changes, and tears the dataflow down with `stop`.

See [../../docs/node_development.md](../../docs/node_development.md) for the node
pattern; this README covers only what is manager-specific.

```
INIT ─▶ STARTING ─(all producers alive)─▶ RUNNING ─(producer silent / STOP)─▶ STOPPING
            └─(startup timeout)──────────────────────────────────────────────▶ STOPPING
```

Every other node subscribes to `program_state: manager/program_state` and breaks its loop
when the value is `stop`. **Liveness:** each producer emits a `status` line ~1 Hz;
the manager treats *any* message from a producer as proof of life (logging the
text when it changes). Unlike worker nodes it has no inline `program_state` self-stop
check — it *owns* `program_state` and tears down on dora's `STOP`.

## States

| state      | meaning |
| ---------- | ------- |
| `INIT`     | node constructed, before it begins waiting |
| `STARTING` | waiting (≤ `STARTUP_TIMEOUT`) for every producer to be alive; → `start` when all are |
| `RUNNING`  | live; checking liveness each tick; `stage` selects an operating mode |
| `STOPPING` | a producer went silent, startup timed out, or dora sent `STOP` → emit `stop`, exit |

## Inputs

| id           | source                | meaning |
| ------------ | --------------------- | ------- |
| `tick`       | `dora/timer/millis/N` | drives the state machine / liveness checks (e.g. 1 Hz) |
| `<producer>` | `<producer>/status`   | one input per configured producer id; any message = alive, payload logged as health |
| `stage`      | any node (optional)   | an operating-mode string (e.g. `teleop`/`policy`), re-broadcast verbatim on `program_state` (only while RUNNING) |

## Outputs

| id          | payload | meaning |
| ----------- | ------- | ------- |
| `program_state` | string  | current program command: `start` / `<mode>` / `stop` |
| `status`    | string  | human-readable health line for logs |

## Config (env)

| var                | default              | meaning |
| ------------------ | -------------------- | ------- |
| `MODE`             | `trossen_stationary` | which program_state transition function to run (registry in `modes/`) |
| `PRODUCERS`        | `""`                 | comma-separated producer ids that must each be alive before `start`. Empty ⇒ `start` on first tick. |
| `STARTUP_TIMEOUT`  | `30`                 | seconds to wait for all producers before giving up → `stop` |
| `LIVENESS_TIMEOUT` | `5`                  | seconds a producer may be silent before it is declared dead → `stop` |
| `STAGES`           | `teleop,policy`      | operating *stages* the `stage` input may select while RUNNING; the first is entered at `start` (distinct from `MODE`) |

## Example wiring

```yaml
  - id: manager
    path: ../manager/main.py
    inputs:
      tick: dora/timer/millis/1000
      left_follower: left_follower/status
      front_cam: front_cam/status
    outputs: [program_state, status]
    env:
      PRODUCERS: left_follower,front_cam
```

## Test

```bash
cd nodes/manager && uv run pytest tests -q
```
