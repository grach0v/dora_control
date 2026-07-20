# trossen-robot

Drives **one** real WXAI arm over Ethernet via the
[`trossen-arm`](https://pypi.org/project/trossen-arm/) SDK. One node = one arm,
identified by `NAME` (its output prefix + status label); a bimanual rig is **two
nodes** in the dataflow (`NAME=left`, `NAME=right`). Mirrors `trossen-sim`'s
per-arm stream interface so the two are swappable.

See [../../docs/node_development.md](../../docs/node_development.md) for the node
pattern (modes, `status` liveness, STOP/`close()` teardown — no signal handlers).

## Inputs

| input | format | |
| --- | --- | --- |
| `<name>_tcp_target` | `float64[7]` `[x,y,z, qx,qy,qz,qw]` (xyzw) | cartesian setpoint (`CONTROL_SPACE=cartesian`) |
| `<name>_joint_target` | `float64[n]` rad | joint setpoint from pinocchio (`CONTROL_SPACE=joint`) |
| `<name>_gripper_target` / `<name>_gripper_joint_target` | `float64[1]` opening in metres | gripper opening — `_gripper_target` in cartesian mode, the gripper part's `_gripper_joint_target` in joint mode |
| `tick` | timer | publish state on each tick |
| `robot_command` | utf8 | `disconnect` → fold to sleep, release, exit |
| `program_state` | utf8 | stops the node on `stop` |

Outputs (follower): `<name>_tcp_pose` (`float64[7]` xyzw), `<name>_joint_state`
(`float64[6]` rad), `<name>_gripper_state` (`float64[1]` m), plus `status`. Every
data message carries a `timestamp` in metadata.

No command is sent until the first target arrives (the web controller seeds its
target from the published pose, so there is no startup jump). Targets farther than
`FOLLOWER__MAX_POS_JUMP` metres from the current TCP are rejected with a `status`
line.

**Graceful disconnect.** On `robot_command` = `disconnect` (the web Disconnect button),
or on any shutdown (dora STOP / program_state stop), the arm is moved to a staged then
folded **sleep** pose (blocking `set_all_positions`) and only then released
(`idle` + `cleanup`) — so it never drops. The disconnect path also exits the node,
which lets the manager stop the rest of the dataflow.

## Modes (`MODE`)

- **`follower`** (default) — drives the arm in position mode, publishes state each `tick`.
  `FOLLOWER__CONTROL_SPACE` picks the command space:
  - `cartesian` (default): consume `<name>_tcp_target`; firmware does Cartesian IK; reject
    targets > `MAX_POS_JUMP` from the current TCP.
  - `joint`: consume `<name>_joint_target` (from the pinocchio node) and command joints via
    `set_all_positions` (gripper rides along); reject jumps > `MAX_JOINT_JUMP`. Pinocchio is
    then the sole owner of IK + collision safety; this guard is a last-resort backstop.
- **`leader`** — the arm is configured backdrivable (external-effort); on each `tick` the
  hand-moved pose is read and published as **state** (`<name>_tcp_pose`, `<name>_joint_state`,
  `<name>_gripper_state`), identical to a follower. A `sync` node bundles the leaders into the
  `command` (joint layout) that pinocchio gates, so leader teleop is collision-checked before
  reaching the followers. A `*_target` arriving at a leader is a wiring bug and raises.
  (Leader real-hardware bring-up is not yet verified.)
- **`base`** *(TODO — not implemented)* — a mobile base (Trossen Mobile) via the
  `trossen-slate` SDK: consume `base_joint_target` (`[v, ω]` body twist, passthrough-gated by
  pinocchio) and publish base odometry as state. Add when the hardware is available.

## Config (env)

Top-level params + mode-specific params namespaced under the mode
(`env_nested_delimiter="__"`).

| var | default | |
| --- | --- | --- |
| `NAME` | `left` | the arm's id: output prefix + status label |
| `IP` | `192.168.1.5` | the arm controller's IP |
| `MODE` | `follower` | `follower` or `leader` |
| `FOLLOWER__CONTROL_SPACE` | `cartesian` | `cartesian` (`tcp_target`) or `joint` (`joint_target`) |
| `FOLLOWER__GOAL_TIME` | `0.0` | horizon (s) for streamed setpoints |
| `FOLLOWER__MAX_POS_JUMP` | `0.10` | cartesian: max metres between current TCP and a new target |
| `FOLLOWER__MAX_JOINT_JUMP` | `0.5` | joint: max rad any joint may jump from the current measured |
| `FOLLOWER__STAGED_POSE` / `FOLLOWER__SLEEP_POSE` | folded / zero | graceful-disconnect poses |

## Test

```sh
uv run pytest -q   # FakeDriver-based; no hardware needed, runs on macOS too
```
