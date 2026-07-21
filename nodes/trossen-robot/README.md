# trossen-robot

Drives **one** real WXAI arm over Ethernet via the
[`trossen-arm`](https://pypi.org/project/trossen-arm/) SDK. One node = one arm,
identified by `NAME` (its output prefix); a bimanual rig is **two nodes** in the
dataflow (`NAME=left`, `NAME=right`). Mirrors the sim nodes' per-arm stream
interface so sim and real are swappable.

See [../../docs/node_development.md](../../docs/node_development.md) for the node
pattern (modes, edge-triggered `node_state`, STOP/`close()` teardown — no signal
handlers).

## Inputs

| input | format | |
| --- | --- | --- |
| `<name>_joint_target` | `float64[n]` rad | joint setpoint from pinocchio (follower) |
| `<name>_gripper_joint_target` | `float64[1]` opening (m) | the gripper part's setpoint (follower) |
| `tick` | timer | publish state on each tick |
| `robot_command` | utf8 | `disconnect` → fold to sleep, release, exit |
| `program_state` | utf8 | stops the node on `disconnect` |

Outputs: `<name>_tcp_pose` (`float64[7]` xyzw), `<name>_joint_state`
(`float64[6]` rad), `<name>_gripper_state` (`float64[1]` m), plus
`<name>_node_state`. Every data message carries a `timestamp` in metadata.

No command is sent until the first state read (never command blind). A joint
target whose largest jump from the current measured joints exceeds
`FOLLOWER__MAX_JOINT_JUMP` is rejected with a warning — a last-resort backstop;
pinocchio owns IK + collision safety and already bounds the step.

**Graceful disconnect.** On `robot_command` = `disconnect` (the web Disconnect button),
or on any shutdown (dora STOP / program_state disconnect), the arm is moved to a staged
then folded **sleep** pose (blocking `set_all_positions`) and only then released
(`idle` + `cleanup`) — so it never drops. The disconnect path also exits the node.

## Modes (`MODE`)

- **`follower`** (default) — joint control only: consume `<name>_joint_target` (from the
  pinocchio node) and command joints via `set_all_positions` (the gripper part's target
  rides along); publish state each `tick`.
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
| `NAME` | `left` | the arm's id: output prefix |
| `IP` | `192.168.1.5` | the arm controller's IP |
| `MODE` | `follower` | `follower` or `leader` |
| `FOLLOWER__GOAL_TIME` | `0.0` | horizon (s) for streamed setpoints |
| `FOLLOWER__MAX_JOINT_JUMP` | `0.5` | max rad any joint may jump from the current measured |
| `FOLLOWER__STAGED_POSE` / `FOLLOWER__SLEEP_POSE` | folded / zero | graceful-disconnect poses |
| `LEADER__STAGED_POSE` / `LEADER__SLEEP_POSE` | folded / zero | leader staging + park poses |
