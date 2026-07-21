# Message formats

The single source of truth for what flows between nodes. Inter-node messages are
**plain pyarrow arrays** — there is no shared codec, no `make_pose` helper, and no
cross-node imports (see [CLAUDE.md](../CLAUDE.md)). Any node could be rewritten in
Rust/C++ and still interoperate, so producers and consumers agree only on the
layouts below. Each node restates the subset it uses at the top of its `main.py`;
this file is the union.

## Conventions

- **Every data message carries a `timestamp`** in its metadata: a `float64` of
  wall-clock seconds (`time.time()`) taken when the sample was captured/formed.
  Recorders use it to align streams; it is the one universal metadata key.
- Arrays are **flat 1-D pyarrow arrays**. Vectors (pose, joints) are fixed-length;
  images are a raveled byte/short buffer whose shape lives in metadata.
- Quaternions are **xyzw order** `[qx, qy, qz, qw]` on the wire. (scipy uses wxyz
  internally — nodes convert at the boundary; don't change the wire order.)
- Control/state messages (`node_state`, `program_state`, `robot_command`, …) are
  single-element `utf8` arrays and carry no required metadata.

## Robot command / state (generic, descriptor-defined)

The robot is a set of **parts** (arm, gripper, base, leg ...) — each a named set of joints
with an optional task (ee) frame. The scene descriptor declares ordered **vector layouts**:
`command_layouts.<name>` and `state_layout`, where each entry is
`{part, space, quantity, dim}` with `space ∈ {joint, cartesian}`, `quantity ∈ {position,
velocity, effort}`. `n` (a part's DOF) and these layouts come from the descriptor, never
hardcoded. (See `assets/.../scenes/*.yaml`.)

**Bundled whole-robot messages** — flat `float64` arrays following a descriptor layout:

| message | layout | meaning |
|---|---|---|
| `command` | `float64[Σ dim]` per `command_layouts.<COMMAND_LAYOUT>` | one whole-robot setpoint snapshot (e.g. `[left pose7 \| left_gripper 1 \| right pose7 \| right_gripper 1]`, or all-joint, or +`base twist`). Drop-oldest. → pinocchio |
| `state` | `float64[Σ dim]` per `state_layout` | one whole-robot measured-joint snapshot. → pinocchio (feedback) + recorders |

**Per-part messages:**

| message | layout | meaning |
|---|---|---|
| `<part>_joint_target` | `float64[n]` | SAFE joint setpoint for one part (output of pinocchio) → sim / robot |
| `<part>_tcp_pose` | `float64[7]` `[x,y,z,qx,qy,qz,qw]` | measured ee pose (sim/robot → web display, rerun) |
| `<part>_joint_state` / `<part>_gripper_state` | `float64[n]` / `float64[1]` | measured per-part state (also assembled into the `state` bundle) |

The **pinocchio** node consumes the `command` bundle (turning Cartesian parts to joints via
IK, joint/gripper/base parts via the gate) + the `state` bundle (feedback), solves the whole
robot together with the descriptor's `constraints` (self-collision + plane/table), and emits
per-part `<part>_joint_target`. Adding a quantity (e.g. velocity) or a part (e.g. a mobile
base) is a descriptor entry + one dispatch branch — no new message ids. Metadata: `timestamp`;
commands use **drop-oldest** queues.

## Images & depth

| message | layout | metadata |
|---|---|---|
| `image` / `cam_*` | `uint8[...]` flat buffer | `encoding` (`rgb8`\|`jpeg`), `width`, `height`, `timestamp` |
| `depth` | `uint16[width*height]` | `width`, `height`, `timestamp` |

- `rgb8`: the raveled HxWx3 frame, so `len == width*height*3`.
- `jpeg`: the encoded JPEG bytes (much smaller); decode with the `encoding` flag.
- `depth`: row-major uint16 millimetres.

Cameras are named per dataflow (`cam_high`, `cam_static1`, …); the name is the
output/input id, the layout above is identical for all of them.

## Control & program_state (utf8, single element)

| message | values | meaning |
|---|---|---|
| `episode_control` / `control` | `start` \| `finish` \| `task=<text>` | recorder gate: open/close an episode, set its task label |
| `robot_command` | `disconnect` | operator → robot: home, release torque, exit (distinct from the `command` setpoint bundle) |
| `node_state` | a state token, see below | a node's logical state, emitted **edge-triggered** (only when it changes). No heartbeat — dora detects node death. |
| `program_state` | `boot` \| `homing` \| `teleop` \| `disconnect` | emitted by **manager** (its mode's state machine) when the program state changes; nodes gate their behaviour on it and tear down on `disconnect`. |

### `node_state` vocabulary

Tokens, not prose — the manager's state machine matches them exactly; human detail
belongs in the node's own log. Common tokens:

| token | meaning |
|---|---|
| `ready` | node is initialized and operational. Every manager-watched producer must report it once to leave `boot`. |
| `homing` / `homed` | pinocchio's homing ramp is running / the robot measurably arrived at the model home (the MJCF `home` keyframe; advances `homing` → `teleop`). |

Node-specific tokens (informative; nothing gates on them yet): retarget
`waiting`/`engaged`, sync `waiting`/`synced`. TODO: tokenize the remaining prose
emitters (pinocchio `HELD (…)`, robot `… rejected …`, camera `capture ended`).

`episode_control`/`robot_command` carry a `timestamp`; `node_state`/`program_state`
do not require metadata.

## Who produces what

- **opencv-camera / realsense-camera** → `<name>_image` (+ `<name>_depth`), `<name>_node_state`;
  self-paced — each captured frame is published as it lands, so the wire rate is the sensor FPS.
- **pinocchio** (IK + safety) → per-part `<part>_joint_target`, `node_state`; consumes the
  `command` bundle (IK/gate) + the `state` bundle (feedback). Whole-robot (one node, all parts).
- **genesis / mujoco-sim** (interchangeable sim backends) → the `state` bundle, per-part
  `<part>_tcp_pose`, `cam_*`, `node_state`; consume per-part `<part>_joint_target`. (Whole-scene;
  no IK — pinocchio owns it.)
- **sync** → a concatenated bundle (`command` or `state`), emitted the moment every configured
  input has a fresh sample (its tick is only a staleness watchdog); consumes the configured
  per-node inputs. Used on the real robot to bundle the per-arm nodes (no built-in dora join).
- **trossen-robot** → one thing's `<name>_tcp_pose`, `<name>_joint_state`, `<name>_gripper_state`,
  `node_state`; consumes `<name>_joint_target` + `<name>_gripper_joint_target` (from
  pinocchio — joint control only, pinocchio owns IK) and `robot_command`. A `leader` publishes its
  state (a `sync` bundles the leaders into the `command`). Bimanual real hardware = two nodes.
- **web-controller** → `episode_control`, `robot_command`, `node_state`, and in `manual` mode the
  `command` bundle (consumes `<part>_tcp_pose` for feedback); `episode` mode emits only
  `episode_control` + the task.
- **manager** → `program_state`; consumes each producer's `node_state` and runs the program
  state machine over those events (see `nodes/manager`).
- **the rerun + lerobot recorders / the rerun visualizer** are sinks: they consume images,
  the `command`/`state` bundles, per-part poses, and `control`; recorders emit `node_state`.
