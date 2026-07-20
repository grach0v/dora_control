# dora_claude — a modular robot-control stack

A robot-control repository built on [dora-rs](https://dora-rs.ai). The goal is to
combine two things that today live in different worlds:

- **ROS-like modularity** — the system is a graph of small, single-purpose nodes
  wired together by a YAML dataflow. Any node (a camera, a robot driver, a
  recorder) can be swapped for another implementation — even in a different
  language — without touching the rest, because nodes only exchange **plain
  Apache Arrow messages** with a documented layout (no shared code, no language
  lock-in).
- **lerobot-like out-of-the-box readiness** — one repository that covers the
  whole robot-learning loop (simulation, teleoperation, visualization, dataset
  recording, and later inference/training), so you can do real work without
  gluing five projects together.

It is meant to scale across robot shapes — one-arm, bimanual, stationary, or
mobile — by composing the same nodes differently.

> **Status:** very early / experimental. The remote dataflows have run on real
> hardware (4 RealSense cameras + the bimanual Trossen kit, teleoperated over
> Tailscale); most of the code wasn't carefully reviewed though (see the verification
> columns below). If you are an expert on some of the nodes and want to help review them, please reach out.

---

## Architecture

- **Nodes exchange plain Apache Arrow messages** — no shared code, no language
  lock-in.
  Every data message is recommened to carry a captured `timestamp` in metadata so a consumer can
  align streams that arrive at different rates. 
- **Producers (cameras, robot) are lightweight and never block the dora loop.**
  Heavy or blocking work — camera capture, MuJoCo rendering (for online
  teleoperation) — runs on a **background thread** that keeps a latest-sample slot.
  Sim/robot nodes publish that latest sample on a `dora/timer` `tick`; camera nodes
  self-pace instead, publishing each captured frame as it lands, so the wire rate
  is the configured sensor FPS. See `opencv-camera` and `mujoco-sim` for the two
  patterns. (TODO: move `genesis` rendering off the dora loop too.)
- **Topic (pub/sub) everywhere.** Setpoint inputs (`*_target`) use a shallow
  drop-oldest queue (`queue_size: 1`) so a consumer tracks the latest command;
  recorders keep every frame (default unbounded queue).
- **Per-node config** is a pydantic-settings model populated from the dataflow's
  `env:` block.
- **One node = one controllable thing** (one arm, one camera), composed by name in
  the dataflow — bimanual real hardware is two `trossen-robot` nodes. How to build
  a node: **[docs/node_development.md](docs/node_development.md)**.
- **Manager node** collects states from other nodes and emits the program state.
- **Nodes often have multiple modes** that they can load, to differentiate different usecases.


## Layout

```
nodes/<name>/            self-contained uv project (own deps + tests)
  <name>/main.py         the dora event loop
  <name>/node_config.py  BaseSettings config read from env
assets/<robot>/             shared asset zoo (data, not code): model + meshes + a
                            scene descriptor read by pinocchio and the sim nodes
dataflows/
  *.yml                    the 7 flows (local sim/real + remote, per robot)
  modules/                 dora modules: arm pairs + camera rig (shared by the flows)
  robot_envs/<host>.env    per-host facts (arm IPs, camera serials; gitignored, copy the .example)
  zenoh/                   legacy zenoh configs (reference only; use --zenoh-peer)
  nodes|assets|out         symlinks to the repo root — dora rejects module node paths
                           outside the dataflow dir; TODO remove when fixed upstream
```

## Running the dataflows

dora is pinned to a **1.0.0-rc1** commit.
```sh
uv sync 
uv run ./scripts/build-dora.sh
```

### Simple run
```sh
uv run dora run dataflows/trossen_sim.yml
```

It opens a rerun viewer with the four cameras and a NiceGUI web UI at
`http://127.0.0.1:8421` (per-arm TCP +/- and gripper buttons). Drive the arms by
hand; press **Start/Finish** to save an episode to a LeRobot dataset + a rerun
`.rrd` under `out/`.


### Propper dora lifecycle

```sh
# 1. Start the dora daemon once (keep it running):
uv run dora up

# 2. Build a dataflow (first time per dataflow: downloads/builds each node's deps):
uv run dora build dataflows/trossen_sim.yml

# 3. Run it (Ctrl-C stops it):
uv run dora start --attach dataflows/trossen_sim.yml

# 4. When you're completely done, tear the daemon down:
uv run dora destroy
```

---

## Nodes

**"Human-verified" = a person has read and approved the code — currently none have.**

| node | what it does | unit tests | human-verified |
| --- | --- | --- | --- |
| `manager` | State machine that collects states from nodes and emits program's new state. | — | ✅ |
| `opencv-camera` | publishes frames from an OpenCV capture (webcam or file) as `<name>_image` at the configured FPS, rgb8 or jpeg | — | ❌ |
| `realsense-camera` | Intel RealSense color + optional depth; hardware-only (no `pyrealsense2` wheel for macOS arm64) | — | ❌ |
| `pinocchio` | whole-robot **IK + collision safety** (Pinocchio + Coal): consumes the `command` + `state` bundles, runs a synchronized whole-robot solve (self-collision + plane constraints, HOLD gate), emits per-part `<part>_joint_target` + per-arm `measured/solution_pose` (model-frame FK). Generic, simulator-independent; reads the scene descriptor | ✅ unit | ❌ |
| `genesis` | **Genesis** sim, driven by per-part `<part>_joint_target` → emits the `state` bundle + per-part `tcp_pose` + cameras. GPU-scale; sub-realtime on this Mac. Reads the descriptor | — | ❌ |
| `mujoco-sim` | **MuJoCo** sim, same contract as `genesis` — the **fast local** backend (realtime here, bg-thread render). Reads the descriptor | e2e (smoke) | ❌ |
| `sync` | reusable aggregator: collect N event-driven inputs, emit one concatenated bundle the moment every input has a fresh sample (tick = staleness watchdog only), `log.warning` on timestamp staleness/skew. Bundles the per-arm hardware nodes (no built-in dora join) | — | ❌ |
| `trossen-robot` | the **real** Trossen robot, **one arm per node** (`NAME`+`IP`, `MODE`=follower/leader; `base` mode TODO for the `trossen-slate` mobile base): a follower streams `tcp_target` (firmware IK) or per-part `joint_target` (`CONTROL_SPACE=joint`, from pinocchio); a leader publishes its hand-moved state | — | ❌ |
| `ur5e-robot` | the **real** UR5e, **one arm per node** (`NAME`+`IP`): servoJ streaming of per-part `joint_target` with joint-jump guards; optional Robotiq gripper | — | ❌ |
| `retarget` | delta-based cross-robot leader→follower mapping (translation `SCALE`, `ALIGN_RPY` frame alignment, gripper range) → the `command` bundle | ✅ unit | ❌ |
| `lerobot` | records cameras + the `state` bundle + the `command` bundle into a `LeRobotDataset` (with video) | — | ❌ |
| `rerun` | one node, two modes (`MODE`): `record` logs streams to a persisted `.rrd` (cameras as H.264 video); `visualize` live-streams them to a rerun viewer (never persists) | e2e (smoke) | ❌ |
| `web-controller` | `manual` mode builds + emits the `command` bundle (closed-loop page, +/- buttons); `episode` mode = episode/task + disconnect only | e2e (smoke) | ❌ |

---

## Dataflows

Under `dataflows/` — 7 flows sharing one control spine (command source →
pinocchio IK/safety → robot → state feedback).

| dataflow | what it does | runs (sim) | human-verified |
| --- | --- | --- | --- |
| `trossen_sim.yml` | LOCAL: web-controller (manual) → **pinocchio** → **mujoco-sim** (Trossen cell) → rerun + lerobot. | ✅ | ❌ |
| `ur5e_sim.yml` | LOCAL: same graph, dual-UR5e cell (no sim cameras yet — the ur5e MJCF has none, so the camera-paced lerobot recorder is disabled); smoke default is trossen_sim. | ✅ | ❌ |
| `trossen_real.yml` | LOCAL real HW: browser Cartesian → pinocchio → trossen pair (joint). No cameras/recording. | — | ❌ |
| `ur5e_real.yml` | LOCAL real HW: browser Cartesian → pinocchio → UR5e pair (servoJ). No cameras/recording. | — | ❌ |
| `remote_trossen_web.yml` | REMOTE real HW: operator browser + rerun ← zenoh → Trossen pair + 4 RealSense + recorders. | — | ❌ |
| `remote_ur5e_web.yml` | REMOTE real HW: same, UR5e cell. | — | ❌ |
| `remote_ur5e_from_trossen.yml` | CROSS-ROBOT: hand-guided Trossen leaders (trossen-mobile) → retarget → pinocchio → UR5e cell. | — | ❌ |

Cross-machine data rides **zenoh**; daemons mesh via `--zenoh-peer` (do NOT set
`ZENOH_CONFIG` on the 1.0 pin — it leaks into nodes and breaks startup).
Robot-specific values (camera serials, arm IPs) live in a host-specific
`dataflows/robot_envs/<host>.env` (gitignored; copy the committed `.example`),
expanded by dora when the YAML is parsed. The robot nodes idle + disconnect the
arms on every shutdown path. Full guide: [dataflows/README.md](dataflows/README.md).

