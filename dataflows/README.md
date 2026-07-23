# Dataflows

A dataflow is a YAML graph wiring together the self-contained nodes under
`nodes/`. Each node entry pulls its own `uv` virtualenv and runs its console
script, so nodes with conflicting deps (mujoco, torch, rerun, …) stay isolated.
Inputs map as `<input_id>: <node_id>/<output_id>`; command inputs use a shallow
drop-oldest queue so a node tracks the latest setpoint. Payloads are plain Arrow
— see [../docs/message_formats.md](../docs/message_formats.md).

All flows share one control spine:

    command source (web / leader arms / policy)
      → pinocchio (IK + collision safety) → <part>_joint_target
      → the robot (mujoco-sim | an arm-pair module)
      → state (bundle) fed back to pinocchio + recorders

## The flows

Named `<cell>_<variant>.yml`, where `<cell>` is exactly the `assets/<cell>/`
directory (flow prefix = scene root = rig identity), so everything for one cell
sorts together. Variants: a sim backend name (`mujoco`, `genesis`) implies local
sim; `real` = local on the robot host; `remote_web` = two-machine browser teleop;
`from_<leader-cell>` = cross-robot.

| Flow | What it is |
|---|---|
| `trossen_stationary_mujoco.yml` | LOCAL Trossen stationary cell in MuJoCo, web teleop + recording |
| `trossen_stationary_genesis.yml` | LOCAL Trossen stationary cell in Genesis (same graph, genesis backend; sub-realtime on a Mac — 100 ms tick, FPS 10) |
| `ur5e_dual_mujoco.yml` | LOCAL dual-UR5e cell in MuJoCo, web teleop + recording (no sim cameras yet — the ur5e MJCF has none) |
| `trossen_stationary_real.yml` | LOCAL (on the robot host) real Trossen cell, web teleop, no cameras/recording |
| `ur5e_dual_real.yml` | LOCAL (on the robot host) real dual UR5e, web teleop, no cameras/recording |
| `trossen_stationary_remote_web.yml` | REMOTE browser teleop + episode recording, real Trossen cell + 4 RealSense |
| `ur5e_dual_remote_web.yml` | REMOTE browser teleop + episode recording, real UR5e cell + 4 RealSense |
| `ur5e_dual_from_trossen_mobile.yml` | CROSS-ROBOT: hand-guided Trossen leaders (trossen-mobile) drive the UR5e cell |

Repeated multi-node blocks are dora modules in [`modules/`](modules), named by
their scope: arm pairs by **robot model** (`trossen_wxai_pair`, `ur5e_pair` —
2 driver nodes + `state_sync`, reusable by any cell with that hardware) and
camera rigs by **cell** (`trossen_stationary_rig` — a rig is a physical fact:
those serials, that host; a new cell gets its own rig module, there is no
"generic" rig). Per-instance nodes publish name-prefixed outputs
(`left_node_state`, `cam_wrist1_image`) because module exports must be distinct
literal names; a `_unstable_deploy` on the module instance applies to all its
inner nodes.

TODO(dora): the `nodes`/`assets`/`out` entries here are symlinks to the repo
root. All three are load-bearing on this pin (audited empirically 2026-07-23):
`nodes` because dora rejects module node paths that resolve outside the
dataflow's directory AND runs node `build:` commands with cwd = this directory;
`assets`/`out` because dora spawns non-deployed nodes (`dora run`, local
`dora start`) with cwd = this directory, while deployed nodes use
`working_dir: ../..` (repo root) — the symlinks let one set of relative
env paths (`SCENE`, `REPO_ROOT`, …) serve both cwds (`dora run` refuses
`working_dir` overrides). Remove them when upstream lifts the module-path
restriction and normalizes the node cwd.

## Run — local flows (single machine)

From the repo root (`uv run` provides the pinned dora CLI):

```sh
uv run dora up                                        # once: coordinator + daemon
uv run dora build dataflows/ur5e_dual_mujoco.yml              # first time: syncs node venvs
uv run dora start --attach dataflows/ur5e_dual_mujoco.yml     # Ctrl-C to stop
uv run dora destroy                                   # when done: tear down
```

Opens a live rerun viewer and a web UI at `http://127.0.0.1:8421`: drive the two
arms, press Start/Finish to record an episode to a LeRobotDataset (with video) +
a per-episode `.rrd` under `out/`.

The real-robot local flows additionally need the host env-file (arm IPs):

```sh
uv run --env-file dataflows/robot_envs/ur5-corner.env dora build  dataflows/ur5e_dual_real.yml
uv run --env-file dataflows/robot_envs/ur5-corner.env dora start --attach dataflows/ur5e_dual_real.yml
```

Safety for first hardware contact: pendants powered, brakes released, Remote
Control mode, e-stop in hand; first action a single small +Z nudge on ONE arm.

## Run — remote flows (robot host + operator laptop)

Host-specific configs are gitignored; copy the templates once and fill them in:

```sh
cp dataflows/robot_envs/trossen-1.env.example dataflows/robot_envs/<host>.env   # serials + arm IPs
```

**Networking on the dora-1.0 pin: use `--zenoh-peer`, NOT `ZENOH_CONFIG`.**
Every node opens its own zenoh session; daemons mesh through one shared
rendezvous endpoint given by `--zenoh-peer` (first daemon to bind it listens,
the rest connect). A `ZENOH_CONFIG` env var actively breaks this: it leaks into
every spawned node, which then all try to bind the same fixed port. Multicast scouting doesn't
cross the office wifi/ethernet subnets, so the flag is required. The coordinator
binds 127.0.0.1 by default — give it `--interface 0.0.0.0` so remote daemons can
register.

**Addresses: LAN IPs vs Tailscale names.** `--zenoh-peer` accepts a DNS name
(`tcp/<host>.taile0e34.ts.net:5456`), but `--coordinator-addr` is parsed as an
IP, so pass `$(tailscale ip -4 <host>)`. The tailnet ACL must allow the robot
hosts to reach each other — as of 2026-07-02 it silently DROPS
trossen-mobile→ur5-corner (both `tagged-devices`), so until that's fixed use LAN
IPs (DHCP — re-check per session: `ipconfig getifaddr en0` / `hostname -I`).

Two-machine web teleop (`trossen_stationary_remote_web.yml` / `ur5e_dual_remote_web.yml`):

```sh
# robot host (tmux, stays up):
uv run dora coordinator --interface 0.0.0.0
uv run dora daemon --machine-id robot --zenoh-peer tcp/<robot-addr>:5456
# operator laptop, terminal 1:
uv run dora daemon --machine-id operator --coordinator-addr <robot-addr> \
  --zenoh-peer tcp/<robot-addr>:5456
# operator laptop, terminal 2 — always build before start:
uv run --env-file dataflows/robot_envs/trossen-1.env \
  dora build dataflows/trossen_stationary_remote_web.yml --coordinator-addr <robot-addr>
uv run --env-file dataflows/robot_envs/trossen-1.env \
  dora start --attach dataflows/trossen_stationary_remote_web.yml --coordinator-addr <robot-addr>
```

The control page (`http://127.0.0.1:8421`): −/+ buttons teleoperate each arm's
TCP and gripper; task field + Start/Finish record episodes to both a
LeRobotDataset and a per-episode `.rrd` under `out/` on the robot host;
**Disconnect** folds/stops the arms live and the manager broadcasts
`program_state: disconnect` (its `controller` input), so every node — cameras,
recorders, viewers — tears down and the whole dataflow stops. Ctrl-C is the
fallback (the robot still homes + releases).

Cross-robot (`ur5e_dual_from_trossen_mobile.yml`, 3 machines — laptop = coordinator
only, `operator` = trossen-mobile, `robot` = ur5-corner): a cross-robot flow
loads *every* involved host's env-file on the build/start commands — dora
expands `${VAR}` at parse time on the launching machine:

```sh
# ur5-corner (tmux):
uv run dora daemon --machine-id robot --coordinator-addr <laptop-addr> \
  --zenoh-peer tcp/<ur5-corner-addr>:5456
# trossen-mobile (tmux):
uv run dora daemon --machine-id operator --coordinator-addr <laptop-addr> \
  --zenoh-peer tcp/<ur5-corner-addr>:5456
# laptop:
uv run dora coordinator --interface 0.0.0.0
uv run --env-file dataflows/robot_envs/ur5-corner.env --env-file dataflows/robot_envs/trossen-mobile.env \
  dora build dataflows/ur5e_dual_from_trossen_mobile.yml --coordinator-addr 127.0.0.1
uv run --env-file dataflows/robot_envs/ur5-corner.env --env-file dataflows/robot_envs/trossen-mobile.env \
  dora start --attach dataflows/ur5e_dual_from_trossen_mobile.yml --coordinator-addr 127.0.0.1
```

The retarget mapping is delta-based (each arm engages where it happens to be —
no startup lunge) with `SCALE` translation scaling and an `ALIGN_RPY` leader→UR
frame yaw; **first run: push one leader gently forward and confirm the UR moves
the expected way in rerun before trusting the alignment.**

## Adding a robot / configuration

No node code changes — that invariant is the test that a new cell fits:

1. `assets/<cell>/` — MJCF model (+ meshes) and a scene descriptor
   (`scenes/*.yaml`; copy the trossen one, it is the schema-by-example).
2. Optionally a `modules/<cell>_rig_module.yml` (its cameras) and/or a
   `modules/<model>_pair_module.yml` (its arms), if not already present.
3. Flows named `<cell>_<variant>.yml`, wiring the shared control spine
   (manager → controller/pinocchio → robot/sim → recorders) to those modules.
4. A `robot_envs/<host>.env.example` for its serials/IPs.

TODO: make the control spine itself composable — a `teleoperation` module you
connect a robot module + a teleoperator-device module to (see
[../docs/plans/teleop-module.md](../docs/plans/teleop-module.md)).

## If a remote flow misbehaves

- **One daemon per machine-id.** A second `operator` daemon steals the
  registration and both drop — check for a stray daemon first.
- Operator daemon exits with "lost connection to coordinator": restart the
  coordinator terminal and re-run (happens on a flaky/relayed link).
- `trossen-arm` must match the controller firmware in major.minor (`1.9.3` ↔
  firmware `v1.9.0`).

## Test

One end-to-end smoke: builds `trossen_stationary_mujoco.yml` (the trossen model is the one
with sim cameras), rewrites it headless (scripted driver instead of the browser,
rerun in-memory), runs it
for ~25 s and asserts a LeRobotDataset with video + a per-episode `.rrd` were
written. Run with any node venv's pytest (it only shells out to `dora`):

```sh
cd dataflows && uv run --project ../nodes/lerobot pytest tests -q
```
