# Plan: managed startup homing (UR slow go-home + Trossen leader staged pose)

**Status: IMPLEMENTED (2026-07-02).** Manager `STAGE_ADVANCE`, pinocchio `homing`
stage, retarget teleop-gating, and the leader staged pose are all live; the staged
flows run it. One non-obvious lesson from the sim validation: the homing ramp must
step a COMMANDED config seeded from the measured state, not re-anchor on the
measured state each step — a part whose measured position lags its actuator (the
2F-85 gripper) otherwise ratchets AWAY from home. Original draft below. Two related features, deliberately designed
as one change because they share the mechanism: the manager reading producer
**status texts** to decide which *stage* of the program we are in. This doubles
as the first real exercise of the status/stage machinery.

## Goal

1. On session start, the UR arms move **slowly** to a configured home/ready pose
   before teleop is allowed — orchestrated by the manager as a program_state stage,
   with completion detected from node `status` messages (not timers).
2. The Trossen leader arms, on connect, power on and move to a configurable
   **staged pose** before going backdrivable — mirroring `lerobot_trossen`
   (`WidowXAILeaderTeleop.configure()`: position mode → `set_all_positions(staged,
   2 s, blocking)` → `external_effort` mode → `set_all_external_efforts([0]*7)`).

## What exists today (leverage, don't rebuild)

- **manager** (`nodes/manager/.../trossen_stationary.py`) already: tracks
  `last_status[producer]` (the status *text*), has a `stage` string broadcast
  verbatim on `program_state` while RUNNING, and transitions STARTING → RUNNING when
  all producers are alive. Today `stage` only changes via the external `stage`
  input; nothing reads status text to advance it.
- **pinocchio control mode** already rate-limits every part toward a goal
  (per-part `max_step`), gates collisions/plane, and knows the descriptor `home`
  (`q_home`). It already consumes `program_state` (stop only).
- **ur5e-robot follower** already refuses jumps (`max_joint_jump`) and never
  commands before its first state read. It has NO homing motion of its own.
- **trossen-robot leader** connects straight into `external_effort` mode: no
  staged move, and — bug — no `set_all_external_efforts(zeros)`, which is what
  actually enables gravity compensation in the reference implementation.
- **retarget** engages (latches leader L0 / follower F0 anchors) as soon as both
  poses have been seen — today that could be *during* a homing motion.

## Design

### Stage machine (manager)

Stages become a first-class sequence with status-driven advancement:

```
STARTING ──all alive──▶ RUNNING[homing] ──status predicate──▶ RUNNING[teleop] ──▶ ...
```

- Config: `STAGES: "homing,teleop"` (already exists as `stages`) plus a new
  advancement rule, e.g. `STAGE_ADVANCE: "homing: pinocchio=homed"` — meaning:
  while in stage `homing`, when producer `pinocchio`'s latest status text equals
  (or starts with) `homed`, broadcast the next stage on `program_state`.
- Implementation is a few lines in `_transition`/`_on_tick`: the manager already
  stores `last_status`; add a check against the active stage's predicate.
- The external `stage` input stays (manual override / future policy switching).
- Flows that don't configure `STAGE_ADVANCE` behave exactly as today
  (single stage, broadcast `start`).

### UR homing motion (pinocchio, NOT the ur5e node)

The slow go-home is executed by **pinocchio**, not the robot node, so the
existing safety stack (per-part `max_step` rate limit, self-collision + plane
gate, joint limits) applies to the homing trajectory for free:

- On `program_state == "homing"`, control mode enters a homing sub-state: it
  **ignores the `command` bundle** and instead, on every `state` message
  (~30 Hz), steps each part from its measured q toward the descriptor `home` by
  `min(part.max_step, homing_max_step)` — new config `HOMING_MAX_STEP`
  (rad/step; e.g. 0.01 @ 30 Hz ≈ 0.3 rad/s ⇒ "slow"), emitting the usual
  per-part `<part>_joint_target`.
- When every part is within `HOMING_TOL` (rad) of home, pinocchio sets its
  status to `homed` (and keeps holding home). The manager sees `homed` →
  broadcasts `teleop`.
- On `program_state == "teleop"` pinocchio resumes normal command-driven control.
- The UR follower node needs **zero changes**: it just keeps executing servoJ
  targets; its `max_joint_jump` backstop stays as the last line of defense.
- Where the home pose lives: the descriptor `home:` per part
  (`assets/ur5e_dual/scenes/workstation_ur5e.yaml`) — one source of truth shared
  with the sim. A different startup pose = edit the descriptor (or a future
  `HOME_OVERRIDE` env if per-session poses are wanted).

### Teleop gating (retarget)

- retarget currently only reacts to `program_state == stop`. Add: only engage/emit
  while `program_state == "teleop"` (configurable stage name, default `teleop`;
  treat legacy `start` as teleop for flows without staging). On any other stage
  it drops its anchors, so engagement always happens against the *homed* pose,
  never mid-homing. This is ~10 lines + tests.

### Trossen leader staged startup (trossen-robot) — **IMPLEMENTED 2026-07-02**

(Shipped ahead of the rest of this plan: staged pose on connect via
`LEADER__STAGED_POSE`, zero external efforts for gravity comp, and a parked
staged→sleep shutdown instead of release-in-place. See modes/leader.py.)

Mirror `lerobot_trossen` in `driver.make_driver` / leader mode `start()`:

1. `set_all_modes(position)`,
2. `set_all_positions(staged_pose, 2.0, blocking)` — new leader config
   `LEADER__STAGED_POSE` (7 floats: 6 joints rad + gripper m), default
   `[0, π/3, π/6, π/5, 0, 0, 0]` (the lerobot default),
3. `set_all_modes(external_effort)`,
4. `set_all_external_efforts([0]*7)` — **add regardless of this plan**: without
   it gravity compensation is likely not engaged (candidate root cause if the
   stage-1 hardware test finds the leaders heavy/stiff).
5. Optionally emit status `staged` before / `leading` after, so the manager can
   include leaders in a homing-stage predicate too (nice-to-have; the blocking
   staged move already delays the leader's first status, which the STARTING
   phase naturally waits for).

Leader disconnect stays release-only (no fold-back): a human is holding the arm.
(lerobot folds staged→sleep on disconnect; revisit if leaders get dropped a lot.)

### Sim behavior

mujoco-sim spawns at descriptor home, so the homing stage completes in ~1 tick —
the staged flows keep working unchanged in sim, and the e2e can assert the
`homing → teleop` transition cheaply.

## Touch list

| where | change |
|---|---|
| manager mode + config | `STAGE_ADVANCE` predicate on status text; broadcast next stage |
| pinocchio control + config | homing sub-state (ignore command, ramp to home, status `homed`), `HOMING_MAX_STEP`, `HOMING_TOL` |
| retarget | engage only in stage `teleop`; drop anchors otherwise |
| trossen-robot leader + driver | staged-pose move on connect, `LEADER__STAGED_POSE`, zero external efforts |
| dataflows | `STAGES: "homing,teleop"` + `STAGE_ADVANCE` on the teleop flows |
| tests | manager transition test (status→stage advance), pinocchio homing ramp test, retarget gating test, leader staged-connect test (fake driver) |

## Open questions

- Homing with the arm far from home can cross the workspace: is pinocchio's
  self-collision + plane gate enough, or do we want a "refuse to home if further
  than X" guard that asks the operator to pre-pose via pendant?
- Should the URs also *fold to a rest pose on disconnect* (mirror of homing —
  same mechanism, one more stage `parking` before stop)? Trivial once homing
  exists; skipped for now (current behavior: stop in place, controller holds).
- Per-arm staged poses for the two leaders (left/right differ?) — start with one
  shared default, override per node env since each leader is its own node.
