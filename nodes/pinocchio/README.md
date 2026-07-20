# pinocchio — IK + collision safety

Whole-robot kinematics + safety for a multi-part cell, built on [Pinocchio](https://github.com/stack-of-tasks/pinocchio)
(+ Coal for collision). It turns a **whole-robot `command` bundle into SAFE per-part joint
targets** and is **simulator-independent**: the same node guards a sim and the real robot.
Everything robot-specific (parts, constraints, vector layouts) comes from the scene
descriptor — nothing is baked to "arms"/"table", so a different robot/setup is a new
descriptor, not node edits.

## Mode: `control`

- **Consumes** `command` (a flat vector per `command_layouts.<COMMAND_LAYOUT>` in the
  descriptor — each part tagged `space ∈ {joint,cartesian}` × `quantity ∈ {position,
  velocity,effort}`) and `state` (measured joints, per `state_layout`).
- **Emits** per-part `<part>_joint_target` (`float64[n]`), plus `status`.

On each `command` it steps **all parts at once** into one combined candidate config —
Cartesian parts via damped-least-squares IK (target projected above the plane), joint parts
toward their target, grippers/base passed through — bounds each step (`MAX_JOINT_STEP`),
then runs the **gate on the combined config**: the descriptor's `self_collision` (cross-group
Coal distance) + `plane` constraint + joint limits. If safe it emits every part's target
**together**; otherwise it **HOLDs** (re-emits the last safe targets) and reports why. Solving
from one bundle is the synchronisation — the collision check sees every part's new pose at
once and outputs leave together. It never commands before the first `state` (seeds from the
real config).

Adding a new capability is a *small* edit: a new `quantity` (e.g. velocity) = one dispatch
branch in `_step`; a new part (e.g. a mobile base) = a descriptor entry (+ a base driver). No
config/message-id growth.

## Config (env)

`MODE` (`control`), **`SCENE`** (required — descriptor path), **`COMMAND_LAYOUT`** (required —
which `command_layouts.<name>`: `cartesian` for web teleop, `joint` for leader/policy),
`COLLISION_CHECK`, `IK_DAMPING`. The per-step bound is **per-part `max_step`** in the descriptor
(units differ — rad for arms, m for grippers); `MAX_JOINT_STEP` is the fallback. Debug port: 5683.

## Tests

`cd nodes/pinocchio && uv run pytest -q` — FakeNode unit tests over the real model + descriptor:
synchronized per-part emit, no-command-before-state, gripper stepping, joint-limit/step clamp,
and the HOLD path. (Velocity quantity is stubbed with a clear `NotImplementedError` branch.)
