# TODO: composable pipeline modules (teleoperation as a module)

**Status: idea, not designed.** Written down so the naming/layout work doesn't
paint us into a corner.

## The goal

Today every flow hand-wires the same control spine (manager + web-controller +
pinocchio + rerun + recorders) around a robot. The end state we want:

    teleoperation module
      ├── plug in: a ROBOT module        (mujoco-sim | genesis | an arm-pair module)
      └── plug in: a TELEOPERATOR module (web-controller | leader arms + retarget | policy)

so a new flow is three `module:` lines plus env, not ~150 lines of spine.

## Why it is not done now

- **dora module limits (on the 1.0 pin):** module outputs must be distinct
  literal names (no templating), a module's inner env is fixed at authoring time
  (only `${VAR}` expansion at parse), modules cannot nest, and module node paths
  must live under the dataflow dir (the symlink workaround). A spine module needs
  at least env parameterization (SCENE, COMMAND_LAYOUT, PRODUCERS, machine
  placement) and ideally module-in-module.
- **The spine repetitions are not homogeneous:** which producers the manager
  watches, which recorders exist, and `_unstable_deploy` machine placement all
  differ per flow. Hiding those in a parameterized module hides exactly the
  wiring you read when a flow misbehaves. Modules are for repeated
  *homogeneous* blocks (arm pairs, camera rigs).

## What would make it viable

1. dora gains module nesting + parameterized module env (watch upstream; the
   symlink TODO in dataflows/README.md tracks the same restriction family).
2. Define the two plug interfaces as message contracts (they already exist
   informally): a ROBOT module = consumes `<part>_joint_target`, produces
   `state` + `<part>_tcp_pose` + `node_state`s; a TELEOPERATOR module =
   produces the `command` bundle + `robot_command`, consumes
   `<part>_measured/solution_pose`.
3. Then the teleoperation module is: manager + pinocchio + recorders wired
   between those two interfaces, with SCENE/COMMAND_LAYOUT passed through.

Until then: flows stay explicit; the `<cell>_<variant>.yml` naming +
"Adding a robot" recipe in dataflows/README.md keep the repetition cheap.
