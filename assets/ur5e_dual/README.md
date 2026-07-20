# `ur5e_dual` — bimanual UR5e workstation asset

Two UR5e arms with Robotiq 2F-85 grippers on a shared bench, facing the same way
(+x), side by side. Used by the descriptor-driven nodes (`pinocchio`, `mujoco-sim`,
`genesis`) exactly like `trossen_stationary` — point `SCENE` at
`scenes/workstation_ur5e.yaml`.

## Layout

```
model/
  ur5e/             vendored UR5e (mujoco_menagerie/universal_robots_ur5e) — generator source
  robotiq_2f85/     vendored Robotiq 2F-85 (mujoco_menagerie/robotiq_2f85) — generator source
  assets/           all arm + gripper meshes, flat (the single MuJoCo meshdir) — generated
  build_scene.py    one-off generator: composes the two arms + grippers + bench
  ur5e_dual.xml     THE universal model: bench + 2 arms + grippers + cameras (no floor/props) —
                    generated; loaded by mujoco-sim, genesis, the rerun 3D view, AND pinocchio
scenes/
  workstation_ur5e.yaml   the scene descriptor (parts, constraints, layouts, cameras)
```

## Cell geometry (real workstation)

- Both arms face forward (+x); side by side along y.
- Base-to-base distance **0.70 m**.
- Bench surface is **0.105 m below** each arm base (arms on risers).

These live as named constants at the top of `build_scene.py`
(`BASE_SEP`, `BASE_RISER`, `TABLE_SURFACE_Z`, `MOUNT_QUAT`).

## Regenerating the model

`ur5e_dual.xml` and `model/assets/` are generated from the vendored menagerie models.
Re-run after changing the layout/robot:

```sh
cd assets/ur5e_dual/model
../../../nodes/mujoco-sim/.venv/bin/python build_scene.py
```

The generator (see its module docstring) composes the two arms + grippers via MuJoCo's
`MjSpec.attach`, then: converts the 2F-85 tendon actuator into a plain driver-joint
position servo in radians (so gripper command/state/ctrl share one unit), renames arm
actuators to their joint names (what `mujoco-sim` looks them up by), and **demotes the
2F-85 mesh collision geometry to non-colliding** — Pinocchio's collision engine (coal)
segfaults on mesh-vs-mesh distance, so collision is primitive-only (arm capsules +
the gripper's box finger pads), like the Trossen cell.

## Provenance

`model/ur5e/` and `model/robotiq_2f85/` are vendored from
[mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie) (see each
dir's `LICENSE`).
