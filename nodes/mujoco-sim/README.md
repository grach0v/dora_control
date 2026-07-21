# mujoco-sim — physics simulation (fast local backend)

A [MuJoCo](https://mujoco.org) simulation of a multi-arm cell, **driven by safe joint
targets** from the pinocchio node (MuJoCo does **not** do IK here — pinocchio owns it).
It is the **fast local alternative to the `genesis` node**: same scene descriptor (the
asset zoo) and the same stream interface, so the two are interchangeable per dataflow.
Pick `mujoco-sim` for quick iteration on a laptop, `genesis` for GPU scale / richer physics.

## Mode: `sim`

- **Consumes:** `<arm>_joint_target` (`float64[n]`, drop-oldest), `<arm>_gripper_target`
  (`float64[1]`, drop-oldest), `tick`, `program_state`.
- **Produces** (identical to genesis / the real robot): `<arm>_tcp_pose` (`float64[7]` xyzw),
  `<arm>_joint_state` (`float64[n]`), `<arm>_gripper_state` (`float64[1]`), `cam_*` images,
  `status`.

On each `tick` it applies the latest joint targets to the actuators, steps physics, and
publishes state + the latest camera frames. **Camera rendering runs on a background thread**
(MuJoCo's renderer is happy off the main thread, unlike Genesis), so the heavy render never
stalls the dora loop. The model, ee site, gripper, and cameras all come from the scene
descriptor named by `SCENE` (it uses the model's own MuJoCo cameras), so another robot/scene
slots in by writing a new descriptor.

## Config (env)

`MODE` (`sim`), `ROBOT_NAME`, `SCENE`, `ARMS` (CSV), `CAMERAS` (CSV), `WIDTH`, `HEIGHT`,
`FPS`, `ENCODING` (`rgb8`|`jpeg`), `JPEG_QUALITY`.

## Tests

No unit tests here on purpose (tests are only for genuinely non-trivial pure
logic — see docs/node_development.md); the e2e smoke
(`cd dataflows && uv run --project ../nodes/lerobot pytest tests -q`) exercises
this node in the full graph.
