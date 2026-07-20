# genesis — physics simulation

A [Genesis](https://github.com/Genesis-Embodied-AI/Genesis) simulation of a multi-arm
cell, **driven by safe joint targets** from the pinocchio node (Genesis does **not** do
IK here — pinocchio owns it). It mirrors the real-robot stream interface, so swapping the
sim for the real robot is a dataflow edit, not code. Replaces the MuJoCo `trossen-sim`.

## Mode: `sim`

- **Consumes:** `<arm>_joint_target` (`float64[n]`, drop-oldest), `<arm>_gripper_target`
  (`float64[1]`, drop-oldest), `tick`, `program_state`.
- **Produces** (identical to the real robot / old sim): `<arm>_tcp_pose` (`float64[7]`
  xyzw), `<arm>_joint_state` (`float64[n]`), `<arm>_gripper_state` (`float64[1]`), `cam_*`
  images, `node_state`.

On each `tick` it applies the latest joint/gripper targets, advances however many physics
steps of real time have elapsed (auto-paced to wall-clock, capped at `MAX_SUBSTEPS` so a
slow machine degrades to slower-than-realtime instead of spiralling), renders the cameras,
and publishes state + frames. Genesis runs **inline on the dora
loop (main thread)** — its renderer needs the main thread (a worker thread crashes with an
NSException on macOS), so unlike the old MuJoCo node there is no render thread. The model,
interactable objects (cube/ball/box), and cameras all come from the scene descriptor named
by `SCENE`, so another robot/scene slots in by writing a new descriptor.

**Performance:** on this Mac the 4-arm scene + render runs sub-realtime (~8–10 Hz with
cameras), so wire the Genesis `tick` modestly (e.g. `dora/timer/millis/100`). On a CUDA GPU
box it runs far faster — raise the tick rate there. State is plain physics; the cost is the
render.

`twin` / `force_preview` modes are reserved (a digital twin that mirrors the real robot and
predicts contact force) — not implemented yet; `MODES` raises if selected.

## macOS

Genesis runs headless on Apple Silicon (offscreen render; no on-screen viewer — the
`cv2.imshow`/Metal conflict). `HEADLESS=True` (default) keeps it offscreen.

## Config (env)

`MODE` (`sim`), `ROBOT_NAME`, `SCENE`, `ARMS` (CSV), `CAMERAS` (CSV), `WIDTH`, `HEIGHT`,
`MAX_SUBSTEPS`, `ENCODING` (`rgb8`|`jpeg`), `JPEG_QUALITY`, `HEADLESS`, `BACKEND`
(`cpu`|`gpu`|`metal`, empty = auto). Publish rate = the `tick` wiring in the dataflow.
Debug port: 5687.

## Tests

Validate real headless render with a manual `uv run dora run …` smoke.
