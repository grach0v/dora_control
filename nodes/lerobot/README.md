# lerobot

A dora node for [LeRobot](https://github.com/huggingface/lerobot). One mode today,
`record`: it records **operator-controlled episodes** into a `LeRobotDataset`. It
subscribes to N camera streams (each named in `CAMERAS`), the state/action input
streams named in `STATE_INPUTS`/`ACTION_INPUTS` (concatenated, in order, into
`observation.state` and `action`), a `control` stream, and `program_state`.

See [../../docs/node_development.md](../../docs/node_development.md) for the node
pattern. Package `lerobot_node` / script `lerobot-node` (not `lerobot`, which would
shadow the LeRobot SDK).

Recording is gated by `control`: `start` begins an episode, `finish` saves it and
goes idle, `task=<text>` sets the task label. The **first** camera in `CAMERAS`
paces recording: one frame is committed per pacing frame, pairing it with the
other cameras' latest frame and each state/action input's sample nearest the
pacing timestamp (within `SYNC_TOLERANCE`; otherwise the frame is skipped as a
gap). On program_state `stop` (or dora STOP) an in-progress episode is flushed.

> **`FPS` must equal the camera's actual frame rate.** Frames are camera-paced but
> dataset timestamps are `frame_index / FPS`; a mismatch yields a time-warped dataset.

## Input queues (memory vs. completeness)

How much the recorder buffers when it falls behind is set **per input edge in the
dataflow YAML** (dora owns the consumer-side queue):

```yaml
image: { source: robot/cam_high, queue_size: 1, queue_policy: drop_oldest }  # freshest only
image: { source: robot/cam_high, queue_size: 100, queue_policy: drop_oldest } # bounded
image: robot/cam_high                                                          # DEFAULT: no limit
```

The shipped dataflows use the **default (no limit)** — memory is the bound, safe
for finite episodes; switch to bounded if you record indefinitely or are tight on
RAM. `drop_oldest` on a recorder leaves a gap where it fell behind.

## Configuration (env)

| var | required | default | meaning |
| --- | --- | --- | --- |
| `REPO_ID` | yes | — | LeRobotDataset repo id, e.g. `user/my_dataset` |
| `REPO_ROOT` | yes | — | on-disk root the dataset is written under |
| `FPS` | no | `30` | dataset fps (timestamps = frame_index / fps) |
| `TASK` | no | `record` | fallback task until a `task=` control arrives |
| `CAMERAS` | yes | — | CSV of camera input ids; each → `observation.images.<id>`; the **first** paces recording |
| `IMAGE_WIDTH` / `IMAGE_HEIGHT` | no | `640` / `480` | expected frame size; must match the cameras |
| `STATE_INPUTS` | yes | — | CSV of input ids concatenated into `observation.state` |
| `STATE_DIM` | yes | — | total concatenated state length (validated) |
| `ACTION_INPUTS` | yes | — | CSV of input ids concatenated into `action` |
| `ACTION_DIM` | yes | — | total concatenated action length (validated) |
| `STATE_KEY` / `ACTION_KEY` | no | `observation.state` / `action` | feature names |
| `SYNC_BUFFER` | no | `100` | per-stream ring-buffer depth for timestamp alignment |
| `SYNC_TOLERANCE` | no | `0.1` | max seconds between the pacing image and a paired stream; beyond this the frame is skipped |

The input ids in `CAMERAS` / `STATE_INPUTS` / `ACTION_INPUTS` must match the input
edge names wired in the YAML.

## YAML

```yaml
  - id: lerobot
    path: ../lerobot_node/main.py
    inputs:
      cam_high: robot/cam_high
      left_joint_state: robot/left_joint_state
      left_tcp_target: record/left_tcp_target
      control: record/episode_control
      program_state: manager/program_state
    outputs: [node_state]
    env:
      REPO_ID: user/pick_cube
      REPO_ROOT: /data/lerobot
      FPS: "30"
      CAMERAS: cam_high
      STATE_INPUTS: left_joint_state
      STATE_DIM: "6"
      ACTION_INPUTS: left_tcp_target
      ACTION_DIM: "7"
```

## Test

`lerobot` is heavy (pulls torch/torchvision/torchcodec). The test runs a real
dataflow with synthetic camera + state + action producers and asserts a dataset
was written (meta/info.json with frames, a data parquet, and an .mp4):

```bash
uv sync --python 3.11
uv run pytest tests -q
```
