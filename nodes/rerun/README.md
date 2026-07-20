# rerun

One dora node for [rerun](https://rerun.io), with two **modes** selected by
`MODE`:

- **`record`** — log incoming robot streams to **per-episode** `.rrd` files on
  disk (`rr.save`, not `rr.spawn`). Gated by a `control` input: `start` begins a
  fresh episode, `finish` writes `<RRD_DIR>/episode_<NNN>.rrd` and goes idle
  (`task=` ignored). Camera images are stored as a true H.264 `rr.VideoStream` (or
  JPEG fallback), so long sessions stay small.
- **`visualize`** — stream the same messages to a live rerun viewer.
  **Stateless**: never `rr.save`, keeps no frames in Python, logs on a worker
  thread fed by a drop-oldest queue so a slow viewer can't block the dora loop.

See [../../docs/node_development.md](../../docs/node_development.md) for the node
pattern. Package `rerun_node` / script `rerun-node` (not `rerun`, which would
shadow the rerun SDK's `import rerun` and its `rerun` viewer CLI).

Each input id becomes a rerun entity path; streams are dispatched by Arrow
layout / metadata (see `docs/message_formats.md`):

| input | record logs | visualize shows |
| --- | --- | --- |
| `image` uint8[N] + `{encoding,width,height}` | H.264 `rr.VideoStream` / JPEG `rr.Image` | `rr.Image` / `rr.EncodedImage` |
| id ends `depth` | (numeric) | `rr.DepthImage` (uint16 mm) |
| `*tcp_pose` / `*tcp_target` float64[7] | `rr.Transform3D` + `rr.Points3D` | `rr.Transform3D` |
| `*joint*` / `*gripper*` / other numeric | `rr.Scalars` | `rr.Scalars` |

## Config (env)

Shared `APP_ID` is top-level; mode-specific params are namespaced under the mode
(`env_nested_delimiter="__"`).

| var | default | |
| --- | --- | --- |
| `MODE` | `visualize` | `record` or `visualize` |
| `APP_ID` | `dora` | rerun application id (shared) |
| `RECORD__RRD_DIR` | `rrd` | output dir for `episode_NNN.rrd` |
| `RECORD__FPS` | `30` | H.264 time base |
| `RECORD__IMAGE_MODE` | `h264` | `h264` or `jpeg` |
| `RECORD__JPEG_QUALITY` | `90` | only when `image_mode=jpeg` |
| `VISUALIZE__SINK` | `connect` | `spawn` / `connect` / `memory` |
| `VISUALIZE__VIEWER_URL` | — | gRPC url of a running viewer (connect) |
| `VISUALIZE__VIEWER_PORT` | `9876` | port for the spawned viewer |
| `VISUALIZE__CAMERAS` | `""` | camera ids to arrange in a grid |
| `VISUALIZE__CAMERA_FRAMES` | `""` | `input=model_camera` pairs mapping image streams onto the scene's camera frustums (ids that match a model camera map automatically; needs `SCENE`) |
| `VISUALIZE__MEMORY_LIMIT` | `2GB` | spawned viewer memory budget |

The `record` mode emits `node_state` edges (ready / episode saved); the
`visualize` mode is a pure sink and emits nothing.

## Test

```sh
uv run pytest -q                       # build-helper unit tests run anywhere
uv run pytest -q tests/test_record.py  # dora-run integration (needs a working dora daemon)
```
