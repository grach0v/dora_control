# opencv-camera

Publishes frames from an OpenCV capture (a webcam device index, or any file /
stream path `cv2.VideoCapture` accepts) on the `<name>_image` output. A background
thread reads frames (capped at `FPS`) and keeps the latest one; the dora loop
publishes **each captured frame as it lands** with its capture `timestamp`, so the
wire rate IS the configured FPS (no `tick`, no timer-phase latency).

See [../../docs/node_development.md](../../docs/node_development.md) for the node
pattern (self-paced producer, edge-triggered `node_state`, drop-oldest on consumers).

## Inputs

| id            | source                | meaning |
| ------------- | --------------------- | ------- |
| `program_state`   | `manager/program_state`   | stops the node on `disconnect` (optional) |

## Outputs

| id       | payload             | meaning |
| -------- | ------------------- | ------- |
| `<name>_image`  | uint8[N] + metadata | `encoding`/`width`/`height` in metadata (see `docs/message_formats.md`) |
| `<name>_node_state` | string          | edge-triggered state token (`ready`, `… capture ended`) |

For `encoding: rgb8`, the payload is the raw H×W×3 RGB matrix bytes
(`N = width*height*3`). For `encoding: jpeg`, it is JPEG-compressed bytes
(`width`/`height` still in metadata). `rgb8` at 30 FPS is large, so consumers that
may fall behind should wire `image` `queue_size: 1` + `queue_policy: drop_oldest`.

## Config (env)

| var            | default | meaning |
| -------------- | ------- | ------- |
| `CAMERA_NAME`  | —       | required; stream id / status label |
| `CAMERA_INDEX` | `0`     | device index (`"0"`) or a file/stream path |
| `WIDTH`        | `640`   | requested capture width |
| `HEIGHT`       | `480`   | requested capture height |
| `FPS`          | `30`    | publish rate |
| `ENCODING`     | `rgb8`  | `rgb8` or `jpeg` |
| `JPEG_QUALITY` | `90`    | JPEG quality when `encoding: jpeg` |

## Tests

No unit tests here on purpose (tests are only for genuinely non-trivial pure
logic — see docs/node_development.md); the e2e smoke
(`cd dataflows && uv run --project ../nodes/lerobot pytest tests -q`) exercises
this node in the full graph.
