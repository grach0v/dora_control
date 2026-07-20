# realsense-camera

Dora node that opens an Intel RealSense device (selected by serial number) via
[`pyrealsense2`](https://pypi.org/project/pyrealsense2/) and publishes its color
stream on `<name>_image` at the configured fps — each captured frameset is
published as it lands, so the wire rate IS the sensor FPS (no `tick`). Optionally
publishes the depth stream on `<name>_depth`. Payloads are plain Apache Arrow
arrays with shape and encoding carried in metadata (see `docs/message_formats.md`).

See [../../docs/node_development.md](../../docs/node_development.md) for the node
pattern (self-paced producer, edge-triggered `node_state`).

> **Depth is not spatially aligned to color.** Color and depth come from the same
> frameset at the same `WIDTH`×`HEIGHT`, but the node does **not** run
> `rs.align(rs.stream.color)`, so per-pixel fusion of `image` and `depth` will be
> off. Add an align step if a consumer needs registered depth.

## Platform note

`pyrealsense2` ships wheels only for `manylinux_x86_64` and `win_amd64` — **not**
macOS. This node builds and runs **only on Linux x86_64 (or Windows) with a
RealSense attached**; on macOS `uv sync` fails at resolution time (expected — the
import is intentionally unguarded) and the tests skip cleanly.

## Outputs

| output   | payload                | metadata                      | notes                              |
| -------- | ---------------------- | ----------------------------- | ---------------------------------- |
| `<name>_image`  | `uint8[N]`             | `encoding`, `width`, `height` | `rgb8` raw or `jpeg` compressed    |
| `<name>_depth`  | `uint16[width*height]` | `width`, `height`             | millimetres; only if `ENABLE_DEPTH`|
| `<name>_node_state` | `utf8[1]`          | —                             | edge-triggered state token         |

## Inputs

| input         | source                | notes                              |
| ------------- | --------------------- | ---------------------------------- |
| `program_state`   | `manager/program_state`   | node stops when value == `disconnect` |

## Configuration (env)

| var            | default | meaning                                            |
| -------------- | ------- | -------------------------------------------------- |
| `CAMERA_NAME`  | —       | required; stream id / status label                 |
| `SERIAL`       | —       | required; RealSense device serial number to open   |
| `WIDTH`        | `640`   | color/depth width                                  |
| `HEIGHT`       | `480`   | color/depth height                                 |
| `FPS`          | `30`    | stream frame rate                                  |
| `ENCODING`     | `rgb8`  | `rgb8` or `jpeg`                                   |
| `JPEG_QUALITY` | `90`    | JPEG quality when `ENCODING=jpeg`                  |
| `ENABLE_DEPTH` | `false` | `true` to also publish the `depth` output          |

## Test (Linux with a RealSense attached)

```bash
cd nodes/realsense-camera && uv run pytest tests -q
```

The dataflow test runs the node via `dora run --stop-after` and asserts a logger
sees color (and, when enabled, depth) frames; it needs a physical device. On a
host without `pyrealsense2` (e.g. macOS) the tests skip cleanly.
