"""`record` mode — log incoming streams to per-episode ``.rrd`` files.

Recording is gated by a ``control`` input (from the web/record node): ``start``
begins a fresh episode, ``finish`` writes ``<rrd_dir>/episode_<NNN>.rrd`` and goes
idle; ``task=`` is ignored. Each input id becomes a rerun entity path; streams are
dispatched by their Arrow layout / metadata (see docs/message_formats.md):

  * image (uint8[N] + {encoding,width,height}) -> H.264 VideoStream (IMAGE_MODE=h264)
    or JPEG-compressed rr.Image (IMAGE_MODE=jpeg)
  * tcp_pose / tcp_target (float64[7])         -> rr.Transform3D + rr.Points3D
  * joint* (float64[J]) / gripper* (float64[1]) / any other numeric -> rr.Scalars

The mode's state (recording flag, episode counter, fallback seq, per-entity video
encoders) lives in the `RecordMode` instance. Control inputs flip the state via an
explicit handler table; any *other* input id is a data stream logged when
recording (idle: ignored) — so an unrecognized id is not a wiring bug here.

Inputs:  control, program_state, plus any <stream> to record.
Outputs: node_state.
"""

from __future__ import annotations

import logging
import io
from pathlib import Path
from typing import Callable, Literal

import av
import numpy as np
import pyarrow as pa
import rerun as rr
from dora import Node
from pydantic import BaseModel

from rerun_node import NODE_ID

logger = logging.getLogger("rerun")


class RecordConfig(BaseModel):
    # Directory where per-episode `.rrd` files are written (episode_NNN.rrd).
    rrd_dir: Path = Path("rrd")
    # Frames-per-second used as the H.264 time base for image streams.
    fps: int = 30
    # "h264" logs camera images as a true video stream (small files);
    # "jpeg" falls back to JPEG-compressed per-frame images.
    image_mode: Literal["h264", "jpeg"] = "h264"
    # JPEG quality (1..100) used only when image_mode == "jpeg".
    jpeg_quality: int = 90


class VideoEncoder:
    """Per-entity H.264 encoder that yields rerun VideoStream samples.

    Opens a raw-H.264 (Annex B) muxer to /dev/null so ``bytes(packet)`` is a
    self-contained sample rerun can decode directly — no bitstream filter or
    container needed. B-frames are disabled (rerun video streams don't support
    them) so packet order matches presentation order.
    """

    def __init__(self, width: int, height: int, fps: int):
        self.container = av.open("/dev/null", mode="w", format="h264")
        self.stream = self.container.add_stream("libx264", rate=fps)
        self.stream.width = width
        self.stream.height = height
        self.stream.pix_fmt = "yuv420p"
        self.stream.max_b_frames = 0
        self.stream.codec_context.options = {"tune": "zerolatency", "preset": "veryfast"}
        self.pts = 0

    def encode(self, rgb: np.ndarray):
        """Encode one RGB frame; return a list of (sample_bytes, is_keyframe)."""
        frame = av.VideoFrame.from_ndarray(rgb, format="rgb24")
        frame.pts = self.pts
        frame.pict_type = av.video.frame.PictureType.NONE
        self.pts += 1
        return [(bytes(p), bool(p.is_keyframe)) for p in self.stream.encode(frame)]

    def flush(self):
        out = [(bytes(p), bool(p.is_keyframe)) for p in self.stream.encode(None)]
        self.container.close()
        return out


def decode_image(value: pa.Array, md: dict) -> np.ndarray:
    """Return an HxWx3 uint8 RGB array from an `image` payload + metadata."""
    encoding = md["encoding"]
    width = int(md["width"])
    height = int(md["height"])
    raw = value.to_numpy(zero_copy_only=False)
    if encoding == "rgb8":
        return raw.reshape(height, width, 3)
    if encoding == "jpeg":
        with av.open(io.BytesIO(raw.tobytes()), mode="r") as container:
            frame = next(container.decode(video=0))
            return frame.to_ndarray(format="rgb24")
    raise ValueError(f"unsupported image encoding {encoding!r}")


def log_event(input_id: str, value: pa.Array, md: dict, encoders: dict, cfg: RecordConfig, seq: int) -> int:
    """Log one data event to the active rerun recording. Returns the updated
    fallback sequence counter (used only when a stream has no timestamp)."""
    ts = md.get("timestamp")
    if ts is not None:
        rr.set_time("capture", timestamp=float(ts))
    else:
        seq += 1
        rr.set_time("seq", sequence=seq)

    if "encoding" in md:  # an `image` stream
        rgb = decode_image(value, md)
        if cfg.image_mode == "jpeg":
            rr.log(input_id, rr.Image(rgb).compress(jpeg_quality=cfg.jpeg_quality))
        else:
            enc = encoders.get(input_id)
            if enc is None:
                height, width = rgb.shape[:2]
                enc = VideoEncoder(width, height, cfg.fps)
                encoders[input_id] = enc
                rr.log(input_id, rr.VideoStream(codec=rr.VideoCodec.H264), static=True)
            for sample, is_keyframe in enc.encode(rgb):
                rr.log(input_id, rr.VideoStream.from_fields(sample=sample, is_keyframe=is_keyframe))
        return seq

    arr = value.to_numpy(zero_copy_only=False).astype(float)
    if input_id.endswith(("tcp_pose", "tcp_target")):
        rr.log(input_id, rr.Transform3D(translation=arr[:3], quaternion=arr[3:7]))  # xyzw
        rr.log(f"{input_id}/position", rr.Points3D([arr[:3]]))
    elif "joint" in input_id:
        for j, v in enumerate(arr):
            rr.log(f"{input_id}/{j}", rr.Scalars(float(v)))
    elif "gripper" in input_id:
        rr.log(input_id, rr.Scalars(float(arr[0])))
    else:  # any other numeric stream: one scalar per component so nothing is lost
        for j, v in enumerate(arr):
            rr.log(f"{input_id}/{j}", rr.Scalars(float(v)))
    return seq


class RecordMode:
    """Recording state machine (idle <-> recording)."""

    def __init__(self, cfg: RecordConfig, node: Node, app_id: str):
        self.cfg = cfg
        self.node = node
        self.app_id = app_id
        self.recording = False
        self.episode = 0
        self.seq = 0
        self.encoders: dict[str, VideoEncoder] = {}
        self.status = ""
        self._handlers: dict[str, Callable] = {
            "program_state": self._on_program_state,
            "control": self._on_control,
        }

    def start(self) -> None:
        self.cfg.rrd_dir.mkdir(parents=True, exist_ok=True)
        logger.info("%s: record -> %s (idle until 'start')", NODE_ID, self.cfg.rrd_dir)
        self._emit_status("ready")

    def _emit_status(self, text: str) -> None:
        self.status = text
        self.node.send_output("node_state", pa.array([text]))

    def handle(self, event) -> bool:
        handler = self._handlers.get(event["id"], self._default)
        return bool(handler(event))

    def close(self) -> None:
        self._finish()  # preserve an in-progress episode on stop / error / signal

    def _finish(self) -> None:
        """Flush encoders and write the in-progress episode's .rrd."""
        if not self.recording:
            return
        for input_id, enc in self.encoders.items():
            for sample, is_keyframe in enc.flush():
                rr.log(input_id, rr.VideoStream.from_fields(sample=sample, is_keyframe=is_keyframe))
        path = self.cfg.rrd_dir / f"episode_{self.episode:03d}.rrd"
        rr.save(str(path))
        self._emit_status(f"{NODE_ID}: saved {path}")
        self.episode += 1
        self.encoders = {}
        self.recording = False

    def _on_program_state(self, event) -> bool:
        return event["value"][0].as_py() == "disconnect"  # True -> stop the loop


    def _on_control(self, event) -> None:
        cmd = event["value"][0].as_py()
        if cmd == "start":
            self._finish()  # safety: close a previous episode if still open
            rr.init(self.app_id, recording_id=f"episode_{self.episode}")
            self.encoders, self.recording = {}, True
        elif cmd == "finish":
            self._finish()

    def _default(self, event) -> None:
        """Data-stream fallback: log any other input id as a stream when recording
        (the input id is its rerun entity path). Idle: nothing to record."""
        if self.recording:
            self.seq = log_event(event["id"], event["value"], event["metadata"], self.encoders, self.cfg, self.seq)
