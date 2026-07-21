"""`visualize` mode — stream incoming streams to a rerun viewer, live.

Visualize-only and **stateless**: each message is logged to a running viewer and
immediately forgotten (never ``rr.save``), so a long session does not grow this
node's memory. The node is always a gRPC *client*; the viewer is a separate
process. Sinks (``VISUALIZE__SINK``):
  * ``spawn``   -> launch the bundled viewer in its OWN session and connect to it.
  * ``connect`` -> connect to a viewer you started yourself (`rerun --port 9876`).
  * ``memory``  -> in-process sink, no viewer/window (headless tests).

Logging runs on a worker thread fed by a small drop-oldest queue, so a slow or
disconnected viewer can never block the dora event loop (the node always sees STOP
promptly). Dispatch is by message shape, not input id, so any number of cameras
each get their own view:
  any input with `encoding` in metadata -> rr.Image / rr.EncodedImage
  *depth                                -> rr.DepthImage (uint16 mm)
  *tcp_pose / *tcp_target               -> rr.Transform3D (translation + xyzw quat)
  everything else (float vectors)       -> rr.Scalars

Inputs:  program_state, plus any camera/state stream wired in YAML.
"""

from __future__ import annotations

import os
import queue
import socket
import subprocess
import threading
import time
from typing import Annotated, Callable, Literal

import pyarrow as pa
import rerun as rr
import rerun.blueprint as rrb
import rerun_cli
from dora import Node
from pydantic import BaseModel, field_validator
from pydantic_settings import NoDecode

from rerun_node.robot3d import RobotScene

# The viewer binary bundled with rerun-sdk.
VIEWER_PATH = os.path.join(os.path.dirname(rerun_cli.__file__), "rerun")


class VisualizeConfig(BaseModel):
    # Where logged data goes: "spawn" (launch a viewer), "connect" (a running
    # viewer at viewer_url), or "memory" (in-process sink; headless CI).
    sink: Literal["spawn", "connect", "memory"] = "connect"
    # gRPC address of a running viewer, e.g. "rerun+http://127.0.0.1:9876/proxy".
    # Empty -> None (let the SDK use its default address).
    viewer_url: str | None = None
    # Port the spawned viewer listens on (sink="spawn").
    viewer_port: int = 9876
    # Camera entity ids to arrange in a grid (comma-separated VISUALIZE__CAMERAS).
    cameras: Annotated[list[str], NoDecode] = []
    # Viewer memory budget (the viewer drops oldest data past it).
    memory_limit: str = "2GB"
    # Optional 3D robot view: path to the scene descriptor. Set -> load the robot model
    # and draw the articulated robot in 3D, driven by the joint-state stream (FK only).
    scene: str | None = None
    # Which input id carries the whole-robot joint-state vector (drives the 3D robot).
    robot_state_input: str = "state"
    # Map camera input ids to model camera names (`input=camera` pairs, comma-separated,
    # e.g. "cam_static1=cam_high,cam_wrist1=cam_left_wrist") for rigs whose input ids
    # differ from the MJCF camera names. An input id that IS a model camera name (the sim)
    # maps automatically. Mapped streams render inside their 3D frustum. Needs `scene`.
    camera_frames: Annotated[dict[str, str], NoDecode] = {}

    @field_validator("cameras", mode="before")
    @classmethod
    def _split_csv(cls, v: object) -> object:
        if isinstance(v, str):
            return [c.strip() for c in v.split(",") if c.strip()]
        return v

    @field_validator("camera_frames", mode="before")
    @classmethod
    def _split_pairs(cls, v: object) -> object:
        if isinstance(v, str):
            return dict(pair.strip().split("=") for pair in v.split(",") if pair.strip())
        return v

    @field_validator("viewer_url", "scene", mode="before")
    @classmethod
    def _empty_to_none(cls, v: object) -> object:
        if isinstance(v, str) and not v.strip():
            return None
        return v


def _listening(port: int) -> bool:
    """True if something already accepts TCP on 127.0.0.1:port."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.25)
        return s.connect_ex(("127.0.0.1", port)) == 0


def start_viewer(cfg: VisualizeConfig) -> subprocess.Popen:
    """Launch the rerun viewer as a detached process in its OWN session and return
    the handle, once it is actually listening. start_new_session=True keeps the
    viewer out of this node's process group, so dora's stop neither waits on nor
    group-signals the window — we own its lifetime explicitly and terminate it in
    close() (see VisualizeMode.close), so the window doesn't linger as an orphan
    after the run is stopped/killed.

    Two guards, because a `spawn` sink that silently attaches to the WRONG viewer is
    the classic "viewer shows old geometry no matter what I edit" bug:
      * If the port is already held, a viewer is already there (a leftover from a
        SIGKILLed run, or one you launched yourself). The new `rerun` below would
        fail to bind and die, and the caller's connect_grpc would attach to that OLD
        viewer — which keeps showing its OLD recording, ignoring this run's data and
        any model edits. So refuse loudly instead of rendering into a stranger.
      * After spawning, wait until the gRPC port is up BEFORE returning, so the
        caller's connect_grpc + one-shot static-mesh logs don't race a not-yet-ready
        viewer and get dropped ("re_grpc_client::write: transport error")."""
    if _listening(cfg.viewer_port):
        raise RuntimeError(
            f"a process is already listening on 127.0.0.1:{cfg.viewer_port} — a leftover "
            f"rerun viewer. This node would connect to it and you'd see ITS stale recording "
            f"(old geometry), ignoring this run and any model edits. Kill it first: "
            f"`pkill -f 'rerun.*{cfg.viewer_port}'` (or set VISUALIZE__VIEWER_PORT to a free port)."
        )
    proc = subprocess.Popen(
        [VIEWER_PATH, "--port", str(cfg.viewer_port), "--memory-limit", cfg.memory_limit],
        start_new_session=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    deadline = time.monotonic() + 15.0
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            raise RuntimeError(f"rerun viewer exited at startup (code {proc.returncode})")
        if _listening(cfg.viewer_port):
            return proc
        time.sleep(0.1)
    proc.terminate()
    raise RuntimeError(f"rerun viewer not listening on :{cfg.viewer_port} after 15s")


def viewer_url(cfg: VisualizeConfig) -> str:
    return f"rerun+http://127.0.0.1:{cfg.viewer_port}/proxy"


def build(entity: str, value: pa.Array, md: dict):
    """Convert one dora message to a (entity, rerun-archetype) pair.

    All array data is copied out of the dora shared-memory buffer here, on the
    dora thread, so the result is safe to log later from the worker thread.
    """
    if "encoding" in md:                       # any camera stream
        buf = value.to_numpy(zero_copy_only=False)
        if md["encoding"] == "jpeg":
            return entity, rr.EncodedImage(contents=buf.tobytes(), media_type="image/jpeg")
        if md["encoding"] == "rgb8":
            h, w = int(md["height"]), int(md["width"])
            return entity, rr.Image(buf.reshape(h, w, 3).copy(), color_model="RGB")
        raise ValueError(f"unsupported image encoding {md['encoding']!r}")
    if entity.endswith("depth"):
        h, w = int(md["height"]), int(md["width"])
        return entity, rr.DepthImage(value.to_numpy(zero_copy_only=False).reshape(h, w).copy(), meter=1000.0)
    if entity.endswith(("tcp_pose", "tcp_target")):
        p = value.to_numpy(zero_copy_only=False)
        return entity, rr.Transform3D(translation=p[:3].copy(), quaternion=rr.Quaternion(xyzw=p[3:7].copy()))
    # joint/gripper float vectors -> all series under ONE entity (one plot).
    return entity, rr.Scalars(value.to_numpy(zero_copy_only=False).tolist())


def build_blueprint(cameras: list[str], robot_origin: str | None):
    """Cameras (grid) and/or the 3D robot, side by side, above a shared time-series
    plot — so every camera + the 3D scene are visible (the viewer's auto-layout
    otherwise surfaces just one)."""
    top = []
    if cameras:
        top.append(rrb.Grid(*[rrb.Spatial2DView(origin=cam, name=cam.rsplit("/", 1)[-1])
                              for cam in cameras]))
    if robot_origin is not None:
        top.append(rrb.Spatial3DView(origin=robot_origin, name="robot"))
    top_view = rrb.Horizontal(*top) if len(top) > 1 else top[0]
    return rrb.Blueprint(
        rrb.Vertical(
            top_view,
            rrb.TimeSeriesView(origin="/", name="state"),
            row_shares=[3, 1],
        ),
        collapse_panels=True,
    )


class VisualizeMode:
    """Stateless live-viewer state machine: `program_state` stop -> stop; any other
    input id is a data stream built into an archetype and pushed onto a drop-oldest
    queue for the worker thread to log. The only state is the queue + worker."""

    def __init__(self, cfg: VisualizeConfig, node: Node, app_id: str):
        self.cfg = cfg
        self.node = node
        self.app_id = app_id
        self.frames: queue.Queue = queue.Queue(maxsize=8)
        self._thread = threading.Thread(target=self._worker, name="rerun-log", daemon=True)
        self._handlers: dict[str, Callable] = {"program_state": self._on_program_state}
        self._viewer: subprocess.Popen | None = None  # spawned viewer, killed in close()
        # Optional 3D robot view (loads the descriptor's model; FK driven by joint state).
        self.robot = RobotScene(cfg.scene) if cfg.scene else None
        # Camera input id -> model camera name: identity for ids that are model cameras
        # (the sim), plus the explicit CAMERA_FRAMES pairs (real rigs). Mapped image
        # streams are logged at the camera's 3D entity, inside its frustum.
        if cfg.camera_frames and self.robot is None:
            raise ValueError("VISUALIZE__CAMERA_FRAMES requires VISUALIZE__SCENE")
        self._cam_of: dict[str, str] = {}
        if self.robot is not None:
            self._cam_of = {name: name for name in self.robot.cameras}
            for input_id, cam in cfg.camera_frames.items():
                if cam not in self.robot.cameras:
                    raise ValueError(f"CAMERA_FRAMES: {cam!r} is not a camera in the model "
                                     f"(has: {sorted(self.robot.cameras)})")
                self._cam_of[input_id] = cam
        self._pinhole_pending = set(self._cam_of)

    def start(self) -> None:
        # Pick a sink; never rr.save (this node holds no persistent recording).
        rr.init(self.app_id, spawn=False)
        if self.cfg.sink == "memory":
            rr.memory_recording()  # headless: keeps data in process, writes nothing
        elif self.cfg.sink == "spawn":
            self._viewer = start_viewer(self.cfg)  # our own viewer, terminated in close()
            rr.connect_grpc(viewer_url(self.cfg))
        else:
            rr.connect_grpc(self.cfg.viewer_url)
        robot_origin = self.robot.prefix if self.robot else None
        if (self.cfg.cameras or self.robot) and self.cfg.sink != "memory":
            # Grid views must target where the images actually land: the camera's 3D
            # entity for streams mapped into a frustum, the bare input id otherwise.
            origins = [self.robot.cameras[self._cam_of[c]] if c in self._cam_of else c
                       for c in self.cfg.cameras]
            rr.send_blueprint(build_blueprint(origins, robot_origin))
        if self.robot is not None:  # link meshes + non-moving link poses are static — log once
            for entity, archetype in self.robot.static_logs():
                rr.log(entity, archetype, static=True)
            for entity, transform in self.robot.static_transforms():
                rr.log(entity, transform, static=True)  # place table / risers / inert arm once
        self._thread.start()

    def handle(self, event) -> bool:
        handler = self._handlers.get(event["id"], self._default)
        return bool(handler(event))

    def close(self) -> None:
        # Stop the worker, then drop rerun's atexit flush hook before returning.
        # For a spawned/remote viewer that final flush blocks on the (now possibly
        # disconnected) viewer well past dora's stop grace -> SIGTERM. Blocking put
        # so the worker reliably receives the sentinel and returns BEFORE we exit —
        # otherwise it can be killed mid-rr.log during teardown and segfault.
        self.frames.put(None)
        rr.unregister_shutdown()
        # Close the live viewer window so it doesn't linger as an orphan after the
        # run stops. terminate() (SIGTERM) lets it shut down; escalate to kill() if
        # it doesn't exit within the stop grace. (On a SIGKILL of the whole run the
        # node gets no close(), so the detached viewer survives — unavoidable.)
        if self._viewer is not None:
            self._viewer.terminate()
            try:
                self._viewer.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self._viewer.kill()

    def _on_program_state(self, event) -> bool:
        return event["value"][0].as_py() == "disconnect"  # True -> stop the loop

    def _default(self, event) -> None:
        """Data-stream fallback: build a {entity: archetype} batch (copying out of
        shared memory on this thread) and push it to the drop-oldest queue for the
        worker to log. The joint-state stream becomes the whole 3D robot (one FK ->
        many link transforms) plus its scalar plot; everything else is one entity."""
        eid = event["id"]
        if self.robot is not None and eid == self.cfg.robot_state_input:
            state = event["value"].to_numpy(zero_copy_only=False)
            batch = self.robot.transforms(state)     # {robot/geom_i: Transform3D, ...}
            batch[eid] = rr.Scalars(state.tolist())  # keep the joint-angle time series
        else:
            entity, archetype = build(eid, event["value"], event["metadata"])
            cam = self._cam_of.get(eid)
            if cam is not None and "encoding" in event["metadata"]:
                entity = self.robot.cameras[cam]  # render inside the camera's 3D frustum
                if eid in self._pinhole_pending:  # one-off: intrinsics need the stream's
                    self._pinhole_pending.discard(eid)  # real resolution, known only now
                    md = event["metadata"]
                    rr.log(*self.robot.camera_pinhole(cam, int(md["width"]), int(md["height"])),
                           static=True)
            batch = {entity: archetype}
        self._push(batch)

    def _push(self, batch: dict) -> None:
        try:
            self.frames.put_nowait(batch)
        except queue.Full:
            # Viewer is behind: drop the oldest queued batch for the newest one.
            try:
                self.frames.get_nowait()
                self.frames.put_nowait(batch)
            except (queue.Empty, queue.Full):
                pass

    def _worker(self) -> None:
        # rr.log can BLOCK when the viewer/sink can't keep up. Logging on a worker
        # (fed by the queue) keeps the dora loop unblocked so it always sees STOP.
        seq = 0
        while True:
            item = self.frames.get()
            if item is None:
                return
            # Coalesce a burst: keep only the newest archetype per entity, so a slow
            # viewer renders the latest frame ASAP and never shows stale ones — and
            # no stream is starved by another (unlike a blind drop-oldest queue).
            batch = dict(item)
            stopping = False
            while True:
                try:
                    nxt = self.frames.get_nowait()
                except queue.Empty:
                    break
                if nxt is None:  # stop sentinel arrived mid-drain: flush, then exit
                    stopping = True
                    break
                batch.update(nxt)
            # One timeline point per coalesced frame (not per entity): all entities in a frame
            # share a seq, so the viewer gets ~30 time points/s instead of ~30*N — a flood of
            # distinct points bloats the viewer's timeline index and shows up as seconds of lag.
            seq += 1
            rr.set_time("log_seq", sequence=seq)
            for entity, archetype in batch.items():
                rr.log(entity, archetype)
            if stopping:
                return
