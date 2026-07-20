"""Web controller node — the operator's browser control surface.

Serves a closed-loop control page (NiceGUI + FastAPI). In ``manual`` mode the node
subscribes to each arm part's measured pose (``<part>_tcp_pose``, pinocchio's
model-frame FK) and shows the **real** live values; a per-field drag slider (drag
the target to a location, anchored where the slider is grabbed) and +/- buttons
(one fine step) move a pose target (seeded from the current pose, so no jump),
plus −/+ gripper buttons. On each ``tick`` the whole-robot **`command` bundle**
(the descriptor's command layout) is emitted — only while the program is in the
``teleop`` stage. ``episode`` mode serves only the task/episode/Disconnect panel.

``node.send_output`` is only ever called from the dora loop, never the HTTP
thread; the two share state under a lock. All payloads are plain Arrow
(docs/message_formats.md).

The page also serves recording + shutdown controls: a task field, Start/Finish
episode buttons (published on ``episode_control`` as ``start``/``finish``/``task=<text>``
for the recorders), and a Disconnect button (published once on ``robot_command`` as
``disconnect``; the manager broadcasts ``program_state: disconnect`` on it).

Inputs:  tick (publish cadence); program_state; `<part>_tcp_pose` +
         `<part>_solution_pose` feedback (manual mode, one per arm part)
Outputs: command (float64 bundle, manual mode only),
         episode_control (utf8: start|finish|task=<text>),
         robot_command (utf8: disconnect), node_state
"""

from __future__ import annotations

import sys
import threading

import uvicorn
from dora import Node
from fastapi import FastAPI
from nicegui import ui

from web_controller.modes import MODES
from web_controller.modes.manual import FIELDS, NODE_ID, State
from web_controller.node_config import WebControllerConfig, load_config


def build_app(state: State) -> FastAPI:
    """REST control surface. The browser uses the NiceGUI page (mounted on this
    same app); these JSON endpoints are the headless/programmatic control path
    (used by the tests). Both front-doors mutate the one shared `state`."""
    app = FastAPI()

    @app.get("/state")
    def get_state() -> dict:
        return state.snapshot()

    @app.post("/nudge")
    def nudge(n: dict) -> dict[str, bool]:
        arm, field = n.get("arm", ""), n.get("field")
        if field in FIELDS and arm in state.arm_parts:
            state.nudge(arm, field, 1 if int(n["dir"]) >= 0 else -1)
        return {"ok": True}

    @app.post("/gripper")
    def gripper(n: dict) -> dict[str, bool]:
        part = n.get("part", "")
        if part in state.gripper_parts:
            state.nudge_gripper(part, 1 if int(n["dir"]) >= 0 else -1)  # +1 open, -1 close
        return {"ok": True}

    @app.post("/task")
    def task(n: dict) -> dict[str, bool]:
        state.set_task(str(n.get("text", "")))
        return {"ok": True}

    @app.post("/episode")
    def episode(n: dict) -> dict[str, bool]:
        cmd = n.get("cmd")
        if cmd == "start":
            state.start_episode()
        elif cmd == "finish":
            state.finish_episode()
        return {"ok": True}

    @app.post("/disconnect")
    def disconnect(n: dict | None = None) -> dict[str, bool]:
        state.request_disconnect()
        return {"ok": True}

    return app


def build_ui(state: State) -> None:
    """Define the control page in pure Python with NiceGUI (no HTML/JS template).

    Drag sliders (begin_drag/drag_to/end_drag) and +/- buttons (nudge) call the shared
    `state` over NiceGUI's websocket, setting the target directly; a per-client timer
    refreshes the displayed current/target values ~5 Hz. The dora loop republishes the
    latest target on each tick (see ManualMode).
    """

    @ui.page("/")
    def index() -> None:
        ui.label("web-controller").classes("text-2xl font-bold")
        ui.label("Live actual TCP from the robot; drag a slider to move the target (the arm "
                 "chases it; releases where you left it), or −/+ for one fine step.").classes(
            "text-sm text-gray-500"
        )
        # Recording + shutdown panel.
        with ui.card():
            task_input = ui.input("task", placeholder="describe the task").on(
                "blur", lambda e: state.set_task(task_input.value or "")
            )
            with ui.row().classes("items-center gap-2"):
                ui.button("Start episode", on_click=state.start_episode).props("color=green")
                ui.button("Finish episode", on_click=state.finish_episode).props("color=orange")
                rec_label = ui.label()
                ui.button("Disconnect", on_click=state.request_disconnect).props("color=red outline")

        labels: dict = {}
        # `episode` mode shows only the recording panel above — no motion controls.
        arm_parts = state.arm_parts if state.cfg.mode == "manual" else []
        gripper_parts = state.gripper_parts if state.cfg.mode == "manual" else []
        for part in arm_parts:
            with ui.card():
                ui.label(f"{part}").classes("text-lg font-bold")
                with ui.grid(columns=4).classes("items-center gap-x-3 gap-y-1"):
                    for head in ("field", "current", "target", "drag → target · ∓ step"):
                        ui.label(head).classes("text-xs text-gray-500")
                    for f in FIELDS:
                        ui.label(f)
                        cur, tgt = ui.label("—"), ui.label("—")
                        with ui.row().classes("items-center gap-1"):
                            ui.button("−", on_click=lambda p=part, fl=f: state.nudge(p, fl, -1)).props("dense flat")
                            # Drag = move the target to a location: on grab we anchor to the current
                            # target; the deflection sets `target = anchor ± span`, so the arm chases
                            # the dragged target (a responsiveness test). Release drops the anchor
                            # (target holds) and recentres the slider (that event is a no-op).
                            slider = ui.slider(min=-1, max=1, step=0.01, value=0).props("dense").style("width: 120px")
                            slider.on("pointerdown", lambda _e, p=part, fl=f: state.begin_drag(p, fl))
                            slider.on_value_change(lambda e, p=part, fl=f: state.drag_to(p, fl, e.value))

                            def _release(_e, s=slider, p=part, fl=f) -> None:
                                state.end_drag(p, fl)
                                s.set_value(0)

                            slider.on("pointerup", _release)
                            slider.on("pointercancel", _release)
                            ui.button("+", on_click=lambda p=part, fl=f: state.nudge(p, fl, 1)).props("dense flat")
                        labels[(part, f)] = (cur, tgt)
        if gripper_parts:
            with ui.card():
                ui.label("grippers").classes("text-lg font-bold")
                for gp in gripper_parts:
                    with ui.row().classes("items-center gap-2"):
                        ui.label(gp)
                        gl = ui.label("—")
                        ui.button("−", on_click=lambda p=gp: state.nudge_gripper(p, -1)).props("dense flat").tooltip("close")
                        ui.button("+", on_click=lambda p=gp: state.nudge_gripper(p, 1)).props("dense flat").tooltip("open")
                        labels[("gripper", gp)] = gl

        def refresh() -> None:
            snap = state.snapshot()
            rec_label.text = "● recording" if snap["recording"] else "idle"
            for part in arm_parts:
                c, t = snap["current"][part], snap["target"][part]
                for i, f in enumerate(FIELDS):
                    cur, tgt = labels[(part, f)]
                    cur.text = f"{c[i]:.3f}" if c else "—"
                    tgt.text = f"{t[i]:.3f}" if t else "—"
            for gp in gripper_parts:
                labels[("gripper", gp)].text = f"{snap['gripper'][gp]:.3f}"

        ui.timer(0.2, refresh)

def run(node: Node, cfg: WebControllerConfig) -> None:
    state = State(cfg)
    app = build_app(state)        # REST control surface (tests)
    build_ui(state)               # NiceGUI page mounted on the same app (browser)
    ui.run_with(app, title="web-controller")
    # uvicorn runs in a daemon thread; the dora loop owns the node and is the
    # only caller of send_output.
    server = uvicorn.Server(
        uvicorn.Config(app, host=cfg.host, port=cfg.port, log_level="warning")
    )
    threading.Thread(target=server.run, daemon=True).start()
    print(f"{NODE_ID}: serving on http://{cfg.host}:{cfg.port}", flush=True)

    # Tick-driven: feedback updates state any time; the mode republishes the
    # latest target on each `tick` (coalescing browser nudges to the tick rate).
    mode = MODES[cfg.mode](cfg, node, state)  # KeyError on a bad MODE = loud
    mode.start()
    try:
        for event in node:
            if event["type"] == "STOP":
                break
            if event["type"] != "INPUT":
                continue
            if mode.handle(event):  # True -> program_state stop
                break
    finally:
        mode.close()


def main() -> int:
    cfg = load_config()
    node = Node()
    run(node, cfg)
    return 0


if __name__ == "__main__":
    sys.exit(main())
