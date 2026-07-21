"""`teleop` mode — the default program lifecycle.

BOOT (every producer reported `ready`) ->
HOMING (pinocchio ramps the robot to the model home) ->
TELEOPERATE ->
DISCONNECT (final; the web controller's disconnect button, wired as the
manager's `controller` input — valid in every state).

Node-state vocabulary the machine relies on (docs/message_formats.md):
`ready` (node initialized and operational — every producer must report it once
to leave BOOT) and pinocchio's `homing` / `homed`.
Everything else is informative only.
"""

from __future__ import annotations

import pyarrow as pa
from statemachine import State, StateMachine

class TeleopProgram(StateMachine):
    boot = State(value="boot", initial=True)
    homing = State(value="homing")
    teleoperate = State(value="teleop")
    disconnect = State(value="disconnect", final=True)

    begin_homing = boot.to(homing, cond="all_ready")
    finish_homing = homing.to(teleoperate, cond="homed")
    # The controller's red button is valid in every state.
    request_disconnect = (
        boot.to(disconnect, cond="disconnect_requested")
        | homing.to(disconnect, cond="disconnect_requested")
        | teleoperate.to(disconnect, cond="disconnect_requested")
    )

    def __init__(self, producers: list[str], node):
        self.producers = producers
        self.node = node
        self.node_state: dict[str, str] = {}   # producer -> latest state token
        self.ready: set[str] = set()           # producers that have reported `ready`
        # Attributes must exist before super().__init__(): entering the initial
        # state already broadcasts via on_enter_state.
        super().__init__()

    def observe(self, producer: str, token: str) -> None:
        """One node-state event: record it, then fire every currently-enabled event."""
        self.node_state[producer] = token
        if token == "ready":
            # Readiness is a latch: a producer counts once it has reported `ready`,
            # even if its state has since moved on (e.g. sync flipping to `synced`).
            self.ready.add(producer)

        for event in self.enabled_events():
            self.send(event.id)

    def all_ready(self) -> bool:
        return self.ready >= set(self.producers)

    def homed(self) -> bool:
        return self.node_state.get("pinocchio") == "homed"

    def disconnect_requested(self) -> bool:
        return self.node_state.get("controller") == "disconnect"

    def on_enter_state(self, state, **kwargs) -> None:
        self.node.send_output("program_state", pa.array([state.value]))
