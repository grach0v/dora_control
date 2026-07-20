"""`episode` mode — episode + task control only, no motion.

For dataflows where motion comes from elsewhere (leader-arm teleop, or a policy): the
page is just the task field + Start/Finish + Disconnect. On each `tick` it drains the
operator's episode/task presses to `episode_control` and a one-shot `disconnect` to
`robot_command` — it never publishes the `command` bundle and takes no pose feedback.
A stray feedback input is a wiring bug and raises.

Inputs:  tick, program_state.
Outputs: episode_control (utf8: start|finish|task=<text>), robot_command (utf8: disconnect), node_state.
"""

from __future__ import annotations

import logging
import time
from typing import Callable

import pyarrow as pa
from dora import Node

from web_controller.modes.manual import NODE_ID, State
from web_controller.node_config import WebControllerConfig

logger = logging.getLogger("web-controller")


class EpisodeMode:
    def __init__(self, cfg: WebControllerConfig, node: Node, state: State):
        self.cfg = cfg
        self.node = node
        self.state = state
        self.disconnecting = False
        self.status = ""
        self._handlers: dict[str, Callable] = {
            "program_state": self._on_program_state,
            "tick": self._on_tick,
        }

    def start(self) -> None:
        logger.info("%s: episode control — open http://%s:%s", NODE_ID, self.cfg.host, self.cfg.port)
        self._emit_status("ready")

    def _emit_status(self, text: str) -> None:
        self.status = text
        self.node.send_output("node_state", pa.array([text]))

    def handle(self, event) -> bool:
        return bool(self._handlers[event["id"]](event))  # KeyError on an unwired input id = loud

    def close(self) -> None:
        pass

    def _on_program_state(self, event) -> bool:
        return event["value"][0].as_py() == "disconnect"


    def _on_tick(self, event) -> None:
        md = {"timestamp": time.time()}
        for msg in self.state.drain_pending():
            self.node.send_output("episode_control", pa.array([msg]), metadata=md)
        if not self.disconnecting and self.state.disconnect_requested:
            self.disconnecting = True
            self.node.send_output("robot_command", pa.array(["disconnect"]), metadata=md)
