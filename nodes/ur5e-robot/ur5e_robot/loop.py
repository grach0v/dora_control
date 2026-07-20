"""The dora event loop, factored out of main.py so it imports no hardware SDK.

`main.py` (which builds the `ur_rtde`-bound driver) and the unit tests both import
`run` from here; keeping it SDK-free is what lets the tests drive a `FollowerMode`
with a fake driver on a host without `ur_rtde` (e.g. macOS — see the node README).
"""

from __future__ import annotations

from dora import Node


def run(node: Node, mode) -> None:
    """Drive the arm until shutdown, then ALWAYS tear the mode down (home/release)
    on any exit path — dora STOP, program_state stop, or error — unless the mode already
    disconnected (operator Disconnect), so the arm is never left energized or homed
    twice. The loop reacts to STOP/program_state promptly, so dora never needs to escalate
    to SIGTERM and the finally always reaches close()."""
    mode.start()
    try:
        for event in node:
            if event["type"] == "STOP":
                break
            if event["type"] != "INPUT":
                continue
            if mode.handle(event):  # True -> program_state stop / operator disconnect
                break
    finally:
        mode.close()
