"""Reusable sync (aggregator) node.

Collects several event-driven inputs, holds the latest of each, and on its own `tick` emits
ONE concatenated bundle (latest-of-each) — the synchronized whole-robot snapshot the rest of
the stack expects (e.g. two leader arms → one `command`, two follower states → one `state`).
It never waits for all inputs (no stall / phase-coupling); instead it WARNS when the inputs'
timestamps are skewed beyond a tolerance, so a silent ~1-frame misalignment can't hide. It is
robot-agnostic: it concatenates the configured inputs in order; the scene descriptor defines
what that vector means.
"""

NODE_ID = "sync"
