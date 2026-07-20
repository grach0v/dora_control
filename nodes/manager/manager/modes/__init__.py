"""Mode registry for the manager node — env ``MODE`` -> program state machine.

Every program machine takes the producer list, exposes ``observe(producer, token)``
and an ``advance`` event, and marks its terminal states ``final``; main.py drives
any of them the same way.
"""

from manager.modes.teleop import TeleopProgram

MODES = {"teleop": TeleopProgram}

__all__ = ["MODES", "TeleopProgram"]
