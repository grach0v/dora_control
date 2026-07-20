"""Pinocchio IK + safety node.

Whole-robot kinematics for a multi-arm cell: turns Cartesian or joint commands into
SAFE joint targets, with self-collision (arm-vs-arm) avoidance and a table-plane
barrier. Simulator-independent — the same node guards a Genesis sim and the real
robot. See modes/control.py for the I/O and the PASS/HOLD safety gate.

The package is named ``pinocchio_node`` (not ``pinocchio``) so it never shadows the
Pinocchio SDK (``import pinocchio``); the console script is ``pinocchio-node``.
"""

NODE_ID = "pinocchio"
