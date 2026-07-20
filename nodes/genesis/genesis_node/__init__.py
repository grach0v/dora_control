"""Genesis simulation node.

Physics + on-board cameras for a multi-arm cell, driven by SAFE joint targets from the
pinocchio node (Genesis does NOT do IK here — pinocchio owns it). Mirrors the real-robot
stream interface so sim and real are swappable in a dataflow. See modes/sim.py.

The package is named ``genesis_node`` (not ``genesis``) so it never shadows the Genesis
SDK (``import genesis``); the console script is ``genesis-node``.
"""

NODE_ID = "genesis"
