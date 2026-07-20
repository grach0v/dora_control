"""MuJoCo simulation node.

Physics + on-board cameras for a multi-arm cell, driven by SAFE joint targets from the
pinocchio node (MuJoCo does NOT do IK here — pinocchio owns it). A fast local alternative
to the genesis node: same scene descriptor (the asset zoo), same stream interface, so the
two are interchangeable per dataflow. Unlike Genesis, MuJoCo renders fine on a background
thread, so cameras render off the dora loop. See modes/sim.py.
"""

NODE_ID = "mujoco-sim"
