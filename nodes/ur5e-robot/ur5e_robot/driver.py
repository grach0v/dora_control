"""UR5e driver — RTDE arm control + a Robotiq 2F-85, with graceful shutdown.

Wraps `ur_rtde` (RTDEControlInterface for `servoJ`/`moveJ`, RTDEReceiveInterface for
the measured joints + TCP pose) and a Robotiq URCap socket client. The mode talks to
this object in the descriptor's units: arm joints in radians, gripper opening in
2F-85 driver-joint radians (0 = open .. `gripper_max_rad` = closed) — the driver maps
that to the Robotiq 0..255 range so command / state / sim ctrl all share one unit.

`ur_rtde` is imported here only; it needs CMake + Boost and has no macOS wheel, so this
module is loaded just on the robot host (main.py builds the driver and injects it into
the mode). The mode and tests never import it.
"""

from __future__ import annotations

import logging

import rtde_control
import rtde_receive

from ur5e_robot.modes.follower import FollowerConfig
from ur5e_robot.robotiq_gripper import RobotiqGripper

logger = logging.getLogger("ur5e-robot")

GRIPPER_PORT = 63352  # Robotiq URCap socket on the UR controller


class URDriver:
    """One UR5e + its 2F-85. Joint control only (servoJ); pinocchio owns IK."""

    def __init__(self, ip: str, cfg: FollowerConfig):
        self.cfg = cfg
        self.rtde_c = rtde_control.RTDEControlInterface(ip)
        self.rtde_r = rtde_receive.RTDEReceiveInterface(ip)
        # The gripper is on the UR controller's URCap socket; skip it when it's wired
        # elsewhere (e.g. directly to the PC) or absent, so the arm still connects/runs.
        self.gripper = None
        if cfg.with_gripper:
            self.gripper = RobotiqGripper(ip, GRIPPER_PORT)
            self.gripper.activate()
        logger.info("UR5e %s connected (RTDE%s)", ip, " + Robotiq" if self.gripper else ", gripper disabled")

    # --- state reads -----------------------------------------------------------------

    def get_actual_q(self) -> list[float]:
        return self.rtde_r.getActualQ()  # 6 arm joints (rad)

    def get_actual_tcp_pose(self) -> list[float]:
        return self.rtde_r.getActualTCPPose()  # [x,y,z, rx,ry,rz] (m, rotation-vector)

    def get_gripper_position(self) -> float:
        """Measured opening as a 2F-85 driver-joint angle (rad), matching the descriptor.
        Reports a constant (open) when the gripper isn't driven by this node."""
        if self.gripper is None:
            return 0.0
        return self.gripper.get_position() / 255.0 * self.cfg.gripper_max_rad

    # --- commands --------------------------------------------------------------------

    def servo_j(self, q: list[float]) -> None:
        """Stream one joint setpoint (speed/accel are ignored by servoJ)."""
        self.rtde_c.servoJ(q, 0.0, 0.0, self.cfg.servo_time, self.cfg.servo_lookahead, self.cfg.servo_gain)

    def gripper_move(self, opening_rad: float) -> None:
        if self.gripper is None:
            return  # gripper not driven by this node
        pos = round(opening_rad / self.cfg.gripper_max_rad * 255.0)
        self.gripper.move(pos, self.cfg.gripper_speed, self.cfg.gripper_force)

    # --- shutdown --------------------------------------------------------------------

    def stop_and_release(self) -> None:
        """Graceful disconnect: smoothly decelerate the servoJ stream and end the control
        script, leaving the arm HOLDING its current pose (the UR controller's brakes keep
        it there) — no surprise motion. The gripper is left exactly as-is, so a held object
        is never dropped on disconnect."""
        self.rtde_c.servoStop()   # decelerate the servoJ motion, hold the current pose
        self.rtde_c.stopScript()  # end the RTDE control program; the controller holds position
        self.rtde_c.disconnect()
        self.rtde_r.disconnect()
        if self.gripper is not None:
            self.gripper.disconnect()
        logger.info("UR5e stopped in place and released")


def make_driver(ip: str, cfg: FollowerConfig) -> URDriver:
    return URDriver(ip, cfg)
