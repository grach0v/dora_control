"""trossen-arm driver setup + graceful shutdown, shared by the robot's modes.

`make_driver` connects an arm in position mode (follower stays there; the leader
mode drives it to its staged pose and then switches itself to backdrivable
external-effort — see modes/leader.py). `home_and_release` folds an arm to its
sleep pose under torque before releasing; `release` just drops torque. These run
only on the real robot.
"""

from __future__ import annotations

import logging

import trossen_arm

logger = logging.getLogger("trossen-robot")


def make_driver(mode: str, ip: str) -> trossen_arm.TrossenArmDriver:
    leader = mode == "leader"
    driver = trossen_arm.TrossenArmDriver()
    driver.configure(
        trossen_arm.Model.wxai_v0,
        trossen_arm.StandardEndEffector.wxai_v0_leader if leader else trossen_arm.StandardEndEffector.wxai_v0_follower,
        ip,
        False,  # don't clear errors silently — a faulted arm should fail loudly
    )
    # Both modes START under position control: the follower is driven by setpoints, the
    # leader first moves to its staged pose (LeaderMode.start) before going backdrivable.
    driver.set_all_modes(trossen_arm.Mode.position)
    return driver


def home_and_release(name: str, driver, staged: list[float], sleep: list[float]) -> None:
    """Move the arm to the staged then sleep pose (still under torque, so it folds
    rather than dropping), then release torque and close. Follower shutdown."""
    driver.set_all_positions(staged, 2.0, True)
    driver.set_all_positions(sleep, 2.0, True)
    driver.set_all_modes(trossen_arm.Mode.idle)
    driver.cleanup()
    logger.info("%s: homed to sleep, disconnected", name)


def release(name: str, driver) -> None:
    """Drop torque and close — for a backdrivable leader (no position homing)."""
    driver.set_all_modes(trossen_arm.Mode.idle)
    driver.cleanup()
    logger.info("%s: released, disconnected", name)
