"""Config for the pinocchio IK + safety node — pydantic-settings from the environment.

Nothing here is robot-specific: the parts, constraints, and the command/state vector layouts
all come from the scene descriptor named by ``SCENE``. The node only needs to know which scene
and which command layout (modality) this dataflow uses.
"""

from __future__ import annotations

from typing import Literal

from pydantic_settings import BaseSettings


class PinocchioConfig(BaseSettings):
    mode: Literal["control"] = "control"
    scene: str                          # required — path to the scene descriptor
    command_layout: str                 # required — which descriptor command_layouts.<name> to consume

    # --- safety gate ---
    collision_check: bool = True        # enforce the descriptor's self_collision + plane constraints
    # Fallback max joint motion per step, used for any part whose descriptor entry omits
    # `max_step`. Per-part `max_step` is preferred (units differ — rad for arms, m for grippers).
    max_joint_step: float = 0.05

    # --- homing stage ---
    # While the manager broadcasts the `homing` program_state stage, the node ignores the
    # command bundle and ramps every part from its measured joints to the model home
    # (the MJCF `home` keyframe), reporting status `homed` on arrival (the manager advances the stage on
    # that). The ramp is deliberately SLOW: max joint motion per state message (rad),
    # ~0.01 @ 30 Hz = 0.3 rad/s.
    homing_max_step: float = 0.01
    # A part counts as home when every joint is within this (rad) of the model home.
    homing_tol: float = 0.02
    # REFUSE to home if any joint would travel further than this (rad) from its measured
    # start. A model home that doesn't match the real cell otherwise means a huge
    # blind joint-space sweep (a real UR was observed swinging its base 130° "around the
    # back"). Refusal = no motion + a loud status; pre-pose the arm or fix the home.
    homing_max_travel: float = 1.2

    # --- IK ---
    # Each command, IK is iterated to a stable goal config (converge-per-tick) and the real
    # motion is then rate-limited toward it by the part's max_step — so a held unreachable
    # target settles on a steady pose instead of wobbling.
    ik_damping: float = 1e-3        # base damping λ₀ (accuracy near a reachable target)
    ik_error_damping: float = 2.0   # error-damped LS gain: λ² += (this·‖e‖)² (smooth when far/singular)
    ik_max_iters: int = 20          # max CLIK iterations per command
    ik_tol: float = 1e-3            # converged when the SE(3) error norm drops below this


def load_config() -> PinocchioConfig:
    return PinocchioConfig()
