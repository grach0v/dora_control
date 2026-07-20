"""Mode registry for the ur5e-robot node — explicit name -> Mode class.

Only `follower` (joint control). The UR5e has no backdrivable leader analog in this
stack, so there is no leader mode (unlike trossen-robot)."""

from ur5e_robot.modes.follower import FollowerMode

MODES = {
    "follower": FollowerMode,
}
