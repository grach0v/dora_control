"""Mode registry for the trossen-robot node — explicit name -> Mode class."""

from trossen_robot.modes.follower import FollowerMode
from trossen_robot.modes.leader import LeaderMode

MODES = {
    "follower": FollowerMode,
    "leader": LeaderMode,
}
