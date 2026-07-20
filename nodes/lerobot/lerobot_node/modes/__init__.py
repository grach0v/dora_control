"""Mode registry for the lerobot node — explicit name -> Mode class."""

from lerobot_node.modes.record import RecordMode

MODES = {
    "record": RecordMode,
}
