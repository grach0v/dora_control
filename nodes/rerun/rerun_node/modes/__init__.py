"""Mode registry for the rerun node — explicit name -> Mode class mapping."""

from rerun_node.modes.record import RecordMode
from rerun_node.modes.visualize import VisualizeMode

MODES = {
    "record": RecordMode,
    "visualize": VisualizeMode,
}
