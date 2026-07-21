"""Mode registry for the genesis node — env ``MODE`` -> mode class."""

from genesis_node.modes.sim import SimMode

MODES = {
    "sim": SimMode,
}
