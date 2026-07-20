"""Mode registry for the genesis node. `twin`/`force_preview` are reserved — selecting
them raises KeyError here (loud), since only `sim` is implemented."""

from genesis_node.modes.sim import SimMode

MODES = {
    "sim": SimMode,
}
