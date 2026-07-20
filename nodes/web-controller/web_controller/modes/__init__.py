"""Mode registry for the web-controller node — explicit name -> Mode class."""

from web_controller.modes.episode import EpisodeMode
from web_controller.modes.manual import ManualMode

MODES = {
    "manual": ManualMode,
    "episode": EpisodeMode,
}
