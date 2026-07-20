"""Mode registry for the realsense-camera node — explicit name -> Mode class."""

from realsense_camera.modes.stream import StreamMode

MODES = {
    "stream": StreamMode,
}
