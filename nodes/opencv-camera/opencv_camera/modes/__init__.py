"""Mode registry for the opencv-camera node — explicit name -> Mode class."""

from opencv_camera.modes.stream import StreamMode

MODES = {
    "stream": StreamMode,
}
