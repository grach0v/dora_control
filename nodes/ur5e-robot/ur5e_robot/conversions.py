"""Pose conversion shared by the robot's modes (within-node, not cross-node).

Feedback-only: the UR5e node is joint-controlled (pinocchio owns IK), so the only
pose handling is turning the arm's MEASURED TCP pose into the repo's wire format for
the web UI / Rerun. UR reports the TCP as ``[x,y,z, rx,ry,rz]`` (a rotation vector);
the wire format is ``pose7 = [x,y,z, qx,qy,qz,qw]`` (xyzw quaternion, see
docs/message_formats.md). No pose -> command path exists anywhere.
"""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation


def tcp_to_pose7(tcp: list[float]) -> list[float]:
    """UR TCP [x,y,z, rx,ry,rz] (rotation-vector) -> [x,y,z, qx,qy,qz,qw] (xyzw)."""
    return [*tcp[:3], *Rotation.from_rotvec(tcp[3:]).as_quat()]


def pose7_to_tcp(pose: np.ndarray) -> list[float]:
    """[x,y,z, qx,qy,qz,qw] (xyzw) -> UR TCP [x,y,z, rx,ry,rz] (rotation-vector).

    Inverse of `tcp_to_pose7`; used only by the round-trip test (no control path
    converts a pose into a command — the arm is commanded in joint space)."""
    return [*pose[:3], *Rotation.from_quat(pose[3:]).as_rotvec()]
