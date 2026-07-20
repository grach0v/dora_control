"""Pose conversions shared by the robot's modes (within-node, not cross-node)."""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation


def pose7_to_cartesian(pose: np.ndarray) -> list[float]:
    """[x,y,z, qx,qy,qz,qw] (xyzw) -> [x,y,z, rx,ry,rz] (angle-axis)."""
    return [*pose[:3], *Rotation.from_quat(pose[3:]).as_rotvec()]


def cartesian_to_pose7(cart: list[float]) -> list[float]:
    """[x,y,z, rx,ry,rz] (angle-axis) -> [x,y,z, qx,qy,qz,qw] (xyzw)."""
    return [*cart[:3], *Rotation.from_rotvec(cart[3:]).as_quat()]
