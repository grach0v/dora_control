"""Pose conversions shared by the robot's modes (within-node, not cross-node)."""

from __future__ import annotations

from scipy.spatial.transform import Rotation


def cartesian_to_pose7(cart: list[float]) -> list[float]:
    """[x,y,z, rx,ry,rz] (angle-axis) -> [x,y,z, qx,qy,qz,qw] (xyzw)."""
    return [*cart[:3], *Rotation.from_rotvec(cart[3:]).as_quat()]
