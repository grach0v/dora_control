"""Unit tests for the retarget follow mode — FakeNode, a minimal descriptor, no dora."""

from __future__ import annotations

import math

import numpy as np
import pyarrow as pa
import pytest
from scipy.spatial.transform import Rotation

from retarget_node.modes.follow import FollowMode
from retarget_node.node_config import RetargetConfig

IDENTITY = [0.0, 0.0, 0.0, 1.0]  # qx qy qz qw

DESCRIPTOR = """
name: test_follower
parts:
  left:
    type: arm
    joints: [j0, j1, j2, j3, j4, j5]
  left_gripper:
    type: gripper
    joints: [g0]
    open: 0.0
    closed: 0.8
  right:
    type: arm
    joints: [j0, j1, j2, j3, j4, j5]
  right_gripper:
    type: gripper
    joints: [g0]
    open: 0.0
    closed: 0.8
command_layouts:
  cartesian:
    - {part: left,          space: cartesian, quantity: position, dim: 7}
    - {part: left_gripper,  space: joint,     quantity: position, dim: 1}
    - {part: right,         space: cartesian, quantity: position, dim: 7}
    - {part: right_gripper, space: joint,     quantity: position, dim: 1}
"""


class FakeNode:
    def __init__(self):
        self.outputs = []

    def send_output(self, output_id, data, metadata=None):
        self.outputs.append((output_id, data, metadata))

    def ids(self):
        return [o[0] for o in self.outputs]

    def last(self, output_id):
        for oid, data, _ in reversed(self.outputs):
            if oid == output_id:
                return data
        raise AssertionError(f"no output {output_id!r} in {self.ids()}")


def program_state(value):
    return {"id": "program_state", "type": "INPUT", "value": pa.array([value])}


def make_mode(tmp_path, **overrides):
    scene = tmp_path / "scene.yaml"
    scene.write_text(DESCRIPTOR)
    cfg = RetargetConfig(scene=str(scene), **overrides)
    node = FakeNode()
    mode = FollowMode(cfg, node)
    mode.start()
    mode.handle(program_state("teleop"))  # retargeting is gated to the teleop stage
    return mode, node


def value(input_id, values):
    return {"id": input_id, "type": "INPUT", "value": pa.array([float(v) for v in values]),
            "metadata": {"timestamp": 0.0}}


def tick():
    return {"id": "tick", "type": "INPUT", "value": pa.array([0])}


def feed_all(mode, left_leader=None, right_leader=None):
    """Seed both arms: leader poses at the origin, follower anchors apart, grippers open."""
    mode.handle(value("left_tcp_pose", left_leader or [0, 0, 0, *IDENTITY]))
    mode.handle(value("right_tcp_pose", right_leader or [0, 0, 0, *IDENTITY]))
    mode.handle(value("left_measured_pose", [0.4, 0.3, 0.5, *IDENTITY]))
    mode.handle(value("right_measured_pose", [0.4, -0.3, 0.5, *IDENTITY]))
    mode.handle(value("left_gripper_state", [0.044]))   # leader open
    mode.handle(value("right_gripper_state", [0.044]))


def command_parts(node):
    cmd = node.last("command").to_numpy()
    assert len(cmd) == 16
    return cmd[0:7], cmd[7], cmd[8:15], cmd[15]  # left pose, left grip, right pose, right grip


def test_waits_until_every_part_engaged(tmp_path):
    mode, node = make_mode(tmp_path)
    mode.handle(value("left_tcp_pose", [0, 0, 0, *IDENTITY]))
    mode.handle(value("left_measured_pose", [0.4, 0.3, 0.5, *IDENTITY]))
    mode.handle(value("left_gripper_state", [0.044]))
    mode.handle(tick())
    assert "command" not in node.ids()      # right arm not seeded -> never command blind
    assert mode.status == "waiting"  # detail (which parts) goes to the log


def test_first_command_is_the_follower_anchor(tmp_path):
    mode, node = make_mode(tmp_path)
    feed_all(mode)
    mode.handle(tick())
    left, lg, right, rg = command_parts(node)
    assert np.allclose(left, [0.4, 0.3, 0.5, *IDENTITY])    # zero delta -> anchor pose
    assert np.allclose(right, [0.4, -0.3, 0.5, *IDENTITY])
    assert lg == pytest.approx(0.0) and rg == pytest.approx(0.0)  # leader open -> follower open
    assert mode.status == "engaged"


def test_translation_scaled(tmp_path):
    mode, node = make_mode(tmp_path, scale=2.0, max_pos_step=10.0)
    feed_all(mode)
    mode.handle(tick())
    mode.handle(value("left_tcp_pose", [0.1, 0.0, 0.05, *IDENTITY]))
    mode.handle(tick())
    left, _, right, _ = command_parts(node)
    assert np.allclose(left[:3], [0.4 + 0.2, 0.3, 0.5 + 0.1])   # leader delta x2
    assert np.allclose(right[:3], [0.4, -0.3, 0.5])             # untouched arm holds anchor


def test_align_rotates_translation_deltas(tmp_path):
    mode, node = make_mode(tmp_path, align_rpy=[0.0, 0.0, math.pi / 2], max_pos_step=10.0)
    feed_all(mode)
    mode.handle(tick())                                              # engage at the origin
    mode.handle(value("left_tcp_pose", [0.1, 0.0, 0.0, *IDENTITY]))  # leader +x ...
    mode.handle(tick())
    left, _, _, _ = command_parts(node)
    assert np.allclose(left[:3], [0.4, 0.3 + 0.1, 0.5], atol=1e-9)   # ... = follower +y


def test_rotation_applied_one_to_one_and_conjugated(tmp_path):
    # scale must not touch rotation; align conjugates it: a leader roll about its x axis
    # becomes a follower roll about the ALIGNED axis (here +y after a 90-degree yaw).
    mode, node = make_mode(tmp_path, scale=3.0, align_rpy=[0.0, 0.0, math.pi / 2],
                           max_rot_step=10.0)
    feed_all(mode)
    mode.handle(tick())  # engage at the origin
    leader_rot = Rotation.from_rotvec([0.3, 0.0, 0.0])
    mode.handle(value("left_tcp_pose", [0, 0, 0, *leader_rot.as_quat()]))
    mode.handle(tick())
    left, _, _, _ = command_parts(node)
    expected = Rotation.from_rotvec([0.0, 0.3, 0.0])  # conjugation maps x-axis -> y-axis
    got = Rotation.from_quat(left[3:7])
    assert (got * expected.inv()).magnitude() == pytest.approx(0.0, abs=1e-9)
    assert np.allclose(left[:3], [0.4, 0.3, 0.5])     # pure rotation -> position holds


def test_leader_jump_is_clamped_per_tick(tmp_path):
    mode, node = make_mode(tmp_path, max_pos_step=0.01)
    feed_all(mode)
    mode.handle(tick())                                              # engage at the origin
    mode.handle(value("left_tcp_pose", [1.0, 0.0, 0.0, *IDENTITY]))  # 1 m teleport
    mode.handle(tick())
    left, _, _, _ = command_parts(node)
    assert np.linalg.norm(left[:3] - [0.4, 0.3, 0.5]) == pytest.approx(0.01)
    mode.handle(tick())
    left, _, _, _ = command_parts(node)
    assert np.linalg.norm(left[:3] - [0.4, 0.3, 0.5]) == pytest.approx(0.02)  # glides, no jump


def test_gripper_range_mapped_and_clipped(tmp_path):
    mode, node = make_mode(tmp_path)
    feed_all(mode)
    mode.handle(value("left_gripper_state", [0.0]))     # leader fully closed
    mode.handle(value("right_gripper_state", [0.022]))  # leader half open
    mode.handle(tick())
    _, lg, _, rg = command_parts(node)
    assert lg == pytest.approx(0.8)                     # follower closed (2F-85 rad)
    assert rg == pytest.approx(0.4)
    mode.handle(value("left_gripper_state", [0.1]))     # beyond open -> clip to open
    mode.handle(tick())
    _, lg, _, _ = command_parts(node)
    assert lg == pytest.approx(0.0)


def test_unknown_input_raises(tmp_path):
    mode, _ = make_mode(tmp_path)
    with pytest.raises(KeyError):
        mode.handle(value("bogus", [0.0]))


def test_inactive_stage_gates_commands_and_reanchors(tmp_path):
    mode, node = make_mode(tmp_path)
    feed_all(mode)
    mode.handle(tick())
    assert "command" in node.ids()

    # homing stage: no commands, anchors dropped
    assert mode.handle(program_state("homing")) is False
    n = len([i for i in node.ids() if i == "command"])
    mode.handle(value("left_tcp_pose", [0.2, 0.0, 0.0, *IDENTITY]))  # leader moves meanwhile
    mode.handle(tick())
    assert len([i for i in node.ids() if i == "command"]) == n
    assert mode.status == "waiting"

    # followers settle at a NEW pose during homing; teleop re-engages against it,
    # so the first command holds that pose (the leader's mid-homing motion is forgotten)
    mode.handle(value("left_measured_pose", [0.5, 0.1, 0.7, *IDENTITY]))
    mode.handle(value("right_measured_pose", [0.5, -0.1, 0.7, *IDENTITY]))
    assert mode.handle(program_state("teleop")) is False
    mode.handle(tick())
    left, _, right, _ = command_parts(node)
    assert np.allclose(left, [0.5, 0.1, 0.7, *IDENTITY])
    assert np.allclose(right, [0.5, -0.1, 0.7, *IDENTITY])
