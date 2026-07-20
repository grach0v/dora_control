"""Unit tests for the pinocchio control mode — FakeNode, real model, no dora.

The node consumes a whole-robot `command` bundle + a `state` bundle (descriptor-defined
layouts) and emits per-part `<part>_joint_target`. Tests synthesize those bundles.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pinocchio as pin
import pyarrow as pa
import pytest
from scipy.spatial.transform import Rotation

from pinocchio_node.modes.control import ControlMode
from pinocchio_node.node_config import PinocchioConfig

SCENE = str(Path(__file__).resolve().parents[3] / "assets/trossen_stationary/scenes/workstation.yaml")


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
                return data.to_numpy()
        raise AssertionError(f"no output {output_id!r} in {self.ids()}")


def program_state(value):
    return {"id": "program_state", "type": "INPUT", "value": pa.array([value])}


def make_mode(command_layout="cartesian", **overrides):
    cfg = PinocchioConfig(scene=SCENE, command_layout=command_layout, **overrides)
    node = FakeNode()
    mode = ControlMode(cfg, node)
    mode.start()
    mode.handle(program_state("teleop"))  # commands are gated to the teleop stage
    return mode, node


def event(input_id, values):
    return {"id": input_id, "type": "INPUT", "value": pa.array([float(v) for v in values])}


def ee_pose7(mode, part, offset=(0.0, 0.0, 0.0)):
    m = mode.model
    pin.framesForwardKinematics(m.model, m.data, m.q_home)
    oMf = m.data.oMf[m.parts[part].ee_frame_id]
    pos = oMf.translation + np.asarray(offset)
    return [*pos, *Rotation.from_matrix(oMf.rotation).as_quat()]


def home_state_vec(mode):
    """State bundle = home joints per the state_layout."""
    return np.concatenate([mode.model.part_q(mode.model.q_home, e["part"]) for e in mode.state_layout])


def cartesian_command_vec(mode, left_offset=(0.0, 0.0, 0.0)):
    return [
        *ee_pose7(mode, "left", left_offset), 0.044,
        *ee_pose7(mode, "right"), 0.044,
    ]


def test_no_output_before_state():
    mode, node = make_mode()
    mode.handle(event("command", cartesian_command_vec(mode, left_offset=(0.05, 0, 0))))
    assert "left_joint_target" not in node.ids()


def test_command_emits_all_parts_together():
    mode, node = make_mode()
    mode.handle(event("state", home_state_vec(mode)))
    mode.handle(event("command", cartesian_command_vec(mode, left_offset=(0.05, 0.03, 0.0))))
    # synchronized: every part's target is emitted, even the unmoved ones
    for oid, dim in [("left_joint_target", 6), ("left_gripper_joint_target", 1),
                     ("right_joint_target", 6), ("right_gripper_joint_target", 1)]:
        assert len(node.last(oid)) == dim
    assert "HELD" not in mode.status
    # the left arm actually stepped toward the offset target
    assert not np.allclose(node.last("left_joint_target"), mode.model.part_q(mode.model.q_home, "left"))


def test_gripper_steps_toward_command():
    mode, node = make_mode()
    mode.handle(event("state", home_state_vec(mode)))
    cmd = cartesian_command_vec(mode, left_offset=(0.05, 0, 0))
    cmd[7] = 0.02  # left_gripper slot (after left pose7)
    mode.handle(event("command", cmd))
    assert np.allclose(node.last("left_gripper_joint_target"), [0.02], atol=1e-6)


def test_unknown_input_raises():
    mode, _ = make_mode()
    with pytest.raises(KeyError):
        mode.handle(event("bogus", [0.0]))


def test_hold_re_emits_last_safe_on_collision():
    mode, node = make_mode()
    mode.model.collision_margin = 5.0  # force every config to read as "in collision"
    mode.handle(event("state", home_state_vec(mode)))
    mode.handle(event("command", cartesian_command_vec(mode, left_offset=(0.05, 0, 0))))
    assert "HELD" in mode.status
    assert np.allclose(node.last("left_joint_target"), mode.model.part_q(mode.model.q_home, "left"))


def test_joint_layout_clamps_to_limits_and_step():
    mode, node = make_mode(command_layout="joint", max_joint_step=0.05)
    mode.handle(event("state", home_state_vec(mode)))
    p = mode.model.parts["left"]
    cmd = [*(p.upper + 5.0), 0.044, *(p.upper + 5.0), 0.044]  # left6, lg1, right6, rg1
    mode.handle(event("command", cmd))
    out = node.last("left_joint_target")
    home = mode.model.part_q(mode.model.q_home, "left")
    assert np.all(out <= p.upper + 1e-9) and np.all(out >= p.lower - 1e-9)
    # the descriptor's per-part max_step takes precedence over the config fallback
    max_step = p.max_step or 0.05
    assert np.all(np.abs(out - home) <= max_step + 1e-9)


def offset_state_vec(mode, joint_offset=0.3):
    """State bundle = home joints with every ARM joint displaced by joint_offset."""
    segs = []
    for e in mode.state_layout:
        q = mode.model.part_q(mode.model.q_home, e["part"]).copy()
        if e["part"] in ("left", "right"):
            q = q + joint_offset
        segs.append(q)
    return np.concatenate(segs)


def test_homing_stage_ramps_to_home_and_reports_homed():
    mode, node = make_mode(homing_max_step=0.01, homing_tol=0.02)
    assert mode.handle(program_state("homing")) is False
    mode.handle(event("state", offset_state_vec(mode, 0.3)))

    # one slow step toward home, not a jump: displaced arm joints move by exactly the cap
    target = node.last("left_joint_target")
    away = mode.model.part_q(mode.model.q_home, "left") + 0.3
    assert np.allclose(target, away - 0.01, atol=1e-9)
    assert mode.status == "homing"

    # commands are ignored while homing (no solution pose = the command path never ran)
    before = len(node.outputs)
    mode.handle(event("command", cartesian_command_vec(mode, (0.05, 0.0, 0.0))))
    assert len(node.outputs) == before

    # once the measured state IS home, the status flips to homed (manager advances on it)
    mode.handle(event("state", home_state_vec(mode)))
    assert mode.status == "homed"

    # entering teleop resumes normal command-driven control
    assert mode.handle(program_state("teleop")) is False
    mode.handle(event("command", cartesian_command_vec(mode)))
    assert "left_solution_pose" in node.ids()


def test_homing_refuses_when_home_is_too_far():
    mode, node = make_mode(homing_max_travel=1.2)
    mode.handle(program_state("homing"))
    mode.handle(event("state", offset_state_vec(mode, 2.0)))  # 2 rad from home > 1.2 cap

    assert "left_joint_target" not in node.ids()   # refuses = zero motion
    assert "refused" in mode.status
    assert "homed" not in mode.status              # must never trip the manager's advance rule

    # a close pose (well under the cap) homes normally
    mode.handle(event("state", offset_state_vec(mode, 0.3)))
    assert "left_joint_target" in node.ids()
