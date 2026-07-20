# retarget

Cross-robot teleoperation retargeting: maps a **leader** robot's hand-guided TCP
motion (e.g. two backdrivable Trossen WXAI arms) onto a **follower** robot of a
different size and mounting (e.g. a bimanual UR5e cell), as the follower scene's
cartesian `command` bundle for the follower-side `pinocchio` node.

The mapping is **delta-based with an engage anchor** (see
`retarget_node/modes/follow.py` for the math): leader displacements — scaled in
translation, 1:1 in rotation, rotated by a configurable leader→follower frame
alignment — are applied on top of the follower's measured pose at engage time.
The emitted message stays an ordinary absolute `command` bundle, so a dropped or
duplicated packet never accumulates error. Gripper openings map linearly from the
leader's range onto the follower descriptor's `open`/`closed` range.

Configuration (env): `SCENE` (the FOLLOWER's scene descriptor) + `COMMAND_LAYOUT`,
`SCALE`, `ALIGN_RPY`, `LEADER_GRIPPER_OPEN`/`_CLOSED`, `MAX_POS_STEP`/`MAX_ROT_STEP`
— see `retarget_node/node_config.py`.

Inputs: `<arm>_tcp_pose` + `<arm>_gripper_state` from the leader nodes,
`<arm>_measured_pose` from the follower-side pinocchio (the engage anchor),
`tick`, `program_state`. Outputs: `command`, `node_state`.

Tests: `uv run pytest -q`.
