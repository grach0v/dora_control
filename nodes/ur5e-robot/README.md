# ur5e-robot

Real-robot node for one **UR5e + Robotiq 2F-85** arm of the bimanual UR5e
workstation. Drives the arm over RTDE (`ur_rtde`) and the gripper over the Robotiq
URCap socket, mirroring the sim nodes' per-arm stream interface so sim and real are
swappable. One node = one arm; the bimanual cell runs two (`NAME=left`, `NAME=right`),
composed in `dataflows/ur5e_dual_real.yml` and the other `dataflows/ur5e_dual_*.yml` flows.

**Joint control only.** pinocchio owns IK + collision safety and emits per-arm
`<name>_joint_target`; this node streams them via `servoJ`. There is no Cartesian /
`servoL` / `tcp_target` path — the measured TCP pose is published only as feedback for
the web UI / Rerun.

| | |
|---|---|
| **Inputs** | `<name>_joint_target` (6, rad), `<name>_gripper_joint_target` (1, rad), `robot_command` (`disconnect`), `tick`, `program_state` |
| **Outputs** | `<name>_tcp_pose` (7, xyzw), `<name>_joint_state` (6), `<name>_gripper_state` (1), `<name>_node_state` |
| **Config** (env) | `NAME`, `IP`, `MODE=follower`, `FOLLOWER__WITH_GRIPPER` (default true), `FOLLOWER__MAX_JOINT_JUMP`, `FOLLOWER__SERVO_*`, `FOLLOWER__GRIPPER_*` |

## Gripper

The Robotiq 2F-85 is driven over the **UR controller's URCap socket** (port 63352 on the
arm IP) — so it's controlled inside this node (one node = one arm + its gripper). The
gripper opening is the 2F-85 driver-joint angle in radians (0 open .. 0.8 closed), which
the driver maps to the Robotiq 0..255 range.

**If the gripper is wired directly to the PC** (not the UR controller), set
`FOLLOWER__WITH_GRIPPER=false`: the arm connects + runs normally, gripper commands are
ignored, and `gripper_state` is reported as a constant (open). **TODO:** a separate
`robotiq-gripper` node talking to the gripper over the PC's serial/USB (Modbus RTU) — that
matches one-node-one-device when the gripper isn't on the UR controller.

## Hardware note (macOS)

`ur_rtde` needs CMake + Boost and has **no macOS wheel**, so it's a platform-gated
dependency (`sys_platform != 'darwin'`) and the SDK import is isolated to `driver.py`.
The project still syncs on this Mac and the unit tests run (they drive a `FollowerMode`
with a fake driver via the SDK-free `loop.run`). The node itself runs on the Linux
robot host. Gripper opening is expressed as the 2F-85 driver-joint angle (rad,
0 = open .. 0.8 = closed), which `driver.py` maps to the Robotiq 0..255 range.

```sh
cd nodes/ur5e-robot && uv run pytest -q
```
