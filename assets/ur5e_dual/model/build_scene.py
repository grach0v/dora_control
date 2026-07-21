"""One-off asset generator for the UR5e dual-arm workstation MJCF.

Run manually (not a node, not imported by any node — CLAUDE.md): it composes the two
vendored menagerie models (`ur5e/ur5e.xml` + `robotiq_2f85/2f85.xml`) into a single,
flat, self-contained `ur5e_dual.xml` (bench + 2 arms + grippers + cameras) using MuJoCo's
MjSpec attach API. That one file is the UNIVERSAL model — loaded by mujoco-sim, genesis,
the rerun 3D view, AND pinocchio. No floor: MuJoCo has a default headlight and genesis
adds its own ground. The committed `ur5e_dual.xml` is the real artifact; re-run this
only when the layout/robot changes.

    cd assets/ur5e_dual/model
    ../../../nodes/mujoco-sim/.venv/bin/python build_scene.py

Why a generator and not hand-authored XML: the 2F-85 is a 8-joint underactuated
linkage; attaching two arms + two grippers by hand is huge and error-prone. MjSpec
attach inlines everything into one flat tree (which Pinocchio's MJCF parser needs).

Workstation geometry (from the real cell):
  * two arms face the SAME way (-x), side by side along y; the table is on -x
  * base-to-base distance = 0.70 m
  * table surface is 0.105 m BELOW each arm base (arms sit on risers)
"""

from __future__ import annotations

import os
import re

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

HERE = os.path.dirname(os.path.abspath(__file__))

# --- workstation geometry (world frame; floor at z=0) -------------------------------
TABLE_SURFACE_Z = 0.75          # table top height off the floor (visual; not constrained by the cell)
BASE_RISER = 0.105              # arm base sits this far ABOVE the table surface (given)
BASE_Z = TABLE_SURFACE_Z + BASE_RISER
BASE_SEP = 0.70                 # base-to-base distance (given)
BASE_X = 0.0
SIDES = {"left": -BASE_SEP / 2.0, "right": +BASE_SEP / 2.0}  # y offset per side (left on -y)

# Cell home pose, emitted as the MJCF `home` keyframe (the single source every consumer
# reads: pinocchio referenceConfigurations, mujoco-sim/genesis key_qpos). Arm joints
# captured from the REAL cell 2026-07-02 (freedrive ready pose: over bench, gripper
# down, elbow-down branch); grippers open (driver joint 0 rad).
HOME = {
    "left":  [0.2079, -2.2915, -0.7666, -1.5145, 1.9397, 0.0121],
    "right": [0.2184, -2.3139, -0.7674, -1.5801, 1.5861, -0.0005],
}
ARM_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
              "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
# Base mount (frame quat, wxyz) for BOTH arms (same -> they stay parallel). This is the REAL
# robot's base frame: a measured joint vector renders matching the physical arm. Do NOT flip it
# to change which way the cell "faces" — that would mis-render the real arm. The default/home
# reach direction is set by the home pose in the scene descriptor instead.
MOUNT_QUAT = [0.0, 0.0, 0.0, 1.0]

GRIP_KP, GRIP_KV = 20.0, 1.0    # gripper driver-joint position-servo gains
DRIVER_RANGE = (0.0, 0.8)       # 2F-85 right_driver_joint range: 0 = open, 0.8 rad = closed

# Arm position-servo gains, STIFFENED from the menagerie defaults (size3 kp2000/kv400,
# size1 kp500/kv100 — both a soft kp/kv=5). Why: pinocchio rate-limits its joint target to
# `max_step` (0.2 rad) ahead of the MEASURED joint, so a soft servo makes the sim track that
# setpoint ~3x slower than the real UR's servoJ (gain 300) does — the sim arm crawled at
# ~1 rad/s while the real arm moves at ~3. These gains restore faithful tracking WITHOUT
# touching max_step (which stays the robot-agnostic velocity limit): kv is kept ~proportional
# to sqrt(kp) so the damping ratio — and zero overshoot — is preserved (measured), and
# forcerange is left at the real UR torque limits (150/28 Nm), so the heavy joints stay
# torque-limited exactly like the hardware. Terminal slew at the 0.2 rad cap: heavy ~2 rad/s,
# wrists ~3 rad/s (≈ a real UR5e joint's ~π rad/s). Re-tune with model/build_scene.py measure
# scripts if the timestep or robot changes.
ARM_KP_KV = (8000.0, 800.0)     # size3 (shoulder_pan/lift, elbow): kp 4x, kv 2x
WRIST_KP_KV = (4500.0, 300.0)   # size1 (wrist_1/2/3): kp 9x, kv 3x (light joints track cleaner)


def add_ee_sites(spec) -> None:
    """Add an `<side>_ee_site` on each wrist_3_link at the gripper's pinch pose.

    The 2F-85's own `pinch` site sits on a chain of ROTATED fixed bodies, and Pinocchio's
    MJCF parser mis-places such sites (it drops the intermediate fixed-body rotations) — so
    its IK ee disagreed with MuJoCo's by a constant offset, and the arm reached the wrong
    pose. A site on wrist_3_link (an actual joint frame) is parsed identically by Pinocchio
    and MuJoCo. We measure the pinch pose relative to wrist_3 from a throwaway compile (it's
    rigid, so config-independent) and place the ee site there; the descriptor points ee_frame
    at it instead of `pinch`."""
    probe = spec.compile()
    data = mujoco.MjData(probe)
    mujoco.mj_kinematics(probe, data)
    for side in SIDES:
        wb = mujoco.mj_name2id(probe, mujoco.mjtObj.mjOBJ_BODY, f"{side}_wrist_3_link")
        ps = mujoco.mj_name2id(probe, mujoco.mjtObj.mjOBJ_SITE, f"{side}_g_pinch")
        wR, wp = data.xmat[wb].reshape(3, 3), data.xpos[wb]
        pR, pp = data.site_xmat[ps].reshape(3, 3), data.site_xpos[ps]
        quat_xyzw = Rotation.from_matrix(wR.T @ pR).as_quat()
        site = spec.body(f"{side}_wrist_3_link").add_site()
        site.name = f"{side}_ee_site"
        site.pos = (wR.T @ (pp - wp)).tolist()
        site.quat = [quat_xyzw[3], *quat_xyzw[:3].tolist()]  # MuJoCo wants wxyz


def stage_meshes() -> None:
    """Copy the vendored arm + gripper meshes flat into `assets/` (the single meshdir)."""
    import shutil

    dst = os.path.join(HERE, "assets")
    os.makedirs(dst, exist_ok=True)
    for src in ("ur5e/assets", "robotiq_2f85/assets"):
        srcdir = os.path.join(HERE, src)
        for fn in os.listdir(srcdir):
            shutil.copy2(os.path.join(srcdir, fn), os.path.join(dst, fn))


def _normalize(v):
    v = np.asarray(v, dtype=float)
    return v / np.linalg.norm(v)


def _axes_to_quat(r, tu):
    """Camera quat (MuJoCo wxyz) from its +x (right) and +y (up) axes. Camera looks
    along -z, so the frame columns are [right, up, right x up]."""
    cz = np.cross(r, tu)
    rot = np.column_stack([r, tu, cz])
    qx, qy, qz, qw = Rotation.from_matrix(rot).as_quat()
    return [qw, qx, qy, qz]


def look_quat(eye, target, up=(0.0, 0.0, 1.0)):
    """MuJoCo camera quat so the camera at `eye` looks at `target`."""
    f = _normalize(np.asarray(target, float) - np.asarray(eye, float))  # forward (view dir)
    r = _normalize(np.cross(f, up))                                     # camera +x (right)
    tu = np.cross(r, f)                                                 # camera +y (up)
    return _axes_to_quat(r, tu)


def main() -> None:
    os.chdir(HERE)

    spec = mujoco.MjSpec()
    spec.modelname = "ur5e_dual"
    spec.compiler.degree = False          # radians
    spec.compiler.autolimits = True
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    wb = spec.worldbody

    # --- table + risers (visual + collision; no floor — not needed: MuJoCo default
    #     headlight lights the scene, genesis adds its own ground, pinocchio ignores it) ----
    table = wb.add_body(name="table")
    table.add_geom(
        name="tabletop", type=mujoco.mjtGeom.mjGEOM_BOX,
        pos=[-0.25, 0.0, TABLE_SURFACE_Z - 0.02], size=[0.5, 0.6, 0.02],
        rgba=[0.65, 0.5, 0.35, 1.0],
    )
    for side, y in SIDES.items():
        table.add_geom(
            name=f"{side}_riser", type=mujoco.mjtGeom.mjGEOM_BOX,
            pos=[BASE_X, y, TABLE_SURFACE_Z + BASE_RISER / 2.0],
            size=[0.05, 0.05, BASE_RISER / 2.0], rgba=[0.2, 0.2, 0.2, 1.0],
        )

    # --- attach the two arms + grippers ---------------------------------------------
    # Both arms nest INSIDE the `table` body so the model has a single root body:
    # Pinocchio's MJCF parser only builds the first worldbody child's subtree (this is
    # how the Trossen model nests both arms under `tabletop_link`).
    for side, y in SIDES.items():
        fr = table.add_frame()
        fr.pos = [BASE_X, y, BASE_Z]
        fr.quat = MOUNT_QUAT
        arm = mujoco.MjSpec.from_file("ur5e/ur5e.xml")
        spec.attach(arm, prefix=f"{side}_", frame=fr)

        # gripper attaches at the arm flange site. Distinct prefix: both ur5e and 2f85
        # have a body named "base", so a shared prefix would collide.
        site = spec.site(f"{side}_attachment_site")
        grip = mujoco.MjSpec.from_file("robotiq_2f85/2f85.xml")
        spec.attach(grip, prefix=f"{side}_g_", site=site)

        # Repurpose the 2F-85's stock tendon actuator (ctrl 0..255) into a plain joint
        # position servo on the driver joint, in RADIANS — so gripper command / state /
        # ctrl share one unit (the node contract), exactly like Trossen's slide gripper.
        act = spec.actuator(f"{side}_g_fingers_actuator")
        act.name = f"{side}_gripper"
        act.trntype = mujoco.mjtTrn.mjTRN_JOINT
        act.target = f"{side}_g_right_driver_joint"
        act.gaintype = mujoco.mjtGain.mjGAIN_FIXED
        act.biastype = mujoco.mjtBias.mjBIAS_AFFINE
        act.gainprm = [GRIP_KP] + [0.0] * 9
        act.biasprm = [0.0, -GRIP_KP, -GRIP_KV] + [0.0] * 7
        act.gear = [1.0, 0, 0, 0, 0, 0]
        act.ctrlrange = list(DRIVER_RANGE)
        act.ctrllimited = mujoco.mjtLimited.mjLIMITED_TRUE
        act.forcerange = [-50.0, 50.0]
        act.forcelimited = mujoco.mjtLimited.mjLIMITED_TRUE

    # --- gravity compensation -------------------------------------------------------
    # The menagerie UR5e/2F-85 have no gravcomp, so the position-servo'd arms sag under
    # gravity to a torque-balanced offset on startup (looks like slow falling, then
    # stiff). Compensate gravity on every robot body so the arms hold their commanded
    # pose exactly — matching how the Trossen model sets gravcomp on its arm bodies.
    for body in spec.bodies:
        if body.name.startswith(("left_", "right_")):
            body.gravcomp = 1.0

    # --- arm actuator names == joint names + stiffer tracking -----------------------
    # MjSpec disambiguated the joints to "<j>_joint" but left the position actuators as
    # "<j>"; mujoco-sim looks up an arm part's actuator BY its joint name, so rename each
    # arm actuator to its target joint. (The gripper is actuator-style — named explicitly
    # and referenced via the descriptor's `actuator:` field — so skip it.) At the same time
    # overwrite the menagerie position-servo gains with the stiffened ARM/WRIST values (see
    # the constants above) so the sim tracks the streamed setpoint about as tightly as the
    # real UR — same affine servo form as the gripper, forcerange/ctrlrange left untouched.
    for a in spec.actuators:
        if "driver" in a.target:
            continue  # gripper: named + tuned above
        a.name = a.target
        kp, kv = WRIST_KP_KV if "wrist" in a.target else ARM_KP_KV
        a.gaintype = mujoco.mjtGain.mjGAIN_FIXED
        a.biastype = mujoco.mjtBias.mjBIAS_AFFINE
        a.gainprm = [kp] + [0.0] * 9
        a.biasprm = [0.0, -kp, -kv] + [0.0] * 7

    # --- gripper collision: primitives only -----------------------------------------
    # The 2F-85 ships mesh collision geometry; Pinocchio's collision engine (coal)
    # SEGFAULTS on mesh-vs-mesh distance, which the safety node runs between the two
    # arms. Demote every collision *mesh* to non-colliding (group 2, contype/conaffinity
    # 0) — the 2F-85's box finger pads remain for grasp contact, so the whole model is
    # primitive-only like the Trossen cell. Arm collision geoms are capsules/cylinders,
    # so this touches only the grippers.
    for geom in spec.geoms:
        if geom.type == mujoco.mjtGeom.mjGEOM_MESH and geom.group == 3:
            geom.group = 2
            geom.contype = 0
            geom.conaffinity = 0

    # No cameras: this cell doesn't use rendered cameras (removed per the real bring-up).

    # --- self-contained mesh paths --------------------------------------------------
    # MuJoCo resolves meshes under a single `meshdir`; the vendored arm/gripper meshes
    # are staged flat into `assets/` by stage_meshes() (basenames don't collide: ur5e
    # is .obj, 2f85 is .stl), so every mesh.file is just its basename.
    stage_meshes()
    for m in spec.meshes:
        m.file = os.path.basename(m.file)
    spec.meshdir = "assets"
    spec.compiler.meshdir = "assets"

    add_ee_sites(spec)  # ee frame both Pinocchio and MuJoCo agree on (see the function)

    model = spec.compile()  # validate before writing
    print(f"compiled ok: nq={model.nq} nu={model.nu} nbody={model.nbody} ncam={model.ncam}")

    # Drop the per-arm "home" keyframes the attach carried over (their qpos is sized for
    # a single arm's tree, not the combined model) and emit ONE whole-cell `home` keyframe
    # instead — qpos per joint by name, ctrl per position actuator = its joint's home.
    xml = re.sub(r"\n\s*<keyframe>.*?</keyframe>", "", spec.to_xml(), flags=re.DOTALL)
    m = mujoco.MjModel.from_xml_string(xml)
    qpos, ctrl = [0.0] * m.nq, [0.0] * m.nu
    for side, home in HOME.items():
        for jn, v in zip(ARM_JOINTS, home):
            j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"{side}_{jn}")
            qpos[m.jnt_qposadr[j]] = v
    for a in range(m.nu):
        j = m.actuator_trnid[a][0]
        ctrl[a] = qpos[m.jnt_qposadr[j]]
    def fmt(v):
        return " ".join(f"{x:g}" for x in v)

    key = f'\n  <keyframe>\n    <key name="home" qpos="{fmt(qpos)}" ctrl="{fmt(ctrl)}"/>\n  </keyframe>'
    xml = xml.replace("\n</mujoco>", key + "\n</mujoco>")
    with open("ur5e_dual.xml", "w") as f:
        f.write(xml)
    print("wrote ur5e_dual.xml (with home keyframe)")


if __name__ == "__main__":
    main()
