#!/usr/bin/env python3
# Copyright 2026 Masaaki Hijikata
# SPDX-License-Identifier: Apache-2.0
"""Generate the parallel-link (passive ankle pitch) variant of Microban's MJCF.

Input is Rhoban/microban's ``src/model/mjcf/robot.xml`` (or an already converted
file). Output is a MuJoCo model in which, per leg, two parallelogram linkages keep
the foot parallel to the pelvis in the pitch direction, so the ankle-pitch servo is
removed, and the head is fixed to the trunk (19 -> 16 actuators; ``--neck-motor``
keeps the neck servo). The mechanism is the one used by the ``ashigaru``
robot: knee block parallel to the pelvis, rear links closing the loops with
``<connect>`` equality constraints. The knee actuator angle is therefore measured
against the pelvis, not against the thigh.

usage:
  python3 tools/make_microban_pl.py <robot.xml> <out.xml> [--naming upstream|lme]
                                    [--keep-motor] [--back 0.020] [--tilt 0.005]
                                    [--scene out_scene.xml]
"""
import argparse, os, re, sys
import numpy as np
import xml.etree.ElementTree as ET

# ---------------------------------------------------------------- naming
UP2LME = {"head": "c_neck_zy"}
for side, s in (("right", "r"), ("left", "l")):
    UP2LME.update({
        f"{side}_shoulder_pitch": f"{s}_shoulder_yp", f"{side}_shoulder_roll": f"{s}_shoulder_xr",
        f"{side}_elbow": f"{s}_elbow_yp",             f"{side}_hip_yaw": f"{s}_hipjoint_zy",
        f"{side}_hip_roll": f"{s}_hipjoint_xr",       f"{side}_hip_pitch": f"{s}_hipjoint_yp",
        f"{side}_knee": f"{s}_knee_yp",               f"{side}_ankle_pitch": f"{s}_ankle_yp",
        f"{side}_ankle_roll": f"{s}_ankle_xr",
        # passive joints added by this script
        f"{side}_knee_block": f"{s}_knee_yp_1", f"{side}_thigh_rear": f"{s}_hipjoint_yp_1",
        f"{side}_shank_rear": f"{s}_knee_yp_2",
    })
LME2UP = {v: k for k, v in UP2LME.items()}

# ---------------------------------------------------------------- geometry
# Rear pivots sit BACK (toward -x of the robot) and slightly tilted so that the
# parallelogram singularity is pushed away from the postures that matter:
#   thigh linkage: rear pivot 5 mm DOWN  -> singular at hip ext +76deg, hip flex -104deg
#   shank linkage: rear pivot 5 mm UP    -> singular at knee flex +104deg, knee ext -76deg
# (positive hip/knee pitch = link swings backward, as in the original model)
LEG = {  # local frames: right femur +x = world back, +y = world up; left femur mirrored
    "right": dict(hipblock="hip_block__configuration_right", femur="femur__configuration_right",
                  tibia="tibia__configuration_right", ankle="ankle_block__configuration_left", sx=+1),
    "left":  dict(hipblock="hip_block__configuration_left",  femur="femur__configuration_left",
                  tibia="tibia__configuration_left",  ankle="ankle_block__configuration_right", sx=-1),
}
HIP_RANGE  = (-1.68, 1.19)   # thigh vs pelvis: -96deg (flex) .. +68deg (ext)
KNEE_RANGE = (-1.19, 1.68)   # shank vs pelvis: -68deg (ext) .. +96deg (flex)
XL330_TORQUE = 0.52          # N*m, XL330-M288-T stall torque at 5 V

def q2m(q):
    w, x, y, z = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
def vec(s): return np.array([float(t) for t in s.split()])
def fmt(a): return " ".join(f"{float(t):.6g}" for t in np.atleast_1d(a))

def header_comments(path):
    """Top-level comments before <mujoco> (ElementTree drops them)."""
    txt = open(path, encoding="utf-8").read()
    i = txt.find("<mujoco")
    return txt[:i].strip() + "\n" if i > 0 else ""

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("src"); ap.add_argument("out")
    ap.add_argument("--naming", choices=["upstream", "lme"], default="upstream",
                    help="joint/actuator names: Microban originals or LegacyMotionEditor convention")
    ap.add_argument("--keep-motor", action="store_true", help="keep <motor> actuators instead of <position>")
    ap.add_argument("--neck-motor", action="store_true",
                    help="keep the neck (head yaw) servo; by default the head is fixed to the trunk")
    ap.add_argument("--back", type=float, default=0.020, help="rear pivot offset behind the front pivot [m]")
    ap.add_argument("--tilt", type=float, default=0.005, help="vertical tilt of the rear pivots [m]")
    ap.add_argument("--scene", help="also write a viewer scene (floor + standing keyframe) to this path")
    a = ap.parse_args()

    head = header_comments(a.src)
    tree = ET.parse(a.src); root = tree.getroot()
    bodies = {b.get("name"): b for b in root.iter("body")}

    # detect source naming and build the rename table toward the requested naming
    src_is_lme = any(j.get("name") == "r_hipjoint_yp" for j in root.iter("joint"))
    if a.naming == "lme":
        rename = {} if src_is_lme else UP2LME
        name = lambda up: UP2LME[up]
    else:
        rename = LME2UP if src_is_lme else {}
        name = lambda up: up
    for el in root.iter():
        if el.tag in ("joint", "motor", "position", "general"):
            for attr in ("name", "joint"):
                if el.get(attr) in rename:
                    el.set(attr, rename[el.get(attr)])

    model_name = "microban_pl" + ("_lme" if a.naming == "lme" else "")
    root.set("model", model_name)

    dflt = root.find("default")
    pcls = ET.SubElement(dflt, "default", {"class": "passive"})
    ET.SubElement(pcls, "joint", {"damping": "0.005", "frictionloss": "0.001", "armature": "0.0001"})

    eq = root.find("equality")
    if eq is None:
        eq = ET.SubElement(root, "equality")

    def inertial(el, mass, pos, diag):
        ET.SubElement(el, "inertial", {"pos": fmt(pos), "mass": f"{mass}", "diaginertia": fmt(diag)})
    def rod(el, v):
        ET.SubElement(el, "geom", {"class": "visual", "type": "capsule", "size": "0.002",
                                   "fromto": "0 0 0 " + fmt(v), "rgba": "0.25 0.25 0.28 1"})

    for side, L in LEG.items():
        hipblock, femur, tibia, ankle = (bodies[L[k]] for k in ("hipblock", "femur", "tibia", "ankle"))
        d1 = np.array([L["sx"] * a.back, -L["sx"] * a.tilt, 0.0])   # thigh rear pivot
        d2 = np.array([L["sx"] * a.back, +L["sx"] * a.tilt, 0.0])   # shank rear pivot
        v1 = vec(tibia.get("pos"))          # hip pitch axis -> knee axis (femur frame)
        v2 = vec(ankle.get("pos"))          # knee axis -> ankle axis (tibia frame)
        tq = tibia.get("quat", "1 0 0 0").replace("-0", "0").split()
        assert tq == ["1", "0", "0", "0"], f"{L['tibia']}: quat must be identity"

        # knee block: passive hinge, stays parallel to the pelvis
        femur.remove(tibia)
        kb = ET.Element("body", {"name": f"{side}_knee_block", "pos": fmt(v1)})
        ET.SubElement(kb, "joint", {"name": name(f"{side}_knee_block"), "type": "hinge",
                                    "axis": "0 0 1", "class": "passive"})
        inertial(kb, 0.003, (0, 0, 0), (1e-7, 1e-7, 1e-7))
        ET.SubElement(kb, "geom", {"class": "visual", "type": "box", "size": "0.004 0.004 0.004",
                                   "pos": fmt(d1), "rgba": "0.25 0.25 0.28 1"})
        tibia.set("pos", "0 0 0"); tibia.set("quat", "1 0 0 0")
        kb.append(tibia)
        sr = ET.SubElement(kb, "body", {"name": f"{side}_shank_rear", "pos": fmt(d2)})
        ET.SubElement(sr, "joint", {"name": name(f"{side}_shank_rear"), "type": "hinge",
                                    "axis": "0 0 1", "class": "passive"})
        inertial(sr, 0.002, v2 / 2, (6.4e-7, 6.4e-7, 2e-8)); rod(sr, v2)
        femur.append(kb)

        fpos, fq = vec(femur.get("pos")), vec(femur.get("quat"))
        tr = ET.SubElement(hipblock, "body", {"name": f"{side}_thigh_rear",
                                              "pos": fmt(fpos + q2m(fq) @ d1), "quat": femur.get("quat")})
        ET.SubElement(tr, "joint", {"name": name(f"{side}_thigh_rear"), "type": "hinge",
                                    "axis": "0 0 1", "class": "passive"})
        inertial(tr, 0.002, v1 / 2, (6.4e-7, 6.4e-7, 2e-8)); rod(tr, v1)

        for j in root.iter("joint"):
            n = j.get("name")
            if n == name(f"{side}_ankle_pitch"):
                j.set("class", "passive"); j.attrib.pop("range", None)
            elif n == name(f"{side}_hip_pitch"):
                j.set("range", fmt(HIP_RANGE))
            elif n == name(f"{side}_knee"):
                j.set("range", fmt(KNEE_RANGE))

        ET.SubElement(eq, "connect", {"name": f"{side}_thigh_loop", "body1": f"{side}_knee_block",
                                      "body2": f"{side}_thigh_rear", "anchor": fmt(d1),
                                      "solref": "0.005 1", "solimp": "0.99 0.999 0.001"})
        ET.SubElement(eq, "connect", {"name": f"{side}_shank_loop", "body1": L["ankle"],
                                      "body2": f"{side}_shank_rear", "anchor": fmt(-d2),
                                      "solref": "0.005 1", "solimp": "0.99 0.999 0.001"})

    # fixed head: the ROBO-ONE rules only require a head, not an actuated one
    removed_joints = {name("right_ankle_pitch"), name("left_ankle_pitch")}
    if not a.neck_motor:
        for b in root.iter("body"):
            for j in list(b.findall("joint")):
                if j.get("name") == name("head"):
                    b.remove(j); removed_joints.add(name("head"))

    # actuators: drop ankle pitch (and neck), motor -> position unless asked otherwise
    act = root.find("actuator")
    ankle_joints = removed_joints
    kept = []
    for el in list(act):
        act.remove(el)
        if el.get("joint") in ankle_joints:
            continue
        if el.tag == "motor" and not a.keep_motor:
            el = ET.Element("position", {"name": el.get("name"), "joint": el.get("joint"),
                                         "kp": "50", "dampratio": "1",
                                         "forcerange": f"{-XL330_TORQUE} {XL330_TORQUE}"})
        kept.append(el)
    act.extend(kept)

    ET.indent(root, space="  ")
    body = ET.tostring(root, encoding="unicode")
    note = (f"<!--\n{model_name}: Microban with parallel-link legs (passive ankle pitch).\n"
            f"Derived from Rhoban/microban src/model/mjcf/robot.xml by tools/make_microban_pl.py\n"
            f"(naming={a.naming}, back={a.back}, tilt={a.tilt}, fixed_head={not a.neck_motor}). See NOTICE.md for attribution.\n-->\n")
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    with open(a.out, "w", encoding="utf-8") as f:
        f.write(head + note + body + "\n")
    print("wrote", a.out)

    if a.scene:
        write_scene(a.out, a.scene)

def write_scene(model_path, scene_path):
    import mujoco
    m = mujoco.MjModel.from_xml_path(model_path); d = mujoco.MjData(m); mujoco.mj_forward(m, d)
    # lowest point of the collision geometry at qpos0 (box corners exact, others by bounding sphere)
    zmin = 1e9
    for g in range(m.ngeom):
        if m.geom_contype[g] == 0 and m.geom_conaffinity[g] == 0:
            continue
        if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX:
            R = d.geom_xmat[g].reshape(3, 3); s = m.geom_size[g]
            for sx in (-1, 1):
                for sy in (-1, 1):
                    for sz in (-1, 1):
                        zmin = min(zmin, (d.geom_xpos[g] + R @ (s * [sx, sy, sz]))[2])
        else:
            zmin = min(zmin, d.geom_xpos[g][2] - m.geom_rbound[g])
    z0 = -zmin + 0.002
    qpos = np.zeros(m.nq); qpos[:7] = [0, 0, z0, 1, 0, 0, 0]
    rel = os.path.relpath(model_path, os.path.dirname(os.path.abspath(scene_path)))
    with open(scene_path, "w", encoding="utf-8") as f:
        f.write(f"""<!-- Copyright 2026 Masaaki Hijikata. SPDX-License-Identifier: Apache-2.0 -->
<!-- Viewer scene for {os.path.basename(model_path)}: floor + standing keyframe. -->
<mujoco model="{os.path.splitext(os.path.basename(model_path))[0]}_scene">
  <include file="{rel}"/>
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <global azimuth="160" elevation="-20"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3" markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
  </asset>
  <worldbody>
    <light pos="0 0 3.5" dir="0 0 -1" directional="true"/>
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>
  </worldbody>
  <keyframe>
    <key name="standing" qpos="{fmt(qpos)}"/>
  </keyframe>
</mujoco>
""")
    print("wrote", scene_path, f"(standing z0={z0:.4f})")

if __name__ == "__main__":
    main()
