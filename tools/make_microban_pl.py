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
# Joint ranges follow from the linkage: a parallelogram degenerates when its link lies
# along the line between its two pivots, i.e. at 90deg +- atan(tilt/back) from straight.
# Thigh: rear pivot DOWN -> flexion singular at 90 + a, extension at 90 - a.
# Shank: rear pivot UP   -> flexion singular at 90 + a, extension at 90 - a.
# Both ranges keep RANGE_MARGIN degrees clear of the singularity.
RANGE_MARGIN = 8.0
XL330_TORQUE = 0.52          # N*m, XL330-M288-T stall torque at 5 V
XL330_MASS = 0.023           # kg
SLIM_ANKLE_MASS = 0.025      # kg: ankle block minus the XL330 that is no longer there

def q2m(q):
    w, x, y, z = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
def vec(s): return np.array([float(t) for t in s.split()])
def fmt(a): return " ".join(f"{float(t):.6g}" for t in np.atleast_1d(a))

DUMMY_MASS = 0.008   # kg, a printed XL330-sized block (20 x 34 x 26 mm shell)


PART_HOOK = {"fn": None}   # set in main() when --parts is given: (body, name, pos, quat) -> adds a mesh geom


def dummy_block(body, pos, quat, part_name=None):
    """Visual + mass of a printed XL330-shaped spacer in a servo's old place: same envelope,
    so the original brackets bolt on unchanged and a real servo can go back in later."""
    if PART_HOOK["fn"] and part_name:
        PART_HOOK["fn"](body, part_name, vec(pos), vec(quat))
    else:
        ET.SubElement(body, "geom", {"class": "visual", "type": "box", "size": "0.010 0.017 0.013",
                                     "pos": fmt(vec(pos) + q2m(vec(quat)) @ np.array([0, -0.0075, -0.0065])),
                                     "quat": quat, "rgba": "0.85 0.6 0.25 1"})
    ine = body.find("inertial")
    m0 = float(ine.get("mass")); m1 = m0 + DUMMY_MASS
    ine.set("mass", f"{m1:.6f}")


def slim_ankle(body, h):
    """Take the ankle-pitch servo out of the ankle block.

    Upstream, the block carries two XL330s: the ankle-pitch servo (its horn on the
    tibia) and the ankle-roll servo. With the pitch joint passive, the first one and
    the idler horn opposite it go, and the U-bracket that wrapped the servo becomes a
    short clevis around the hinge. Its collision box keeps the bracket's plane and
    thickness but is cut to +-h about the axis. Mass: 47 g -> SLIM_ANKLE_MASS.
    """
    for g in list(body.findall("geom")):
        at = g.attrib
        if at.get("mesh") == "xl330":
            body.remove(g); dummy_block(body, at.get("pos"), at.get("quat", "1 0 0 0"), "xl330_dummy_ankle_hinge")
        elif at.get("mesh") in ("dc15_a01_horn_idle2_dummy", "bts2_m2_6x8", "dc15_a01_idle_cap_dummy"):
            body.remove(g)
        elif at.get("class") == "collision":
            sx, sy, sz = (float(v) for v in at["size"].split())
            px, py, pz = (float(v) for v in at["pos"].split())
            at["size"] = fmt((sx, h, sz)); at["pos"] = fmt((px, np.sign(py) * h, pz))
    ine = body.find("inertial")
    scale = SLIM_ANKLE_MASS / float(ine.get("mass"))   # dummy_block already added its 8 g above
    ine.set("mass", f"{SLIM_ANKLE_MASS + DUMMY_MASS}")
    ine.set("fullinertia", " ".join(f"{float(v) * scale:.4e}" for v in ine.get("fullinertia").split()))


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
    ap.add_argument("--back", type=float, default=0.020, help="thigh linkage: rear pivot offset behind the front pivot [m]")
    ap.add_argument("--tilt", type=float, default=0.010, help="thigh linkage: rear pivot DOWN by this much [m]")
    ap.add_argument("--shank-back", type=float, default=0.025,
                    help="shank linkage: rear pivot offset [m] (25 mm clears the tibia's rear edge and the ankle-roll servo)")
    ap.add_argument("--shank-tilt", type=float, default=0.025,
                    help="shank linkage: rear pivot UP by this much [m] (lifts the ankle pivot above the foot and the roll servo)")
    ap.add_argument("--link-x", type=float, default=0.015,
                    help="lateral position of the two bars of each linkage [m], +-link_x from the leg centre plane "
                         "(15 mm: just outside the hip/ankle bracket side walls, inside the arm's swing)")
    ap.add_argument("--keep-ankle-servo", action="store_true",
                    help="keep the ankle-pitch servo's geometry and mass in the ankle block (by default "
                         "it is removed and the bracket shrunk to a clevis, which is what lets the knee fold deeper)")
    ap.add_argument("--clevis", type=float, default=0.006,
                    help="half-height of the slimmed ankle bracket's collision box [m]")
    ap.add_argument("--scene", help="also write a viewer scene (floor + standing keyframe) to this path")
    ap.add_argument("--dof", type=int, choices=[16, 14, 12], default=16,
                    help="16: passive ankle pitch + fixed head. 14: additionally no shoulder roll "
                         "(legs 5x2 with hip yaw, arms 2x2 -- the chosen layout). 12: no hip yaw either")
    ap.add_argument("--meshdir", help="override the compiler meshdir written into the output")
    ap.add_argument("--parts", default="",
                    help="directory of the printed-part STLs (tools/make_parts.py), as a path RELATIVE TO MESHDIR; "
                         "when given, the links, knee yoke, axle caps and spacer blocks are drawn with those meshes "
                         "instead of boxes (which LegacyMotionEditor's preview cannot show)")
    ap.add_argument("--forearm-ext", type=float, default=0.0,
                    help="lengthen each forearm by this much [m] with a printed extension (a capsule "
                         "past the hand, 10 g per 40 mm). ROBO-ONE Light allows 26 cm from the shoulder.")
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

    model_name = {16: "microban_pl", 14: "microban_pl14", 12: "microban_pl12"}[a.dof] + ("_lme" if a.naming == "lme" else "")
    root.set("model", model_name)

    asset = root.find("asset")
    PART_MESHES = ("thigh_bar", "shank_bar", "knee_yoke_right", "knee_yoke_left", "knee_axle_cap",
                   "xl330_dummy_shoulder_roll", "xl330_dummy_neck", "xl330_dummy_hip_yaw", "xl330_dummy_ankle_hinge")
    if a.parts:
        for nm in PART_MESHES:
            ET.SubElement(asset, "mesh", {"name": f"part_{nm}", "file": f"{a.parts}/{nm}.stl", "scale": "0.001 0.001 0.001"})

    def part(body, nm, pos, quat, rgba="0.85 0.6 0.25 1"):
        ET.SubElement(body, "geom", {"class": "visual", "type": "mesh", "mesh": f"part_{nm}", "pos": fmt(pos),
                                     "quat": fmt(quat), "rgba": rgba})
    PART_HOOK["fn"] = part if a.parts else None

    dflt = root.find("default")
    pcls = ET.SubElement(dflt, "default", {"class": "passive"})
    ET.SubElement(pcls, "joint", {"damping": "0.005", "frictionloss": "0.001", "armature": "0.0001"})

    eq = root.find("equality")
    if eq is None:
        eq = ET.SubElement(root, "equality")

    def inertial(el, mass, pos, diag):
        ET.SubElement(el, "inertial", {"pos": fmt(pos), "mass": f"{mass}", "diaginertia": fmt(diag)})
    def rod(el, v, bar="thigh_bar", lateral=None):
        # printed bars, pin to pin, in sagittal planes offset laterally (local z is the pitch
        # axis = world +y): thigh bars on both sides at +-link_x, the shank bar outboard only
        ang = np.arctan2(v[1], v[0]); q = (np.cos(ang / 2), 0.0, 0.0, np.sin(ang / 2))
        offsets = (-a.link_x, a.link_x) if lateral is None else lateral
        if a.parts:
            for zx in offsets:   # STL: centred at the origin along x, 3 mm thick in +z
                part(el, bar, v / 2 + np.array([0, 0, zx - 0.0015]), q, "0.25 0.25 0.28 1")
            return
        for zx in offsets:
            ET.SubElement(el, "geom", {"class": "visual", "type": "box",
                                       "size": fmt((np.linalg.norm(v) / 2 + 0.004, 0.004, 0.0015)),
                                       "pos": fmt(v / 2 + np.array([0, 0, zx])), "quat": fmt(q),
                                       "rgba": "0.25 0.25 0.28 1"})

    ang1 = np.degrees(np.arctan2(a.tilt, a.back)); ang2 = np.degrees(np.arctan2(a.shank_tilt, a.shank_back))
    HIP_RANGE = (-np.radians(90 + ang1 - RANGE_MARGIN), np.radians(90 - ang1 - RANGE_MARGIN))   # +ve = thigh back
    KNEE_RANGE = (-np.radians(90 - ang2 - RANGE_MARGIN), np.radians(90 + ang2 - RANGE_MARGIN))  # +ve = shank back

    for side, L in LEG.items():
        hipblock, femur, tibia, ankle = (bodies[L[k]] for k in ("hipblock", "femur", "tibia", "ankle"))
        if not a.keep_ankle_servo:
            slim_ankle(ankle, a.clevis)
        d1 = np.array([L["sx"] * a.back, -L["sx"] * a.tilt, 0.0])               # thigh rear pivot (back, down)
        d2 = np.array([L["sx"] * a.shank_back, +L["sx"] * a.shank_tilt, 0.0])   # shank rear pivot (back, up)
        v1 = vec(tibia.get("pos"))          # hip pitch axis -> knee axis (femur frame)
        v2 = vec(ankle.get("pos"))          # knee axis -> ankle axis (tibia frame)
        tq = tibia.get("quat", "1 0 0 0").replace("-0", "0").split()
        assert tq == ["1", "0", "0", "0"], f"{L['tibia']}: quat must be identity"

        # The femur/tibia body origins sit on the INBOARD clevis plate, 14.5 mm from the leg's
        # centre plane, so lateral placements are measured from the knee servo's centre:
        # c_z in the femur body frame (local z = pitch axis = world +y), x_c in the femur
        # mesh (= Onshape leg) frame.
        sg = next(x for x in tibia.findall("geom") if x.get("mesh") == "xl330")
        # leg centre = midway between the horn face (mesh z -22.5) and the idler face (+6.5)
        p_servo = v1 + vec(sg.get("pos")) + q2m(vec(sg.get("quat", "1 0 0 0"))) @ np.array([0, -0.0075, -0.0080])
        c_z = float(p_servo[2])
        fg = next(x for x in femur.findall("geom") if x.get("mesh") == L["femur"])
        fgp, fgq = vec(fg.get("pos")), vec(fg.get("quat", "1 0 0 0")); R_fg = q2m(fgq)
        x_c = float((R_fg.T @ (p_servo - fgp))[0])
        knee_mesh = R_fg.T @ (v1 - fgp)
        out_sign = 1.0 if side == "right" else -1.0        # mesh +x is world -y: outboard on the right, inboard on the left
        out_z = -1.0 if side == "right" else 1.0           # femur body +z is world +y

        # knee block: passive hinge, stays parallel to the pelvis
        femur.remove(tibia)
        kb = ET.Element("body", {"name": f"{side}_knee_block", "pos": fmt(v1)})
        ET.SubElement(kb, "joint", {"name": name(f"{side}_knee_block"), "type": "hinge",
                                    "axis": "0 0 1", "class": "passive"})
        inertial(kb, 0.006, (0, 0, 0), (2e-7, 2e-7, 2e-7))
        if a.parts:
            # the parts are modelled in the Onshape leg frame with the leg centre at x = 0; that
            # frame is the femur mesh geom's frame shifted to the actual centre x_c
            part(kb, f"knee_yoke_{side}", fgp + R_fg @ np.array([x_c, 0, 0]) - v1, fgq, "0.25 0.25 0.28 1")
            # axle cap: flange on the outer plate face (centre +- 18.5 mm), stub pointing outboard
            cap_pos = fgp + R_fg @ np.array([x_c + out_sign * 0.0185, knee_mesh[1], knee_mesh[2]])
            r2 = np.sqrt(0.5); q_rot = np.array([r2, 0, out_sign * r2, 0])        # cap +z -> mesh +-x
            w1, x1, y1, z1 = fgq; w2, x2, y2, z2 = q_rot
            cap_q = (w1*w2 - x1*x2 - y1*y2 - z1*z2, w1*x2 + x1*w2 + y1*z2 - z1*y2,
                     w1*y2 - x1*z2 + y1*w2 + z1*x2, w1*z2 + x1*y2 - y1*x2 + z1*w2)
            part(femur, "knee_axle_cap", cap_pos, cap_q, "0.6 0.2 0.2 1")
        # box placeholders (shown when no --parts): a bar behind the knee carrying both
        # rear-link pivots and spanning the two link planes, plus an outboard arm
        mid = (d1 + d2) / 2
        if not a.parts:
            ET.SubElement(kb, "geom", {"class": "visual", "type": "box",
                                       "size": fmt((0.004, abs(d2[1] - d1[1]) / 2 + 0.006, a.link_x + 0.0015)),
                                       "pos": fmt(mid + np.array([0, 0, c_z])), "rgba": "0.25 0.25 0.28 1"})
            ET.SubElement(kb, "geom", {"class": "visual", "type": "box",
                                       "size": fmt((abs(mid[0]) / 2 + 0.006, 0.005, 0.0015)),
                                       "pos": fmt((mid[0] / 2, mid[1] / 2, c_z + out_z * 0.022)), "rgba": "0.25 0.25 0.28 1"})
        tibia.set("pos", "0 0 0"); tibia.set("quat", "1 0 0 0")
        kb.append(tibia)
        sr = ET.SubElement(kb, "body", {"name": f"{side}_shank_rear", "pos": fmt(d2)})
        ET.SubElement(sr, "joint", {"name": name(f"{side}_shank_rear"), "type": "hinge",
                                    "axis": "0 0 1", "class": "passive"})
        inertial(sr, 0.002, v2 / 2, (6.4e-7, 6.4e-7, 2e-8)); rod(sr, v2, "shank_bar", (c_z + out_z * 0.019,))
        femur.append(kb)

        fpos, fq = vec(femur.get("pos")), vec(femur.get("quat"))
        tr = ET.SubElement(hipblock, "body", {"name": f"{side}_thigh_rear",
                                              "pos": fmt(fpos + q2m(fq) @ d1), "quat": femur.get("quat")})
        ET.SubElement(tr, "joint", {"name": name(f"{side}_thigh_rear"), "type": "hinge",
                                    "axis": "0 0 1", "class": "passive"})
        inertial(tr, 0.002, v1 / 2, (6.4e-7, 6.4e-7, 2e-8)); rod(tr, v1, "thigh_bar", (c_z - 0.016, c_z + 0.016))

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
        # the neck servo itself sits in the trunk, just under the head: drop it and its mass
        trunk = bodies["trunk"]
        for g in list(trunk.findall("geom")):
            if g.get("mesh", "").startswith("xl330") and abs(vec(g.get("pos"))[2] - 0.1195) < 1e-3:
                trunk.remove(g); dummy_block(trunk, g.get("pos"), g.get("quat", "1 0 0 0"), "xl330_dummy_neck")
        ine = trunk.find("inertial")
        m0 = float(ine.get("mass")); m1 = m0 - XL330_MASS
        ine.set("mass", f"{m1:.6f}")
        ine.set("fullinertia", " ".join(f"{float(v) * m1 / m0:.5e}" for v in ine.get("fullinertia").split()))

    drop = []
    if a.dof <= 14:
        drop.append("shoulder_roll")   # joint on the humerus, servo at the top of the humerus
    if a.dof == 12:
        drop.append("hip_yaw")         # joint on the hip bracket, servo in the bottom of the trunk
    for side in ("right", "left"):
        for base in drop:
            for b in root.iter("body"):
                for j in list(b.findall("joint")):
                    if j.get("name") == name(f"{side}_{base}"):
                        b.remove(j); removed_joints.add(name(f"{side}_{base}"))
    if a.dof == 12:
        # Hip yaw servos: one XL330 each side at y = +-36 mm in the bottom of the trunk.
        # Without them each hip bracket bolts straight to the trunk.
        trunk = bodies["trunk"]
        n_removed = 0
        for g in list(trunk.findall("geom")):
            pz = vec(g.get("pos", "0 0 0"))
            if g.get("mesh", "").startswith("xl330") and abs(abs(pz[1]) - 0.036) < 1e-3 and abs(pz[2] - 0.0065) < 1e-3:
                trunk.remove(g); n_removed += 1; dummy_block(trunk, g.get("pos"), g.get("quat", "1 0 0 0"), "xl330_dummy_hip_yaw")
        assert n_removed == 2, f"expected the two hip-yaw servos in the trunk, found {n_removed}"
        ine = trunk.find("inertial")
        m0 = float(ine.get("mass")); m1 = m0 - 2 * XL330_MASS
        ine.set("mass", f"{m1:.6f}")
        ine.set("fullinertia", " ".join(f"{float(v) * m1 / m0:.5e}" for v in ine.get("fullinertia").split()))
    if a.dof <= 14:
        # Shoulder roll servos: the XL330 at the top of each humerus (the other one in the
        # humerus is the elbow's). The humerus bolts to the shoulder bracket, arm at the side.
        for hb in ("humerus__configuration_right", "humerus__configuration_left"):
            hum = bodies[hb]; n_removed = 0
            for g in list(hum.findall("geom")):
                if g.get("mesh") == "xl330" and abs(vec(g.get("pos"))[2] + 0.0065) < 1e-3:
                    hum.remove(g); n_removed += 1; dummy_block(hum, g.get("pos"), g.get("quat", "1 0 0 0"), "xl330_dummy_shoulder_roll")
            assert n_removed == 1, f"{hb}: shoulder-roll servo not found"
            ine = hum.find("inertial")
            m0 = float(ine.get("mass")); m1 = m0 - XL330_MASS
            ine.set("mass", f"{m1:.6f}")
            ine.set("fullinertia", " ".join(f"{float(v) * m1 / m0:.5e}" for v in ine.get("fullinertia").split()))

    if a.forearm_ext > 0:
        # The radius runs along its local -y from the elbow; the upstream hand collision
        # cylinder is centred 67 mm down it. Add a capsule from there to 67 mm + ext.
        for rb in ("radius", "radius_2"):
            rad = bodies[rb]
            sy = -1.0 if rb == "radius" else 1.0
            y0, y1 = sy * 0.067, sy * (0.067 + a.forearm_ext)
            for cls, extra in (("collision", {}), ("visual", {"rgba": "0.3 0.3 0.32 1"})):
                ET.SubElement(rad, "geom", {"class": cls, "type": "capsule", "size": "0.005",
                                            "fromto": fmt((0, y0, 0.0135)) + " " + fmt((0, y1, 0.0135)), **extra})
            ine = rad.find("inertial")
            m0 = float(ine.get("mass")); madd = 0.010 * a.forearm_ext / 0.040
            c0 = vec(ine.get("pos")); c1 = np.array([0, (y0 + y1) / 2, 0.0135])
            ine.set("pos", fmt((c0 * m0 + c1 * madd) / (m0 + madd))); ine.set("mass", f"{m0 + madd:.6f}")
            fi = [float(v) for v in ine.get("fullinertia").split()]
            L = a.forearm_ext; I = madd * L * L / 12
            fi[0] += I; fi[2] += I          # rod inertia about the two axes across it
            ine.set("fullinertia", " ".join(f"{v:.5e}" for v in fi))

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

    if a.meshdir:
        root.find("compiler").set("meshdir", a.meshdir)
    ET.indent(root, space="  ")
    body = ET.tostring(root, encoding="unicode")
    note = (f"<!--\n{model_name}: Microban with parallel-link legs (passive ankle pitch).\n"
            f"Derived from Rhoban/microban src/model/mjcf/robot.xml by tools/make_microban_pl.py\n"
            f"(dof={a.dof}, naming={a.naming}, thigh back/tilt={a.back}/{a.tilt}, shank back/tilt={a.shank_back}/{a.shank_tilt}, link_x={a.link_x}, fixed_head={not a.neck_motor}, slim_ankle={not a.keep_ankle_servo}, forearm_ext={a.forearm_ext}).\n"
            f"Hip range {np.degrees(HIP_RANGE[0]):.0f}..{np.degrees(HIP_RANGE[1]):.0f} deg, knee {np.degrees(KNEE_RANGE[0]):.0f}..{np.degrees(KNEE_RANGE[1]):.0f} deg (pelvis-relative). See NOTICE.md for attribution.\n-->\n")
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
