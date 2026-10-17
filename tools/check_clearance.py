#!/usr/bin/env python3
# Copyright 2026 Masaaki Hijikata
# SPDX-License-Identifier: Apache-2.0
"""Kinematic interference check of the added parts (link bars, knee yoke, axle cap, ear, standoff).

Sweeps one leg's (default right, --left) hip pitch and knee over their joint ranges (the passive joints follow
the parallelogram kinematics), runs MuJoCo collision detection on each pose and prints the
deepest penetration that involves one of the printed parts. Model self-collision pairs that
do not involve a part are ignored (they are Microban's own, and the knee range already stops
at the ankle-block/hip-block contact).

usage: python3 tools/check_clearance.py model/microban_pl12/microban_pl12.xml [--arm] [--left]
  --arm   also report contacts with the arm hanging straight down (shoulder/elbow at 0).
          The forearm then passes the shank bar and knee yoke with 0.5 mm interference in
          squats (hip -30/knee 90, hip -60/knee 60); in practice the arm is never there.
"""
import sys, numpy as np, mujoco

path = sys.argv[1]; with_arm = "--arm" in sys.argv; side = "left" if "--left" in sys.argv else "right"
m = mujoco.MjModel.from_xml_path(path); d = mujoco.MjData(m)
J = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
Q = lambda n: m.jnt_qposadr[J(n)]
B = lambda i: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[i])
def G(i):
    if m.geom_type[i] == mujoco.mjtGeom.mjGEOM_MESH:
        return mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_MESH, m.geom_dataid[i]) or ""
    return {5: "cyl", 6: "box", 3: "cap"}.get(int(m.geom_type[i]), "geom")
S, s_ = side, side[0]
names = {"hip": f"{S}_hip_pitch", "knee": f"{S}_knee", "kb": f"{S}_knee_block", "ank": f"{S}_ankle_pitch",
         "tr": f"{S}_thigh_rear", "sr": f"{S}_shank_rear"}
if J(names["hip"]) < 0:   # LME naming
    names = {"hip": f"{s_}_hipjoint_yp", "knee": f"{s_}_knee_yp", "kb": f"{s_}_kneeblock_yp", "ank": f"{s_}_anklepassive_yp",
             "tr": f"{s_}_thighlink_yp", "sr": f"{s_}_shanklink_yp"}
other = "left" if side == "right" else "right"
hr = np.degrees(m.jnt_range[J(names["hip"])]); kr = np.degrees(m.jnt_range[J(names["knee"])])
hips = list(range(int(np.ceil(hr[0] / 15)) * 15, int(hr[1]) + 1, 15)) + [round(hr[0], 1)]
knees = list(range(int(np.ceil(kr[0] / 15)) * 15, int(kr[1]) + 1, 15)) + [round(kr[1], 1)]
print(f"{path}: hip {hr[0]:.0f}..{hr[1]:.0f}, knee {kr[0]:.0f}..{kr[1]:.0f} deg; relative knee flexion limited to 130")
print("deepest part penetration [mm] ('ok' = none, '-' = outside the leg's reach); arm", "hanging" if with_arm else "ignored")
print("    hip\\knee " + " ".join(f"{k:>6.0f}" for k in knees))
bad = []
for hip in hips:
    row = []
    for knee in knees:
        if knee - hip > 130 or knee - hip < -10: row.append("   -  "); continue
        d.qpos[:] = 0; d.qpos[3] = 1
        d.qpos[Q(names["hip"])] = np.radians(hip); d.qpos[Q(names["knee"])] = np.radians(knee)
        d.qpos[Q(names["kb"])] = -np.radians(hip); d.qpos[Q(names["ank"])] = -np.radians(knee)
        d.qpos[Q(names["tr"])] = np.radians(hip); d.qpos[Q(names["sr"])] = np.radians(knee)
        mujoco.mj_forward(m, d); worst = (0.0, "")
        for c in d.contact[:d.ncon]:
            a, b = G(c.geom1), G(c.geom2); ba, bb = B(c.geom1), B(c.geom2)
            if not (a.startswith("part_") or b.startswith("part_") or "knee_block" in ba + bb or "kneeblock" in ba + bb):
                continue
            if (other in ba + bb or other[0] + "_" in (ba[:2], bb[:2])) and f"configuration_{other}" not in ba + bb: continue
            if not with_arm and any(k in ba + bb for k in ("radius", "humerus", "shoulder", "elbow")): continue
            if c.dist < worst[0] - 1e-4:
                worst = (c.dist, f"{a[5:] or ba.split('_')[0]} x {b[5:] or bb.split('_')[0]}")
        if worst[0] < -0.0003: row.append(f"{worst[0]*1000:5.1f} "); bad.append((hip, knee, worst))
        else: row.append("  ok  ")
    print(f"    {hip:8.0f} " + " ".join(row))
for hip, knee, (dist, what) in bad: print(f"  hip {hip:.0f} knee {knee:.0f}: {what}, {dist*1000:.1f} mm")
print("RESULT:", "OK, no part interference inside the joint ranges" if not bad else f"{len(bad)} poses with interference")
sys.exit(1 if bad else 0)
