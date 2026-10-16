#!/usr/bin/env python3
# Copyright 2026 Masaaki Hijikata
# SPDX-License-Identifier: Apache-2.0
"""Check a parallel-link leg model: loop closure, foot-parallel sweep, standing test.

usage: python3 tools/verify_parallel_leg.py model/microban_pl/microban_pl.xml
Works with either naming convention (Microban originals or LME names).
"""
import os, sys, tempfile
import numpy as np, mujoco
import xml.etree.ElementTree as ET
np.set_printoptions(precision=4, suppress=True)

ALIAS = {"hip": ("{side}_hip_pitch", "{s}_hipjoint_yp"), "knee": ("{side}_knee", "{s}_knee_yp"),
         "ankle": ("{side}_ankle_pitch", "{s}_ankle_yp")}
FOOT = {"right": "foot", "left": "foot_2"}

def build_scene(src):
    """Model + flat floor, trunk lifted so the soles start 1 cm above the ground."""
    src = os.path.abspath(src)
    t = ET.parse(src); r = t.getroot()
    comp = r.find("compiler"); comp.set("meshdir", os.path.join(os.path.dirname(src), comp.get("meshdir", "")))
    wb = r.find("worldbody")
    ET.SubElement(wb, "geom", {"name": "floor", "type": "plane", "size": "2 2 0.1", "friction": "0.8"})
    ET.SubElement(wb, "light", {"pos": "0 0 1"})
    m0 = mujoco.MjModel.from_xml_path(src); d0 = mujoco.MjData(m0); mujoco.mj_forward(m0, d0)
    zmin = min(d0.geom_xpos[g][2] - m0.geom_rbound[g] for g in range(m0.ngeom)
               if m0.geom_contype[g] or m0.geom_conaffinity[g])
    wb.find("body").set("pos", f"0 0 {-zmin + 0.01:.4f}")
    out = os.path.join(tempfile.mkdtemp(prefix="pl_verify_"), "scene.xml"); t.write(out)
    return out

def main(src):
    m = mujoco.MjModel.from_xml_path(build_scene(src)); d = mujoco.MjData(m)
    J = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
    B = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
    joint_act = {int(m.actuator_trnid[i, 0]): i for i in range(m.nu)}
    def jid(kind, side):
        s = side[0]
        for pat in ALIAS[kind]:
            j = J(pat.format(side=side, s=s))
            if j >= 0: return j
        raise KeyError(f"{kind} joint of {side} leg not found")
    def act(kind, side):
        j = jid(kind, side)
        if j not in joint_act: raise KeyError(f"no actuator on {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j)}")
        return joint_act[j]
    def q(kind, side): return d.qpos[m.jnt_qposadr[jid(kind, side)]]
    print(f"model: {src}\n  nq={m.nq} nu={m.nu} neq={m.neq} mass={m.body_mass.sum():.3f} kg")
    for side in ("right", "left"):
        assert jid("ankle", side) not in joint_act, "ankle pitch is still actuated"

    def closure():
        res = []
        for e in range(m.neq):
            if m.eq_type[e] != mujoco.mjtEq.mjEQ_CONNECT: continue
            b1, b2 = m.eq_obj1id[e], m.eq_obj2id[e]
            p1 = d.xpos[b1] + d.xmat[b1].reshape(3, 3) @ m.eq_data[e, 0:3]
            p2 = d.xpos[b2] + d.xmat[b2].reshape(3, 3) @ m.eq_data[e, 3:6]
            res.append(np.linalg.norm(p1 - p2))
        return max(res) if res else 0.0
    Rt0 = None
    def foot_tilt(side):
        Rt = d.xmat[B("trunk")].reshape(3, 3); Rf = d.xmat[B(FOOT[side])].reshape(3, 3)
        Rerr = (Rt.T @ Rf) @ Rt0[side].T
        return np.degrees(np.arccos(np.clip((np.trace(Rerr) - 1) / 2, -1, 1)))
    mujoco.mj_forward(m, d)
    Rt0 = {side: d.xmat[B("trunk")].reshape(3, 3).T @ d.xmat[B(FOOT[side])].reshape(3, 3) for side in FOOT}

    print(f"\n[1] loop closure residual at qpos0: {closure()*1e3:.4f} mm")

    ok = True
    m.opt.gravity[:] = 0
    print("\n[2] foot-parallel sweep, gravity off (hip, knee [deg] -> closure [mm], foot tilt R/L [deg], hip tracking err [deg])")
    for hip, knee in [(0, 0), (-30, 30), (-60, 60), (30, -30), (-60, 0), (0, 60), (-90, 0), (60, 60)]:
        mujoco.mj_resetData(m, d)
        for side in FOOT:
            d.ctrl[act("hip", side)] = np.radians(hip); d.ctrl[act("knee", side)] = np.radians(knee)
        for _ in range(1500): mujoco.mj_step(m, d)
        lo, hi = np.degrees(m.jnt_range[jid("hip", "right")])   # the test pose may exceed the joint range
        err = np.degrees(q("hip", "right")) - min(max(hip, lo), hi)
        tR, tL = foot_tilt("right"), foot_tilt("left")
        flag = "" if (tR < 1 and tL < 1 and abs(err) < 2) else "   <-- check"
        ok &= flag == ""
        print(f"   {hip:4d} {knee:4d} -> {closure()*1e3:6.3f}   {tR:5.2f} {tL:5.2f}   {err:6.2f}{flag}")

    m.opt.gravity[:] = (0, 0, -9.81)
    print("\n[3] standing on a floor for 3 s (hip, knee [deg] -> trunk height, trunk tilt, foot tilt R/L)")
    for hip, knee in [(0, 0), (-30, 30), (-60, 60)]:
        mujoco.mj_resetData(m, d)
        for side in FOOT:
            d.ctrl[act("hip", side)] = np.radians(hip); d.ctrl[act("knee", side)] = np.radians(knee)
        for _ in range(1500): mujoco.mj_step(m, d)
        Rt = d.xmat[B("trunk")].reshape(3, 3); tilt = np.degrees(np.arccos(np.clip(Rt[2, 2], -1, 1)))
        flag = "" if tilt < 5 else "   <-- fell"
        ok &= flag == ""
        print(f"   {hip:4d} {knee:4d} -> z={d.xpos[B('trunk')][2]:.3f} m  tilt={tilt:5.2f}  "
              f"foot {foot_tilt('right'):5.2f} {foot_tilt('left'):5.2f}{flag}")
    print("\nRESULT:", "OK" if ok else "CHECK FAILED")
    return 0 if ok else 1

if __name__ == "__main__":
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else "model/microban_pl/microban_pl.xml"))
