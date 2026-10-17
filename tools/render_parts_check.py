#!/usr/bin/env python3
# Copyright 2026 Masaaki Hijikata
# SPDX-License-Identifier: Apache-2.0
"""Render the added parts (green) on a semi-transparent robot, close up on the right knee.

Left tile: outboard side view with the hip flexed 45 deg and the knee bent 45 deg (the
walking crouch); right tile: 3/4 view from the front in the same pose; bottom row: the
deepest symmetric squat the ranges allow (hip -60, knee 60).

usage: MUJOCO_GL=egl python3 tools/render_parts_check.py model/microban_pl12/scene.xml docs/images/parts_check.png
"""
import sys, numpy as np, mujoco
from PIL import Image

scene, out = sys.argv[1], sys.argv[2]
m = mujoco.MjModel.from_xml_path(scene); d = mujoco.MjData(m)
A = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
hip = [A(n) for n in ("right_hip_pitch", "left_hip_pitch", "r_hipjoint_yp", "l_hipjoint_yp") if A(n) >= 0]
knee = [A(n) for n in ("right_knee", "left_knee", "r_knee_yp", "l_knee_yp") if A(n) >= 0]
# everything that is not one of our parts becomes see-through
for g in range(m.ngeom):
    if m.geom_group[g] != 2: continue
    mesh = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_MESH, m.geom_dataid[g]) if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH else ""
    if not (mesh or "").startswith("part_"):
        m.geom_rgba[g, 3] = 0.25
kb = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "right_knee_block")
if kb < 0: kb = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "r_kneeblock_yp_link")
r = mujoco.Renderer(m, 480, 640)
cam = mujoco.MjvCamera(); cam.type = mujoco.mjtCamera.mjCAMERA_FREE
opt = mujoco.MjvOption(); opt.geomgroup[:] = 0; opt.geomgroup[0] = 1; opt.geomgroup[2] = 1
rows = []
for h, k in [(-45, 45), (-60, 60)]:
    mujoco.mj_resetDataKeyframe(m, d, 0)
    d.ctrl[hip] = np.radians(h); d.ctrl[knee] = np.radians(k)
    for _ in range(1500): mujoco.mj_step(m, d)
    look = d.xpos[kb] if kb >= 0 else d.xpos[1]
    tiles = []
    for az, el, dist in ((90, -5, 0.30), (40, -15, 0.32)):
        cam.lookat[:] = look + [0, 0, 0.01]; cam.distance = dist; cam.azimuth = az; cam.elevation = el
        r.update_scene(d, cam, opt); tiles.append(r.render())
    rows.append(np.concatenate(tiles, axis=1))
Image.fromarray(np.concatenate(rows, axis=0)).save(out); print("wrote", out)
