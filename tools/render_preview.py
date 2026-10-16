#!/usr/bin/env python3
# Copyright 2026 Masaaki Hijikata
# SPDX-License-Identifier: Apache-2.0
"""Render preview images of the model (side and 3/4 views, standing and crouched).

usage: MUJOCO_GL=egl python3 tools/render_preview.py model/microban_pl/scene.xml docs/images/preview.png
"""
import sys, numpy as np, mujoco
from PIL import Image

scene, out = sys.argv[1], sys.argv[2]
m = mujoco.MjModel.from_xml_path(scene); d = mujoco.MjData(m)
A = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
hip = [A(n) for n in ("right_hip_pitch", "left_hip_pitch", "r_hipjoint_yp", "l_hipjoint_yp") if A(n) >= 0]
knee = [A(n) for n in ("right_knee", "left_knee", "r_knee_yp", "l_knee_yp") if A(n) >= 0]
r = mujoco.Renderer(m, 480, 640)
cam = mujoco.MjvCamera(); cam.type = mujoco.mjtCamera.mjCAMERA_FREE
opt = mujoco.MjvOption(); opt.geomgroup[:] = 0; opt.geomgroup[0] = 1; opt.geomgroup[2] = 1  # floor + visual meshes
rows = []
for h, k in [(0, 0), (-45, 45)]:
    mujoco.mj_resetDataKeyframe(m, d, 0)
    d.ctrl[hip] = np.radians(h); d.ctrl[knee] = np.radians(k)
    for _ in range(1500): mujoco.mj_step(m, d)
    trunk = d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "trunk")]
    tiles = []
    for az in (90, 135):
        cam.lookat[:] = trunk + [0, 0, -0.01]; cam.distance = 0.6; cam.azimuth = az; cam.elevation = -10
        r.update_scene(d, cam, opt); tiles.append(r.render())
    rows.append(np.concatenate(tiles, axis=1))
Image.fromarray(np.concatenate(rows, axis=0)).save(out); print("wrote", out)
