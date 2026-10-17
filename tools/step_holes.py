#!/usr/bin/env python3
# Copyright 2026 Masaaki Hijikata. SPDX-License-Identifier: Apache-2.0
"""List the circular edges (holes, bosses) of STEP parts by radius: how the hole patterns in
docs/parts_ja.md were read from Microban's cad/step files. usage: python tools/step_holes.py a.step b.step"""
import sys, cadquery as cq
for f in sys.argv[1:]:
    shp = cq.importers.importStep(f)
    bb = shp.val().BoundingBox()
    print(f"== {f.split('/')[-1]}  bbox {bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f} mm  solids={len(shp.solids().vals())}")
    circ = {}
    for e in shp.edges().vals():
        if e.geomType() == 'CIRCLE':
            r = round(e.radius(), 2); c = e.Center()
            circ.setdefault(r, []).append((round(c.x,1), round(c.y,1), round(c.z,1)))
    for r in sorted(circ):
        pts = sorted(set(circ[r]))
        if len(pts) <= 12: print(f"   r={r:5.2f}  n={len(pts):2d}  {pts[:8]}")
        else: print(f"   r={r:5.2f}  n={len(pts):2d}")
