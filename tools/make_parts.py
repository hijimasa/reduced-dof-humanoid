#!/usr/bin/env python3
# Copyright 2026 Masaaki Hijikata
# SPDX-License-Identifier: Apache-2.0
"""Printable drafts of the parts the parallel-link conversion adds (CadQuery).

All numbers are in the LEG frame of Microban's Onshape parts (femur_right.step,
tibia_right.step, ankle_dbl_block.step share it): x lateral (+ = outboard on the right
leg), y forward, z up, millimetres. Pivot layout (docs/parts_ja.md):

    hip pitch axis   (y  3.0, z 132.7)    knee axis (8.0, 70.7)    ankle axis (0.0, 9.5)
    thigh linkage    hip pivot (-17.0, 122.7)  ->  knee pivot (-12.0,  60.7)   bar 62.2 mm
    shank linkage    knee pivot (-17.0,  95.7)  ->  ankle pivot (-25.0, 34.5)  bar 61.7 mm

These are DRAFTS for fit-checking against the originals in Onshape, not release parts:
the screw holes that mate with Microban's brackets are placed from the STEP files'
hole patterns, but nothing here has been printed.

usage: <venv with cadquery>/bin/python tools/make_parts.py model/parts
"""
import math, os, sys
import cadquery as cq

OUT = sys.argv[1] if len(sys.argv) > 1 else "model/parts"
os.makedirs(OUT, exist_ok=True)

# ---- layout (mm, leg frame) --------------------------------------------------------
HIP, KNEE, ANKLE = (3.0, 132.7), (8.0, 70.7), (0.0, 9.5)
THIGH_BACK, THIGH_DOWN = 20.0, 10.0
SHANK_BACK, SHANK_UP = 25.0, 25.0
P1H = (HIP[0] - THIGH_BACK, HIP[1] - THIGH_DOWN)      # thigh bar, hip end
P1K = (KNEE[0] - THIGH_BACK, KNEE[1] - THIGH_DOWN)    # thigh bar, knee end
P2K = (KNEE[0] - SHANK_BACK, KNEE[1] + SHANK_UP)      # shank bar, knee end
P2A = (ANKLE[0] - SHANK_BACK, ANKLE[1] + SHANK_UP)    # shank bar, ankle end
BAR_T, BAR_W, PIN_D = 3.0, 8.0, 2.2                    # bar thickness, width, M2 clearance
PLATE_X = 18.5          # outer face of the femur / tibia clevis plates
THIGH_BAR_X = (14.5, 17.5)   # thigh bars: both sides, just outside the hip bracket wall (13.5)
SHANK_BAR_X = (17.5, 20.5)   # shank bar: outboard only (the inner side has no room between the legs)
YOKE_ARM_X = (20.5, 23.5)    # knee yoke's arm to the knee axle, outside the shank bar

def dist(p, q): return math.hypot(q[0] - p[0], q[1] - p[1])

def bar(length, name):
    """Flat link: two Ø2.2 holes `length` apart, rounded ends, 3 x 8 mm section."""
    b = (cq.Workplane("XY").slot2D(length + BAR_W, BAR_W).extrude(BAR_T)
         .faces(">Z").workplane().pushPoints([(-length / 2, 0), (length / 2, 0)]).hole(PIN_D))
    cq.exporters.export(b, f"{OUT}/{name}.stl"); return b

def knee_yoke(side="right"):
    """Block behind the knee carrying both rear-link pivots, with one outboard arm to the
    knee axle. Pivots: thigh bars on both faces at P1K, shank bar on the arm at P2K.
    The block stays inside the clevis plates (|x| < 14.5) and behind the knee servo
    (y < -8), the arm runs outside the plates below the hanging arm."""
    sx = 1 if side == "right" else -1
    y0, y1 = -21.0, -9.0                       # block depth: behind the knee servo (y > -2) and the femur (y > -6)
    z0, z1 = P1K[1] - 6.0, P2K[1] + 6.0        # 54.7 .. 101.7
    blk = cq.Workplane("XY").box(29.0, y1 - y0, z1 - z0, centered=(True, False, False)).translate((0, y0, z0))
    # lightening pocket through the block (keeps 3 mm walls)
    blk = blk.cut(cq.Workplane("XY").box(23.0, y1 - y0 - 6, z1 - z0 - 24, centered=(True, False, False)).translate((0, y0 + 3, z0 + 12)))
    # thigh pivot: M2 through hole along x at P1K
    blk = blk.cut(cq.Workplane("YZ").center(P1K[0], P1K[1]).circle(PIN_D / 2).extrude(40, both=True))
    # outboard arm: plate at x in YOKE_ARM_X from the block to the knee axle
    ax0, ax1 = (YOKE_ARM_X if sx > 0 else (-YOKE_ARM_X[1], -YOKE_ARM_X[0]))
    arm = (cq.Workplane("YZ").workplane(offset=ax0)
           .polyline([(y0, z0 + 8), (y0, z1 - 8), (P2K[0] + 6, P2K[1] + 6), (KNEE[0] + 2, KNEE[1] + 9),
                      (KNEE[0] + 9, KNEE[1]), (KNEE[0] + 2, KNEE[1] - 9), (y1 + 2, z0 + 2)]).close()
           .extrude(ax1 - ax0))
    # bridge from the block face to the arm (fills x from 14.5 to the arm, behind the plates)
    bridge = cq.Workplane("XY").box(abs(ax1) - 14.5, 8.0, 20.0, centered=(False, True, True)).translate((sx * 14.5 if sx > 0 else -abs(ax1), -15.0, (z0 + z1) / 2))
    yoke = blk.union(arm).union(bridge)
    # knee axle bore and shank pivot on the arm
    yoke = yoke.cut(cq.Workplane("YZ").center(KNEE[0], KNEE[1]).circle(4.1).extrude(60, both=True))
    yoke = yoke.cut(cq.Workplane("YZ").center(P2K[0], P2K[1]).circle(PIN_D / 2).extrude(60, both=True))
    cq.exporters.export(yoke, f"{OUT}/knee_yoke_{side}.stl"); return yoke

def knee_axle_cap():
    """Bolts onto the knee servo horn through the femur plate with the four M2 horn screws
    (PCD 12, lengthened to M2x12) and carries an Ø8 stub axle for the yoke, from the
    plate's outer face (x 18.5) out to x 24.5; M2 tapped hole in the tip for a retainer."""
    cap = (cq.Workplane("XY").circle(9.0).extrude(2.0)                   # 2 mm flange on the plate
           .faces(">Z").workplane().circle(4.0).extrude(6.0))            # Ø8 stub, 6 mm
    cap = cap.faces("<Z").workplane().pushPoints([(6, 0), (-6, 0), (0, 6), (0, -6)]).hole(2.2)
    cap = cap.faces(">Z").workplane().hole(1.6, 5.0)                    # M2 tap drill in the stub
    cq.exporters.export(cap, f"{OUT}/knee_axle_cap.stl"); return cap

def standoff(length, name):
    """Spacer between a bracket wall and a bar, M2 clearance through."""
    s = cq.Workplane("XY").circle(3.0).extrude(length).faces(">Z").workplane().hole(PIN_D)
    cq.exporters.export(s, f"{OUT}/{name}.stl"); return s

def xl330_dummy(name, horn=True):
    """Placeholder block with the XL330's envelope (20 x 34 x 26, horn axis at the origin,
    horn face at z = -19.5) for a servo position that is no longer driven: shoulder roll,
    neck. Back-face holes 4 x Ø2.2 on a 16 x 25.5 mm rectangle as read from the
    brackets' STEP files; verify against the real servo before printing."""
    body = cq.Workplane("XY").box(20, 34, 26, centered=(True, False, False)).translate((0, -24.5, -19.5))
    body = body.faces("<Y").workplane().pushPoints([(-8, -8.0), (8, -8.0), (-8, 17.5), (8, 17.5)]).hole(2.2, 8)
    if horn:   # fixed horn disc: PCD 12 M2 holes and the Ø7.2 centre boss, as on the idle horn
        disc = cq.Workplane("XY").circle(8.0).extrude(-3.0).translate((0, 0, -19.5))
        disc = disc.faces("<Z").workplane().pushPoints([(6, 0), (-6, 0), (0, 6), (0, -6)]).hole(1.6)
        body = body.union(disc).union(cq.Workplane("XY").circle(3.6).extrude(-1.5).translate((0, 0, -22.5)))
    cq.exporters.export(body, f"{OUT}/{name}.stl"); return body

if __name__ == "__main__":
    L1, L2 = dist(P1H, P1K), dist(P2K, P2A)
    assert abs(L1 - dist(HIP, KNEE)) < 1e-6 and abs(L2 - dist(KNEE, ANKLE)) < 1e-6, "parallelogram: bar = front link"
    bar(L1, "thigh_bar")           # x4 (two per leg)
    bar(L2, "shank_bar")           # x2 (one per leg, outboard)
    knee_yoke("right"); knee_yoke("left")
    knee_axle_cap()                # x2
    standoff(THIGH_BAR_X[0] - 13.5, "standoff_thigh_hip")    # 1 mm: hip bracket wall (13.5) -> thigh bar
    standoff(SHANK_BAR_X[0] - 13.5, "standoff_shank_ankle")  # 4 mm: ankle bracket wall (13.5) -> shank bar
    xl330_dummy("xl330_dummy_shoulder_roll")   # x2
    xl330_dummy("xl330_dummy_neck")            # x1
    print(f"thigh bar {L1:.1f} mm, shank bar {L2:.1f} mm, pivots: thigh {P1H}->{P1K}, shank {P2K}->{P2A}")
    print("wrote", sorted(os.listdir(OUT)))
