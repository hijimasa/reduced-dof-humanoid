#!/usr/bin/env python3
# Copyright 2026 Masaaki Hijikata
# SPDX-License-Identifier: Apache-2.0
"""Printable drafts of the parts the parallel-link conversion adds (CadQuery).

All numbers are in the LEG frame of Microban's Onshape parts (femur_right.step,
tibia_right.step, ankle_dbl_block.step share it): x lateral (+ = outboard on the right
leg), y forward, z up, millimetres. Pivot layout (docs/parts_ja.md):

    hip pitch axis   (y  3.0, z 132.7)    knee axis (8.0, 70.7)    ankle axis (0.0, 9.5)
    thigh linkage    hip pivot ( 25.0, 142.7)  ->  knee pivot ( 30.0,  80.7)   bar 62.2 mm (in FRONT)
    shank linkage    knee pivot (-17.0,  95.7)  ->  ankle pivot (-25.0, 34.5)  bar 61.7 mm (behind)

These are DRAFTS for fit-checking against the originals in Onshape, not release parts:
the screw holes that mate with Microban's brackets are placed from the STEP files'
hole patterns, but nothing here has been printed.

usage: <venv with cadquery>/bin/python tools/make_parts.py model/parts [path/to/microban/cad/step/foot.step]
"""
import math, os, sys
import cadquery as cq

OUT = sys.argv[1] if len(sys.argv) > 1 else "model/parts"
os.makedirs(OUT, exist_ok=True)

# ---- layout (mm, leg frame) --------------------------------------------------------
HIP, KNEE, ANKLE = (3.0, 132.7), (8.0, 70.7), (0.0, 9.5)
THIGH_BACK, THIGH_DOWN = -22.0, -10.0     # negative: the thigh linkage runs in FRONT of the thigh, pivots 10 mm up
SHANK_BACK, SHANK_UP = 25.0, 25.0
P1H = (HIP[0] - THIGH_BACK, HIP[1] - THIGH_DOWN)      # thigh bar, hip end
P1K = (KNEE[0] - THIGH_BACK, KNEE[1] - THIGH_DOWN)    # thigh bar, knee end
P2K = (KNEE[0] - SHANK_BACK, KNEE[1] + SHANK_UP)      # shank bar, knee end
P2A = (ANKLE[0] - SHANK_BACK, ANKLE[1] + SHANK_UP)    # shank bar, ankle end
BAR_T, BAR_W, PIN_D = 3.0, 8.0, 2.2                    # bar thickness, width, M2 clearance
PLATE_X = 18.5          # outer face of the femur / tibia clevis plates
THIGH_BAR_X = (21.0, 24.0)   # outboard of the knee axle cap flange (x 18.5..20.5), 0.5 mm clear: at hip
                             # flexion beyond 100 deg the (front) thigh bar sweeps across the knee axis
SHANK_BAR_X = (24.5, 27.5)   # one plane further out: at deep hip flexion the (front) thigh bar crosses
                             # back over the knee and would otherwise hit the shank bar's upper end
YOKE_ARM_X = (28.0, 31.0)    # knee yoke plate, outside both bars

def dist(p, q): return math.hypot(q[0] - p[0], q[1] - p[1])

def bar(length, name):
    """Flat link: two Ø2.2 holes `length` apart, rounded ends, 3 x 8 mm section."""
    b = (cq.Workplane("XY").slot2D(length + BAR_W, BAR_W).extrude(BAR_T)
         .faces(">Z").workplane().pushPoints([(-length / 2, 0), (length / 2, 0)]).hole(PIN_D))
    cq.exporters.export(b, f"{OUT}/{name}.stl"); return b

def knee_yoke(side="right"):
    """One flat plate OUTBOARD of the leg (x 20.5..23.5 on the right) that ties the knee axle to
    the two rear-link pivots: a Ø8.2 bore on the knee axis, the thigh pin at P1K and the
    shank pin at P2K, each pin reaching inboard to its bar. No block behind the knee and no
    inner bar: the first draft's crossbar ran straight through the bars' paths and the inner
    thigh bars of the two legs were only 8 mm apart, so both were dropped. The plate sits
    below the hanging forearm (which starts at z 112) and outside the femur/tibia plates."""
    sx = 1 if side == "right" else -1
    x0, x1 = (YOKE_ARM_X if sx > 0 else (-YOKE_ARM_X[1], -YOKE_ARM_X[0]))
    pts = [KNEE, P1K, P2K]
    # plate outline: the triangle of the three pivots offset by 5 mm, plus a Ø16 boss round the
    # knee bore. 5 mm (not 7) keeps the top of the plate 2 mm lower: in the walk the hanging
    # forearm passes 3 mm above it when the hip is flexed 45 deg
    tri = cq.Workplane("YZ").workplane(offset=x0).polyline(pts).close().offset2D(5.0).extrude(x1 - x0)
    tri = tri.union(cq.Workplane("YZ").workplane(offset=x0).center(*KNEE).circle(8.0).extrude(x1 - x0))
    plate = tri.cut(cq.Workplane("YZ").center(*KNEE).circle(4.1).extrude(60, both=True))
    # thigh pin: Ø2 (M2 screw) through the plate, with an integral Ø6 spacer down to the bar
    # plane (thigh bar at THIGH_BAR_X, shank bar at SHANK_BAR_X: the spacer fills the gap)
    for p, bar_x in ((P1K, THIGH_BAR_X), (P2K, SHANK_BAR_X)):
        gap = (x0 - bar_x[1] if sx > 0 else bar_x[1] - abs(x1)) - 0.5   # between the bar and the plate, 0.5 mm clearance
        if gap > 0.1:
            sp = cq.Workplane("YZ").workplane(offset=(x0 - gap) if sx > 0 else -abs(x1)).center(*p).circle(3.0).extrude(gap)
            plate = plate.union(sp)
        plate = plate.cut(cq.Workplane("YZ").center(*p).circle(PIN_D / 2).extrude(60, both=True))
    cq.exporters.export(plate, f"{OUT}/knee_yoke_{side}.stl"); return plate


def hip_front_ear():
    """Carries the thigh bar's upper pivot, P1H = 22 mm in front of and 10 mm above the hip
    pitch axis, where nothing of Microban's exists: a 3 mm plate against the outer face of the
    hip bracket's side wall (x 13.5), reaching forward past the bracket (y <= 17) to the pivot,
    with a Ø6 boss out to the bar plane. The two screw positions into the bracket wall are
    placeholders to be set against the real part."""
    pl = (cq.Workplane("YZ").workplane(offset=13.5)
          .polyline([(2, 124), (2, 140), (P1H[0] + 6, P1H[1] + 5), (P1H[0] + 6, P1H[1] - 6), (14, 124)]).close().extrude(3.0))
    boss = cq.Workplane("YZ").workplane(offset=16.5).center(*P1H).circle(3.0).extrude(THIGH_BAR_X[0] - 0.5 - 16.5)
    ear = pl.union(boss).cut(cq.Workplane("YZ").center(*P1H).circle(PIN_D / 2).extrude(40, both=True))
    for y, z in ((6, 128), (6, 137)):
        ear = ear.cut(cq.Workplane("YZ").center(y, z).circle(1.1).extrude(40, both=True))
    cq.exporters.export(ear, f"{OUT}/hip_front_ear.stl"); return ear


def knee_axle_cap():
    """Bolts onto the knee servo horn through the femur plate with the four M2 horn screws
    (PCD 12, lengthened to M2x12) and carries an Ø8 stub axle for the yoke, from the
    plate's outer face (x 18.5) out to x 32.5; M2 tapped hole in the tip for a retainer."""
    cap = (cq.Workplane("XY").circle(9.0).extrude(2.0)                   # 2 mm flange on the plate
           .faces(">Z").workplane().circle(4.0).extrude(12.0))           # Ø8 stub through the yoke plate at 28..31
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

def ankle_hinge_dummy(name="xl330_dummy_ankle_hinge"):
    """Goes where the ankle-pitch servo was, inside the unchanged ankle_dbl_block: the XL330
    envelope with the bracket's back-face holes, and in place of horn and idler two Ø5.9
    stub axles that ride in the tibia plates' existing Ø6 holes (plates are 4.5 mm off the
    servo faces, 4 mm thick). Each stub is retained from outside by an M2 screw and washer
    (M2 tap drill in the stub). The shank bar's pivot is NOT on this part: it is an M2
    hole added to the ankle bracket's side wall at (y -25.0, z 34.5)."""
    body = cq.Workplane("XY").box(20, 34, 26, centered=(True, False, False)).translate((0, -24.5, -19.5))
    body = body.faces("<Y").workplane().pushPoints([(-8, -8.0), (8, -8.0), (-8, 17.5), (8, 17.5)]).hole(2.2, 8)
    # The clevis is not symmetric about the servo: on the horn side the plate's inner face is
    # 4.5 mm off the servo face (horn 3 mm + 1.5), on the idler side the plate sits directly
    # on the servo face (the idle cap lives in the plate's hole). Spacers follow suit.
    for z_face, direction, gap in ((-19.5, -1, 4.5), (6.5, 1, 0.5)):
        spacer = cq.Workplane("XY").circle(6.0).extrude(gap * direction).translate((0, 0, z_face))
        stub = cq.Workplane("XY").circle(2.95).extrude(5.5 * direction).translate((0, 0, z_face + gap * direction))  # through the 4 mm plate + 1.5
        body = body.union(spacer).union(stub)
    body = body.cut(cq.Workplane("XY").circle(0.8).extrude(60, both=True))   # M2 tap drill along the axis, both stubs
    cq.exporters.export(body, f"{OUT}/{name}.stl"); return body


def foot_sole80(step_path, out_dir, sole=80.0):
    """Microban's foot with the toe end of the sole plate cut back so the ground-contact
    plate is `sole` mm long (ROBO-ONE Light: <= 55 % of the leg length; 90 mm on this
    146 mm leg is too long). The ankle bracket sits over the rear of the plate, so only
    the toe is cut. A derivative of Microban's CAD: CC BY-NC-SA 4.0, see
    model/parts_derived/LICENSE."""
    f = cq.importers.importStep(step_path)
    bb = f.val().BoundingBox()                       # plate y -41.5 .. 48.5, y = forward
    keep_to = bb.ymin + sole
    short = f.cut(cq.Workplane("XY").box(200, 200, 200).translate((0, keep_to + 100, 0)))
    os.makedirs(out_dir, exist_ok=True)
    cq.exporters.export(short, f"{out_dir}/foot_sole{int(sole)}.stl")
    cq.exporters.export(short, f"{out_dir}/foot_sole{int(sole)}.step")
    return short


if __name__ == "__main__":
    L1, L2 = dist(P1H, P1K), dist(P2K, P2A)
    assert abs(L1 - dist(HIP, KNEE)) < 1e-6 and abs(L2 - dist(KNEE, ANKLE)) < 1e-6, "parallelogram: bar = front link"
    bar(L1, "thigh_bar")           # x2 (one per leg, outboard)
    bar(L2, "shank_bar")           # x2 (one per leg, outboard)
    knee_yoke("right"); knee_yoke("left")
    knee_axle_cap()                # x2
    standoff(SHANK_BAR_X[0] - 13.5, "standoff_shank_ankle")  # 9 mm: ankle bracket wall (13.5) -> shank bar
    hip_front_ear()
    xl330_dummy("xl330_dummy_shoulder_roll")   # x2
    xl330_dummy("xl330_dummy_neck")            # x1
    xl330_dummy("xl330_dummy_hip_yaw")         # x2, the 12-servo kit's standard; swap for real XL330s to get 14
    ankle_hinge_dummy()                        # x2
    # optional: the shortened foot, from a Microban checkout (second argument = path to cad/step/foot.step)
    if len(sys.argv) > 2 and os.path.exists(sys.argv[2]):
        foot_sole80(sys.argv[2], os.path.join(os.path.dirname(OUT.rstrip("/")) or ".", "parts_derived"))
        print("wrote the 80 mm foot into parts_derived/")
    print(f"thigh bar {L1:.1f} mm, shank bar {L2:.1f} mm, pivots: thigh {P1H}->{P1K}, shank {P2K}->{P2A}")
    print("wrote", sorted(os.listdir(OUT)))
