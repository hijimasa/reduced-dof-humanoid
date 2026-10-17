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

def xl330_dummy(name, horn=True, horn_rot=0.0):
    """Placeholder block with the XL330's envelope (20 x 34 x 26, horn axis at the origin,
    horn face at z = -19.5) for a servo position that is no longer driven: shoulder roll,
    neck. Back-face holes 4 x Ø2.2 on a 16 x 25.5 mm rectangle as read from the
    brackets' STEP files; verify against the real servo before printing.
    horn_rot turns the fixed horn's hole pattern about the axis [deg]: the shoulder-roll
    blocks use +-30 so the arm hangs abducted 30 deg (see make_microban_pl.py --arm-abduct)."""
    body = cq.Workplane("XY").box(20, 34, 26, centered=(True, False, False)).translate((0, -24.5, -19.5))
    body = body.cut(cq.Workplane("XY").box(16, 30, 22, centered=(True, False, False)).translate((0, -22.5, -17.5)))   # 2 mm shell, ~7 g
    body = body.faces("<Y").workplane().pushPoints([(-8, -8.0), (8, -8.0), (-8, 17.5), (8, 17.5)]).hole(2.2, 8)
    if horn:   # fixed horn disc: PCD 12 M2 holes and the Ø7.2 centre boss, as on the idle horn
        c, s_ = math.cos(math.radians(horn_rot)), math.sin(math.radians(horn_rot))
        pts = [(6 * c, 6 * s_), (-6 * c, -6 * s_), (-6 * s_, 6 * c), (6 * s_, -6 * c)]
        disc = cq.Workplane("XY").circle(8.0).extrude(-3.0).translate((0, 0, -19.5))
        disc = disc.faces("<Z").workplane().pushPoints(pts).hole(1.6)
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


# ---------------------------------------------------------------------------------------------
# Integrated parts: the printed servo-shaped spacers merged with the part they bolt to.
# Geometry of the interfaces comes from Microban's robot.xml (servo poses) and STEP files.
# The ankle block and upper arm are derived from Microban's CAD (CC BY-NC-SA 4.0) and are
# written to parts_derived/; the neck post is an original part.
# ---------------------------------------------------------------------------------------------
def _cyl(r, p0, p1):
    """Solid cylinder of radius r from point p0 to point p1 (mm)."""
    v = cq.Vector(*p1) - cq.Vector(*p0)
    return cq.Workplane("XY").add(cq.Solid.makeCylinder(r, v.Length, cq.Vector(*p0), v.normalized()))

def _box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))

def _pcd_points(r, n, rot_deg, axis):
    """n points on a circle of radius r in the plane normal to `axis` ('x','y' or 'z'), rotated rot_deg."""
    pts = []
    for k in range(n):
        a = math.radians(rot_deg + 360.0 * k / n)
        u, v = r * math.cos(a), r * math.sin(a)
        pts.append({"x": (0, u, v), "y": (v, 0, u), "z": (u, v, 0)}[axis])
    return pts

def ankle_block_integrated(step_dir, out_dir, side):
    """Microban's ankle_dbl_block (the cap that ties the two ankle servos together) merged
    with a printed hinge body in the place of the ankle-pitch servo.

    Frame = the block's own STEP frame (x lateral, y forward, z up); this is also the leg
    frame: pitch axis along x through (y 0, z 9.5), roll axis along y through (x 0, z 9.5).
    The pitch servo occupied x -11.5..14.5, y +-10, z 0..34 with its horn at x -11.5 (horn
    disc -11.5..-14.5) and its idle cap at x 14.5; the tibia plates sit at |x| 14.5..18.5 with
    Ø6 holes on the axis. The cap bolted to the servo's top; here the hinge body is fused to it.
    The same part orientation is used in both legs (the block is not mirrored upstream), so
    the horn side (-x) is inboard on the right leg and outboard on the left; only the shank-bar
    pivot boss changes side: outboard = +x on the right leg, -x on the left."""
    cap = cq.importers.importStep(os.path.join(step_dir, "ankle_dbl_block.step"))
    body = _box(-9.0, 11.5, -6.0, 6.0, 9.5, 34.0)             # column from the hinge boss up to the cap
    pocket = _box(-7.4, 9.9, -4.4, 4.4, 14.0, 31.0)           # hollow, 1.6 mm walls
    web = _box(-9.0, 11.5, -1.0, 1.0, 9.5, 34.0)              # centre web
    boss = _cyl(6.0, (-11.5, 0, 9.5), (14.0, 0, 9.5))         # Ø12 hinge boss between the plates
    collar_h = _cyl(6.0, (-11.5, 0, 9.5), (-14.2, 0, 9.5))    # horn-side spacer, 0.3 mm off the plate (14.5)
    stub_h = _cyl(2.95, (-11.5, 0, 9.5), (-20.0, 0, 9.5))     # through the 4 mm plate + 1.5 for an M2 washer
    collar_i = _cyl(6.0, (14.0, 0, 9.5), (14.2, 0, 9.5))
    stub_i = _cyl(2.95, (14.0, 0, 9.5), (20.0, 0, 9.5))
    part = cap.union(body.cut(pocket)).union(web).union(boss).union(collar_h).union(stub_h).union(collar_i).union(stub_i)
    sx = 1.0 if side == "right" else -1.0                     # outboard side for the shank-bar pivot
    piv = _cyl(3.0, (sx * 12.5, P2A[0], P2A[1]), (sx * (SHANK_BAR_X[0] - 0.5), P2A[0], P2A[1]))
    gus = _box(min(sx * 12.5, sx * 13.5), max(sx * 12.5, sx * 13.5), P2A[0] - 3, P2A[0] + 3, 31.0, P2A[1] + 3)
    part = part.union(piv).union(gus)
    part = part.cut(_cyl(PIN_D / 2, (sx * 10, P2A[0], P2A[1]), (sx * 30, P2A[0], P2A[1])))     # M2 clearance through the boss
    part = part.cut(_cyl(0.8, (-21, 0, 9.5), (-15, 0, 9.5))).cut(_cyl(0.8, (15, 0, 9.5), (21, 0, 9.5)))   # M2 tap drills in the stub tips
    os.makedirs(out_dir, exist_ok=True)
    cq.exporters.export(part, f"{out_dir}/ankle_block_integrated_{side}.stl")
    cq.exporters.export(part, f"{out_dir}/ankle_block_integrated_{side}.step")
    return part

def upper_arm_integrated(step_dir, out_dir, side, abduct=30.0):
    """Microban's humerus plate merged with a column in the place of the shoulder-roll servo.

    Frame = the humerus STEP frame. The roll servo occupied x -42.7..-22.7, y -11.5..14.5,
    z 0..34 with its horn axis along +y (the robot's forward axis), horn face at y -11.5
    (horn disc to -14.5, bolted to the shoulder bracket) and idle cap at y 14.5 (in the
    bracket's other plate). The column carries a fixed horn disc whose PCD 12 pattern is
    turned by +-abduct about +y, so the arm hangs abducted (right +, left -: the model turns
    the right humerus -30 deg about this axis, the pattern that stays with the bracket
    therefore appears at +30 in the arm's frame), and an idle-side stub into the bracket."""
    hum = cq.importers.importStep(os.path.join(step_dir, "humerus.step"))
    cx, zc = -32.7, 24.5
    col = _box(cx - 6.0, cx + 6.0, -8.0, 11.5, 0.0, 30.0)
    hollow = _box(cx - 4.4, cx + 4.4, -6.4, 9.9, 2.0, 28.0)
    web = _box(cx - 6.0, cx + 6.0, -1.0, 1.0, 0.0, 30.0)
    hub = _cyl(6.0, (cx, -9.5, zc), (cx, 13.0, zc))                 # Ø12 hub through the column on the axis
    disc = _cyl(8.0, (cx, -9.5, zc), (cx, -14.5, zc))               # fixed horn: disc out to the bracket's inner face
    spig = _cyl(3.6, (cx, -14.5, zc), (cx, -16.0, zc))              # Ø7.2 spigot into the bracket's horn hole
    idle = _cyl(2.95, (cx, 13.0, zc), (cx, 18.5, zc))               # idle-side stub through the bracket plate
    part = hum.union(col.cut(hollow)).union(web).union(hub).union(disc).union(spig).union(idle)
    rot = abduct if side == "right" else -abduct
    for (dx, dy, dz) in _pcd_points(6.0, 4, rot, "y"):
        part = part.cut(_cyl(0.8, (cx + dx, -17.0, zc + dz), (cx + dx, -8.0, zc + dz)))        # M2 tap drills, PCD 12
    part = part.cut(_cyl(0.8, (cx, 19.0, zc), (cx, 13.5, zc)))                                 # M2 tap in the idle stub
    os.makedirs(out_dir, exist_ok=True)
    cq.exporters.export(part, f"{out_dir}/upper_arm_integrated_{side}.stl")
    cq.exporters.export(part, f"{out_dir}/upper_arm_integrated_{side}.step")
    return part

def neck_post(name="neck_post"):
    """Replaces the neck servo, which Microban mounts upside down: its horn is bolted to the
    floor of a pocket in the trunk top (z -19.5 in the servo's frame, PCD 12 M2) and the head
    bolts to the servo's idle side (z 6.5, PCD 12). Frame = the servo mesh frame (horn axis
    +z). Lower block fills the pocket (7.5 mm deep) for location; a hollow stalk carries the
    head at the same height as before; tapped holes top and bottom."""
    base = _box(-9.8, 9.8, -24.3, 9.3, -19.5, -12.0).cut(_box(-8.2, 8.2, -22.7, 7.7, -17.5, -12.0))   # 2 mm floor, 1.6 mm walls
    stalk = _cyl(9.0, (0, -7.5, -12.0), (0, -7.5, 6.5)).cut(_cyl(6.0, (0, -7.5, -10.0), (0, -7.5, 4.5)))
    top = _cyl(8.0, (0, 0, 3.5), (0, 0, 6.5)).union(_box(-8, 8, -15.5, 0, 3.5, 6.5))   # head seat on the axis, bridged to the stalk
    part = base.union(stalk).union(top)
    for (dx, dy, dz) in _pcd_points(6.0, 4, 0.0, "z"):
        part = part.cut(_cyl(0.8, (dx, dy, -20), (dx, dy, -13)))      # bottom: M2 taps on the horn's PCD 12 (trunk floor)
        part = part.cut(_cyl(0.8, (dx, dy, 0.5), (dx, dy, 7)))        # top: M2 taps on PCD 12 for the head
    cq.exporters.export(part, f"{OUT}/{name}.stl"); return part


if __name__ == "__main__":
    L1, L2 = dist(P1H, P1K), dist(P2K, P2A)
    assert abs(L1 - dist(HIP, KNEE)) < 1e-6 and abs(L2 - dist(KNEE, ANKLE)) < 1e-6, "parallelogram: bar = front link"
    bar(L1, "thigh_bar")           # x2 (one per leg, outboard)
    bar(L2, "shank_bar")           # x2 (one per leg, outboard)
    knee_yoke("right"); knee_yoke("left")
    knee_axle_cap()                # x2
    standoff(SHANK_BAR_X[0] - 13.5, "standoff_shank_ankle")  # 9 mm: ankle bracket wall (13.5) -> shank bar
    hip_front_ear()
    # shoulder roll: the horn pattern is indexed +30 deg (right) / -30 deg (left) about the roll
    # axis, which is the block's +z in both arms, so the arm hangs 30 deg abducted. (The humerus
    # is turned -30 / +30 in the model; the horn stays with the shoulder bracket.)
    xl330_dummy("xl330_dummy_shoulder_roll_right", horn_rot=30.0)
    xl330_dummy("xl330_dummy_shoulder_roll_left", horn_rot=-30.0)
    xl330_dummy("xl330_dummy_neck")            # x1
    xl330_dummy("xl330_dummy_hip_yaw")         # x2, the 12-servo kit's standard; swap for real XL330s to get 14
    ankle_hinge_dummy()                        # x2
    neck_post()                                # x1, integrated replacement for xl330_dummy_neck
    # optional: parts derived from Microban's CAD (second argument = path to microban/cad/step or to foot.step)
    if len(sys.argv) > 2 and os.path.exists(sys.argv[2]):
        step_dir = sys.argv[2] if os.path.isdir(sys.argv[2]) else os.path.dirname(sys.argv[2])
        derived = os.path.join(os.path.dirname(OUT.rstrip("/")) or ".", "parts_derived")
        foot_sole80(os.path.join(step_dir, "foot.step"), derived)
        vols = {}
        for side in ("right", "left"):
            vols[f"ankle_block_integrated_{side}"] = ankle_block_integrated(step_dir, derived, side).val().Volume()
            vols[f"upper_arm_integrated_{side}"] = upper_arm_integrated(step_dir, derived, side).val().Volume()
        for n in ("ankle_dbl_block", "humerus"):
            vols[n + " (original)"] = cq.importers.importStep(os.path.join(step_dir, n + ".step")).val().Volume()
        vols["neck_post"] = neck_post().val().Volume()
        vols["xl330_dummy (shell)"] = xl330_dummy("xl330_dummy_hip_yaw").val().Volume()
        print("wrote the 80 mm foot, the integrated ankle blocks and upper arms into parts_derived/")
        print("volumes [cm3]:", {k: round(v / 1000, 2) for k, v in vols.items()})
    print(f"thigh bar {L1:.1f} mm, shank bar {L2:.1f} mm, pivots: thigh {P1H}->{P1K}, shank {P2K}->{P2A}")
    print("wrote", sorted(os.listdir(OUT)))
