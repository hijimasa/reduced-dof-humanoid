#!/usr/bin/env python3
# Copyright 2026 Masaaki Hijikata
# SPDX-License-Identifier: Apache-2.0
"""Printable drafts of the parts the parallel-link conversion adds (CadQuery).

All numbers are in the LEG frame of Microban's Onshape parts (femur_right.step,
tibia_right.step, ankle_dbl_block.step share it): x lateral (+ = outboard on the right
leg), y forward, z up, millimetres. Pivot layout (docs/parts_ja.md):

    hip pitch axis   (y  3.0, z 132.7)    knee axis (8.0, 70.7)    ankle axis (0.0, 9.5)
    thigh linkage    hip pivot ( 30.0, 144.7)  ->  knee pivot ( 35.0,  82.7)   bar 62.2 mm (in FRONT)
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
THIGH_BACK, THIGH_DOWN = -27.0, -12.0     # negative: the thigh linkage runs in FRONT of the thigh, pivots 12 mm up (r 29.5 from
                                          # the hip axis: outside the femur plates' swept radius of 25.8)
SHANK_BACK, SHANK_UP = 25.0, 25.0
P1H = (HIP[0] - THIGH_BACK, HIP[1] - THIGH_DOWN)      # thigh bar, hip end
P1K = (KNEE[0] - THIGH_BACK, KNEE[1] - THIGH_DOWN)    # thigh bar, knee end
P2K = (KNEE[0] - SHANK_BACK, KNEE[1] + SHANK_UP)      # shank bar, knee end
P2A = (ANKLE[0] - SHANK_BACK, ANKLE[1] + SHANK_UP)    # shank bar, ankle end
BAR_T, BAR_W, PIN_D = 3.0, 8.0, 2.2                    # bar thickness, width, M2 clearance
PLATE_X = 18.5          # outer face of the femur / tibia clevis plates
THIGH_BAR_X = (22.5, 25.5)   # outboard of the hip bracket's pivot plate (x 19.5..22), 0.5 mm clear: at hip
                             # flexion beyond 100 deg the (front) thigh bar sweeps across the knee axis
SHANK_BAR_X = (26.0, 29.0)   # one plane further out: at deep hip flexion the (front) thigh bar crosses
                             # back over the knee and would otherwise hit the shank bar's upper end
YOKE_ARM_X = (29.5, 32.5)    # knee yoke plate, outside both bars

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


def hip_block_integrated(step_dir, out_dir, side):
    """Microban's hip bracket (hip_dbl_block_{side}.step: the cap under the hip roll and pitch servos,
    with a front tab for the roll-axis idler) with the thigh bar's upper pivot grown out of it.
    Frame = the leg frame (x lateral, +x outboard on the right; y forward; z up), mapped into the
    bracket's STEP frame with p_step = diag(-1, -1, 1) p_leg + t (t from robot.xml).

    What the support has to clear (leg frame, right leg):
    - the femur's clevis plates at x 14.5..18.5, a wide fan around the hip axis (y 3, z 132.7)
      that sweeps everything in FRONT of the axis as the hip flexes (its front edge reaches r 40
      and more), and the region below/behind the axis as it extends; the quadrant behind and
      above the axis is never swept;
    - the pelvis-side hip bracket (hip.step): a spine within x +-9 (mostly +-6.5) over the roll
      servo, so the sides beyond x 11 are free;
    - the hip block rolls +-35 deg about the roll axis (y axis through x 0, z 132.7).
    So the support leaves the bracket's rear skirt BELOW the axis (a low bridge at y -30..-14,
    z 116..123, crossing under the plates where the femur never is), runs forward as an 18 mm tall
    rail OUTSIDE the plates (x 19..22), rises beside the axis (y -1..9, where the pelvis bracket's
    top plate, y -24..-1 / z 149..154, cannot meet it under roll) and carries the pivot
    P1H = (30, 144.7) on an upper plate in front. Everything inboard of the bar and in front of
    the axis is swept by the femur plates, so there is no shorter way from the bracket to the
    pivot; the detour is sized so the bar's pull (along the bar, nearly vertical) loads each
    member in-plane. The thigh bar sits at x 22.5..25.5, just outside this plate. The __main__
    report checks the part in CAD against the femur over the hip range and against the pelvis-
    side bracket over the roll range."""
    t = {"right": (0.0, -9.75, -105.71), "left": (27.0, -9.75, -105.71)}[side]
    sx = 1.0 if side == "right" else -1.0
    X = lambda x0, x1: (min(sx * x0, sx * x1), max(sx * x0, sx * x1))
    # 3 mm plates between the femur plates (18.5) and the thigh bar (22.5), 0.5 mm clear of each.
    # The bar pulls along its own length (nearly vertical, up to ~18 N = knee stall torque / 29.5 mm
    # lever), so every member is loaded in its strong, in-plane direction; the rail is 18 mm tall
    # to take the 0.9 N*m bending moment about x (about 7 MPa in PLA; 7 mm tall it would be 43 MPa).
    low = _box(*X(11.0, 22.0), -30.0, -14.0, 116.0, 123.0)      # under the plates, behind the axis: the femur is never there
    rail = _box(*X(19.0, 22.0), -30.0, 9.0, 116.0, 134.0)       # outside the plates, at or below the axis height
    riser = _box(*X(19.0, 22.0), -1.0, 9.0, 116.0, 154.0)       # beside the axis: clear of the pelvis bracket's top plate (y -24..-1, z 149..154) when rolled
    upper = _box(*X(19.0, 22.0), 0.0, P1H[0] + 4.0, 137.0, 154.0)
    eye = _cyl(4.5, (sx * 19.0, P1H[0], P1H[1]), (sx * 22.0, P1H[0], P1H[1]))
    sup = low.union(rail).union(riser).union(upper).union(eye)
    try:
        sup = sup.edges("|X").fillet(2.0)        # round the in-plane corners of the plate outline
    except Exception:
        pass
    sup = sup.cut(_cyl(PIN_D / 2, (sx * 15, P1H[0], P1H[1]), (sx * 30, P1H[0], P1H[1])))
    sup = sup.rotate((0, 0, 0), (0, 0, 1), 180).translate(t)           # leg -> bracket STEP frame
    blk = cq.importers.importStep(os.path.join(step_dir, f"hip_dbl_block_{side}.step"))
    part = blk.union(sup)
    assert len(part.solids().vals()) == 1, f"hip_block_integrated_{side}: support not fused ({len(part.solids().vals())} solids)"
    os.makedirs(out_dir, exist_ok=True)
    cq.exporters.export(part, f"{out_dir}/hip_block_integrated_{side}.stl")
    cq.exporters.export(part, f"{out_dir}/hip_block_integrated_{side}.step")
    return part

def knee_axle_cap():
    """Bolts onto the knee servo horn through the femur plate with the four M2 horn screws
    (PCD 12, lengthened to M2x12) and carries an Ø8 stub axle for the yoke, from the
    plate's outer face (x 18.5) out to x 34; M2 tapped hole in the tip for a retainer."""
    cap = (cq.Workplane("XY").circle(9.0).extrude(2.0)                   # 2 mm flange on the plate
           .faces(">Z").workplane().circle(4.0).extrude(13.5))           # Ø8 stub through the yoke plate at 29.5..32.5
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

def shoulder_arm_simple(side, abduct=30.0):
    """Shoulder bracket + upper arm as ONE stout printed part (original design; only Microban's
    interfaces are kept, from robot.xml and the STEP hole patterns). Frame = the humerus STEP
    frame (y = the robot's forward axis, the former roll axis).

    - shoulder-pitch horn: a 4 mm plate at x -52.2..-48.2 with the horn's PCD 12 M2 holes around
      (y 0, z 24.5) and a Ø7.5 centre hole. The screws are driven from the outboard side, so Ø5
      access holes continue through the body (stepped hole: head on the plate, driver through).
    - elbow servo: Microban's humerus-plate pattern, 25.4 x 16 (x -45.4 / -20.0, y +-8), through a
      5 mm plate whose underside has a 1.5 mm pocket (26.4 x 20.4) that locates the servo's end;
      the servo's end face stays at z -2.5 as upstream.
    The arm (elbow plate + 16 x 16 post) is turned by the abduction angle about the former roll
    axis (x -32.7, z 24.5; right -abduct, left +abduct as in the model); a 24 x 18 x 21 body joins
    the horn plate to the post. Walls 3 mm, outer edges chamfered, no overhangs when printed on
    the elbow plate."""
    cx, zc = -32.7, 24.5
    # elbow servo mount, as on Microban's humerus plate: the XL330 has no holes in its end face; it is
    # held by the top corner holes of its HORN-side face (x -44.2) and idler-side face, screws along
    # the horn axis (x). The plate's holes are at x -45.4 and -20.0, (y +-8, z -4.5): two short tabs
    # hang from the top plate, 2.4 mm thick, outside each face, with Ø2.2 holes along x.
    top = _box(-47.5, -17.5, -11.5, 11.5, -2.5, 1.5).edges("|Z").chamfer(1.5)
    tab_h = _box(-46.6, -44.2, -11.5, 11.5, -8.0, -2.5)        # horn side: holes at x -45.4
    tab_i = _box(-20.6, -18.2, -11.5, 11.5, -8.0, -2.5)        # idler side: holes at x -20.0
    elbow = top.union(tab_h).union(tab_i)
    for y in (-8.0, 8.0):
        elbow = elbow.cut(_cyl(1.1, (-48.0, y, -4.5), (-43.0, y, -4.5))).cut(_cyl(1.1, (-22.0, y, -4.5), (-17.0, y, -4.5)))
    post = _box(cx - 8.0, cx + 8.0, -8.0, 8.0, 0.0, 32.0).cut(_box(cx - 5.0, cx + 5.0, -5.0, 5.0, 3.0, 29.0))
    arm = elbow.union(post).rotate((cx, 0, zc), (cx, 1, zc), -abduct if side == "right" else abduct)
    plate = _box(-52.2, -48.2, -11.0, 11.0, 12.0, 37.0).edges("|X").chamfer(1.5)
    body = _box(-48.2, -24.0, -9.0, 9.0, 14.0, 35.0).cut(_box(-45.0, -27.0, -5.0, 5.0, 18.0, 31.0))
    part = plate.union(body).union(arm)
    part = part.cut(_cyl(3.75, (-53, 0, zc), (-47, 0, zc)))                      # horn centre boss
    for (y, z) in ((6.0, zc), (-6.0, zc), (0.0, 18.5), (0.0, 30.5)):
        part = part.cut(_cyl(1.1, (-53.0, y, z), (-47.0, y, z)))                # M2 clearance in the plate
        part = part.cut(_cyl(2.5, (-48.2, y, z), (-16.0, y, z)))                # driver access through body and post
    for r in (1.5, 1.0, 0.6):                                                   # round the outer edges; OCC may refuse the larger radii
        try:
            part = part.edges("not %CIRCLE").fillet(r); break
        except Exception:
            continue
    cq.exporters.export(part, f"{OUT}/shoulder_arm_simple_{side}.stl")
    cq.exporters.export(part, f"{OUT}/shoulder_arm_simple_{side}.step")
    return part

def head_integrated(step_dir, out_dir):
    """Microban's head with the neck grown out of it as one printed part. The servo-horn joint on
    the head's underside (PCD 12 holes, recess) is filled by a Ø24 pad and a plain Ø18 stalk
    centred on the former servo axis runs down into the pocket base, so nothing of the old horn
    interface remains. Frame = the head STEP frame, which is the trunk-top frame; the neck servo
    sat at (36, 7.79, 167.9) with its axis +z: pocket floor z 148.4, trunk top surface z 155.9,
    head underside z 174.4. Fixed from inside the trunk by the four M2 screws of the horn's PCD 12."""
    head = cq.importers.importStep(os.path.join(step_dir, "head.step"))
    ax = (36.0, 7.79)
    base = _box(-9.8, 9.8, -24.3, 9.3, -19.5, -12.0).cut(_box(-8.2, 8.2, -22.7, 7.7, -17.5, -12.0)).translate((ax[0], ax[1], 167.9))
    stalk = _cyl(9.0, (ax[0], ax[1], 155.9 - 0.5), (ax[0], ax[1], 175.0)).cut(_cyl(6.0, (ax[0], ax[1], 158.0), (ax[0], ax[1], 172.5)))
    pad = _cyl(12.0, (ax[0], ax[1], 174.4), (ax[0], ax[1], 178.0))
    part = head.union(pad).union(stalk).union(base)
    for (dx, dy, dz) in _pcd_points(6.0, 4, 0.0, "z"):
        part = part.cut(_cyl(0.8, (ax[0] + dx, ax[1] + dy, 147.5), (ax[0] + dx, ax[1] + dy, 154.5)))   # M2 taps, screws from inside the trunk
    os.makedirs(out_dir, exist_ok=True)
    cq.exporters.export(part, f"{out_dir}/head_integrated.stl")
    cq.exporters.export(part, f"{out_dir}/head_integrated.step")
    return part


if __name__ == "__main__":
    L1, L2 = dist(P1H, P1K), dist(P2K, P2A)
    assert abs(L1 - dist(HIP, KNEE)) < 1e-6 and abs(L2 - dist(KNEE, ANKLE)) < 1e-6, "parallelogram: bar = front link"
    bar(L1, "thigh_bar")           # x2 (one per leg, outboard)
    bar(L2, "shank_bar")           # x2 (one per leg, outboard)
    knee_yoke("right"); knee_yoke("left")
    knee_axle_cap()                # x2
    standoff(SHANK_BAR_X[0] - 13.5, "standoff_shank_ankle")  # 9 mm: ankle bracket wall (13.5) -> shank bar
    # shoulder roll: the horn pattern is indexed +30 deg (right) / -30 deg (left) about the roll
    # axis, which is the block's +z in both arms, so the arm hangs 30 deg abducted. (The humerus
    # is turned -30 / +30 in the model; the horn stays with the shoulder bracket.)
    xl330_dummy("xl330_dummy_shoulder_roll_right", horn_rot=30.0)
    xl330_dummy("xl330_dummy_shoulder_roll_left", horn_rot=-30.0)
    xl330_dummy("xl330_dummy_neck")            # x1
    xl330_dummy("xl330_dummy_hip_yaw")         # x2, the 12-servo kit's standard; swap for real XL330s to get 14
    ankle_hinge_dummy()                        # x2
    vols = {"shoulder_arm_simple_right": shoulder_arm_simple("right").val().Volume(),
            "shoulder_arm_simple_left": shoulder_arm_simple("left").val().Volume(),
            "xl330_dummy (shell)": xl330_dummy("xl330_dummy_hip_yaw").val().Volume()}
    # optional: parts derived from Microban's CAD (second argument = path to microban/cad/step or to foot.step)
    if len(sys.argv) > 2 and os.path.exists(sys.argv[2]):
        step_dir = sys.argv[2] if os.path.isdir(sys.argv[2]) else os.path.dirname(sys.argv[2])
        derived = os.path.join(os.path.dirname(OUT.rstrip("/")) or ".", "parts_derived")
        foot_sole80(os.path.join(step_dir, "foot.step"), derived)
        for side in ("right", "left"):
            vols[f"ankle_block_integrated_{side}"] = ankle_block_integrated(step_dir, derived, side).val().Volume()
            vols[f"hip_block_integrated_{side}"] = hip_block_integrated(step_dir, derived, side).val().Volume()
        vols["head_integrated"] = head_integrated(step_dir, derived).val().Volume()
        for n in ("ankle_dbl_block", "hip_dbl_block_right", "head"):
            vols[n + " (original)"] = cq.importers.importStep(os.path.join(step_dir, n + ".step")).val().Volume()
        print("wrote the 80 mm foot, the integrated ankle blocks and the head into parts_derived/")
    print("volumes [cm3]:", {k: round(v / 1000, 2) for k, v in vols.items()})
    print(f"thigh bar {L1:.1f} mm, shank bar {L2:.1f} mm, pivots: thigh {P1H}->{P1K}, shank {P2K}->{P2A}")
    print("wrote", sorted(os.listdir(OUT)))
