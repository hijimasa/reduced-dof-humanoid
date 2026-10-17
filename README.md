# reduced-dof-humanoid

[日本語版 README はこちら](README_ja.md)

Reduced-DOF humanoid for ROBO-ONE Light, derived from Rhoban's
[Microban](https://github.com/Rhoban/microban) (unofficial, not affiliated with
Rhoban or the author of Microban).

**Status: experimental, not yet built.** One generator script produces three
MuJoCo models of Microban. Where a servo is removed, the part it sat in is
reprinted with the function grown into it (ankle hinge, shoulder, neck); only
the hip-yaw position keeps a printed block of the servo's envelope, so the
14-servo upgrade is a drop-in. The printed parts are CAD drafts: they have been
checked against the model and against Microban's CAD, but not printed or
assembled.

| Model | Layout | Mass in the model |
| --- | --- | --- |
| 16-servo (intermediate step) | passive parallel-link ankle pitch, fixed head | 0.809 kg |
| **12-servo (standard build)** | additionally no shoulder roll and no hip yaw: legs 4 × 2, arms 2 × 2, printed XL330-shaped spacers where the hip-yaw servos were | 0.726 kg |
| **14-servo (upgrade)** | the 12-servo build with the two hip-yaw spacers swapped for real servos, so it turns on the spot instead of scuffing its soles | 0.756 kg |

Microban itself has 19 servos and weighs 0.819 kg in its model.

<p align="center">
  <img src="docs/images/preview12.png" alt="12-servo parallel-link Microban: side and 3/4 views, standing and crouched" width="640">
</p>

## Why

Pollen Robotics' Microduck walks with 15 servos and no arms or ankle roll,
driven by a learned policy. It suggests that the usual 19–20 servo layout of
small humanoids is more than a walking robot strictly needs. ROBO-ONE Light,
however, requires two actuated arms, a head, and 10 mm foot clearance when
walking in all four directions, so the question is where the axes can be cut
while staying inside those rules and while keeping keyframe-style control
practical. The study behind this repository is in
[docs/design_ja.md](docs/design_ja.md) (Japanese).

| Joint | Decision | Reason |
| --- | --- | --- |
| Ankle pitch | passive (done here) | Two parallelograms per leg keep the foot parallel to the pelvis, as on the `ashigaru` robot. |
| Neck | remove (done here) | Rules only require a separate head, not an actuated one. A fixed head keeps the humanoid silhouette at zero servo cost. |
| Hip yaw | optional: spacer in the 12-servo build, servo in the 14-servo upgrade | Without it the feet are always parallel and turning needs the soles to slip, which depends on the floor (Robovie-nano manages the same way). |
| Shoulder roll | remove (done here) | Rules require one actuated axis per arm; shoulder pitch + elbow is enough for punching and getting up. |
| Hip roll, hip pitch, knee, ankle roll | keep | Needed for lateral stepping, 10 mm foot lift and flat-footed weight transfer with position-controlled servos. |

Standard build: legs 4 × 2, arms 2 × 2, fixed head → 12 servos, 0.726 kg in the
model including the printed spacers. Upgrade: + 2 hip-yaw servos → 14 servos,
0.756 kg. Microban: 19 servos, 0.819 kg.

## What is in the repository

```
model/microban_pl12/     12-servo model (standard): upstream joint names, LME joint names, viewer scene
model/microban_pl14/     14-servo model (upgrade)
model/microban_pl/       16-servo model, plus Microban's meshes (unmodified, CC BY-NC-SA 4.0) in assets/
model/parts/             printable DRAFTS of the added parts (STL, from tools/make_parts.py)
model/parts_derived/     parts derived from Microban's CAD (CC BY-NC-SA 4.0): 80 mm foot, integrated ankle and hip blocks, head with neck, long forearm
tools/
  make_microban_pl.py    generates the models above from Microban's robot.xml
  make_parts.py          CadQuery script: link bars, knee yoke, axle cap, shoulder + arm, hip-yaw spacer, and the parts derived from Microban's STEP (ankle and hip blocks, head, forearm); reports CAD clearance checks
  verify_parallel_leg.py loop closure, foot-parallel sweep and standing tests
  check_clearance.py     sweeps hip/knee over their ranges and reports any part-vs-robot interference
  render_preview.py      renders the preview images
  render_parts_check.py  renders the added parts on a transparent robot (docs/images/parts_check.png)
  step_holes.py          reads hole patterns out of Microban's STEP parts
motion/                  the motion tools and robot profiles used for the verification below (copied from
                         LegacyMotionEditor's tools/ with paths adjusted; see motion/README.md), and the
                         generated LME projects in motion/projects/
docs/
  design_ja.md           DOF reduction study: ROBO-ONE Light rules, Microduck, per-joint decisions, results, cost (Japanese)
  parts_ja.md            design memo for the added parts: pivot layout, packaging constraints, part list, open fit checks (Japanese)
```

Each model directory holds `<name>.xml` (Microban's joint names), `<name>_lme.xml`
(LegacyMotionEditor's names) and `scene.xml` (floor and a standing keyframe).

### Fixed head

The head body and mesh are kept, but the neck yaw joint and its servo are
removed, so the head bolts directly to the trunk. Pass `--neck-motor` to the
generator to keep the servo.

### The parallel-link leg

Per leg, a passive knee block is inserted between the femur and the tibia and
two passive link bars close two parallelogram loops (`<connect>` constraints).
The knee block, and therefore the foot, stay parallel to the pelvis in pitch.
Consequences:

- The ankle-pitch joint is passive; its servo and horn can be removed from the
  physical robot.
- The knee servo now acts between the knee block and the tibia, so the knee
  command is the shank angle **relative to the pelvis**, not to the thigh.
  Existing Microban motions must be re-authored.
- Thigh linkage: one bar **behind** the thigh, pivots 22 mm behind and 8 mm
  below the hip and knee axes (singular at 110.6° flexion / 69.4° extension).
  The hip-side pivot then sits beside the hip bracket's rear skirt, in the
  quadrant the femur's clevis plates never sweep, on a short lug grown out of
  the bracket. Shank linkage: one bar behind the shank, pivots 30 mm behind and
  30 mm above the knee and ankle axes, which lifts the ankle pivot clear of the
  ankle-roll servo and the foot (singular at 135° / 45°). Joint ranges follow
  with an 8° margin: hip −100°…+62°, knee −37°…+127° (pelvis-relative).
- Every part that is new or changed for this robot is drawn in green (printed
  servo-shaped spacers in amber), including the shortened foot. The green parts also collide (convex hulls), so interference between
  the mechanism and the rest of the robot shows up in simulation;
  `tools/check_clearance.py` sweeps the ranges and reports none inside them.

### Parts that change on the printed robot

The generator edits the model to match the parts the physical conversion needs
(`--keep-ankle-servo`, `--neck-motor`, `--elbow-flexion-only` and `--arm-abduct` are the knobs):

| Part | Change | Effect in the model |
| --- | --- | --- |
| Ankle block | The ankle-pitch XL330 and its idler horn come out. Microban's ankle block (the cap over the two servos) is reprinted as one part with a hinge body in the servo's place: Ø5.9 stub axles on the pitch axis into the tibia's existing plate holes, and the shank bar's pivot support (root block, beam, arm) on the outboard side (`parts_derived/ankle_block_integrated_{right,left}`). | 47 g → 33 g per ankle. The knee folds to 132° before the ankle meets the hip bracket, instead of 117°. |
| Hip bracket | Reprinted with the thigh bar's upper pivot grown out of its rear skirt (`parts_derived/hip_block_integrated_{right,left}`). | +3.6 g per side. |
| Neck | The XL330 in the top of the trunk comes out (Microban mounts it upside down: horn in the trunk, head on the servo body). The head is reprinted with the neck grown out of it (`parts_derived/head_integrated`): the old horn seat on its underside is filled by a pad, a plain Ø18 stalk on the former servo axis runs down into a base that fits the servo pocket, and four M2 screws from inside the trunk hold it on the horn's PCD 12. The head stays where it was. | −17 g. |
| Forearm | Microban's forearm is lengthened by 40 mm with a Ø10 rod grown out of the hand (`parts_derived/radius_long`): the arm is then 183 mm from the shoulder axis, closer to KXR / Robovie-nano proportions and more reach for the get-ups. ROBO-ONE Light allows 26 cm from the shoulder and wants the hands 50 mm above the floor while walking: they hang 86 mm up with straight arms in the walking crouch, 114 mm with the elbows bent as the gaits use them. `--forearm-ext 0` keeps the original. | +4 g per arm. |
| Shoulder roll | The XL330 at the top of each humerus comes out. Shoulder bracket and upper arm become one stout printed part (`parts/shoulder_arm_simple_{right,left}`) that keeps only Microban's interfaces: the shoulder-pitch horn's PCD 12 plate, and the elbow servo held exactly as Microban's humerus plate holds it (an XL330 has no holes in its end face: four screws along the horn axis into the top corner holes of its horn-side and idler-side faces, through two short tabs at x −45.4 / −20.0), with the arm turned 30° about the former roll axis. A 24 × 18 body (3 mm walls, filleted edges) joins plate and 16 × 16 post; the four horn screws are reached through Ø5 stepped access holes along it. A hanging arm sits right over the knee's link stack; abducted, the forearm can be bent forward (60° in the gaits) like a walking human without touching it. `--arm-abduct` sets the angle. | −30 g. |
| Hip yaw (12-servo) | The two XL330s in the bottom of the trunk come out; spacers go in. The 14-servo upgrade puts the servos back. | −30 g. No turning on the spot. |
| Foot | The sole plate is cut back 10 mm at the toe to an 80 mm contact length (ROBO-ONE Light: at most 55 % of the leg length), `parts_derived/foot_sole80`. | Collision plates shortened to match. |
| Link bars | Per leg one thigh bar and one shank bar, both behind the leg (62.2 / 61.7 mm between pins), 3 × 8 mm, M2 pins, outboard of the leg in separate planes. | Close the parallelogram loops. |
| Knee yoke | A triangular plate outboard of the knee carrying both bar pivots, riding on a printed axle cap bolted to the knee servo horn. | 6 g. |

Total mass: 0.833 kg (upstream model) → 0.809 kg (16-servo) → 0.756 kg (14-servo) → 0.726 kg (12-servo). Printed parts are counted at 1.0 g/cm³ (PLA, three walls, about 40 % infill): integrated ankle block 17.3 cm³ (cap alone 9.2), hip bracket 13.3 cm³ (9.7), shoulder + upper arm 13.5 cm³, the neck grown out of the head 6.4 cm³, forearm extension 4 g, XL330-shaped hip-yaw spacer 7.7 cm³ (2 mm shell). `--dummies` builds the earlier variant with servo-shaped spacer blocks at the ankle, shoulder and neck instead; the hip-yaw position always keeps the spacer so the 14-servo upgrade stays a drop-in.

<p align="center">
  <img src="docs/images/integrated_parts.png" alt="Hip bracket lug, integrated ankle block, shoulder + arm and head with neck on a transparent robot" width="900">
</p>

<p align="center">
  <img src="docs/images/parts_hip_shoulder.png" alt="The integrated hip bracket, ankle block and shoulder + arm part on their own" width="900">
</p>

### Cost

From Microban's own bill of materials (servos $23.90 each, everything else
about $113: Raspberry Pi Zero 2W, Robot HAT, batteries, BMS, charger, switch,
SD card, screws, shims, filament). The conversion adds about $5 of M2 screws,
washers and filament for the new parts.

| Build | Servos | Servo cost | Total | vs Microban ($567) |
| --- | --- | --- | --- | --- |
| Microban | 19 | $454 | $567 | — |
| 12-servo standard | 12 | $287 | ≈ $405 | −$162 (−29 %) |
| 14-servo upgrade | 14 | $335 | ≈ $453 | −$114 (−20 %) |

The U2D2 configuration tools ($56) are the same for all three and are not
included. Prices are the BOM's at the time of writing; ROBOTIS' Japanese
distributors price the XL330 somewhat higher.

### The printed parts (drafts)

`tools/make_parts.py` (needs `cadquery`) writes STL drafts of everything the
conversion adds, laid out in the Onshape frame of Microban's leg parts so they
can be dropped into the original assembly for a fit check. With `--parts` the
model generator draws these STLs in place of its box placeholders, so the
mechanism shows up in any viewer that renders meshes (LegacyMotionEditor's
preview included); their lateral positions are measured from the leg's centre
plane, not from the body origins, which sit on the inboard clevis plate. The placement was
checked against the model with collision on (below, originals transparent):
link bars 3 × 8 mm with M2 pins stacked outboard of the leg in three planes
(shank bar 26–29 mm from the leg's centre plane, knee yoke plate 29.5–32.5 mm,
thigh bar 35–38 mm, outside the yoke and its axle stub so the rear bar can pass
the knee axis near the extension limit), the yoke on an axle cap bolted to the
knee servo horn, the thigh bar's upper pivot on a lug grown out of the hip bracket's rear
skirt (`parts_derived/hip_block_integrated`: a flange on the wall, a taper from
20 × 22 to 10 × 10 mm over 23 mm and a filleted root, so the root is not a
right angle), the shank bar's pivot grown out of the ankle block (a root block on the cap's
rear corner, an 8 × 8 mm beam routed outside the band the tibia's plates sweep,
and an arm outside the plates to the pin), and a dummy XL330 block only at the
(12-servo) hip-yaw position. The ankle's passive hinge reuses the tibia's
existing Ø6 plate holes. Both bracket extensions were checked in CAD against the
femur over the hip-pitch range and against the pelvis-side bracket over the
hip-roll range.

The layout went through several rounds, all checked by sweeping hip and knee
over their ranges with MuJoCo's collision detection (`tools/check_clearance.py`),
by replaying the gaits, and by CAD intersection checks of the bracket extensions
against the femur and the pelvis-side bracket (`tools/make_parts.py` reports
them). A front thigh bar was used for a while to keep the bar away from a
hanging forearm; with the arms abducted that reason disappeared, and the front
bar's pivot support had to detour around the femur's plates. The rear bar puts
the pivot right next to the hip bracket. With both bars behind the leg, the
thigh bar has to sit outside the yoke: inside it, the yoke's shank pin sweeps
through it, and the knee axle stub meets it beyond 57° of hip extension. No
contact remains inside the joint ranges, even with the arms hanging straight
down. Why the pivots sit where
they do, and what is still unverified, is in [docs/parts_ja.md](docs/parts_ja.md).

<p align="center">
  <img src="docs/images/parts_check.png" alt="Drafted parts placed on the model, originals transparent" width="640">
</p>

### Verification

`tools/verify_parallel_leg.py` checks the model in MuJoCo:

| Test | Result |
| --- | --- |
| Loop closure residual at the initial pose | 0.000 mm |
| Foot tilt relative to the trunk over 8 hip/knee combinations (gravity off) | ≤ 0.5° |
| Standing on a floor for 3 s, straight legs | trunk tilt 0.08° |
| Standing with hip −30° / knee +30° | trunk tilt 0.12°, feet flat |
| Standing with hip −60° / knee +60° | trunk tilt 0.33°, feet flat |
| Deepest symmetric squat before self-collision | 132° relative knee flexion (117° with the original ankle block) |

With LegacyMotionEditor's motion tools (keyframe walk re-solved through the
leg IK, get-ups found by physics beam search and re-timed per model), under
servo zero-offset and initial-tilt disturbances, 8 trials each:

| Action | 12-servo | 14-servo |
| --- | --- | --- |
| Walk forward / back, sidestep | 8/8 (about 32 mm/s) | 8/8 |
| Turn | 8/8 (scuffing, about 33° per cycle) | 8/8 (hip yaw, 72° / 141°) |
| Jab left / right | 8/8 | 8/8 |
| Get up from prone | 8/8 (6.7 s, nine poses) | 8/8 (same poses, re-timed: 5.2 s) |
| Get up from supine | 8/8 (5.1 s, eight poses; 40/40 over 40 trials) | 8/8 (same poses, re-timed: 4.8 s; 39/40) |

Both rises were searched on the final geometry (`motion/getup_supine.py
--pitch 90` and `--pitch -90`): with the 40 mm longer forearms the prone search
stopped falling into the kneeling local optimum that had caught fifteen of
sixteen seeds on the earlier geometry, and two of eight seeds stood with 5/8 and
6/8 before re-timing; the nine-pose rise from seed 72, re-timed by
`motion/tune_getup.py`, is the one in the profiles. It rolls the robot over and
stands it up facing the other way. From supine the robot sits up on its hands,
folds the hip to the new −100° limit (the rear thigh link's wider flexion
range), squats and stands: eight poses from seed 80, 4/8 at the search cadence
and 8/8 after re-timing, each build timed on its own model. The earlier supine
scheme (a one-pose roll onto the front, then the prone rise) held 7/8 and took
8 s; its roll poses and a prone-to-supine roll are kept in `motion/candidates/`. The
gaits walk with the elbows bent 60° forward and the arms abducted 30° by the
printed shoulder parts. The motion tools
(keyframe IK, perturbed simulation checks, gait and get-up tuners, the beam
search) and the robot profiles are in `motion/`, copied from LegacyMotionEditor
so that the tables can be reproduced from this repository alone:

```bash
LME_ROBOT=microban_12 python3 motion/build_project.py        # -> motion/projects/LME_microban_12_save.xml
LME_ROBOT=microban_12 python3 motion/sim_check.py --trials 8  # the table above
```

The walk is a statically stable keyframe gait; Microban's own learned walk
(commands up to 0.7 m/s) is a dynamic one and out of scope here.

With the original Microban model, commanding the same two hip/knee values
without adjusting the ankle tips the robot over; here the linkage keeps the
soles flat automatically.

The elbow keeps Microban's own symmetric ±125° range: the forearm clears the
upper arm either way, and the get-ups found so far push with the elbow bent
backward. Limiting it to forward flexion (`--elbow-flexion-only`) dropped the
get-up's success rate from 15/16 to at best 5/16 in the searches tried on an
earlier geometry.

## Usage

Requirements: Python 3.10+, `mujoco` (≥ 3.1), `numpy`; `Pillow` for rendering;
`cadquery` for the parts.

```bash
# view
python -m mujoco.viewer --mjcf model/microban_pl12/scene.xml      # 12-servo standard
python -m mujoco.viewer --mjcf model/microban_pl14/scene.xml      # 14-servo upgrade
# regenerate from a Microban checkout
python tools/make_microban_pl.py path/to/microban/src/model/mjcf/robot.xml \
    model/microban_pl12/microban_pl12.xml --dof 12 --meshdir ../microban_pl/assets --parts ../../parts \
    --scene model/microban_pl12/scene.xml
python tools/make_microban_pl.py path/to/microban/src/model/mjcf/robot.xml \
    model/microban_pl12/microban_pl12_lme.xml --dof 12 --naming lme --meshdir ../microban_pl/assets --parts ../../parts
# verify
python tools/verify_parallel_leg.py model/microban_pl12/microban_pl12.xml
python tools/check_clearance.py model/microban_pl12/microban_pl12.xml       # part interference sweep
# part drafts and preview images
python tools/make_parts.py model/parts path/to/microban/cad/step   # 2nd arg: also the parts derived from Microban's STEP
MUJOCO_GL=egl python tools/render_preview.py model/microban_pl12/scene.xml docs/images/preview12.png
MUJOCO_GL=egl python tools/render_parts_check.py model/microban_pl12/scene.xml docs/images/parts_check.png
```

Generator options: `--dof 16|14|12`; `--naming lme`; `--back`/`--tilt` (thigh
linkage, negative = in front / above) and `--shank-back`/`--shank-tilt` (shank
linkage) set the pivot geometry and the joint ranges follow; `--link-x` and
`--shank-link-x` the lateral planes of the bars;
`--clevis` the slimmed ankle bracket; `--parts <dir relative to meshdir>` draws
the STL drafts instead of boxes (`--derived` names the directory of the parts
derived from Microban's CAD); `--arm-abduct` the fixed abduction built into the
shoulder part; `--forearm-ext` the forearm extension; `--dummies` the earlier
variant with servo-shaped spacer blocks instead of the integrated parts;
`--keep-ankle-servo`, `--neck-motor`, `--keep-motor` (Microban's torque
actuators, for RL), `--elbow-flexion-only` restore or vary the rest.

For [LegacyMotionEditor](https://github.com/hijimasa/LegacyMotionEditor) use the
`*_lme.xml` files: the editor detects the four closed loops and hides the eight
passive joints from its sliders, and the passive joints carry names that are not
in LME's Meridim slot table, so nothing it publishes can collide with a servo.

## ROBO-ONE Light rule check

Measured on the model (leg length = hip-pitch axis to sole with the legs
straight, as the rules define it):

| Rule (1 kg class) | Limit here | Microban as is | This repository |
| --- | --- | --- | --- |
| Sole length ≤ 55 % of leg length (leg 146 mm) | 80.4 mm | 90 mm | 80 mm contact plate (`--sole-length`, default); the printed foot must be shortened to match |
| Sole width ≤ 35 % of leg length | 51.2 mm | 45 mm | 45 mm |
| Soles must not overlap seen from above | — | 27 mm apart | 27 mm apart |
| No squat walking: knee not closed to 90° or less | > 90° | — | walk re-tuned with a taller stance (see `docs/design_ja.md`) |

## Roadmap

1. Fit-check the drafted parts against the original parts in Onshape and print
   them; the open points are listed in `docs/parts_ja.md`.
2. Play the projects in LegacyMotionEditor / MuJoCo Studio and on the robot;
   the motions are tuned against MuJoCo only.
3. For a dynamic gait, retarget Microban's training pipeline
   (`mjlab_microban`) to this model.

## License

- Code, documentation and the generated MJCF files: **Apache-2.0** (the upstream
  `robot.xml` is itself Apache-2.0; its notice is retained).
- Meshes in `model/microban_pl/assets/`: **CC BY-NC-SA 4.0**, copyright Marc
  Duclusaud, unmodified copies from Microban. The parts in
  `model/parts_derived/` are modified copies of Microban's CAD parts and carry
  the same license. This makes the model as a whole **non-commercial**; see
  [NOTICE.md](NOTICE.md).
- Parts in `model/parts/`: original designs of this repository, Apache-2.0.

If you use Microban-derived content, please also cite Microban:

```bibtex
@software{mduclusaud-microban,
    author = {Duclusaud, Marc},
    doi = {10.5281/zenodo.20784789},
    month = jun,
    title = {{Microban: A Compact, Fully 3D-Printable Open-Source Humanoid Robot}},
    url = {https://github.com/Rhoban/microban},
    version = {1.0.0},
    year = {2026}
}
```
