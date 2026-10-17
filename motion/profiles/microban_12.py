"""microban_12: the ROBO-ONE Light target layout -- legs 4 x 2, arms 2 x 2, fixed head.

microban_pl (parallel-link legs, passive ankle pitch, no neck) with the hip yaw and the
shoulder roll removed as well. 12 servos. Model from reduced-dof-humanoid:
tools/make_microban_pl.py --dof 12 --naming lme.

What changes for the motions:
  * no hip yaw -> no turning on the spot. Like the ROBOTIS Mini, the turns use
    TURN_SCUFF (a differential push of the two soles in double support); the numbers
    are the Mini's as a seed and are verified, not tuned, for this robot.
  * no shoulder roll -> armset() drops the roll; the jab is thrown straight ahead with
    no hip twist (P_TWIST 0) and no roll.
  * the leg IK solves four joints for a flat sole at a position; heading is whatever
    the leg leaves it, as on the Mini.
Everything else (gait geometry and timing) is microban's, re-solved on this leg.
"""
from profiles.microban_pl import *          # noqa: F401,F403

OUT = 'motion/projects/LME_microban_12_save.xml'
MODEL = 'model/microban_pl12/microban_pl12_lme.xml'
ROBOT_NAME = 'Microban 12'

LEG_ORDER = ['hip_roll', 'hip_pitch', 'knee', 'ankle_roll']
JOINT_ORDER = [
    'l_shoulder_yp', 'l_elbow_yp',
    'l_hipjoint_xr', 'l_hipjoint_yp', 'l_knee_yp', 'l_ankle_xr',
    'r_shoulder_yp', 'r_elbow_yp',
    'r_hipjoint_xr', 'r_hipjoint_yp', 'r_knee_yp', 'r_ankle_xr',
]
ARM_HOME = dict(l_shoulder_yp=10.0, l_elbow_yp=-60.0, r_shoulder_yp=10.0, r_elbow_yp=-60.0)

# no hip yaw: scuff turns (see build_project.scuff_turn_cycle); seed values from the Mini
PHI_S, PHI_B = 0.0, 0.0
TURN_SCUFF = (0.044, 20, 70)
TURN_SLIP = {}

JAB = dict(JAB, P_TWIST=0.0, G_ROLL=0.0, P_ROLL=0.0)

GETUP_SUPINE, F_RISE_U, F_HOLD_U = [], [], []
SKIP_ACTIONS = ('bow', 'kick', 'Attack_upper_l', 'Attack_upper_r')

# ---- supine: roll onto the front, then use the prone rise ---------------------------
# tools/getup_roll.py --to prone --robust 2 on this model: ONE pose (60 frames + the
# search's 1.0 s hold) turns the chest from face-up to -0.99 with tilt 0.01 in every
# perturbed copy -- the arms swing over and one hip rolls out, and the lighter trunk
# (no hip-yaw or neck servos) follows. The prone rise then joins in full.
ROLL_TO_PRONE = [
    ('roll_0', dict(r_shoulder_yp=119.17, r_elbow_yp=-1.65, r_hipjoint_xr=35.00, r_hipjoint_yp=-17.52, r_knee_yp=17.78, r_ankle_xr=4.47, l_shoulder_yp=109.15, l_elbow_yp=18.15, l_hipjoint_xr=-25.10, l_hipjoint_yp=17.37, l_knee_yp=10.92, l_ankle_xr=-7.62), 60),
    ('hold_0', dict(), 100),
]
F_SETTLE_ROLL = 100

# ---- prone get-up -----------------------------------------------------------------
# tools/getup_supine.py --pitch 90 --seed 0 on this model: four leg numbers per pose (hip
# roll, hip pitch, knee (pelvis-relative), ankle roll); the middle arm number is the
# shoulder roll the search still samples but no joint takes. The rise rolls the robot
# over onto its back/side and sits up facing the other way, kneels, squats and stands --
# 7 poses, each listed twice (move, then hold). Durations from tools/tune_getup.py
# face_down, second pass (pop 80, gens 15, trials 8): 5.6 s -> 3.9 s, and the schedule
# held 32/32 verification trials in the tuner.
# 2026-10-17: rise re-searched for the arms abducted 30 deg (--arm-abduct) with the printed parts
# colliding: tools/getup_supine.py --pitch 90 --seed 32 --robust 3 (found on a model with the hip
# flexion cap at 100, but the rise uses hip -20..+58 only, so it is the same on the standard model).
# Six poses: rolls over and stands facing the other way. Timed by tune_getup.py (pop 90, gens 25,
# trials 8): 4.9 s -> 4.6 s, 98% of its screens.
_PRONE_POSES = [
    # (leg: hip roll, hip pitch, knee, ankle roll), (shoulder pitch, [unused], elbow)
    ((-18.950396, 57.555902, 3.311391, -19.612827), (-43.052088, 162.767617, 72.520268)),
    ((-24.719447, 57.555902, 76.663522, 7.718129), (-57.193811, 202.870646, -13.893520)),
    ((-12.254259, 56.829073, 107.773167, -27.673930), (-17.608691, 198.257037, -34.532642)),
    ((17.276841, 0.048001, 107.963070, -29.581076), (9.014764, 229.612424, 38.541114)),
    ((35.000000, -19.786351, 89.267794, -35.000000), (15.428489, 290.357207, 7.388185)),
    ((31.868273, 5.401025, 10.549636, 8.239007), (-48.799855, 226.869233, -7.866885)),
]
GETUP_PRONE = [p for p in _PRONE_POSES for _ in (0, 1)]
F_RISE_D = [61, 16, 35, 11, 61, 12, 41, 16, 43, 26, 34, 21]
PRONE_PREP = [('lie_flat', (0, 90, 0), 46)]
F_HOME_D = 22

# ---- ROBO-ONE Light rules -----------------------------------------------------------
# Sole: the model's ground-contact plate is 80 mm (Microban's 90 mm exceeds 55% of the
# 146 mm leg). Squat walking: the knee may not close to 90 deg or less, so the walk is
# tuned with a taller stance than tune_gait would otherwise pick (Z0 <= -0.141 keeps the
# swing knee above ~100 deg).
GAIT_GLOBAL = dict(Z0=(-0.152, -0.141, 0), LIFT=(0.012, 0.022, 0), WY=(0.024, 0.055, 0),
                   X0=(-0.004, 0.016, 0))

# ---- walk, re-tuned under the ROBO-ONE Light rules (80 mm sole, knee > 90 deg) -------------
# tools/tune_gait.py walk with Z0 bounded to <= -0.141: 25.0 of the tuner's speed score at
# 100% survival, 30/30 finalists holding 96%+ over 24 trials. Taller than the earlier walk
# (Z0 -0.133 -> -0.142) and a little slower, with the swing knee staying above 100 deg.
# X0 moved +3 mm (0.0056 -> 0.0086) when the arms went to abduct 30 / elbows forward 60: the
# forearms ahead of the trunk shift the CoM forward and walk_back fell 4/8 at the old value
# (0.0036 -> 0/8, 0.0076 and 0.0096 -> 8/8 everywhere).
X0, Z0, LIFT, WY, WY_D = 0.0086, -0.141809, 0.0198224, 0.0391393, -0.00128318
STEP, ARM_SWING = 0.0222771, 14.6795
F_SHIFT, F_LIFT, F_SWING, F_PLACE = 65, 6, 40, 11
