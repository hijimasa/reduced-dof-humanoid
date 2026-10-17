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

SKIP_ACTIONS = ('bow', 'kick', 'Attack_upper_l', 'Attack_upper_r')


# ---- prone get-up -----------------------------------------------------------------
# tools/getup_supine.py --pitch 90 --seed 0 on this model: four leg numbers per pose (hip
# roll, hip pitch, knee (pelvis-relative), ankle roll); the middle arm number is the
# shoulder roll the search still samples but no joint takes. The rise rolls the robot
# over onto its back/side and sits up facing the other way, kneels, squats and stands --
# 7 poses, each listed twice (move, then hold). Durations from tools/tune_getup.py
# face_down, second pass (pop 80, gens 15, trials 8): 5.6 s -> 3.9 s, and the schedule
# held 32/32 verification trials in the tuner.
# 2026-10-17: rise re-searched on the FINAL geometry (rear thigh link, hip lug, long forearms,
# abducted arms): motion/getup_supine.py --pitch 90 --seed 72 --robust 3 --steps 20 on microban_12,
# nine poses (6/8 at the search cadence); timed by motion/tune_getup.py (pop 90, gens 25,
# trials 8): 6.9 s -> 6.7 s, 100% of its screens.
_PRONE_POSES = [
    # (leg: hip roll, hip pitch, knee, ankle roll), (shoulder pitch, [unused], elbow)
    ((13.434479, 62.016952, -4.767457, -22.618828), (-7.597739, 89.824309, 120.718426)),
    ((28.939760, 62.016952, 22.000046, -11.640316), (-124.436640, 67.187588, 72.714403)),
    ((35.000000, 62.016952, 127.000106, -35.000000), (-19.409325, 25.990932, 21.805967)),
    ((35.000000, 62.016952, 105.354450, 3.279516), (-26.879386, 40.272662, 15.975284)),
    ((-35.000000, 62.016952, 68.121740, 1.759976), (-18.287690, -38.225134, -16.538399)),
    ((-35.000000, 34.243016, 66.955918, -8.581963), (36.759169, -38.921473, -78.686083)),
    ((-35.000000, -15.882427, 46.400884, -18.002783), (54.516067, -67.019046, -55.271585)),
    ((35.000000, 46.610811, -20.041844, 35.000000), (76.734172, -33.818359, -30.086216)),
    ((35.000000, 15.738871, 3.021146, -2.630319), (60.546171, -59.333977, 8.319482)),
]
GETUP_PRONE = [p for p in _PRONE_POSES for _ in (0, 1)]
F_RISE_D = [45, 17, 52, 16, 38, 25, 55, 16, 22, 13, 53, 23, 53, 21, 52, 14, 34, 17]
PRONE_PREP = [('lie_flat', (0, 90, 0), 27)]
F_HOME_D = 76

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

# ---- supine: a DIRECT rise (2026-10-17) ---------------------------------------------
# motion/getup_supine.py --pitch -90 --seed 80 --robust 3 --steps 20 on the final geometry: the
# 40 mm longer forearms let the robot sit up on its hands, the hip then folds to the new -100 deg
# limit and it squats and stands. Eight poses, 4/8 at the search cadence; motion/tune_getup.py
# face_up (pop 90, gens 25, trials 8): 6.2 s -> 5.3 s, 100% of its screens, 15/24 finalists held
# 40 verification trials. Replaces the roll-onto-the-front + prone-rise scheme (7/8).
_SUPINE_POSES = [
    # (leg: hip roll, hip pitch, knee, ankle roll), (shoulder pitch, [unused], elbow)
    ((35.000000, -100.000043, -37.000010, -10.957765), (82.365255, 102.292400, -125.000000)),
    ((-3.862485, -21.985642, -31.083634, -35.000000), (130.208685, 61.544520, -118.957559)),
    ((5.696653, -54.053162, -14.820980, -7.412180), (103.126907, 78.885826, -90.077569)),
    ((35.000000, -100.000043, 20.947117, -8.653312), (46.898822, 81.335915, -32.626929)),
    ((26.018069, -100.000043, 8.294092, -11.464974), (70.107278, 130.187932, -43.369711)),
    ((-35.000000, -80.311343, 6.987364, 35.000000), (25.148267, -4.625093, -5.245178)),
    ((-35.000000, -100.000043, 70.803735, 35.000000), (141.561042, 29.652828, -27.876010)),
    ((-35.000000, 18.043450, -12.368676, 35.000000), (180.000000, 42.351237, -61.384149)),
]
GETUP_SUPINE = _SUPINE_POSES
F_RISE_U = [39, 31, 27, 40, 49, 47, 49, 39]
F_HOLD_U = [17, 11, 16, 18, 17, 14, 21, 22]
SUPINE_PREP = [('lie_flat', (0, 150, 0), 19)]
F_HOME_U = 58
# The old one-pose roll onto the front (supine = roll + prone rise, 7/8) is kept in
# motion/candidates/roll_to_prone_12.pkl; putting it back here would restore that scheme.
ROLL_TO_PRONE = []
