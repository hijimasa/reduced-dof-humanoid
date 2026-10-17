"""microban_14: the chosen layout -- legs 5 x 2 (hip yaw kept), arms 2 x 2, fixed head.

microban_pl (parallel-link legs, passive ankle pitch, no neck) with only the shoulder
roll removed. 14 servos, 0.719 kg. Model from reduced-dof-humanoid:
tools/make_microban_pl.py --dof 14 --naming lme.

Hip yaw stays, so turning on the spot uses microban's turn_cycle and does not depend on
the floor's friction (the 12-servo variant had to scuff). Walking, sidestepping and the
jab are microban's, re-solved on the 5-joint leg; the jab keeps its hip twist and loses
its shoulder roll. Get-ups are searched on this model.
"""
from profiles.microban_pl import *          # noqa: F401,F403

OUT = 'motion/projects/LME_microban_14_save.xml'
MODEL = 'model/microban_pl14/microban_pl14_lme.xml'
ROBOT_NAME = 'Microban 14'

JOINT_ORDER = [
    'l_shoulder_yp', 'l_elbow_yp',
    'l_hipjoint_zy', 'l_hipjoint_xr', 'l_hipjoint_yp', 'l_knee_yp', 'l_ankle_xr',
    'r_shoulder_yp', 'r_elbow_yp',
    'r_hipjoint_zy', 'r_hipjoint_xr', 'r_hipjoint_yp', 'r_knee_yp', 'r_ankle_xr',
]
ARM_HOME = dict(l_shoulder_yp=10.0, l_elbow_yp=-60.0, r_shoulder_yp=10.0, r_elbow_yp=-60.0)
JAB = dict(JAB, G_ROLL=0.0, P_ROLL=0.0)

GETUP_SUPINE, F_RISE_U, F_HOLD_U = [], [], []
SKIP_ACTIONS = ('bow', 'kick', 'Attack_upper_l', 'Attack_upper_r')

# ---- supine: roll onto the front, then use the prone rise ---------------------------
# tools/getup_roll.py --to prone --robust 2 on this model: one pose (60 frames + 1.0 s
# hold) turns the chest face-down (-0.99, tilt 0.00) in every perturbed copy.
ROLL_TO_PRONE = [
    ('roll_0', dict(r_shoulder_yp=65.69, r_elbow_yp=-25.81, r_hipjoint_zy=49.57, r_hipjoint_xr=35.00, r_hipjoint_yp=-18.70, r_knee_yp=4.57, r_ankle_xr=-23.26, l_shoulder_yp=144.50, l_elbow_yp=-19.72, l_hipjoint_zy=-27.91, l_hipjoint_xr=-35.00, l_hipjoint_yp=-9.69, l_knee_yp=-0.16, l_ankle_xr=21.50), 60),
    ('hold_0', dict(), 100),
]
F_SETTLE_ROLL = 100

# ---- prone get-up -----------------------------------------------------------------
# The 12-servo rise (tools/getup_supine.py --pitch 90 --seed 0 on microban_12, timed by
# tune_getup.py) with the hip yaw held at zero: five leg numbers (hip yaw, hip roll, hip
# pitch, knee (pelvis-relative), ankle roll). Searches run on THIS model kept wandering
# off into hip-yaw twists that only worked nominally; the yaw-free rise carried over at
# 10/16 prone, 13/16 supine (roll + rise) before re-timing, 46 g heavier than where it
# was found. Each pose is listed twice: move, then hold.
# 2026-10-17: rise re-searched on the FINAL geometry (rear thigh link, hip lug, long forearms,
# abducted arms): motion/getup_supine.py --pitch 90 --seed 72 --robust 3 --steps 20 on microban_12,
# nine poses (6/8 at the search cadence); timed by motion/tune_getup.py (pop 90, gens 25,
# trials 8): 6.9 s -> 6.7 s, 100% of its screens. Re-timed again on THIS model: 6.5 s -> 5.2 s
# (prone 8/8, supine 7/8 in sim_check).
_PRONE_POSES = [
    # (leg: hip yaw, hip roll, hip pitch, knee, ankle roll), (shoulder pitch, [unused], elbow)
    ((0.0, 13.434479, 62.016952, -4.767457, -22.618828), (-7.597739, 89.824309, 120.718426)),
    ((0.0, 28.939760, 62.016952, 22.000046, -11.640316), (-124.436640, 67.187588, 72.714403)),
    ((0.0, 35.000000, 62.016952, 127.000106, -35.000000), (-19.409325, 25.990932, 21.805967)),
    ((0.0, 35.000000, 62.016952, 105.354450, 3.279516), (-26.879386, 40.272662, 15.975284)),
    ((0.0, -35.000000, 62.016952, 68.121740, 1.759976), (-18.287690, -38.225134, -16.538399)),
    ((0.0, -35.000000, 34.243016, 66.955918, -8.581963), (36.759169, -38.921473, -78.686083)),
    ((0.0, -35.000000, -15.882427, 46.400884, -18.002783), (54.516067, -67.019046, -55.271585)),
    ((0.0, 35.000000, 46.610811, -20.041844, 35.000000), (76.734172, -33.818359, -30.086216)),
    ((0.0, 35.000000, 15.738871, 3.021146, -2.630319), (60.546171, -59.333977, 8.319482)),
]
GETUP_PRONE = [p for p in _PRONE_POSES for _ in (0, 1)]
# Re-timed on THIS model by tools/tune_getup.py (pop 90, gens 25, trials 8): 4.7 s -> 3.8 s, 100% of
# its screens (the 12-servo schedule gave 1/8 here).
F_RISE_D = [30, 13, 49, 11, 47, 22, 56, 17, 23, 11, 53, 22, 10, 14, 42, 10, 34, 12]
PRONE_PREP = [('lie_flat', (0, 90, 0), 14)]
F_HOME_D = 28

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
