"""microban_pl: Microban with parallel-link legs (passive ankle pitch) and a fixed head.

Model from the reduced-dof-humanoid repository (tools/make_microban_pl.py --naming lme):
per leg two parallelograms keep the sole parallel to the pelvis, so the ankle-pitch servo
is gone and the KNEE angle is measured against the pelvis, not the thigh. 16 actuators.

Everything the walk needs carries over from microban unchanged: the gait keyframes are
foot targets in the trunk frame with the sole flat, which is exactly the constraint the
linkage enforces for free, and the leg lengths are microban's. Only the IK output
differs (no ankle pitch, pelvis-relative knee).

What does NOT carry over is anything that tilts the trunk over flat soles -- the bow,
the kick's support leg, and every get-up pose microban searched, since those have the
sole at an angle to the pelvis. The get-ups have to be searched again on this model
(tools/getup_supine.py works unchanged: it samples LEG_ORDER, which is five joints here).
"""
from profiles.microban import *          # noqa: F401,F403  -- microban's numbers, overridden below

OUT = 'motion/projects/LME_microban_pl_save.xml'
MODEL = 'model/microban_pl/microban_pl_lme.xml'
MODEL_TYPE = 'mjcf'
ROBOT_NAME = 'Microban PL'

# No ankle pitch: the linkage sets it. Hip yaw is still there, so turning in place works.
LEG_ORDER = ['hip_yaw', 'hip_roll', 'hip_pitch', 'knee', 'ankle_roll']

# No neck, no ankle pitch.
JOINT_ORDER = [
    'l_shoulder_yp', 'l_shoulder_xr', 'l_elbow_yp',
    'l_hipjoint_zy', 'l_hipjoint_xr', 'l_hipjoint_yp', 'l_knee_yp', 'l_ankle_xr',
    'r_shoulder_yp', 'r_shoulder_xr', 'r_elbow_yp',
    'r_hipjoint_zy', 'r_hipjoint_xr', 'r_hipjoint_yp', 'r_knee_yp', 'r_ankle_xr',
]

# The linkage's passive joints: posed by nothing, declared so the model check passes.
# (named so that none of them is in LegacyMotionEditor's JOINT_TO_MERIDIM table: the old
# 'r_knee_yp_1' was an alias of the knee's own slot and LME's publishing of the passive
# angle into it made the knee flip between two targets)
EXTRA_JOINTS = ('l_kneeblock_yp', 'l_shanklink_yp', 'l_thighlink_yp', 'l_anklepassive_yp',
                'r_kneeblock_yp', 'r_shanklink_yp', 'r_thighlink_yp', 'r_anklepassive_yp')

# How the passive joints follow the actuated ones (tools/ik.py applies this, since
# mj_forward does not solve the <connect> loops). Knee block = -hip pitch keeps the knee
# block parallel to the pelvis; ankle pitch = -knee keeps the sole parallel to it.
# The two rear links mirror their front link.
PASSIVE_COUPLING = {}
for _s in ('l', 'r'):
    PASSIVE_COUPLING.update({
        f'{_s}_kneeblock_yp': [(f'{_s}_hipjoint_yp', -1.0)],
        f'{_s}_anklepassive_yp': [(f'{_s}_knee_yp', -1.0)],
        f'{_s}_thighlink_yp': [(f'{_s}_hipjoint_yp', 1.0)],
        f'{_s}_shanklink_yp': [(f'{_s}_knee_yp', 1.0)],
    })

ARM_HOME = dict(l_shoulder_yp=10.0, l_shoulder_xr=10.0, l_elbow_yp=-60.0,
                r_shoulder_yp=10.0, r_shoulder_xr=-10.0, r_elbow_yp=-60.0)

# microban's get-ups are joint-space poses with the sole at an angle to the pelvis,
# which this leg cannot make. Empty until tools/getup_supine.py has run on this model.
GETUP_SUPINE, F_RISE_U, F_HOLD_U = [], [], []

# bow / kick pitch the trunk over flat soles (impossible here); the uppercut has not
# been re-checked; the get-ups are not searched yet.
SKIP_ACTIONS = ('bow', 'kick', 'Attack_upper_l', 'Attack_upper_r', 'standup_face_down', 'standup_face_up')

SIM_KP = 50.0
SIM_FORCE = 0.52       # the model's forcerange: XL330-M288-T stall torque at 5 V

# ---- prone get-up -----------------------------------------------------------------
# tools/getup_supine.py --pitch 90 on THIS model (physics beam search in joint space,
# five leg numbers per pose: hip yaw, hip roll, hip pitch, knee (pelvis-relative), ankle
# roll). The search's own cadence was 45 frames to each pose then a 0.2 s hold, so each
# pose is listed twice below: the move, then the hold (F_RISE_D 45 / 20).
# Found standing at step 11: up 1.00, CoM 161 mm, sole 1.00, foot yaw -20 deg.
_PRONE_POSES = [
    # (leg: yaw, roll, hip pitch, knee, ankle roll), (shoulder pitch, roll, elbow)
    ((-21.772508, 25.748190, 28.971567, 75.649492, -35.000000), (95.623116, 159.784840, 31.518630)),
    ((-103.243188, 13.389046, 68.181978, 96.256910, -35.000000), (112.569452, 180.000000, -49.835454)),
    ((-156.632714, 20.647496, 68.181978, 96.256910, -20.589995), (27.434611, 180.000000, -40.768653)),
    ((-155.023493, 31.805520, 68.181978, 96.256910, 25.715237), (-89.998888, 180.000000, -15.687512)),
    ((-170.624563, -23.983578, 68.181978, 96.256910, 35.000000), (-47.713394, 160.913056, 82.668816)),
    ((-133.841670, 15.930986, 68.181978, 96.256910, 16.239913), (-65.892300, 180.000000, 65.840226)),
    ((-130.074018, 12.494640, 40.860904, 96.256910, 8.067084), (-58.382564, 180.000000, 36.554998)),
    ((-174.727704, 31.103920, 0.351746, 96.256910, -3.581480), (-51.075663, 175.652774, 20.339512)),
    ((-170.323630, 16.329430, -3.611724, 96.256910, -17.728853), (-36.556458, 166.770174, 7.000120)),
    ((-188.027360, 7.573295, 2.818883, 79.846692, -11.932560), (-8.277559, 157.179089, 2.203612)),
    ((-203.935447, 28.319432, -11.996382, 61.269917, -16.308884), (-38.045451, 162.897997, -22.571536)),
    ((-217.957757, 7.765845, 5.157252, 37.748217, -8.061535), (-22.512220, 160.110197, -45.708989)),
]
GETUP_PRONE = []   # v1 poses above are kept for reference; re-searched on the v2 model below
# Per-keyframe durations from tools/tune_getup.py face_down (pop 60, gens 12, hi-scale 2):
# 8.84 s -> 7.83 s and the pass rate from 1/8 to 89% of its 6-trial screens.
F_RISE_D = [20, 25, 49, 23, 45, 16, 70, 11, 22, 21, 23, 31, 16, 31, 44, 12, 34, 27, 28, 18, 37, 15, 54, 18]
# The search started from legs at zero with the arms rolled out flat (shoulder roll 90).
PRONE_PREP = [("lie_flat", (0, 90, 0), 43)]
# Unwinding 220 deg of hip yaw while standing takes longer than microban's 34 frames.
F_HOME_D = 50

# ---- supine: roll onto the front, then use the prone rise ---------------------------
# tools/getup_roll.py --to prone --robust 2 on this model: three poses, 60 frames each
# with the search's 1.0 s hold after every one, take the chest from +0.92 (face-up) to
# -1.00 with tilt 0.00 -- square on the front -- in every perturbed copy. The prone rise
# then joins in full, lie_flat included, after F_SETTLE_ROLL frames at rest.
ROLL_TO_PRONE = [
    ('roll_0', dict(r_shoulder_yp=49.89, r_shoulder_xr=-156.56, r_elbow_yp=7.74, r_hipjoint_zy=7.43, r_hipjoint_xr=13.19, r_hipjoint_yp=33.24, r_knee_yp=26.16, r_ankle_xr=9.06, l_shoulder_yp=56.52, l_shoulder_xr=116.63, l_elbow_yp=-21.40, l_hipjoint_zy=-60.00, l_hipjoint_xr=35.00, l_hipjoint_yp=45.49, l_knee_yp=2.38, l_ankle_xr=-35.00), 60),
    ('hold_0', dict(), 100),
    ('roll_1', dict(r_shoulder_yp=-58.47, r_shoulder_xr=-178.24, r_elbow_yp=51.05, r_hipjoint_zy=60.00, r_hipjoint_xr=35.00, r_hipjoint_yp=-27.17, r_knee_yp=27.11, r_ankle_xr=19.11, l_shoulder_yp=38.76, l_shoulder_xr=168.48, l_elbow_yp=-43.86, l_hipjoint_zy=26.44, l_hipjoint_xr=35.00, l_hipjoint_yp=68.18, l_knee_yp=24.32, l_ankle_xr=-35.00), 60),
    ('hold_1', dict(), 100),
    ('roll_2', dict(r_shoulder_yp=-49.79, r_shoulder_xr=-169.10, r_elbow_yp=70.87, r_hipjoint_zy=60.00, r_hipjoint_xr=35.00, r_hipjoint_yp=-41.22, r_knee_yp=44.02, r_ankle_xr=13.04, l_shoulder_yp=-49.93, l_shoulder_xr=139.58, l_elbow_yp=-38.43, l_hipjoint_zy=-4.56, l_hipjoint_xr=-9.18, l_hipjoint_yp=42.58, l_knee_yp=52.87, l_ankle_xr=-35.00), 60),
    ('hold_2', dict(), 100),
]
F_SETTLE_ROLL = 100
