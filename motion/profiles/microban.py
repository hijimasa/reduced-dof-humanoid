"""microban: the numbers behind save/LME_microban_save.xml.

Everything here was authored or searched for THIS robot -- 300 mm tall, XL330 servos,
40 x 22.5 mm soles. The generators in build_project.py read one of these profile
modules (see robot_profile.py) and hold nothing robot-specific themselves.
"""

OUT = 'motion/projects/LME_microban_save.xml'   # the 19-servo original: its model is not in this repository; this profile only supplies base numbers
MODEL = './model/microban_mjcf/microban_mjcf.xml'
MODEL_TYPE = 'mjcf'


JOINT_ORDER = [
    'c_neck_zy',
    'l_shoulder_yp', 'l_shoulder_xr', 'l_elbow_yp',
    'l_hipjoint_zy', 'l_hipjoint_xr', 'l_hipjoint_yp', 'l_knee_yp',
    'l_ankle_yp', 'l_ankle_xr',
    'r_shoulder_yp', 'r_shoulder_xr', 'r_elbow_yp',
    'r_hipjoint_zy', 'r_hipjoint_xr', 'r_hipjoint_yp', 'r_knee_yp',
    'r_ankle_yp', 'r_ankle_xr',
]


# ---- gait parameters (metres / degrees, trunk frame: +x fwd, +y left, +z up) ----
# Tuned by tools/tune_gait.py: a cross-entropy search over these numbers, scoring
# distance per cycle but rejecting any candidate that fell in even one perturbation
# trial. The stance came out much lower than the hand-picked one (Z0 -150 -> -132 mm):
# a lower centre of mass raises the inverted-pendulum frequency, which is what lets the
# weight transfer be quick, and that transfer is the whole cost of a statically stable
# walk. 17 -> 38 mm/s at 8/8 trials.
X0, Y0, Z0 = 0.0013, 0.036, -0.1321   # home foot pose
LIFT = 0.0205                        # swing foot clearance
WY = 0.0413                          # lateral trunk shift onto the support foot
WY_D = 0.0                           # left/right trim on that shift, cancels sideways drift
STEP = 0.0246                        # half step length (stride = 2 * STEP)
SIDE = 0.0153                        # sidestep width per step
INSET = -0.0055                      # trunk parks just past the support sole: the swing leg
                                     # pulls the whole-body CoM back inboard by about this much
PHI_S, PHI_B = 18.0, 36.75           # turn per step, small / big

# Frames per keyframe at 100 fps. The lateral weight transfer dominates: moving
# the CoM 2*WY in T seconds peaks at ~(pi^2/2)*2*WY/T^2, which has to stay under
# (foot half width)*g/(CoM height) or the robot tips. Hence the long SHIFT.
F_SHIFT, F_LIFT, F_SWING, F_PLACE, F_HOME = 71, 9, 45, 8, 30
# Turning and sidestepping get their own timings: they are not the same balance problem.
# A turn spins the trunk over a planted sole, so its swing can be brief; a sidestep ends
# with a wider stance than it started, so its weight transfer is the longest of the three.
F_SHIFT_T, F_LIFT_T, F_SWING_T, F_PLACE_T = 88, 8, 32, 22
F_SHIFT_S, F_LIFT_S, F_SWING_S, F_PLACE_S = 83, 9, 32, 29
# The sidestep also stands taller than the walk. Hip roll is only +-35 deg, and the
# crouched walking stance already spends most of it, which caps the step at 7 mm; a
# taller stance buys the roll range back.
Z0_S = -0.1469
F_SETTLE = 40                        # frames spent lying flat before the arms plant
F_RISE = 280                         # frames per get-up keyframe (see tools/getup_beam.py)
# Per-keyframe durations for the prone rise, from tools/tune_getup.py and then stretched
# by 1.4. The uniform 280 frames were the beam search's fixed cadence, not a requirement.
# The tuner's own schedule (5.4 s) gets up in every trial at the nominal disturbance but
# only 71% of the time at 0.8 deg / 2.5 deg, where the 21.6 s original manages 89%; the
# stretch buys that margin back and then some -- 92.5% at 7.5 s, better than the
# original on both counts. The long one is rise_4, the pose that closes the straddle
# while standing; the last two are nearly free, the robot is already upright by then.
F_RISE_D = [132, 91, 84, 104, 179, 15, 14]

# Boot's midpoint: nearly straight legs, on the way down from the zero pose.
BOOT_Z = -0.160
# microban's zero pose already stands with its arms down, so Boot starts from it as-is.
BOOT_ZERO = {}

ARM_SWING = 3.6        # counter-swing amplitude of the arms during the walk
F_HOME_D, F_HOME_U = 34, 14   # settling back into HOME at the end of each get-up        # counter-swing amplitude of the arms during the walk

ARM_HOME = dict(l_shoulder_yp=10.0, l_shoulder_xr=10.0, l_elbow_yp=-20.0,
                r_shoulder_yp=10.0, r_shoulder_xr=-10.0, r_elbow_yp=-20.0,
                c_neck_zy=0.0)


# ---- prone get-up -----------------------------------------------------------------
# Found by tools/getup_search.py (roadmap of statically stable flat-foot poses, with
# the HANDS counted as support while they are down) and tools/getup_beam.py (beam
# search with the physics simulator in the loop, because a static roadmap says which
# poses hold but not which transitions actually happen -- the trunk often rotates the
# wrong way and the robot noses over).
#
# The motion splays the legs into a near-180 deg straddle (the feet reach 232 mm apart)
# and closes them again while standing, which is what makes it work: the wide stance
# buys lateral margin, and the leg twist that the low poses need (hip yaw to -179 deg,
# inside the -240..+60 hardware range) is unwound while upright.
#
# Verified: 16/16 trials from prone to HOME with servo-offset and tilt disturbances.
# Keep the poses UNINTERPOLATED -- inserting intermediate keyframes drops it to 0/16,
# since only the roadmap poses themselves are statically stable.
GETUP_PRONE = [
    # (leg: yaw, roll, hip pitch, knee, ankle pitch, ankle roll), (shoulder pitch, roll, elbow)
    ((-178.65, -18.89, 64.45, 67.02, -48.77, 3.90), (-63.75, 25.31, -40.67)),
    ((-170.95, -9.53, 27.80, 87.85, -76.30, 13.07), (-26.85, 9.33, -14.68)),
    ((-158.01, 6.24, -10.06, 55.49, -48.96, -7.64), (-3.83, 27.45, -43.06)),
    ((-114.90, 4.46, -27.94, 43.07, -17.97, -10.49), (-15.97, 12.81, -22.54)),
    ((-78.92, -13.21, -27.43, 3.05, 25.61, 6.97), (1.90, 39.42, -20.84)),
    ((-38.57, -26.57, -11.00, -19.76, 28.03, 28.45), (20.29, 38.51, -41.57)),
    ((2.35, -11.31, 9.17, -20.07, 7.52, 11.16), (0.09, 20.35, -47.96)),
]


# ---- supine get-up ----------------------------------------------------------------
# Found by tools/getup_supine.py: a physics beam search that samples poses freely in
# joint space instead of drawing them from the flat-foot roadmap, because none of the
# poses this needs -- sitting, tucked, balanced on the backside -- are statically stable
# on the feet and so none of them are in that roadmap.
#
# The old answer was to roll onto the front first and reuse the prone rise, which cost
# 2.6 s and half the sequence. This gets up directly. What makes it work is folding the
# legs in as the trunk comes up (knee 110 deg, both hips rolled fully out to 35 deg) so
# the feet arrive under the mass, and swinging the arms overhead-and-back to push: the
# earlier attempt pressed on the arms alone, stalled at about 45 deg of trunk pitch, and
# sat there with its feet 110 mm in front of the centre of mass.
GETUP_SUPINE = [
    # (leg: yaw, roll, hip pitch, knee, ankle pitch, ankle roll), (shoulder pitch, roll, elbow)
    ((36.91, -5.89, 90.00, 50.95, -34.83, 35.00), (19.62, 112.29, 60.64)),
    ((60.00, -35.00, 61.23, 37.78, -74.55, 35.00), (21.40, 92.68, 59.54)),
    ((-23.71, -10.65, -19.43, 23.80, -12.25, 13.91), (-19.52, 26.94, -4.44)),
]

# Each rise pose is followed by a hold at the same angles: the search evaluated every
# candidate after a settling pause, so the transitions were only ever shown to work with
# one. tools/tune_getup.py tunes both numbers; these are its schedule stretched by 1.4.
# The tuned 1.57 s version got up 98% of the time and had a servo at its torque ceiling
# 69% of the time -- it was winning against gravity by a margin the XL330s did not
# really have. Stretching it costs 0.6 s, drops peak-torque time to 51% and makes it
# 240/240.
F_RISE_U = [32, 81, 35]
F_HOLD_U = [22, 11, 13]


# A bow wants straighter legs than the walk: at the walking stance (Z0 = -132 mm) the
# knee is already at 89 deg and bowing on top of that reads as a squat, not a bow. The
# action stands tall first, and BOW_Z is as tall as the leg IK will go while the hips
# still have the range to pitch the trunk over.
BOW_ANGLE, BOW_Z = 35.0, -0.163
BOW_XO = (-0.070, 0.050)     # how far the feet may travel while the bisection balances the bow
BOW_ARMS = dict(arm_pitch=0.0, arm_roll=6.0, elbow=-8.0)


# Jab, tuned by tools/tune_punch.py: a target post is put in the scene at the reach of
# the original jab and the search maximises the peak contact force it takes. A candidate
# is rejected unless it (a) never touches the target before the strike, (b) is off it
# again by the end, (c) is still standing afterwards, and (d) is ALSO still standing when
# the same punch is thrown at thin air -- scored against the target alone the search finds
# a punch that throws the whole body forward and lets the target catch it, which lands
# face down in 38 of 40 trials when it misses.
#
# The target is a POST about as wide as a torso, not a wall. The first version of this was
# a 0.5 m slab, and a slab cannot tell a straight punch from a diagonal one: the search
# took 33 deg of shoulder roll and 28 deg of hip twist for the extra reach and still
# scored 46 N, because there was slab either side of wherever the fist ended up. Against
# something the size of a robot that punch measures 0.0 N -- it goes clean past.
#
# Against the post: 15.0 N peak and 254 mNs of impulse, standing in all 60 trials at
# 0.8 deg / 2.5 deg disturbance, hit or miss. The shoulder roll came back to 3 deg, which
# is the whole point -- the fist now goes where the robot is looking.
JAB = dict(STANCE_Z=-0.1471, FRONT_X=-0.0100, REAR_X=0.0037, STANCE_Y=0.0410,
           G_PITCH=-17.38, G_ROLL=56.91, G_ELBOW=-61.50,
           P_PITCH=-105.08, P_ROLL=3.36, P_ELBOW=4.78, P_OFFARM=43.91,
           P_TWIST=45.00, P_LEAN=-0.0029)
F_JAB_GUARD, F_JAB_PUNCH, F_JAB_HOLD, F_JAB_BACK = 51, 18, 5, 37
F_JAB_HOME = 40      # settling back into HOME after the punch


# ---- front kick -------------------------------------------------------------------
# The finisher, on the Triangle button. Tuned by tools/tune_attack.py, which puts a
# second robot in the scene and scores whether the move actually puts it on the floor --
# not force into a post. That distinction matters here: microban's shoulder roll only
# abducts, so a two-handed shove keeps both palms at y = +-89 mm, outside the opponent's
# silhouette, and measures as a miss however hard it is thrown. A leg has no such
# problem, carries far more mass than an arm, and lands at chest height where the
# toppling moment is largest.
#
# Against another microban at 160 mm: opponent down 100% of trials, its trunk dropping
# 135 mm, and the kicker still standing in 100% -- including when the kick is thrown at
# nothing, which is the case that decides whether it is usable at all.
KICK = dict(
    STANCE_Z=-0.1471,     # support foot height
    SUP_X=-0.0003,        # support foot fore/aft
    STANCE_Y=0.0322,      # half stance width
    WY_K=0.0384,          # lateral shift onto the support leg before the foot leaves
    CH_HIP=-83.85,        # chamber: knee up ...
    CH_KNEE=98.15,        # ... heel tucked
    K_HIP=-68.69,         # strike: hip drives the thigh forward ...
    K_KNEE=15.77,         # ... and the knee snaps out
    K_ANK=19.06,
    A_PITCH=27.44,        # arms as a counterweight
    A_ROLL=40.12,
    A_ELBOW=-32.57,
    K_LEAN=-7.18,         # trunk pitched slightly away from the kick
)
F_K_SHIFT, F_K_CHAMBER, F_K_KICK, F_K_HOLD, F_K_BACK, F_K_DOWN = 77, 31, 26, 8, 19, 27
F_K_HOME = 40     # settling back into HOME after the kick


# ---- project metadata --------------------------------------------------------------
ROBOT_NAME = 'Microban'
SERVO = ('XL330-M288-T', '10.7861')
CAM_DISTANCE, CAM_FOCAL = '0.42', ('0.005', '0.0', '-0.06')
SKIP_ACTIONS = ()

# The two keyframes that get the robot from wherever it fell into the pose the searched
# rise starts from: (name, (shoulder pitch, roll, elbow), frames), legs left at zero.
PRONE_PREP = [('lie_flat', (0, 90, 0), 42),
              ('plant_hands', (-110, 40, -95), 39),
              ('push_chest', (-100, 30, -20), 20)]
SUPINE_PREP = [('lie_flat', (0, 150, 0), 30)]

# microban's model turns every joint the way the canonical names read.
SIGN = {}

# ... and hangs its arms at the sides at zero.
ARM_ZERO = {}

# ---- what the simulators need to know ----------------------------------------------
SIM_KP = 50.0          # position actuator gain, from the MJCF "microban" default class
SIM_FORCE = 0.64       # XL330 torque ceiling: BAM_MAX_CURRENT (1.75 A) * kt (0.366 Nm/A)
SIM_MAX_SPEED = 10.786  # XL330-M288-T, rad/s (matches the project's JointSettings)
FOREARM_BODIES = ('radius', 'radius_2')   # the links that count as hands when they are on the floor
SOLE_HALF = (0.040, 0.0225)   # sole plate half extents, for the static stability check

# ---- the parameter space tools/tune_gait.py searches --------------------------------
# LIFT bottoms out at 12 mm on purpose: the search will happily take the swing foot
# down to 8 mm because nothing in the sim scuffs, but a real floor is not that flat.
GAIT_GLOBAL = dict(Z0=(-0.170, -0.132, 0), LIFT=(0.012, 0.032, 0), WY=(0.024, 0.055, 0),
                   X0=(-0.004, 0.016, 0))
GAIT_WALK = dict(F_SHIFT=(14, 90, 1), F_LIFT=(5, 30, 1), F_SWING=(8, 50, 1),
                 F_PLACE=(5, 30, 1), STEP=(0.008, 0.040, 0), ARM=(0.0, 30.0, 0),
                 WY_D=(-0.008, 0.008, 0))
GAIT_SIDE = dict(Z0_S=(-0.172, -0.128, 0), F_SHIFT_S=(14, 95, 1), F_LIFT_S=(5, 30, 1),
                 F_SWING_S=(8, 50, 1), F_PLACE_S=(5, 30, 1), SIDE=(0.006, 0.032, 0),
                 INSET=(-0.010, 0.008, 0))
GAIT_TURN = dict(F_SHIFT_T=(14, 90, 1), F_LIFT_T=(5, 30, 1), F_SWING_T=(8, 60, 1),
                 F_PLACE_T=(5, 30, 1), PHI=(4.0, 52.0, 0))

# ---- where tools/getup_search.py samples for stable poses ---------------------------
# (trunk pitch lo, hi in deg, CoM height lo, hi in mm). microban stands with its CoM
# at 137 mm.
GETUP_BANDS = dict(
    hands=(40, 115, 35, 130),
    mid=(15, 80, 60, 160),
    top=(-20, 35, 95, 180),
    supine_hands=(-120, -20, 35, 130),
    supine_free=(-90, -10, 60, 165),
    start_comz=(60, 100),
    goal_comz=128,
)
# CoM height above which tools/getup_beam.py calls the robot standing.
GETUP_STAND_COMZ = 138

# (gait, get-up) frame multipliers for build_project.py --compat
COMPAT_SCALES = (1.40, 1.20)

# microban rises from its front directly; no roll-over needed.
ROLL_TO_SUPINE = []

# ---- uppercut ------------------------------------------------------------------------
# Lengths in metres, angles canonical. Z_LOW/Z_MID/Z_HIGH are the crouch, the guard and
# the extension: the difference between LOW and HIGH is the distance the whole body
# travels upward through the strike, which is where the impulse comes from.
UPPER = dict(
    STANCE_X=0.0013, STANCE_Y=0.0400,
    Z_MID=-0.1400, Z_LOW=-0.1240, Z_HIGH=-0.1640,
    G_PITCH=-17.38, G_ROLL=56.91, G_ELBOW=-61.50,     # the jab's guard
    L_PITCH=35.00, L_ROLL=8.00, L_ELBOW=-95.00,       # fist loaded behind the hip
    U_PITCH=-150.00, U_ROLL=4.00, U_ELBOW=-80.00,     # fist driven up past the shoulder
    U_LEAN=0.0040, U_TWIST=30.00,
)
F_U_GUARD, F_U_LOAD, F_U_RISE, F_U_HOLD, F_U_BACK = 45, 30, 30, 6, 40
F_U_HOME = 40

# Fight-balance: slow every action by this factor (see build_project --handicap).
SPEED_HANDICAP = 1.0

# Fight-balance: stretch attack wind-ups only (see build_project --telegraph).
ATTACK_TELEGRAPH = 1.0

# This model's geoms collide the ordinary way; no explicit floor pairs needed.
FLOOR_PAIRS = ()

# Every joint in this model is posed by the project.
EXTRA_JOINTS = ()

TURN_SLIP = {}   # microban has hip yaw and turns on the spot

TURN_ARM = 0.0
F_SETTLE_ROLL = 0    # extra hold between a roll-over and the rise; see build_project.py
TURN_SCUFF = ()      # (a, f_push, f_ret) for a hip-yaw-less scuff turn; see build_project.py
FIST_BODY = '{s}_el_link'   # body carrying the striking point; {s} is 'l'/'r'
