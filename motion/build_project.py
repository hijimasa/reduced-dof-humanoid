"""Generate an LME project, mirroring LME_sample_save.xml's action set.

    python3 motion/build_project.py              # microban
    LME_ROBOT=op3 python3 motion/build_project.py

Leg keyframes are authored as foot poses in the trunk frame and turned into joint
angles by the IK in ik.py, so every walking pose keeps the soles flat -- which is
what lets a second robot reuse them: the same foot trajectory, re-solved against its
own legs. Everything robot-specific lives in profiles/<name>.py.
"""
import os, sys, math
import xml.etree.ElementTree as ET
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from joint_names import jn, LEG_ORDER, LEG_MIRROR, NLEG, NPARAM
from ik import solve_leg, LIMITS, JOINTS
from robot_profile import constants

# every number in this file belongs to one robot; they live in profiles/<name>.py
globals().update(constants())

# Playback-speed safety factor, per action family. This one is NOT robot-specific -- it
# is a property of MuJoCo Studio, which advances physics by a fixed 1/60 s per rendered
# frame (LegacyMotionEditor_MuJoCoStudio.py, "# Physics") while LME goes on publishing at
# the project's 100 fps of wall-clock time. A Studio that only manages F frames per
# second therefore replays the motion at F/60 of its designed speed, and the cost of that
# is entirely one-sided. Measured on microban, 16 trials each:
#
#   effective speed   0.50   0.60   0.70   0.80   0.88   1.00   1.45
#   equivalent fps      30     36     42     48     53     60      --
#   walk forward      0/16   0/16   0/16   0/16  12/16  15/16  16/16
#   sidestep          0/16   4/16  14/16  16/16  16/16  16/16  16/16
#   turns             0/16  12/16  15/16  15/16  15/16  15/16  16/16
#   get-ups           0/16   1/16  16/16  16/16  16/16  16/16  16/16
#   jab / bow / kick 16/16  16/16  16/16  16/16  16/16  16/16  16/16
#
# Slower is always safe; faster is fatal, and the walk has no margin at all above the
# speed it was tuned at. Raising these trades speed for the frame rate the motion
# survives: at 1.0 the walk needs Studio to hold ~57 fps, which is not something a
# machine can be relied on to do while it is also rendering.
GAIT_SCALE = 1.0     # walk, sidestep, turn -- the fragile ones
RISE_SCALE = 1.0     # the two get-ups
HANDICAP = SPEED_HANDICAP   # every action, from the profile; see --handicap
TELEGRAPH = ATTACK_TELEGRAPH   # attack wind-ups only; see --telegraph

# JOINT_ORDER is what the project poses; a model may carry more joints than that.
# ToddlerBot has wrists, a parallel waist and the gear halves of its linkages -- real
# degrees of freedom that nothing here drives. They are declared rather than inferred,
# so a joint going missing from a rename still fails loudly instead of being written off
# as an extra.
_missing = set(JOINT_ORDER) - set(JOINTS)
_unclaimed = set(JOINTS) - set(JOINT_ORDER) - set(EXTRA_JOINTS)
assert not _missing, f'JOINT_ORDER names joints the model does not have: {_missing}'
assert not _unclaimed, f'model joints neither posed nor declared extra: {_unclaimed}"'


# Whatever head joints this robot has, centred. microban has one; OP3 adds a tilt that
# nothing here drives, and a pose that simply omitted it would not be a complete pose.
HEAD_ZERO = {j: 0.0 for j in JOINT_ORDER if j.startswith('c_')}


def sg(joint):
    """Which way this robot's joint turns, in the canonical sense.

    Hand-authored angles below are written the way microban's joints turn: elbow and
    knee positive is flexion, pitch positive swings forward, and the two sides of the
    body share a convention. A robot whose model does not agree -- OP3 mirrors its
    right arm and leg where microban does not -- declares the difference once, in
    SIGN, instead of having it smeared through every pose. Angles that came out of the
    IK are already in the model's own terms and never go through here.
    """
    return SIGN.get(joint, 1)

_cache = {}


def legs(rx, ry, rz, lx, ly, lz, ryaw=0.0, lyaw=0.0):
    """Both legs from foot targets in the trunk frame -> joint angles (deg)."""
    out = {}
    for side, (x, y, z, yaw) in (('right', (rx, ry, rz, ryaw)), ('left', (lx, ly, lz, lyaw))):
        key = (side, round(x, 6), round(y, 6), round(z, 6), round(yaw, 4))
        if key not in _cache:
            q, pe, re = solve_leg(side, (x, y, z), yaw)
            if pe > 1e-5 or re > 1e-4:
                raise SystemExit(f'IK failed {key}: pos {pe:.2e} rot {re:.2e}')
            _cache[key] = q
        out.update(_cache[key])
    return out


def pose(rf, lf, ryaw=0.0, lyaw=0.0, **upper):
    """rf/lf are (x, y, z) foot targets; upper overrides arm/head angles."""
    q = legs(*rf, *lf, ryaw=ryaw, lyaw=lyaw)
    q.update(ARM_HOME)
    q.update(upper)
    return q


ZERO = {n: 0.0 for n in JOINT_ORDER}
HOME = pose((X0, -Y0, Z0), (X0, Y0, Z0))
CROUCH = pose((X0, -Y0, Z0 + 0.012), (X0, Y0, Z0 + 0.012))


def walk_cycle(step, arm=None):
    """8 keyframes of a statically stable walk. step > 0 walks forward.

    `arm` is the counter-swing amplitude; tune_gait.py searches it, so it comes from
    the profile rather than being fixed here.
    """
    arm = ARM_SWING if arm is None else arm
    b, f = -step, step          # trailing / leading foot x
    k = []

    def P(rx, ry_off, rz, lx, ly_off, lz, sh_r, sh_l):
        return pose((X0 + rx, -Y0 + ry_off, Z0 + rz), (X0 + lx, Y0 + ly_off, Z0 + lz),
                    r_shoulder_yp=ARM_HOME['r_shoulder_yp'] + sg('r_shoulder_yp') * sh_r,
                    l_shoulder_yp=ARM_HOME['l_shoulder_yp'] + sg('l_shoulder_yp') * sh_l)

    # The two half cycles do not shift by the same amount. They should, by symmetry,
    # but the measured walk drifted ~6 mm sideways per step: the swing leg's own inertia
    # pulls the centre of mass differently depending on which way the trunk is already
    # leaning, and WY_D is the trim that cancels it.
    wl, wr = WY + WY_D, WY - WY_D
    # trunk shifted onto the left foot -> feet move -wl in the trunk frame
    k.append(('shift_L', P(b, -wl, 0, f, -wl, 0, -arm, arm), F_SHIFT))
    k.append(('lift_R', P(b, -wl, LIFT, f, -wl, 0, -arm, arm), F_LIFT))
    k.append(('swing_R', P(f, -wl, LIFT, b, -wl, 0, arm, -arm), F_SWING))
    k.append(('place_R', P(f, -wl, 0, b, -wl, 0, arm, -arm), F_PLACE))
    k.append(('shift_R', P(f, wr, 0, b, wr, 0, arm, -arm), F_SHIFT))
    k.append(('lift_L', P(f, wr, 0, b, wr, LIFT, arm, -arm), F_LIFT))
    k.append(('swing_L', P(b, wr, 0, f, wr, LIFT, -arm, arm), F_SWING))
    k.append(('place_L', P(b, wr, 0, f, wr, 0, -arm, arm), F_PLACE))
    return k


def turn_slip_cycle(step_r, step_l, arm=None):
    """8 keyframes that turn by stepping ASYMMETRICALLY, for a leg with no hip yaw.

    turn_cycle() spins the trunk over a planted sole by yawing the feet, which takes a
    hip yaw joint. Without one the soles cannot rotate relative to the trunk at all, so
    the turn has to come from somewhere else: give one leg a longer stride than the
    other and the body swings round the short side, the planted sole scuffing to absorb
    the difference. It is how a differential drive steers, and it is what a five-joint
    leg has available.

    Positive (step_l > step_r) turns to the RIGHT: the left leg covers more ground.

    The stride difference alone is not enough: with no hip yaw the trunk can only rotate
    relative to a planted sole by SLIPPING it, and a difference big enough to break that
    friction is also big enough to fall over (measured: stable settings yaw 1 degree,
    the ones that yaw 140 fall in every trial). So the turn is driven by the arms as
    well -- swung IN PHASE rather than antiphase, which puts a yaw reaction into the
    trunk instead of cancelling one, timed to land while a foot is off the ground and
    the sole under the other is easiest to scuff round.
    """
    arm = ARM_SWING if arm is None else arm
    yaw_arm = TURN_ARM * (1.0 if step_r > step_l else -1.0)
    br, fr = -step_r, step_r
    bl, fl = -step_l, step_l

    def P(rx, ry_off, rz, lx, ly_off, lz, sh_r, sh_l):
        return pose((X0 + rx, -Y0 + ry_off, Z0 + rz), (X0 + lx, Y0 + ly_off, Z0 + lz),
                    r_shoulder_yp=ARM_HOME['r_shoulder_yp'] + sg('r_shoulder_yp') * sh_r,
                    l_shoulder_yp=ARM_HOME['l_shoulder_yp'] + sg('l_shoulder_yp') * sh_l)

    wl, wr = WY + WY_D, WY - WY_D
    a, b = arm + yaw_arm, -arm + yaw_arm     # in-phase component rides on the swing
    return [('shift_L', P(br, -wl, 0, fl, -wl, 0, -arm, arm), F_SHIFT_T),
            ('lift_R', P(br, -wl, LIFT, fl, -wl, 0, -arm, arm), F_LIFT_T),
            ('swing_R', P(fr, -wl, LIFT, bl, -wl, 0, a, b), F_SWING_T),
            ('place_R', P(fr, -wl, 0, bl, -wl, 0, a, b), F_PLACE_T),
            ('shift_R', P(fr, wr, 0, bl, wr, 0, arm, -arm), F_SHIFT_T),
            ('lift_L', P(fr, wr, 0, bl, wr, LIFT, arm, -arm), F_LIFT_T),
            ('swing_L', P(br, wr, 0, fl, wr, LIFT, b, a), F_SWING_T),
            ('place_L', P(br, wr, 0, fl, wr, 0, b, a), F_PLACE_T)]


def scuff_turn_cycle(a, f_push, f_ret, arm=None):
    """2 keyframes that turn on the spot by scuffing BOTH soles, for a leg with no hip yaw.

    turn_slip_cycle() above is the honest failure. It tried to yaw the trunk over one
    planted sole, and the sums say that cannot work here: scuffing a single sole round
    takes 98 mN.m, and the arms swung in phase supply 2.8 of it and the swing leg 6.5.

    What that reasoning missed is that the LEGS can push on the ground, and they are not
    weak. Stand in double support and drive one foot forward and the other back by the
    same `a`: the feet cannot move, so the trunk turns instead, and the angle it turns
    through is set by geometry rather than by anything having to overcome anything --
    atan(a / Y0), which for a = 36 mm on a 33 mm half-stance is a great deal of yaw. Both
    soles scuff, but they scuff about a point BETWEEN them, driven by a couple the hip
    pitches make against the floor rather than by the arms waving.

    That much is symmetric and would undo itself on the way back, so the two strokes are
    deliberately not alike: the push is fast enough to break the soles loose and the
    return is slow enough that they stay stuck and the trunk keeps what it gained. It is
    the same trick as shuffling a heavy box round on a smooth floor, and it is what the
    "surge one way, creep back" timing below is for -- f_push 8 frames against f_ret 40.

    Positive `a` turns to the RIGHT (the left foot goes forward). Measured on the Mini:
    -27.5 deg per 0.48 s cycle with both feet on the ground the whole time.
    """
    def P(rx, lx):
        return pose((X0 + rx, -Y0, Z0), (X0 + lx, Y0, Z0))

    return [('push', P(-a, a), f_push),
            ('recover', P(0.0, 0.0), f_ret)]


def sidestep_cycle(d):
    """8 keyframes stepping sideways by d each half cycle (d > 0 steps left).

    The trunk is always parked INSET inboard of the *support foot*, not at a fixed
    offset from the stance midpoint -- after a step out the stance is wider, and a
    midpoint-relative shift leaves the CoM off the edge of the support sole.
    """
    def P(rw, rz, lw, lz, ty):
        return pose((X0, rw - ty, Z0_S + rz), (X0, lw - ty, Z0_S + lz))

    rw, lw = -Y0, Y0                       # foot y in the world, trunk starts at 0
    k = []
    if d > 0:                              # lead with the left foot
        ty = rw + INSET
        k += [('shift_R', P(rw, 0, lw, 0, ty), F_SHIFT_S),
              ('lift_L', P(rw, 0, lw, LIFT, ty), F_LIFT_S),
              ('move_L', P(rw, 0, lw + d, LIFT, ty), F_SWING_S),
              ('place_L', P(rw, 0, lw + d, 0, ty), F_PLACE_S)]
        lw += d
        ty = lw - INSET
        k += [('shift_L', P(rw, 0, lw, 0, ty), F_SHIFT_S),
              ('lift_R', P(rw, LIFT, lw, 0, ty), F_LIFT_S),
              ('move_R', P(rw + d, LIFT, lw, 0, ty), F_SWING_S),
              ('place_R', P(rw + d, 0, lw, 0, ty), F_PLACE_S)]
    else:                                  # lead with the right foot
        ty = lw - INSET
        k += [('shift_L', P(rw, 0, lw, 0, ty), F_SHIFT_S),
              ('lift_R', P(rw, LIFT, lw, 0, ty), F_LIFT_S),
              ('move_R', P(rw + d, LIFT, lw, 0, ty), F_SWING_S),
              ('place_R', P(rw + d, 0, lw, 0, ty), F_PLACE_S)]
        rw += d
        ty = rw + INSET
        k += [('shift_R', P(rw, 0, lw, 0, ty), F_SHIFT_S),
              ('lift_L', P(rw, 0, lw, LIFT, ty), F_LIFT_S),
              ('move_L', P(rw, 0, lw + d, LIFT, ty), F_SWING_S),
              ('place_L', P(rw, 0, lw + d, 0, ty), F_PLACE_S)]
    return k


def turn_cycle(phi):
    """8 keyframes turning phi per half cycle (phi > 0 turns left / CCW).

    The trunk rotation happens in the 'spin' keyframe: the loaded foot's yaw goes
    back to 0 in the trunk frame, which can only mean the trunk itself rotated.
    The airborne foot keeps yaw 0 and is carried round to the new heading.
    """
    def P(ry_off, rz, ryaw, ly_off, lz, lyaw):
        return pose((X0, -Y0 + ry_off, Z0 + rz), (X0, Y0 + ly_off, Z0 + lz),
                    ryaw=ryaw, lyaw=lyaw)

    o = WY if phi > 0 else -WY      # feet shift +WY when the trunk leans onto the right foot
    if phi > 0:
        return [('shift_R', P(o, 0, 0, o, 0, 0), F_SHIFT_T),
                ('lift_L', P(o, 0, 0, o, LIFT, 0), F_LIFT_T),
                ('turn_L', P(o, 0, 0, o, LIFT, phi), F_SWING_T),
                ('place_L', P(o, 0, 0, o, 0, phi), F_PLACE_T),
                ('shift_L', P(-o, 0, 0, -o, 0, phi), F_SHIFT_T),
                ('lift_R', P(-o, LIFT, 0, -o, 0, phi), F_LIFT_T),
                ('spin', P(-o, LIFT, 0, -o, 0, 0), F_SWING_T),
                ('place_R', P(-o, 0, 0, -o, 0, 0), F_PLACE_T)]
    return [('shift_L', P(o, 0, 0, o, 0, 0), F_SHIFT_T),
            ('lift_R', P(o, LIFT, 0, o, 0, 0), F_LIFT_T),
            ('turn_R', P(o, LIFT, phi, o, 0, 0), F_SWING_T),
            ('place_R', P(o, 0, phi, o, 0, 0), F_PLACE_T),
            ('shift_R', P(-o, 0, phi, -o, 0, 0), F_SHIFT_T),
            ('lift_L', P(-o, 0, phi, -o, LIFT, 0), F_LIFT_T),
            ('spin', P(-o, 0, 0, -o, LIFT, 0), F_SWING_T),
            ('place_L', P(-o, 0, 0, -o, 0, 0), F_PLACE_T)]


# ---------------- hand-authored whole-body sequences ----------------
def upper(**kw):
    q = dict(ZERO)
    q.update(kw)
    return q


def arm_angle(joint, v):
    """One canonical arm angle -> this robot's model angle.

    ARM_ZERO carries the model angle that means 'arm hanging at the side', which is
    zero on microban and +-90 on OP3, whose shoulders are modelled in a T-pose. Every
    arm angle in this file goes through here; a pose that skips it lands 90 degrees out
    on OP3 and is correct by accident on microban.
    """
    zero = ARM_ZERO.get(joint)
    return sg(joint) * v if zero is None else zero + sg(joint) * v


def armset(p, r, e):
    """Both arms from canonical angles: pitch forward, roll out, elbow bent."""
    # A robot without a shoulder roll (the 12-servo microban_12) simply has no joint to
    # put the roll on; the arm hangs at the side and the other two take the pose.
    return {j: arm_angle(j, v) for j, v in
            (('l_shoulder_yp', p), ('r_shoulder_yp', p), ('l_shoulder_xr', r),
             ('r_shoulder_xr', -r), ('l_elbow_yp', e), ('r_elbow_yp', e))
            if j in JOINT_ORDER}


def legset(h, k, a, **kw):
    q = {n: 0.0 for n in JOINT_ORDER}
    for side in ('left', 'right'):
        for base, v in (('hip_pitch', h), ('knee', k), ('ankle_pitch', a)):
            if jn(side, base) in JOINT_ORDER:      # no ankle-pitch servo on the parallel-link legs
                q[jn(side, base)] = sg(jn(side, base)) * v
    q.update(kw)
    return q


LEG_NAMES = list(LEG_ORDER)


def getup_pose(legs, arm):
    """One get-up keyframe: the right leg as given, the left mirrored."""
    q = {n: 0.0 for n in JOINT_ORDER}
    for i, n in enumerate(LEG_NAMES):
        q[jn('right', n)] = sg(jn('right', n)) * legs[i]
        q[jn('left', n)] = sg(jn('left', n)) * LEG_MIRROR[i] * legs[i]
    q.update(armset(*arm))
    return q


# The rise only works from face-down, so this is the face-down action verbatim: no
# roll-over, the arms plant straight away. (Both actions used to lead with the roll so
# that either button worked whichever way the robot had landed; that cost 2.6 s here
# for nothing, and the gamepad already has a separate button per orientation.)
# lie_flat's own duration is not tuned: the simulation starts from this pose, so it
# is only ever the move from whatever pose the robot fell in, which is unknown.
STANDUP_FACE_DOWN = [(n, legset(0, 0, 0, **armset(*a)), f) for n, a, f in PRONE_PREP] \
    + [(f'rise_{i}', getup_pose(*p), F_RISE_D[i]) for i, p in enumerate(GETUP_PRONE)]



# lie_flat is where the robot waits for the floor to catch it, so its own duration
# is whatever it takes to fold the arms out from an unknown fallen pose, not a
# tuned number -- the simulation starts from this pose and never plays the move.
STANDUP_FACE_UP = [(n, legset(0, 0, 0, **armset(*a)), f) for n, a, f in SUPINE_PREP]
for _i, _p in enumerate(GETUP_SUPINE):
    STANDUP_FACE_UP.append((f'rise_{_i}', getup_pose(*_p), F_RISE_U[_i]))
    STANDUP_FACE_UP.append((f'hold_{_i}', getup_pose(*_p), F_HOLD_U[_i]))

# A robot that cannot rise from its front can still roll onto its back and use the rise
# it does have. ROLL_TO_SUPINE holds whole-body poses, not the 9-parameter mirrored kind
# the roadmap uses: rolling is asymmetric by nature, and it is momentum rather than
# static stability that turns the trunk over. microban does the same thing in reverse --
# its verified rise is the prone one -- and dropped the roll only because it had both.
# The supine sequence joins in FULL, lie_flat included. That keyframe is not redundant
# just because the roll already left the robot on its back: the rise is carried by
# momentum, and it has to start from rest in the pose it was searched from. Skipping it
# hands the rise the roll's leftover velocity, and the last keyframe -- the big one that
# swings the trunk up -- overshoots and throws the robot onto its face.
def _roll_keys(roll=None):
    """The roll poses, where an empty pose means 'hold the one before it'.

    The hold is not padding: the rise that follows is a momentum move searched from a
    robot at rest, and handing it the roll's leftover velocity makes its last keyframe
    overshoot.
    """
    out, prev_roll = [], {}
    for n, q, f in (ROLL_TO_SUPINE if roll is None else roll):
        prev_roll = q or prev_roll
        out.append((n, dict(ZERO, **prev_roll), f))
    return out


if ROLL_TO_SUPINE:
    # Between the roll and the rise the robot has to come to a STOP, and it has to do it
    # in the pose the rise starts from. Measured on the Mini: the roll leaves the trunk
    # turning at 1.79 rad/s, and the rise -- searched from a robot at rest -- loses about
    # a third of the height its first keyframe is worth and never recovers. Holding for
    # F_SETTLE_ROLL frames in STANDUP_FACE_UP's own first pose brings it to 0.001 rad/s,
    # at which point the state matches the standalone rise to the millimetre.
    # Holding in the ROLL's last pose instead does not work and is worse than no hold at
    # all: that pose has the shoulders and hips at their stops, so the servos sit pegged
    # against the floor for the whole hold, and the robot creeps out of square.
    _flat_n, _flat_q, _flat_f = STANDUP_FACE_UP[0]
    STANDUP_FACE_DOWN = ([(n, legset(0, 0, 0, **armset(*a)), f) for n, a, f in PRONE_PREP]
                         + _roll_keys()
                         + [(_flat_n, _flat_q, _flat_f + F_SETTLE_ROLL)]
                         + STANDUP_FACE_UP[1:])

# The mirror image: a robot whose verified rise is the PRONE one rolls onto its front
# first. microban_pl is this case -- its parallel-link legs cannot make the sitting
# poses a direct supine rise needs (the sole is slaved to the pelvis), but a face-down
# rise was found, so face-up = roll over + that rise. Same settle logic as above.
ROLL_TO_PRONE = globals().get('ROLL_TO_PRONE', [])
if ROLL_TO_PRONE:
    _flat_n, _flat_q, _flat_f = STANDUP_FACE_DOWN[0]
    STANDUP_FACE_UP = ([(n, legset(0, 0, 0, **armset(*a)), f) for n, a, f in SUPINE_PREP]
                       + _roll_keys(ROLL_TO_PRONE)
                       + [(_flat_n, _flat_q, _flat_f + F_SETTLE_ROLL)]
                       + STANDUP_FACE_DOWN[1:])


def _roty(deg):
    c, sn = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return [[c, 0.0, sn], [0.0, 1.0, 0.0], [-sn, 0.0, c]]


def tilted_legs(theta, x_off, z, yaw=0.0, feet=None):
    """Both legs, soles flat on the floor, with the trunk pitched `theta` deg forward.

    `x_off`/`z` place both feet symmetrically relative to the trunk origin, in the world
    frame: positive x is in front. Pass `feet` as ((rx, ry, rz), (lx, ly, lz)) instead
    when the stance is split. Pitching the trunk without moving the feet throws the mass
    out past the toes, so the caller has to choose the offset deliberately -- bow_pose
    solves it for balance.
    """
    import numpy as np
    R = np.array(_roty(-theta))          # R_trunk^T, keeps the soles flat in the world
    if feet is None:
        feet = ((x_off, -Y0, z), (x_off, Y0, z))
    q = {}
    for side, p in (('right', feet[0]), ('left', feet[1])):
        sol, pe, re = solve_leg(side, R @ np.array(p, float), yaw, R_extra=R)
        if pe > 1e-5 or re > 1e-4:
            raise SystemExit(f'tilted_legs IK failed theta={theta} foot={p}')
        q.update(sol)
    return q


def bow_pose(theta, arm_pitch=0.0, arm_roll=6.0, elbow=-8.0, z=None):
    """Standing bow: the trunk pitches `theta` deg forward, the soles stay on the floor.

    Pitching the trunk throws the centre of mass out over the toes, so the feet cannot
    stay where they are -- the hips have to travel back by however far the mass moved
    forward. That offset is solved for here rather than guessed: bisect the foot x
    offset until the pose's own centre of mass sits over the middle of the soles.
    """
    import numpy as np
    from ik import _m, _d, qadr, LEG
    import mujoco
    R = np.array(_roty(-theta))          # R_trunk^T, keeps the soles flat in the world
    zz = Z0 if z is None else z
    arms = dict(armset(arm_pitch, arm_roll, elbow), **HEAD_ZERO)

    def build(xo):
        q = tilted_legs(theta, xo, zz)
        q.update(arms)
        return q

    def com_x(q):
        _d.qpos[:] = 0.0
        _d.qpos[3] = 1.0
        for n, v in q.items():
            _d.qpos[qadr(n)] = math.radians(v)
        mujoco.mj_forward(_m, _d)
        # the model holds the trunk upright at the origin, so rotate its CoM into the
        # world the pitched trunk actually lives in
        return float((np.array(_roty(theta)) @ _d.subtree_com[0])[0])

    lo, hi = BOW_XO
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        q = build(mid)
        if com_x(q) - mid > 0:           # mass still ahead of the soles: move them forward
            lo = mid
        else:
            hi = mid
    xo = 0.5 * (lo + hi)
    if min(abs(xo - BOW_XO[0]), abs(xo - BOW_XO[1])) < 1e-4:
        raise SystemExit(f'bow: balance point ran into the search bound at x={xo:.4f}')
    return build(xo)


def bow_keys():
    tall = bow_pose(0.0, z=BOW_Z, **BOW_ARMS)
    down = bow_pose(BOW_ANGLE, z=BOW_Z, **BOW_ARMS)
    return [('stand_tall', tall, 55), ('bow_down', down, 65), ('bow_hold', down, 45),
            ('bow_up', tall, 65), ('settle', tall, 25)]


def jab(side):
    """Guard -> jab -> retract. `side` is the punching arm, and the lead foot."""
    p = JAB
    sgn = 1.0 if side == 'left' else -1.0
    other = 'right' if side == 'left' else 'left'
    y, z = p['STANCE_Y'], p['STANCE_Z']
    rx = p['FRONT_X'] if side == 'right' else p['REAR_X']
    lx = p['FRONT_X'] if side == 'left' else p['REAR_X']

    def stance(lean):
        return legs(rx - lean, -y, z, lx - lean, y, z)

    guard = stance(0.0)
    guard.update(armset(p['G_PITCH'], p['G_ROLL'], p['G_ELBOW']))
    guard.update(HEAD_ZERO)
    punch = stance(p['P_LEAN'])
    punch.update({k: v for k, v in guard.items()
                  if 'shoulder' in k or 'elbow' in k or k in HEAD_ZERO})
    sp, sr, el = jn(side, 'shoulder_pitch'), jn(side, 'shoulder_roll'), jn(side, 'elbow')
    punch[sp] = arm_angle(sp, p['P_PITCH'])
    if sr in JOINT_ORDER:
        punch[sr] = arm_angle(sr, sgn * p['P_ROLL'])
    punch[el] = arm_angle(el, p['P_ELBOW'])
    op = jn(other, 'shoulder_pitch')
    punch[op] = arm_angle(op, p['G_PITCH'] + p['P_OFFARM'])
    if 'c_neck_zy' in HEAD_ZERO:          # a robot without a neck cannot look at it
        punch['c_neck_zy'] = arm_angle('c_neck_zy', sgn * 12.0)
    # The hip twist needs hip yaw, which not every robot has; without it the punch is
    # thrown from a square stance.
    for hip in ('l_hipjoint_zy', 'r_hipjoint_zy'):
        if hip in JOINT_ORDER:
            punch[hip] = sg(hip) * sgn * p['P_TWIST']
    return guard, punch


def kick_keys(side='left'):
    """Front kick with `side`'s leg; the other leg supports."""
    p = KICK
    pre = 'l_' if side == 'left' else 'r_'
    y, z = p['STANCE_Y'], p['STANCE_Z']
    sgn = 1.0 if side == 'left' else -1.0     # the trunk moves over the OTHER foot

    def support(lean, shift=1.0):
        o = p['WY_K'] * shift * sgn
        return tilted_legs(lean, 0.0, 0.0,
                           feet=((p['SUP_X'], -y + o, z), (p['SUP_X'], y + o, z)))

    def swing(q, hip, knee, ank):
        # the kicking leg leaves the IK behind: a chambered knee is not a foot position
        q = {k: v for k, v in q.items() if not k.startswith(pre + 'hip')
             and not k.startswith(pre + 'knee') and not k.startswith(pre + 'ank')}
        # canonical angles, like every other hand-authored pose in this file: the
        # kicking leg leaves the IK behind, so nothing else is converting them
        if pre + 'hipjoint_zy' in JOINT_ORDER:
            q[pre + 'hipjoint_zy'] = 0.0
        q.update({pre + 'hipjoint_xr': 0.0,
                  pre + 'hipjoint_yp': sg(pre + 'hipjoint_yp') * hip,
                  pre + 'knee_yp': sg(pre + 'knee_yp') * knee,
                  pre + 'ankle_yp': sg(pre + 'ankle_yp') * ank,
                  pre + 'ankle_xr': 0.0})
        return q

    def arm(q):
        q.update(_arms(p['A_PITCH'], p['A_ROLL'], p['A_ELBOW']))
        return q

    ready = arm(support(0.0, 0.0))
    shifted = arm(support(0.0))
    cham = arm(swing(support(p['K_LEAN']), p['CH_HIP'], p['CH_KNEE'], 0.0))
    strike = arm(swing(support(p['K_LEAN']), p['K_HIP'], p['K_KNEE'], p['K_ANK']))
    return [('shift', shifted, F_K_SHIFT), ('chamber', cham, F_K_CHAMBER),
            ('kick', strike, F_K_KICK), ('hold', strike, F_K_HOLD),
            ('recover', cham, F_K_BACK), ('down', ready, F_K_DOWN)]


# both arms at the same pitch/elbow, rolled symmetrically outward
def _arms(pitch, roll, elbow):
    return dict(armset(pitch, roll, elbow), **HEAD_ZERO)


# ---- uppercut -----------------------------------------------------------------------
# The jab already lands ABOVE a big opponent's centre of mass; what it lacks is impulse.
# Measured against OP3, microban's jab delivers 1.09 N.s and its front kick 4.20 N.s --
# the leg carries four times the arm's momentum, which is why the kick wins on toppling
# moment despite landing at a third of the height.
#
# So the uppercut does not try to punch harder with the arm. It drops into a crouch and
# then EXTENDS THE LEGS through the strike, so the whole body's upward momentum arrives
# at the fist. The arm swings from behind the hip to above the shoulder over the same
# keyframe; the legs are what actually pay for the blow.
def uppercut_keys(side):
    """Crouch -> load -> drive up -> recover. `side` is the punching arm."""
    p = UPPER
    sgn = 1.0 if side == 'left' else -1.0
    other = 'right' if side == 'left' else 'left'
    y = p['STANCE_Y']
    x = p['STANCE_X']

    def stance(z, lean=0.0):
        return legs(x - lean, -y, z, x - lean, y, z)

    def arms(pitch, roll, elbow, punch=None):
        q = armset(pitch, roll, elbow)
        q.update(HEAD_ZERO)
        if punch is not None:
            pp, pr, pe = punch
            q[jn(side, 'shoulder_pitch')] = arm_angle(jn(side, 'shoulder_pitch'), pp)
            if jn(side, 'shoulder_roll') in JOINT_ORDER:
                q[jn(side, 'shoulder_roll')] = arm_angle(jn(side, 'shoulder_roll'), sgn * pr)
            q[jn(side, 'elbow')] = arm_angle(jn(side, 'elbow'), pe)
        return q

    guard = stance(p['Z_MID'])
    guard.update(arms(p['G_PITCH'], p['G_ROLL'], p['G_ELBOW']))
    load = stance(p['Z_LOW'])
    load.update(arms(p['G_PITCH'], p['G_ROLL'], p['G_ELBOW'],
                     punch=(p['L_PITCH'], p['L_ROLL'], p['L_ELBOW'])))
    rise = stance(p['Z_HIGH'], p['U_LEAN'])
    rise.update(arms(p['G_PITCH'], p['G_ROLL'], p['G_ELBOW'],
                     punch=(p['U_PITCH'], p['U_ROLL'], p['U_ELBOW'])))
    for q in (load, rise):
        for hip in ('l_hipjoint_zy', 'r_hipjoint_zy'):
            if hip in JOINT_ORDER:          # the twist needs hip yaw; not every robot has it
                q[hip] = sg(hip) * sgn * p['U_TWIST']
    return [('guard', guard, F_U_GUARD), ('load', load, F_U_LOAD),
            ('rise', rise, F_U_RISE), ('hold', rise, F_U_HOLD),
            ('recover', guard, F_U_BACK)]


# ---------------- XML emission ----------------
POSE_ATTRS = dict(branching_enabled='False', branch_outputs_swapped='False',
                  branch_if_left='UserVal_0', branch_if_op='==', branch_if_right='UserVal_1',
                  branch_if_uv_enabled='True', branch_if_formula_enabled='False',
                  branch_if_formula='Form1:foo', branch_if_pad_enabled='False',
                  branch_if_pad_button='L1', branch_if_pad_analog_enabled='False',
                  branch_if_pad_analog_axis='Lx', branch_if_pad_analog_op='>=',
                  branch_if_pad_analog_threshold='0')


def check(name, q):
    for j, v in q.items():
        lo, hi = LIMITS[j]
        if not (lo - 1e-6 <= v <= hi + 1e-6):
            raise SystemExit(f'{name}: {j}={v:.2f} outside [{lo:.1f},{hi:.1f}]')


class Action:
    def __init__(self, title, scale=1.0):
        self.title = title
        # HANDICAP slows EVERY action, on top of whatever per-family scale it already
        # has. It is a fight-balance knob, not a robustness one: a bigger robot that
        # moves at the same rate as a small one covers more ground per second and swings
        # a longer arm through it. Note what it does NOT change -- how hard the robot is
        # to knock over. That is set by its mass and its ankle torque, and stays put.
        self.scale = scale * HANDICAP
        self.nodes = []
        self.edges = []
        self.start = None

    def add_pose(self, nid, name, q, frames=10, x=50.0, y=100.0, easing='3: easeInOutSine'):
        check(f'{self.title}/{name}', q)
        # Stretching a keyframe can only ever relax the servo speed limiter, never
        # violate it, so the scale needs no floor beyond one frame.
        frames = max(1, int(round(frames * self.scale)))
        self.nodes.append(('pose', nid, name, q, frames, x, y, easing, {}))
        return nid

    def add_branch(self, nid, name, x, y, **kw):
        self.nodes.append(('branch', nid, name, None, None, x, y, None, kw))
        return nid

    def add_jump(self, nid, name, target, x, y):
        self.nodes.append(('jump', nid, name, None, None, x, y, None, {'target': target}))
        return nid

    def link(self, a, b, port=0, label='default', priority=0):
        self.edges.append((a, b, port, label, priority))


def emit(root, actions):
    order = {a.title: i for i, a in enumerate(actions)}

    def index_of(target):
        """Jumps name the action they go to; a skipped action leaves no dangling index."""
        return target if isinstance(target, int) else order[target]

    acts = ET.SubElement(root, 'Actions', currentIndex='0')
    for i, a in enumerate(actions):
        ae = ET.SubElement(acts, 'Action', index=str(i), title=a.title)
        mf = ET.SubElement(ae, 'MotionFormulas')
        for k in ('Form1:foo', 'Form2:bar'):
            ET.SubElement(mf, 'Formula', key=k).text = '\n'.join(['(Not Available Now)'] * 5)
        ne = ET.SubElement(ae, 'Nodes')
        for kind, nid, name, q, frames, x, y, easing, kw in a.nodes:
            if kind == 'pose':
                attrs = dict(id=nid, type='pose', name=name, pos_x=f'{x}', pos_y=f'{y}',
                             duration='1.0', frames=str(frames), **POSE_ATTRS)
                n = ET.SubElement(ne, 'Node', attrs)
                ang = ET.SubElement(n, 'Angles')
                eas = ET.SubElement(n, 'Easings')
                for j in JOINT_ORDER:
                    ET.SubElement(ang, 'Angle', joint=j, value=f'{round(q[j], 3)}')
                    ET.SubElement(eas, 'Easing', joint=j, value=easing)
                ports = ET.SubElement(n, 'OutputPorts')
                ET.SubElement(ports, 'Port', label='default', priority='0')
            elif kind == 'branch':
                attrs = dict(id=nid, type='branch', name=name, pos_x=f'{x}', pos_y=f'{y}',
                             branching_enabled='True', branch_outputs_swapped='False',
                             branch_if_left='UserVal_0', branch_if_op='==',
                             branch_if_right='UserVal_1', branch_if_uv_enabled='False',
                             branch_if_formula_enabled='False', branch_if_formula='Form1:foo',
                             branch_if_pad_enabled=kw.get('pad_en', 'False'),
                             branch_if_pad_button=kw.get('button', 'L1'),
                             branch_if_pad_analog_enabled=kw.get('ana_en', 'False'),
                             branch_if_pad_analog_axis=kw.get('axis', 'Lx'),
                             branch_if_pad_analog_op=kw.get('op', '>='),
                             branch_if_pad_analog_threshold=kw.get('th', '0'))
                n = ET.SubElement(ne, 'Node', attrs)
                ports = ET.SubElement(n, 'OutputPorts')
                ET.SubElement(ports, 'Port', label='else', priority='0')
                ET.SubElement(ports, 'Port', label='then', priority='10')
            else:
                n = ET.SubElement(ne, 'Node', id=nid, type='jump', name=name,
                                  pos_x=f'{x}', pos_y=f'{y}',
                                  jump_target_action_index=str(index_of(kw['target'])),
                                  jump_type='action', jump_target_function='')
                ports = ET.SubElement(n, 'OutputPorts')
                ET.SubElement(ports, 'Port', label='default', priority='0')
        ee = ET.SubElement(ae, 'Edges')
        for f, t, port, label, prio in a.edges:
            ET.SubElement(ee, 'Edge', {'from': f, 'to': t, 'from_port': str(port),
                                       'label': label, 'priority': str(prio)})
        ET.SubElement(ee, 'Edge', {'from': 'start', 'to': a.start, 'from_port': '0',
                                   'label': '', 'priority': '0'})
        ET.SubElement(ae, 'Playback', start_node_id=a.start, interpolation='0: linear',
                      fps='100', branch_mode='default-first')


def chain(a, keys, frames, prefix='pose', x=50.0, y0=100.0, dy=90.0):
    """keys entries are (name, angles) or (name, angles, frames)."""
    ids = []
    for i, k in enumerate(keys):
        name, q = k[0], k[1]
        fr = k[2] if len(k) > 2 else frames
        ids.append(a.add_pose(f'{prefix}_{i}', f'{i}_{name}', q, fr, x, y0 + dy * i))
    return ids


def cyclic_action(title, keys, exit_axis, exit_op, exit_th, back_to='Base', scale=1.0):
    """home -> cycle keyframes -> branch (stick still held? loop : leave) -> home -> Base."""
    a = Action(title, scale)
    ids = []
    for i, (name, q, fr) in enumerate([('home', HOME, F_HOME)] + keys):
        ids.append(a.add_pose(f'pose_{i}', f'{i}_{name}', q, fr,
                              50.0 + 220.0 * (i % 4), 100.0 + 140.0 * (i // 4)))
    tail = a.add_pose(f'pose_{len(ids)}', 'exit_home', HOME, F_HOME,
                      50.0 + 220.0 * (len(ids) % 4), 100.0 + 140.0 * (len(ids) // 4))
    br = a.add_branch('branch_0', 'keep_going', 700.0, 620.0, ana_en='True',
                      axis=exit_axis, op=exit_op, th=str(exit_th))
    jp = a.add_jump('jump_0', f'-> Base', back_to, 700.0, 760.0)
    a.start = ids[0]
    for i in range(len(ids) - 1):
        a.link(ids[i], ids[i + 1])
    a.link(ids[-1], br)
    a.link(br, tail, port=0, label='else', priority=0)
    a.link(br, ids[1], port=1, label='then', priority=10)
    a.link(tail, jp)
    return a


# The keyframes an opponent can read: the robot is loading, not yet striking. Scaling
# only these lengthens the tell without slowing the blow, which is what makes an attack
# duckable rather than merely slow. A uniformly slowed attack is still unreadable, it
# just arrives later.
# Not 'shift'. The kick's weight transfer is a wind-up in the sense that it comes first,
# but stretching it just leaves the robot balanced on one leg for 2.4 s and it drifts off
# (10/10 -> 8/10). The chamber -- knee up, heel tucked -- is the unmistakable tell, and
# doubling that alone is both readable and stable.
TELEGRAPH_KEYS = ('guard', 'chamber', 'load')


def linear_action(title, keys, frames, back_to='Base', scale=1.0, telegraph=1.0):
    if telegraph != 1.0:
        keys = [(n, q, (max(1, round(f * telegraph)) if n in TELEGRAPH_KEYS else f))
                for n, q, f in keys]
    a = Action(title, scale)
    ids = chain(a, keys, frames)
    jp = a.add_jump('jump_0', '-> Base', back_to, 320.0, 100.0 + 90.0 * len(ids))
    a.start = ids[0]
    for i in range(len(ids) - 1):
        a.link(ids[i], ids[i + 1])
    a.link(ids[-1], jp)
    return a


def build():
    actions = []

    # 0 Boot
    a = Action('Boot')
    ids = chain(a, [('zero', dict(ZERO, **BOOT_ZERO)),
                    ('half', pose((X0, -Y0, BOOT_Z), (X0, Y0, BOOT_Z))),
                    ('home', HOME)], 60)
    a.nodes[0] = ('pose',) + a.nodes[0][1:7] + ('0: linear', {})
    jp = a.add_jump('jump_0', '-> Base', 'Base', 320.0, 400.0)
    a.start = ids[0]
    a.link(ids[0], ids[1]); a.link(ids[1], ids[2]); a.link(ids[2], jp)
    actions.append(a)

    # 2..9 gaits
    g = dict(scale=GAIT_SCALE)
    actions.append(cyclic_action('walk_forward', walk_cycle(STEP), 'Ly', '>=', 15, **g))
    actions.append(cyclic_action('walk_back', walk_cycle(-STEP), 'Ly', '<=', -15, **g))
    actions.append(cyclic_action('walk_sidestep_left', sidestep_cycle(SIDE), 'Lx', '<=', -15, **g))
    actions.append(cyclic_action('walk_sidestep_right', sidestep_cycle(-SIDE), 'Lx', '>=', 15, **g))
    # Turning in place spins the trunk over a planted sole, which takes hip yaw. A robot
    # without one (the ROBOTIS Mini) skips these rather than building a turn of zero
    # degrees.
    if 'walk_turn_left_small' not in SKIP_ACTIONS:
        if TURN_SCUFF:
            # no hip yaw: turn on the spot by scuffing both soles with a differential
            # push in double support. One cycle is worth more than a small turn needs,
            # so both sizes run the same cycle and stop on their own yaw condition.
            a, fp, fr = TURN_SCUFF
            for name, sign, cmp_, lim in (('left', -1.0, '<=', -19), ('right', 1.0, '>=', 19)):
                actions.append(cyclic_action(f'walk_turn_{name}_small',
                                             scuff_turn_cycle(sign * a, fp, fr),
                                             'Rx', cmp_, lim, **g))
            for name, sign, cmp_, lim in (('left', -1.0, '<=', -60), ('right', 1.0, '>=', 60)):
                actions.append(cyclic_action(f'walk_turn_{name}_big',
                                             scuff_turn_cycle(sign * a, fp, fr),
                                             'Rx', cmp_, lim, **g))
        elif TURN_SLIP:
            # no hip yaw: turn by stepping asymmetrically instead of yawing the feet
            s, b = TURN_SLIP['small'], TURN_SLIP['big']
            actions.append(cyclic_action('walk_turn_left_small',
                                         turn_slip_cycle(*s), 'Rx', '<=', -19, **g))
            actions.append(cyclic_action('walk_turn_right_small',
                                         turn_slip_cycle(*s[::-1]), 'Rx', '>=', 19, **g))
            actions.append(cyclic_action('walk_turn_left_big',
                                         turn_slip_cycle(*b), 'Rx', '<=', -60, **g))
            actions.append(cyclic_action('walk_turn_right_big',
                                         turn_slip_cycle(*b[::-1]), 'Rx', '>=', 60, **g))
        else:
            actions.append(cyclic_action('walk_turn_left_small', turn_cycle(PHI_S), 'Rx', '<=', -19, **g))
            actions.append(cyclic_action('walk_turn_right_small', turn_cycle(-PHI_S), 'Rx', '>=', 19, **g))
            actions.append(cyclic_action('walk_turn_left_big', turn_cycle(PHI_B), 'Rx', '<=', -60, **g))
            actions.append(cyclic_action('walk_turn_right_big', turn_cycle(-PHI_B), 'Rx', '>=', 60, **g))

    # 10/11 standups
    if 'standup_face_down' not in SKIP_ACTIONS:
        actions.append(linear_action('standup_face_down',
                                     STANDUP_FACE_DOWN + [('home', HOME, F_HOME_D)], 50,
                                     scale=RISE_SCALE))
    if 'standup_face_up' not in SKIP_ACTIONS:
        actions.append(linear_action('standup_face_up',
                                     STANDUP_FACE_UP + [('home', HOME, F_HOME_U)], 60,
                                     scale=RISE_SCALE))

    # 14 bow, appended after the jabs but dispatched from Base like the rest
    # 12/13 jabs
    for side, title in (('left', 'Attack_jab1_l'), ('right', 'Attack_jab1_r')):
        guard, punch = jab(side)
        a = Action(title)
        # The punch keyframe is the one deliberately fast move, but not faster than the
        # servo: the elbow travels ~105 deg and the XL330 tops out at 10.79 rad/s (6.2 deg
        # per frame at 100 fps), so under 17 frames LME's own speed limiter would clip the
        # command and the pose would never actually be reached.
        ids = chain(a, [('guard', guard, max(1, round(F_JAB_GUARD * TELEGRAPH))),
                        ('punch', punch, F_JAB_PUNCH),
                        ('hold', punch, F_JAB_HOLD), ('recover', guard, F_JAB_BACK),
                        ('home', HOME, F_JAB_HOME)], 20)
        jp = a.add_jump('jump_0', '-> Base', 'Base', 320.0, 460.0)
        a.start = ids[0]
        for i in range(len(ids) - 1):
            a.link(ids[i], ids[i + 1])
        a.link(ids[-1], jp)
        actions.append(a)

    # 14 ceremonial bow, on the Circle button. Guarded like the kick: a bow pitches the
    # trunk over flat soles, which a parallel-link leg (sole slaved to the pelvis) cannot
    # do, and bow_keys() would exit on the IK failure before the skip list is applied.
    if 'bow' not in SKIP_ACTIONS:
        actions.append(linear_action('bow', bow_keys() + [('home', HOME, 45)], 50))

    # 16 uppercut, on the Square button
    if UPPER and 'Attack_upper_l' not in SKIP_ACTIONS:
        for side, title in (('left', 'Attack_upper_l'), ('right', 'Attack_upper_r')):
            actions.append(linear_action(title, uppercut_keys(side)
                                         + [('home', HOME, F_U_HOME)], 30,
                                         telegraph=TELEGRAPH))

    # 15 front kick, on the Triangle button
    if 'kick' not in SKIP_ACTIONS:
        actions.append(linear_action('kick', kick_keys('left') + [('home', HOME, F_K_HOME)],
                                     50, telegraph=TELEGRAPH))
    # after the skip list, not before: a dispatch entry may only point at an action that
    # survives it
    built = {a.title for a in actions if a.title not in SKIP_ACTIONS} | {'Base'}
    # 1 Base: the gamepad dispatcher. Built LAST, because it needs to know which
    # actions exist before it can name them, then spliced back into slot 1 where the
    # rest of the project (and the operator) expects it.
    a = Action('Base')
    home = a.add_pose('pose_0', 'home', HOME, 1, 50.0, 100.0)
    checks = [
        ('bow', dict(pad_en='True', button='CIR'), 'bow'),
        ('kick', dict(pad_en='True', button='TRI'), 'kick'),
        ('upper_r', dict(pad_en='True', button='SQR'), 'Attack_upper_r'),
        ('upper_l', dict(pad_en='True', button='CRS'), 'Attack_upper_l'),
        ('standup_fd', dict(pad_en='True', button='DOWN'), 'standup_face_down'),
        ('standup_fu', dict(pad_en='True', button='UP'), 'standup_face_up'),
        ('jab_r', dict(pad_en='True', button='R1'), 'Attack_jab1_r'),
        ('jab_l', dict(pad_en='True', button='L1'), 'Attack_jab1_l'),
        ('walk_fwd', dict(ana_en='True', axis='Ly', op='>=', th='15'), 'walk_forward'),
        ('walk_back', dict(ana_en='True', axis='Ly', op='<=', th='-15'), 'walk_back'),
        ('sidestep_r', dict(ana_en='True', axis='Lx', op='>=', th='15'), 'walk_sidestep_right'),
        ('sidestep_l', dict(ana_en='True', axis='Lx', op='<=', th='-15'), 'walk_sidestep_left'),
        ('turn_r', dict(ana_en='True', axis='Rx', op='>=', th='19'), 'walk_turn_right_small'),
        ('turn_l', dict(ana_en='True', axis='Rx', op='<=', th='-19'), 'walk_turn_left_small'),
        ('turn_r_big', dict(ana_en='True', axis='Rx', op='>=', th='60'), 'walk_turn_right_big'),
        ('turn_l_big', dict(ana_en='True', axis='Rx', op='<=', th='-60'), 'walk_turn_left_big'),
    ]
    # Filter against the actions that actually got built, not against SKIP_ACTIONS: an
    # action can also be absent because its profile simply has no data for it (OP3 has
    # no uppercut), and a dispatch entry pointing at one that was never built is a
    # dangling jump.
    checks = [c for c in checks if c[2] in built]
    # big turns are checked first so their wider threshold wins over the small ones
    order = checks[-2:] + checks[:-2]
    bids = []
    for i, (name, kw, target) in enumerate(order):
        b = a.add_branch(f'branch_{i}', f'{name}_check', 260.0, 100.0 + 90.0 * i, **kw)
        j = a.add_jump(f'jump_{i}', f'-> {name}', target, 620.0, 100.0 + 90.0 * i)
        a.link(b, j, port=1, label='then', priority=10)
        bids.append(b)
    for i in range(len(bids) - 1):
        a.link(bids[i], bids[i + 1], port=0, label='else', priority=0)
    a.link(bids[-1], home, port=0, label='else', priority=0)
    a.add_jump('jump_loop', '-> Base', 'Base', 50.0, 240.0)
    a.link(home, 'jump_loop')
    a.start = bids[0]
    actions.insert(1, a)

    return [a for a in actions if a.title not in SKIP_ACTIONS]


def main():
    import argparse
    global GAIT_SCALE, RISE_SCALE, HANDICAP, TELEGRAPH, OUT
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--compat', action='store_true',
                    help='build the variant that survives a Studio holding only ~40 fps '
                         'instead of 60: gaits 1.4x longer, get-ups 1.2x. Writes to '
                         '..._compat_save.xml so both can be shipped side by side.')
    ap.add_argument('--telegraph', type=float, default=None,
                    help='stretch attack wind-ups by this factor, leaving the strike '
                         'itself at full speed, so an opponent can read and retreat')
    ap.add_argument('--handicap', type=float, default=None,
                    help='slow every action by this factor, for balancing a fight '
                         'between robots of different size')
    ap.add_argument('--gait-scale', type=float, default=None)
    ap.add_argument('--rise-scale', type=float, default=None)
    ap.add_argument('--out', default=None)
    args = ap.parse_args()
    if args.compat:
        # Per robot: OP3's rise already carries its slowdown in the profile's own frame
        # counts (its safe speed window is narrow in BOTH directions -- too fast and the
        # servos cannot hold it, too slow and it loses the momentum it rises on), so
        # multiplying again here overshoots the window rather than adding margin.
        GAIT_SCALE, RISE_SCALE = COMPAT_SCALES
        OUT = OUT.replace('_save.xml', '_compat_save.xml')
    if args.telegraph is not None:
        TELEGRAPH = args.telegraph
    if args.handicap is not None:
        HANDICAP = args.handicap
    if args.gait_scale is not None:
        GAIT_SCALE = args.gait_scale
    if args.rise_scale is not None:
        RISE_SCALE = args.rise_scale
    if args.out:
        OUT = args.out

    root = ET.Element('StopMotionProject', version='2')
    ET.SubElement(root, 'URDFPath').text = MODEL
    ET.SubElement(root, 'ModelType').text = MODEL_TYPE
    ET.SubElement(root, 'RobotName').text = ROBOT_NAME
    jo = ET.SubElement(root, 'JointOrder')
    for j in JOINT_ORDER:
        ET.SubElement(jo, 'Joint', name=j)
    js = ET.SubElement(root, 'JointSettings')
    for j in JOINT_ORDER:
        # rev is NOT the SIGN map. SIGN says which way the MODEL's joint turns, and every
        # angle written here is already in the model's own terms. rev is LME's UI-to-model
        # negation (LegacyMotionEditor._joint_is_rev): setting it makes the editor apply
        # the negative of every stored angle, which renders the robot with its arms over
        # its head. It also has to be written explicitly, because LME defaults it to True
        # for r_*_xr joints when the attribute is absent.
        ET.SubElement(js, 'Joint', name=j, display_name=j, rev='False',
                      max_speed_rad_s=SERVO[1], speed_preset_name=SERVO[0])
    presets = [('KRS2552', 7.142833333333333), ('KRS2572', 7.142833333333333),
               ('KRS4034', 6.159965061988786), ('RS304MD', 6.544984694978735),
               ('RS303MR', 9.51989840500307), ('RS305CR', 9.51989840500307),
               ('SG-90', 8.726646259971648), ('XL330-M288-T', 10.786134777324957),
               ('XC330-T181', 10.890854532444616), ('XD540-T150', 7.3303828583761845),
               ('XD540-T270', 4.084070449666731), ('XM540-W270', 3.141592653589793),
               ('STS3215-C046', 11.728612573401895), ('STS3215-C044', 9.00589894029074),
               ('STS3032', 11.623892818282235), ('RobStride-RS00', 32.98672286269283),
               ('RobStride-RS01', 32.98672286269283), ('RobStride-RS02', 32.98672286269283),
               ('RobStride-RS03', 20.420352248333657), ('RobStride-RS04', 20.943951023931955),
               ('RobStride-RS05', 50.26548245743669), ('RobStride-RS06', 50.26548245743669)]
    if SERVO[0] not in {n for n, _ in presets}:
        presets.append((SERVO[0], float(SERVO[1])))
    jsp = ET.SubElement(root, 'JointSpeedPresets')
    for n, v in presets:
        ET.SubElement(jsp, 'Preset', name=n, speed_rad_s=repr(v))
    jl = ET.SubElement(root, 'JointLayout')
    ET.SubElement(jl, 'Order').text = ','.join(JOINT_ORDER)
    for j in JOINT_ORDER:
        ET.SubElement(jl, 'Joint', name=j, group='R' if j.startswith('right') else 'L')
    jg = ET.SubElement(root, 'JointGroupPresets', currentIndex='-1')
    p = ET.SubElement(jg, 'Preset', name='LiftBody')
    for j in JOINT_ORDER:
        en = '1' if j.endswith(('hip_pitch', 'knee', 'ankle_pitch')) else '0'
        sc = '-2.0' if j.endswith('knee') else '1.0'
        ET.SubElement(p, 'Member', name=j, enabled=en, scale=sc)
    p = ET.SubElement(jg, 'Preset', name='Arms')
    for j in JOINT_ORDER:
        en = '1' if ('shoulder' in j or 'elbow' in j) else '0'
        ET.SubElement(p, 'Member', name=j, enabled=en, scale='1.0')
    ET.SubElement(root, 'ViewSettings', bg_color_a='0.1,0.1,0.1', bg_color_b='0.3,0.3,0.3',
                  bg_gradient_type='vertical', bg_slider_value='40', light_slider_value='56')
    hp = ET.SubElement(root, 'HomePosition')
    for j in JOINT_ORDER:
        ET.SubElement(hp, 'Joint', name=j, angle=f'{round(HOME[j], 3)}')
    cp = ET.SubElement(root, 'CameraPresets')
    cams = [('A', 'Front', 0, 0), ('B', 'Side', 90, 0), ('C', 'Quo', 45, 15),
            ('D', 'Top', 0, 90), ('E', '', 0, 0)]
    for n, lab, az, el in cams:
        ET.SubElement(cp, 'Camera', name=n, label=lab, azimuth=str(az), elevation=str(el),
                      distance=CAM_DISTANCE, focal_x=CAM_FOCAL[0], focal_y=CAM_FOCAL[1],
                      focal_z=CAM_FOCAL[2])
    ET.SubElement(root, 'UserValueSession')
    emit(root, build())
    ET.indent(root, space='  ')
    ET.ElementTree(root).write(OUT, encoding='utf-8', xml_declaration=True)
    print(f'wrote {OUT}  (gaits x{GAIT_SCALE:g}, get-ups x{RISE_SCALE:g}'
          + (f', everything x{HANDICAP:g}' if HANDICAP != 1.0 else '') + ')')


if __name__ == '__main__':
    main()
