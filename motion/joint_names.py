"""microban's own joint names <-> the canonical names LME/Meridim already knows.

LegacyMotionEditor_Utils.JOINT_TO_MERIDIM maps joint names to Meridim slots, and it
ships with the canonical RobotLabelBridge names. Naming microban's joints that way
means the model works with an unmodified LME -- no patch to share alongside it.

The physical robot's firmware (Rhoban microban, src/constants.py) uses the names on the
left, so anything that talks to real hardware has to translate back through here.
"""

MICROBAN_TO_LME = {
    'head':                 'c_neck_zy',
    'left_shoulder_pitch':  'l_shoulder_yp',
    'left_shoulder_roll':   'l_shoulder_xr',
    'left_elbow':           'l_elbow_yp',
    'left_hip_yaw':         'l_hipjoint_zy',
    'left_hip_roll':        'l_hipjoint_xr',
    'left_hip_pitch':       'l_hipjoint_yp',
    'left_knee':            'l_knee_yp',
    'left_ankle_pitch':     'l_ankle_yp',
    'left_ankle_roll':      'l_ankle_xr',
    'right_shoulder_pitch': 'r_shoulder_yp',
    'right_shoulder_roll':  'r_shoulder_xr',
    'right_elbow':          'r_elbow_yp',
    'right_hip_yaw':        'r_hipjoint_zy',
    'right_hip_roll':       'r_hipjoint_xr',
    'right_hip_pitch':      'r_hipjoint_yp',
    'right_knee':           'r_knee_yp',
    'right_ankle_pitch':    'r_ankle_yp',
    'right_ankle_roll':     'r_ankle_xr',
}
LME_TO_MICROBAN = {v: k for k, v in MICROBAN_TO_LME.items()}

# Logical leg joint order, root to toe, used to index the leg-pose vectors. Most robots
# here have six; the ROBOTIS Mini has five, with no hip yaw, so the profile gets to say.
# A five-joint leg can still put a sole where it is wanted and keep it flat -- that is
# three position and two orientation constraints -- it just cannot choose the foot's
# heading, which is why turning in place is not available to it.
try:
    from robot_profile import profile
    LEG_ORDER = list(getattr(profile(), 'LEG_ORDER',
                             ['hip_yaw', 'hip_roll', 'hip_pitch', 'knee',
                              'ankle_pitch', 'ankle_roll']))
except Exception:
    LEG_ORDER = ['hip_yaw', 'hip_roll', 'hip_pitch', 'knee', 'ankle_pitch', 'ankle_roll']
ARM_ORDER = ['shoulder_pitch', 'shoulder_roll', 'elbow']


def jn(side, base):
    """Canonical name for a leg/arm joint, e.g. jn('right', 'hip_pitch')."""
    return MICROBAN_TO_LME[f'{side}_{base}']


# Which way a leg joint mirrors between left and right, by base name. The lists the
# get-up tools use are derived from LEG_ORDER so they stay the right length for a robot
# whose leg has five joints instead of six.
MIRROR_BY_NAME = {'hip_yaw': -1.0, 'hip_roll': -1.0, 'hip_pitch': 1.0,
                  'knee': 1.0, 'ankle_pitch': 1.0, 'ankle_roll': -1.0}
LEG_MIRROR = [MIRROR_BY_NAME[n] for n in LEG_ORDER]
NLEG = len(LEG_ORDER)
NPARAM = NLEG + 3          # legs plus shoulder pitch, shoulder roll, elbow
