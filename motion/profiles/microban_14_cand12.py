"""microban_14 driven with the 12-servo prone rise (hip yaw held at 0), for evaluation."""
import os, pickle
from profiles.microban_14 import *   # noqa: F401,F403
_PRONE_POSES = [((0.0,) + tuple(float(v) for v in x[:4]), tuple(float(v) for v in x[4:7]))
                for x in pickle.load(open(os.environ.get('PL_PRONE_PKL', 'motion/log/getup_prone.pkl'), 'rb'))]
GETUP_PRONE = [p for p in _PRONE_POSES for _ in (0, 1)]
if 'PL_FRAMES' in os.environ:    # "move,hold" cadence of the search, before re-timing
    _MV, _HD = (int(v) for v in os.environ['PL_FRAMES'].split(','))
    F_RISE_D = [_MV, _HD] * len(_PRONE_POSES); PRONE_PREP = [('lie_flat', (0, 90, 0), 44)]
else:
    _frames = [28, 72, 44, 18, 14, 15, 8, 14, 16, 16, 11, 42, 12, 48, 10, 23]   # tune_getup on microban_12
    F_RISE_D = _frames[1:15]; PRONE_PREP = [('lie_flat', (0, 90, 0), _frames[0])]; F_HOME_D = _frames[15]
SKIP_ACTIONS = ('bow', 'kick', 'Attack_upper_l', 'Attack_upper_r')
