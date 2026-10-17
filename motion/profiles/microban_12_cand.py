"""microban_12 with the prone rise taken from a search result on disk, for evaluation.
    LME_ROBOT=microban_12_cand PL_PRONE_PKL=motion/log/getup_prone.pkl python3 motion/build_project.py --out motion/projects/_cand.xml
"""
import os, pickle
from profiles.microban_12 import *   # noqa: F401,F403
_PRONE_POSES = [(tuple(float(v) for v in x[:4]), tuple(float(v) for v in x[4:7]))
                for x in pickle.load(open(os.environ['PL_PRONE_PKL'], 'rb'))]
GETUP_PRONE = [p for p in _PRONE_POSES for _ in (0, 1)]
_MV, _HD = (int(v) for v in os.environ.get('PL_FRAMES', '45,20').split(','))   # the search's cadence: move, hold
F_RISE_D = [_MV, _HD] * len(_PRONE_POSES)
PRONE_PREP = [('lie_flat', (0, 90, 0), 44)]
SKIP_ACTIONS = ('bow', 'kick', 'Attack_upper_l', 'Attack_upper_r', 'standup_face_up')
