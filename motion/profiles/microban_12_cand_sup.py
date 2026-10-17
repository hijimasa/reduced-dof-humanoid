"""microban_12 with a DIRECT supine rise taken from a search result on disk, for evaluation.
    LME_ROBOT=microban_12_cand_sup PL_SUPINE_PKL=motion/log/getup_supine_s80.pkl python3 motion/build_project.py --out motion/projects/_cand.xml
The roll-over path (ROLL_TO_PRONE) is switched off so standup_face_up uses these poses.
"""
import os, pickle
from profiles.microban_12 import *   # noqa: F401,F403
_SUP = [(tuple(float(v) for v in x[:4]), tuple(float(v) for v in x[4:7]))
        for x in pickle.load(open(os.environ['PL_SUPINE_PKL'], 'rb'))]
GETUP_SUPINE = _SUP
_MV, _HD = (int(v) for v in os.environ.get('PL_FRAMES', '45,20').split(','))   # the search's cadence: move, hold
F_RISE_U = [_MV] * len(_SUP)
F_HOLD_U = [_HD] * len(_SUP)
SUPINE_PREP = [('lie_flat', (0, 150, 0), 44)]
ROLL_TO_PRONE = []
SKIP_ACTIONS = ('bow', 'kick', 'Attack_upper_l', 'Attack_upper_r', 'standup_face_down')
