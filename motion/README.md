# Motion tools (copied from LegacyMotionEditor's `tools/`)

These scripts were written for [LegacyMotionEditor](https://github.com/hijimasa/LegacyMotionEditor)
and are copied here, with their paths adjusted, so that the motion checks in the
top-level README can be reproduced from this repository alone. Run everything
from the repository root with `LME_ROBOT=microban_12` or `microban_14`:

```bash
LME_ROBOT=microban_12 python3 motion/build_project.py          # keyframe IK -> motion/projects/LME_microban_12_save.xml (loads in LME)
LME_ROBOT=microban_12 python3 motion/sim_check.py --trials 8    # perturbed replay of every action in MuJoCo
LME_ROBOT=microban_12 python3 motion/tune_gait.py walk          # CEM re-tuning of the walk parameters
LME_ROBOT=microban_12 python3 motion/getup_supine.py --pitch 90 --seed 0 --robust 3 --out motion/log/getup_prone.pkl   # beam search for a prone rise
LME_ROBOT=microban_12 python3 motion/getup_supine.py --pitch -90 --seed 80 --robust 3 --out motion/log/getup_supine.pkl  # the same search from supine
LME_ROBOT=microban_12_cand PL_PRONE_PKL=motion/log/getup_prone.pkl python3 motion/build_project.py --out motion/projects/_cand.xml
LME_ROBOT=microban_12_cand PL_PRONE_PKL=motion/log/getup_prone.pkl python3 motion/sim_check.py --project motion/projects/_cand.xml --only standup_face_down
LME_ROBOT=microban_12 python3 motion/tune_getup.py face_down    # CEM re-timing of the rise in the profile
```

- `profiles/microban_12.py`, `profiles/microban_14.py`: the robots' numbers (joint order, IK targets, gait and
  get-up parameters, the rise poses) on top of `microban_pl.py` and `microban.py`.
- `ik.py`, `joint_names.py`, `robot_profile.py`: leg IK with the passive-joint coupling, name mapping.
- `build_project.py`: writes an LME project (keyframes for the walks, turns, jabs and get-ups).
- `sim_check.py`: replays a project in MuJoCo under servo-offset and tilt perturbations; the tables in the README come from it.
- `tune_gait.py`, `tune_getup.py`, `getup_supine.py`, `getup_roll.py`: tuners and searches.
- `projects/`: the generated LME projects for the 12- and 14-servo builds.
- `candidates/`: pickled search results (both rises in the profiles, the old rolls).

Needs `mujoco` and `numpy`. The scripts have Python 3.12 f-strings in places.
