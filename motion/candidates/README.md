# Search results kept for reference

Pickled pose lists from `motion/getup_supine.py` / `motion/getup_roll.py` on the final
12-servo geometry (each entry: leg parameters followed by shoulder pitch, [unused], elbow;
the roll is a whole-body pose in the model's actuator order).

- `getup_prone_s72.pkl`: the nine-pose prone rise that is in the profiles (seed 72).
- `getup_supine_direct_s80.pkl`: a direct supine rise (sit up on the hands, hip flexed to
  -100 deg), 4/8 at the search cadence before re-timing. Evaluate with
  `LME_ROBOT=microban_12_cand_sup PL_SUPINE_PKL=motion/candidates/getup_supine_direct_s80.pkl`.
- `roll_to_supine.pkl`: a one-pose roll from prone onto the back (all perturbed copies face up),
  for a prone get-up built as roll + direct supine rise instead of the current scheme.
