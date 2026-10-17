# Search results kept for reference

Pickled pose lists from `motion/getup_supine.py` / `motion/getup_roll.py` on the final
12-servo geometry (each entry: leg parameters followed by shoulder pitch, [unused], elbow;
the rolls are whole-body poses).

- `getup_prone_s72.pkl`: the nine-pose prone rise that is in the profiles (seed 72).
- `getup_supine_direct_s80.pkl`: the eight-pose direct supine rise that is in the profiles
  (seed 80; sit up on the hands, hip flexed to -100 deg), 4/8 at the search cadence before
  re-timing. Evaluate the raw pickle with
  `LME_ROBOT=microban_12_cand_sup PL_SUPINE_PKL=motion/candidates/getup_supine_direct_s80.pkl`.
- `roll_to_prone_12.pkl`, `roll_to_prone_14.pkl`: the one-pose roll from supine onto the front
  (`(name, {actuator: deg}, frames)` tuples, as `ROLL_TO_PRONE` in a profile) that the earlier
  supine get-up used before the prone rise (7/8 on both builds). Assigning it to
  `ROLL_TO_PRONE` in the profile restores that scheme.
- `roll_to_supine.pkl`: a one-pose roll from prone onto the back (all perturbed copies face up),
  in the model's actuator order, for a prone get-up built as roll + direct supine rise.
