# Notice and attribution

This repository is an **unofficial derivative** of
[Microban](https://github.com/Rhoban/microban) by Marc Duclusaud (Rhoban team,
LaBRI, University of Bordeaux). It is not affiliated with, nor endorsed by,
Rhoban or the author of Microban.

Upstream version used: `Rhoban/microban` commit
`d0a128d180ecfce69eaaf31b295a0fce31280bfd` (tag `v1.0.0` + 10 commits, 2026-09-10).

## Provenance and licenses by path

| Path | Origin | License |
| --- | --- | --- |
| `model/microban_pl/assets/*.stl` | Unmodified copies of upstream `src/model/mjcf/assets/` (25 meshes referenced by the model). Exported from the Microban CAD. | CC BY-NC-SA 4.0 (Marc Duclusaud). See `model/microban_pl/assets/LICENSE`. |
| `model/microban_pl/microban_pl.xml`, `microban_pl_lme.xml` | Generated from upstream `src/model/mjcf/robot.xml` by `tools/make_microban_pl.py`. The upstream file carries an Apache-2.0 notice, which is retained in the generated files. | Apache-2.0 (upstream notice: Marc Duclusaud; modifications: Masaaki Hijikata) |
| `model/microban_pl/scene.xml` | Written by `tools/make_microban_pl.py` | Apache-2.0 (Masaaki Hijikata) |
| `tools/`, `docs/`, `README.md` | Original work of this repository | Apache-2.0 (Masaaki Hijikata); documentation text may be reused under the same terms |

Because the meshes are CC BY-NC-SA 4.0, **the model as a whole may not be used
commercially** (kits, pre-printed parts, assembled robots) without written
permission from the author of Microban. See the upstream README, section
"License & Commercial Use".

## Modifications made to the upstream MJCF

All changes are applied programmatically by `tools/make_microban_pl.py`:

- Per leg, a passive "knee block" body is inserted between the femur and the
  tibia, and two passive rear links (thigh and shank) are added. Two `<connect>`
  equality constraints per leg close the parallelogram loops so that the foot
  stays parallel to the pelvis in pitch.
- The ankle-pitch joints become passive (no actuator, no range).
- The neck yaw joint and its actuator are removed; the head body is fixed to the
  trunk (`--neck-motor` keeps it). Actuator count goes from 19 to 16.
- The knee actuator now acts between the pelvis-parallel knee block and the
  tibia, so its angle is measured against the pelvis rather than the thigh.
- Hip-pitch and knee ranges are derived from the linkage geometry to stay 8°
  clear of its singularities (with the default 20 mm / 10 mm pivots: hip -109° to
  +55°, knee -55° to +109°, pelvis-relative).
- The ankle-pitch servo's geometry is removed from the ankle block, the block's
  collision box is cut down to a clevis around the hinge and its mass reduced
  (`--keep-ankle-servo` restores it). The neck servo's geometry and mass are
  removed from the trunk together with the neck joint.
- `<motor>` actuators are replaced by `<position>` actuators (kp = 50,
  critically damped, force limit ±0.52 N·m) for keyframe-style control. Use
  `--keep-motor` to keep the upstream torque actuators.
- A `--naming lme` option renames joints to the LegacyMotionEditor convention.

The upstream copyright and license header of `robot.xml` is preserved at the
top of every generated model file, followed by a note describing the
derivation.

## Other references

- Pollen Robotics / Hugging Face **Microduck** (15-DOF biped without arms or
  ankle roll), which motivated the degree-of-freedom reduction study in
  `docs/design_ja.md`. No files from Microduck are used here.
- ROBO-ONE competition rules, 二足歩行ロボット協会 (Biped Robot Association),
  https://www.robo-one.com/ . Rule excerpts in the documentation are quoted for
  design reference only.
