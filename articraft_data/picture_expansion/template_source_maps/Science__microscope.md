# Science / microscope — template source map

pattern: mixed
parents:
- rec_build-a-realistic-articulated-3d-model-of-a-micr_20260609_183637_825621_5f8b9008 ← picture/Science/microscope/ (`base` root; `eyepiece_tube`, `nosepiece_turret`+`head_to_nosepiece`(continuous), `stage`+`arm_to_stage`(prismatic), `condenser`, `focus_knob`+`arm_to_focus_knob`(continuous); objectives looped n=3 at 120°). Covers Slot A=monocular_inclined, Slot B=condenser_lamp, Slot C=side_coaxial_knob+vertical_stage, Multiplicity N=3 objectives.

Compound optical microscope. `base`/arm is the root; the `nosepiece_turret` rotates on a CONTINUOUS
joint, the `stage` rises on a PRISMATIC, and the `focus_knob` turns on a CONTINUOUS joint. The batch
isolates head/eyepiece type, illumination, and focus/stage mechanism as independent slots, with
objective count on the nosepiece as the multiplicity axis (single CONTINUOUS turret joint, rigid lenses).

## Slot 候选覆盖

### Slot A:head / eyepiece
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| monocular_inclined | rec_build-...-micr_20260609_183637_825621_5f8b9008 (parent) | `eyepiece_tube`, `head_to_nosepiece`(continuous) | single inclined monocular tube | converged(parent) |
| binocular | rec_microscope_var_binoc | `binocular_head`, `head_to_nosepiece`(continuous) | twin inclined binocular head | converged(workbench, rating pending sync) |
| vertical_straight | rec_microscope_var_vtube | `eyepiece_tube`, `head_to_nosepiece`(continuous) | vertical straight (non-inclined) tube | converged(workbench, rating pending sync) |

### Slot B:illumination
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| condenser_lamp | rec_build-...-micr_20260609_183637_825621_5f8b9008 (parent) | `condenser`, stage prismatic | fixed sub-stage condenser + lamp | converged(parent) |
| tilting_mirror | rec_microscope_var_mirror | `mirror_fork`, `mirror`, mirror tilt(revolute about -Y) | tilting reflector mirror in a fork, adds a REVOLUTE | converged(workbench, rating pending sync) |
| led_disc | rec_microscope_var_leddisc | `illuminator` | flat LED illuminator disc replacing the condenser | converged(workbench, rating pending sync) |

### Slot C:focus / stage
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| side_coaxial_knob_vertical_stage | rec_build-...-micr_20260609_183637_825621_5f8b9008 (parent) | `focus_knob`(continuous), `stage`+`arm_to_stage`(prismatic) | side coaxial focus knob + vertical stage lift | converged(parent) |
| dual_coarse_fine_knob | rec_microscope_var_dualknob | `coarse_focus`, `fine_focus`(continuous), `stage`(prismatic) | separate coarse + fine focus knobs (3 continuous) | converged(workbench, rating pending sync) |
| mechanical_xy_stage | rec_microscope_var_xystage | `x_carriage`, `y_carriage`(prismatic X+Y), `drive_knob_{i}` | mechanical XY stage, two PRISMATIC carriages + drive knobs | converged(workbench, rating pending sync) |

## Multiplicity / Copy Logic
- count_param: `objective_count`(objectives on the nosepiece turret)
- N 样本已覆盖: {2, 3, 4, 6} → rec_microscope_var_obj2(@180°) / parent(@120°) / rec_microscope_var_obj4(@90°) / rec_microscope_var_obj6(@60°)
- 模板建议 N_range: [2, 6]
- copied object / naming / placement / joint policy: objectives looped on the `nosepiece_turret`, spaced equiangularly (360/N), rigid on the turret; the turret carries the single shared CONTINUOUS `head_to_nosepiece` joint (no per-lens joints).

## 组合数预审
Slot A(3) × Slot B(3) × Slot C(3) × Multiplicity(4) = 108 ≥ 10 ✓.

## 排除项(未来 compatibility matrix 素材)
- Trinocular folds into the binocular Slot-A candidate (camera port is decoration).
- Arm / base form is a continuous param, not a slot.
- N=1 (single objective) and N>6 are out of vocabulary for the nosepiece turret.
