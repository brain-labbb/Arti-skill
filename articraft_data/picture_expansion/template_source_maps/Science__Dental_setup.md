# Science / Dental setup — template source map

pattern: mixed
parents:
- rec_build-a-realistic-articulated-3d-model-of-a-dent_20260609_183622_759989_90de439f ← picture/Science/Dental setup/ (`base` root; `backrest`+`backrest_recline`(revolute), `light_arm`+`light_arm_swing`(revolute), `light_head`+`light_head_tilt`(revolute); instruments loop n=5, stool legs loop n=5). Covers Slot A=single_body_recline, Slot B=single_rigid_arm, Slot C=round_dish_head, Multiplicity N=5.

Dental treatment unit. `base` is the root carrying a reclining patient chair (`backrest`), a swinging
operatory `light_arm` ending in a tilting `light_head`, a `delivery_column`, and a `stool`. The batch
isolates chair recline articulation, light-arm linkage topology, and delivery/head form as independent
slots, with instrument/handpiece count as the multiplicity axis (FIXED hoses on the delivery head).

## Slot 候选覆盖

### Slot A:chair recline
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| single_body_recline | rec_build-...-dent_20260609_183622_759989_90de439f (parent) | `backrest`, `backrest_recline`(revolute) | one-piece reclining backrest, single revolute | converged(parent) |
| two_section_backrest | rec_dental_setup_var_seatback2 | `seat`, `backrest`, recline revolute | split seat + backrest, separate revolute recline | converged(workbench, rating pending sync) |
| three_section_footrest | rec_dental_setup_var_seatback3 | `backrest`, `footrest`, recline + footrest revolutes | three-section chair adding articulated `footrest` (4 revolutes) | converged(workbench, rating pending sync) |

### Slot B:light-arm linkage
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| single_rigid_arm | rec_build-...-dent_20260609_183622_759989_90de439f (parent) | `light_arm`, `light_arm_swing`(revolute) | one rigid arm, single swing revolute | converged(parent) |
| two_segment_elbow | rec_dental_setup_var_armelbow | `upper_arm`, `forearm`, elbow revolutes | two-segment arm with elbow joint (4 revolutes) | converged(workbench, rating pending sync) |
| parallelogram | rec_dental_setup_var_armparallel | `arm_yaw`, `arm_bar_{i}`, `light_carriage`, yaw+bar revolutes | yaw post + parallelogram `arm_bar_{i}` linkage to a balanced carriage (6 revolutes) | converged(workbench, rating pending sync) |

### Slot C:delivery / head
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| round_dish_head | rec_build-...-dent_20260609_183622_759989_90de439f (parent) | `light_head`, `light_head_tilt`(revolute) | round dish operatory light head, tilt revolute | converged(parent) |
| led_panel | rec_dental_setup_var_ledhead | `light_head`, `light_head_tilt`(revolute) | flat rectangular LED panel head, tilt revolute | converged(workbench, rating pending sync) |
| swing_tray_delivery | rec_dental_setup_var_swingtray | `delivery_arm`, delivery swing revolute | swing-out delivery tray arm off the column (adds 1 revolute) | converged(workbench, rating pending sync) |

## Multiplicity / Copy Logic
- count_param: `instrument_count`(handpieces/instruments on delivery head)
- N 样本已覆盖: {3, 5, 7} → rec_dental_setup_var_instr3 / parent / rec_dental_setup_var_instr7
- 模板建议 N_range: [2, 8]
- copied object / naming / placement / joint policy: instruments looped with per-instrument hose, evenly placed on the delivery head, FIXED to the head (no independent joints); changing N changes only the loop count.

## 组合数预审
Slot A(3) × Slot B(3) × Slot C(3) × Multiplicity(3) = 81 ≥ 10 ✓.

## 排除项(未来 compatibility matrix 素材)
- Stool form is not an articulation axis (rigid `stool` companion in all variants).
- Cuspidor (spittoon) folded into Slot C delivery rather than a separate slot.
- Instrument N>8 is the sampler's job; samples only demonstrate the loop copy logic.
