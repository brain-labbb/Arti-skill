# Science / Surgical chair — template source map

pattern: mixed
parents:
- rec_build-a-realistic-articulated-3d-model-of-a-surg_20260609_183631_452724_821a91c4 ← picture/Science/Surgical chair/ (`base` root; `seat`+`seat_lift`(prismatic), `left_arm`+`left_arm_swing`(revolute), `right_arm`+`right_arm_swing`(revolute); 5 casters loop, 2 pedals loop; backrest a fixed stalk). Covers Slot A=five_star_caster, Slot B=fixed_stalk, Multiplicity N=5 casters.

Surgical/procedure chair. `base` is the root carrying a `seat` that lifts on a PRISMATIC `seat_lift`,
with two armrests each swinging on a REVOLUTE. The batch isolates base form and backrest mechanism as
independent slots, with caster-leg count as the multiplicity axis (`caster_{i}` looped equiangularly).

## Slot 候选覆盖

### Slot A:base
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| five_star_caster | rec_build-...-surg_20260609_183631_452724_821a91c4 (parent) | `base`, `seat_lift`(prismatic), `caster_{i}` loop | five-star caster base, seat lift prismatic | converged(parent) |
| fixed_pedestal | rec_surgical_chair_var_pedestal | `base`, `seat_lift`(prismatic) | fixed solid pedestal column, no casters | converged(workbench, rating pending sync) |
| four_leg_frame | rec_surgical_chair_var_fourleg | `base`, `seat_lift`(prismatic), 4-leg frame | four-leg square frame base | converged(workbench, rating pending sync) |
| cross_caster | rec_surgical_chair_var_xcaster | `base`, `seat_lift`(prismatic), `caster_{i}`(4) | four-arm cross caster base | converged(workbench, rating pending sync) |

### Slot B:backrest mechanism
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| fixed_stalk | rec_build-...-surg_20260609_183631_452724_821a91c4 (parent) | `seat`, fixed backrest stalk | rigid fixed backrest stalk (3 joints: lift + 2 arms) | converged(parent) |
| reclining_revolute | rec_surgical_chair_var_recline | `backrest`, recline revolute | reclining backrest on a REVOLUTE (4 joints) | converged(workbench, rating pending sync) |
| split_headrest_tilt | rec_surgical_chair_var_headrest | `headrest`, headrest tilt revolute | split backrest with tilting `headrest` (4 joints) | converged(workbench, rating pending sync) |
| footrest_legrest | rec_surgical_chair_var_legrest | `legrest`, `legrest_hinge`(revolute about -Y) | leg-rest that swings up on a REVOLUTE (4 joints) | converged(workbench, rating pending sync) |

## Multiplicity / Copy Logic
- count_param: `caster_count`(caster legs)
- N 样本已覆盖: {3, 4, 5} → rec_surgical_chair_var_caster3 / rec_surgical_chair_var_xcaster / parent
- 模板建议 N_range: [3, 6]
- copied object / naming / placement / joint policy: casters looped as `caster_{i}`, placed equiangularly (360/N) around the base hub, FIXED to the base (rolling is decoration, no joints).

## 组合数预审
Slot A(4) × Slot B(4) × Multiplicity(3) = 48 ≥ 10 ✓.

## 排除项(未来 compatibility matrix 素材)
- Armrest count is not an axis (always 2 swinging arms in all variants).
- Footrest also lives as a Slot-B candidate (legrest) rather than its own slot.
- Base × backrest combinations are the sampler's job; no cross-axis combo variants made.
