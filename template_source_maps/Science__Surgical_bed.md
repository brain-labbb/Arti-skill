# Science / Surgical bed — template source map

pattern: mixed
parents:
- rec_build-a-realistic-articulated-3d-model-of-a-surg_20260609_183628_574040_3fe8e15f ← picture/Science/Surgical bed/ (`base` root; `seat` carries `back`+`seat_to_back`(revolute) and `leg`+`seat_to_leg`(revolute), plus `head`; mattress sections hand-written seat/back/leg). Covers Slot A=pedestal_crossfoot, Slot B=arm_rails+horseshoe_head, Multiplicity N=3 sections.

Articulating surgical/exam bed. `base` (pedestal + crossfoot) is the root; the `seat` is the spine
carrying a linear chain of articulating mattress sections (`back`, `leg`, `head`) each on a REVOLUTE
off the seat. The batch isolates the base/support form and parallel accessories as independent slots,
with articulating-section count as the multiplicity axis (loop-rewrite of the hand-written sections
into a uniform `section_{i}` chain).

## Slot 候选覆盖

### Slot A:base / support
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| pedestal_crossfoot | rec_build-...-surg_20260609_183628_574040_3fe8e15f (parent) | `base`, `seat`, `seat_to_back`/`seat_to_leg`(revolute) | central pedestal column on a crossfoot | converged(parent) |
| four_leg_splayed | rec_surgical_bed_var_fourleg | `base`, `seat`, section revolutes | four splayed legs replacing the pedestal | converged(workbench, rating pending sync) |
| caster_trolley | rec_surgical_bed_var_casterbase | `chassis`, `caster_{i}`, section revolutes | mobile trolley chassis on looped `caster_{i}` | converged(workbench, rating pending sync) |
| twin_column_H | rec_surgical_bed_var_twincolumn | `base`, `seat`, section revolutes | twin-column H-frame base | converged(workbench, rating pending sync) |

### Slot B:accessories
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| arm_rails_horseshoe_head | rec_build-...-surg_20260609_183628_574040_3fe8e15f (parent) | `head`, `seat`, `back`, `leg` | arm rails plus horseshoe head support | converged(parent) |
| side_safety_rails_IV_pole | rec_surgical_bed_var_siderails | `base`, `seat`, `back`, `leg`, rail accessories | side safety rails + IV pole accessories | converged(workbench, rating pending sync) |
| padded_arm_boards | rec_surgical_bed_var_armboards | `base`, `seat`, `back`, `leg`, arm-board accessories | padded swing-out arm boards | converged(workbench, rating pending sync) |

## Multiplicity / Copy Logic
- count_param: `section_count`(articulating mattress sections)
- N 样本已覆盖: {2, 3, 4} → rec_surgical_bed_var_sections2 / parent (back/seat/leg) / rec_surgical_bed_var_sections4
- 模板建议 N_range: [2, 5]
- copied object / naming / placement / joint policy: hand-written sections rewritten as a `section_{i}` chain off the `seat` spine; one uniform REVOLUTE per section (observed 1 revolute @ N=2, 3 revolute @ N=4); each section hinges off the seat.

## 组合数预审
Slot A(4) × Slot B(3) × Multiplicity(3) = 36 ≥ 10 ✓.

## 排除项(未来 compatibility matrix 素材)
- PRISMATIC column-height lift folded into Slot A base (single image, no separate height-only sample).
- Color / scale not articulation axes.
- Base × accessory combinations are the sampler's job; no cross-axis combo variants made.
