# Science / Syringe — template source map

pattern: mixed
parents:
- rec_build-a-realistic-articulated-3d-model-of-a-syri_20260609_183634_604959_042137c8 ← picture/Science/Syringe/ (`barrel_assembly` root + `plunger`+`barrel_to_plunger`(prismatic) hero joint; graduation ticks looped n=16; hub side-slots loop). Covers Slot A=luer_lock_needle, Slot B=flat_wings, Slot C=plus_cross, Slot D=straight_cylindrical, Multiplicity N=16 ticks.

Medical syringe. `barrel_assembly` is the root; the `plunger` slides on the single hero PRISMATIC
`barrel_to_plunger` (preserved in every variant). The batch isolates tip/hub type, flange form,
plunger-rod cross-section, and barrel form as independent slots, with graduation-tick count as a
purely-visual multiplicity axis (inlined `scale_marks`, no joints).

## Slot 候选覆盖

### Slot A:tip / hub
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| luer_lock_needle | rec_build-...-syri_20260609_183634_604959_042137c8 (parent) | `barrel_assembly` hub, `barrel_to_plunger`(prismatic) | threaded luer-lock hub with needle | converged(parent) |
| slip_tip | rec_syringe_var_sliptip | `barrel_assembly` tip, `barrel_to_plunger`(prismatic) | plain slip (friction) tip, no lock collar | converged(workbench, rating pending sync) |
| blunt_cannula | rec_syringe_var_bluntcan | `barrel_assembly` tip, `barrel_to_plunger`(prismatic) | blunt fill cannula tip | converged(workbench, rating pending sync) |
| safety_shield | rec_syringe_var_safetysh | `barrel_assembly` shield, `barrel_to_plunger`(prismatic) | retractable safety shield over the needle | converged(workbench, rating pending sync) |

### Slot B:finger flange
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| flat_wings | rec_build-...-syri_20260609_183634_604959_042137c8 (parent) | `barrel_assembly` flange | flat opposed finger wings | converged(parent) |
| ring_collar | rec_syringe_var_ringflng | `barrel_assembly` flange | full ring/collar finger grip | converged(workbench, rating pending sync) |
| ribbed_grip | rec_syringe_var_ribgrip | `barrel_assembly` flange (rib loop) | ribbed/textured finger grip flange | converged(workbench, rating pending sync) |

### Slot C:plunger rod (2-candidate slot)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| plus_cross | rec_build-...-syri_20260609_183634_604959_042137c8 (parent) | `plunger`, `barrel_to_plunger`(prismatic) | plus/cross cross-section rod | converged(parent) |
| flat_plate | rec_syringe_var_flatrod | `plunger`, `barrel_to_plunger`(prismatic) | flat-plate cross-section rod | converged(workbench, rating pending sync) |

### Slot D:barrel form
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| straight_cylindrical | rec_build-...-syri_20260609_183634_604959_042137c8 (parent) | `barrel_assembly`, `plunger`(prismatic) | straight cylindrical barrel | converged(parent) |
| slim_insulin | rec_syringe_var_slimbody | `barrel_assembly`, `plunger`(prismatic) | slim small-diameter insulin barrel | converged(workbench, rating pending sync) |
| wide_stepped | rec_syringe_var_widebody | `barrel_assembly`, `plunger`(prismatic) | wide stepped/tapered large barrel | converged(workbench, rating pending sync) |

## Multiplicity / Copy Logic
- count_param: `tick_count`(graduation marks)
- N 样本已覆盖: {10, 16, 24} → rec_syringe_var_ticks10 / parent / rec_syringe_var_ticks24
- 模板建议 N_range: [6, 40]
- copied object / naming / placement / joint policy: graduation ticks looped and merged into one inlined `scale_marks` visual along the barrel; evenly spaced; NO joints (purely decorative). Changing N changes only the loop count (the tiny topology diff is expected and correct).

## 组合数预审
Slot A(4) × Slot B(3) × Slot C(2) × Slot D(3) = 72 ≥ 10 ✓.

## 排除项(未来 compatibility matrix 素材)
- The hero `barrel_to_plunger` PRISMATIC is preserved in every variant.
- Slot C has only 2 honest candidates (cross vs. flat rod) — documented degrade, not a 3-candidate slot.
- Pure scale (volume) is a continuous param, not an axis; capacity is folded into Slot D barrel form.
