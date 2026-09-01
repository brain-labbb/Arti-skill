# Bathroom / Hair dryer — template source map

pattern: mixed
parents:
- rec_pink-compact-hair-dryer-with-a-detachable-rotata_20260605_144857_361026_8f70ba30 ← picture/Bathroom/Hair dryer/001.png (pink compact dryer, tapered barrel, rotatable concentrator nozzle, pistol handle with two sliding switches, rear grille, cord + plug). Covers Slot A=concentrator_nozzle, Slot B=pistol_handle, Slot C=radial_rear_filter.

Compact handheld hair dryer. The core root is `body`; independent child mechanisms include
the nozzle rotation (`barrel_to_nozzle`, CONTINUOUS), sliding switches (`body_to_power_switch`
/ `body_to_heat_switch`, PRISMATIC), and in some variants a handle/rear-filter articulation.
The variant batch isolates three future slots: nozzle attachment, handle/grip structure, and
rear intake/filter mechanism.

## Slot 候选覆盖

### Slot A:nozzle attachment (`nozzle` / barrel front outlet)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| concentrator_nozzle | rec_pink-compact-hair-dryer-with-a-detachable-rotata_20260605_144857_361026_8f70ba30 (parent) | `nozzle`, `barrel_to_nozzle` | black tapered concentrator with flattened rectangular outlet; continuous rotation around barrel axis | converged |
| diffuser_nozzle | rec_hair_dryer_var_diffuser_nozzle | `diffuser`, `barrel_to_diffuser`, diffuser fingers/perforations | round bowl diffuser with short rounded fingers and perforated face | converged |
| comb_pick_nozzle | rec_hair_dryer_var_comb_nozzle | `nozzle`, `barrel_to_nozzle`, comb tooth visuals | narrow comb pick nozzle with evenly spaced teeth at the outlet | converged |
| wide_smoothing_nozzle | rec_hair_dryer_var_wide_smoothing_nozzle | `nozzle`, `barrel_to_nozzle` | broad flattened smoothing slit nozzle with rounded rectangular lips | converged |

### Slot B:handle / grip structure
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| pistol_handle | rec_pink-compact-hair-dryer-with-a-detachable-rotata_20260605_144857_361026_8f70ba30 (parent) | `body` handle shell, `body_to_power_switch`, `body_to_heat_switch` | fixed pistol grip with two prismatic sliding switches | converged |
| folding_travel_handle | rec_hair_dryer_var_folding_handle | `handle`, body-to-handle hinge plus retained switch joints | travel folding handle with visible revolute hinge at the barrel/handle root | converged |
| open_loop_grip | rec_hair_dryer_var_loop_grip_handle | loop grip geometry, switch shelf, switch parts | open loop handle with oval finger cutout and integrated switch shelf | converged |

### Slot C:rear intake / filter
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| radial_fixed_grille | rec_pink-compact-hair-dryer-with-a-detachable-rotata_20260605_144857_361026_8f70ba30 (parent) | `body.rear_filter` | fixed rear radial grille/cap on the barrel end | converged |
| twist_ring_filter | rec_hair_dryer_var_twist_rear_filter | rear ring/filter part and release joint | removable twist-ring circular grille with radial ribs and bayonet-style ring | converged |
| hinged_lint_screen | rec_hair_dryer_var_hinged_rear_filter | rear lint screen hinge | hinged flip-open rear lint screen with perforated grille surface | converged |

## Multiplicity / Copy Logic
- count_param: 无核心同构子件 multiplicity；diffuser fingers / grille ribs / comb teeth are local repeated visuals within a slot candidate, emitted by loop where present.
- N 样本已覆盖: 无。
- 模板建议 N_range: 无；teeth/finger/rib counts should be local candidate parameters, not topology-level multiplicity.
- copied object / naming / placement / joint policy: local repeated nozzle/filter features should use loop-emitted visual names such as `finger_{i}`, `tooth_{i}`, or `rib_{i}` with regular angular/linear placement.

## 组合数预审
Slot A(4) × Slot B(3) × Slot C(3) = 36 ≥ 10 ✓.

## 排除项(未来 compatibility matrix 素材)
- No blocked cells in this batch; all planned cells converged.
- folding_travel_handle and rear-filter mechanisms add articulation beyond the parent; template side should ensure added hinge axes do not collide with switch placement.
- Pure color/material changes and continuous body scaling are not slot candidates.
