# Facade Element / Air conditioner outdoor unit - template source map

pattern: parallel_children around equipment body
parents:
- rec_build-a-realistic-articulated-3d-model-of-a-air-_20260609_185849_813335_0710784e ← picture/Facade Element/Air conditioner outdoor unit/001.png (single-fan rectangular condenser body: one round wire-grille axial fan on the front face, recessed side service panel, refrigerant lines, base mounting; `housing_to_fan` continuous). Covers Slot A=parent_front_fan_layout (single front fan), Slot B=parent_front_grille, Slot C=base_or_feet_mount. Source of louvered_front, side_service_door, top_discharge_fan, wall_brackets.
- rec_build-a-realistic-articulated-3d-model-of-a-air-_20260609_185900_637532_0ffac329 ← picture/Facade Element/Air conditioner outdoor unit/003.png (dual-fan side-discharge condenser body: two round front grilles each with an axial fan rotor, `left_fan`/`right_fan` with `housing_to_left_fan`/`housing_to_right_fan` continuous, control/access panel, feet). Covers Slot A=parent_front_fan_layout (multi-fan front layout). Source of three_fans (fan multiplicity 2→3).

(A third sibling build record exists in this picture set — rec_build-a-realistic-articulated-3d-model-of-a-air-_20260609_185856_985635_d332e6ea ← picture/.../002.png — but it produced no variants, so it is not a fork parent here.)

Outdoor condenser unit family with body shell, fan/discharge layout, front grille/louver, service panel,
and mounting support. Variants isolate fan count/layout, front/service cover treatment, and wall bracket
support.

## Slot 候选覆盖

### Slot A:fan / discharge layout
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| parent_front_fan_layout | parents | front fan rotor(s), grille | inherited outdoor condenser front fan layout | converged |
| three_front_fans | rec_ac_unit_var_three_fans | `fan_{i}` loop, rotor joints | three front fan rotors emitted by loop | converged |
| top_discharge_fan | rec_ac_unit_var_top_discharge_fan | top fan rotor/grille | top-discharge fan layout | converged |

### Slot B:front grille / service skin
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| parent_front_grille | parents | grille/fan guard visuals | inherited condenser front grille | converged |
| louvered_front | rec_ac_unit_var_louvered_front | louver slat loop | louvered front grille | converged |
| side_service_door | rec_ac_unit_var_side_service_door | side door hinge | side-hinged service access door | converged |

### Slot C:mounting support
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| base_or_feet_mount | parents | base feet / shell support | inherited ground/base mounting | converged |
| wall_brackets | rec_ac_unit_var_wall_brackets | bracket arms/rails | wall bracket mounting frame | converged |

## Multiplicity / Copy Logic
- count_param: `fan_count` and local `louver_count`.
- N 样本已覆盖: fan_count includes parent samples plus 3; louver_count sampled as local repeated grille slats.
- 模板建议 N_range: fan_count [1, 3] for this evidence set.
- copied object / naming / placement / joint policy: fan rotors should be `fan_{i}` / `rotor_{i}` with regular horizontal placement and identical continuous rotation joints.

## 组合数预审
Slot A(3) x Slot B(3) x Slot C(2) = 18 >= 10 ✓.

## 排除项(未来 compatibility matrix 素材)
- No blocked cells in this batch; all planned outdoor-unit variants converged.
- top_discharge_fan can conflict with dense top service geometry; preserve clear top grille clearance.
- wall_brackets are mounting support and should not modify fan/grille topology.
