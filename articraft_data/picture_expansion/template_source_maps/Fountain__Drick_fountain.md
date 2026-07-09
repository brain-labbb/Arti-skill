# Fountain / Drick fountain - template source map

pattern: mixed drinking fountain
parents:
- rec_build-a-realistic-articulated-3d-model-of-a-dric_20260609_215049_780247_b6678542 <- picture/Fountain/Drick fountain/001.png (teal-blue painted-steel pylon drinking fountain with a stainless catch basin and gooseneck spout, brushed-steel faceplate with engraved bottle pictogram, a chrome PRISMATIC push-button valve, and a perforated bottle-rest grille shelf). Covers Slot A=pedestal_body, Slot B=parent_basin_spout, Slot C=single_push_button.

Drinking fountain family with body/mount, basin, spout or bottle-filler module, and actuator controls.
Variants isolate basin shape, spout/filler module, body mounting, and push/foot controls.

## Slot 候选覆盖

### Slot A:body / mounting
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| pedestal_body | rec_build-a-realistic-articulated-3d-model-of-a-dric_20260609_215049_780247_b6678542 (parent) | pylon_body, pylon_to_basin | floor/pedestal drinking fountain body | converged |
| wall_mounted_body | rec_drick_fountain_var_wall_mounted_body | mounting_plate, body | wall-mounted compact fountain body | converged |

### Slot B:basin / water outlet module
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| parent_basin_spout | rec_build-a-realistic-articulated-3d-model-of-a-dric_20260609_215049_780247_b6678542 (parent) | catch_basin, spout | inherited basin and spout | converged |
| round_basin | rec_drick_fountain_var_round_basin | catch_basin (round bowl) | circular catch basin | converged |
| bottle_filler | rec_drick_fountain_var_bottle_filler | bottle_filler_arch, basin_to_filler | upper bottle-filler spout module | converged |
| bubbler_spout | rec_drick_fountain_var_bubbler_spout | spout (arched bubbler) | arched bubbler spout over basin | converged |

### Slot C:actuator controls
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| single_push_button | parent | push_button, faceplate_to_button | one push actuator | converged |
| foot_pedal | rec_drick_fountain_var_foot_pedal | foot_pedal, pylon_to_pedal | lower foot-pedal actuator | converged |
| dual_push_buttons | rec_drick_fountain_var_dual_push_buttons | button_{i}, faceplate_to_button_{i} | two push buttons on the front/top control area | converged |

## Multiplicity / Copy Logic
- count_param: `button_count`.
- N 样本已覆盖: button_count {1, 2}.
- 模板建议 N_range: [1, 2] for this evidence set.
- copied object / naming / placement / joint policy: push buttons should be `button_{i}` or semantic hot/cold-style buttons with regular spacing and identical prismatic/revolute press policy.

## 组合数预审
Slot A(2) x Slot B(4) x Slot C(3) = 24 >= 10 ✓.

## 排除项(未来 compatibility matrix 素材)
- No blocked cells in this batch; all planned drinking fountain variants converged.
- bottle_filler changes the outlet module only; it should not force a wall-mounted body unless selected separately.
- foot_pedal and dual_push_buttons are alternate control candidates, not color/material variants.
