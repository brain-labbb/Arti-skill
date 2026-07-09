# Equipment / Lock — template source map

pattern: mixed
parents:
- rec_build-a-realistic-articulated-3d-model-of-a-lock_20260609_180057_850280_fa3217d6 ← picture/Equipment/Lock/001.png (brass keyed padlock, tall U shackle, keyway disc/slot). Covers Slot A=rect_brass_body, Slot B=tall_u_shackle, Slot C=keyway_access.
- rec_build-a-realistic-articulated-3d-model-of-a-lock_20260609_180101_520334_f96f0b30 ← picture/Equipment/Lock/002.png (armored combination padlock, orange/black body, U shackle, four rotating dials). Covers Slot A=armored_combo_body, Slot B=tall_u_shackle, Slot C=dial_stack_access, dial_count=4.

Padlock family. Core motion is a shackle release/lift, plus either keyed access or a
rotating dial stack. The two parents occupy distinct access families; variants fill body
shell, shackle protection/form, access mechanism, and dial-count multiplicity evidence.

## Slot 候选覆盖

### Slot A:body shell
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| rect_brass_body | rec_build-a-realistic-articulated-3d-model-of-a-lock_20260609_180057_850280_fa3217d6 (parent) | `body` brass shell, keyway visuals | simple rectangular brass keyed padlock body | converged |
| armored_combo_body | rec_build-a-realistic-articulated-3d-model-of-a-lock_20260609_180101_520334_f96f0b30 (parent) | `body`, faceplate, corner rivets, dial window | armored orange/black combination-lock body with dial recess | converged |
| laminated_steel_body | rec_lock_var_laminated_body | `body` laminated shell layers | stacked horizontal laminated steel plate body | converged |
| round_discus_body | rec_lock_var_round_disc_body | `body` circular/discus shell | round discus-style body with partially recessed shackle | converged |

### Slot B:shackle form / protection
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| tall_u_shackle | rec_build-a-realistic-articulated-3d-model-of-a-lock_20260609_180057_850280_fa3217d6 (parent) | `shackle`, `body_to_shackle` | tall exposed U shackle with prismatic release lift | converged |
| shrouded_u_shackle | rec_lock_var_shrouded_shackle | `shackle`, `body_to_shackle`, protective shoulders | short shrouded U shackle protected by raised body shoulders | converged |
| straight_bar_hasp | rec_lock_var_straight_bar_shackle | straight bar shackle and release lift | straight sliding hasp bar across the top with prismatic release | converged |

### Slot C:access mechanism
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| keyway_access | rec_build-a-realistic-articulated-3d-model-of-a-lock_20260609_180057_850280_fa3217d6 (parent) | `body.keyway_disc`, `body.keyway_slot` | visible keyed disc and vertical slot on the front face | converged |
| dial_stack_access | rec_build-a-realistic-articulated-3d-model-of-a-lock_20260609_180101_520334_f96f0b30 (parent) | `dial_{i}`, `dial_{i}` joints, dial window | rotating combination dial stack in a front window | converged |
| keyway_dust_cover | rec_lock_var_keyway_dust_cover | `dust_cover`, cover hinge, keyway underneath | hinged swinging dust cover over the keyway, plus original shackle release | converged |

## Multiplicity / Copy Logic
- count_param: `dial_count`.
- N 样本已覆盖: {3, 4, 5} → rec_lock_var_three_dials / combination parent / rec_lock_var_five_dials.
- 模板建议 N_range: [3, 5] for compact padlocks; wider ranges need more dial-window/body evidence.
- copied object / naming / placement / joint policy: copied object = one numbered dial wheel with axle/number markings; naming = `dial_{i}` part/joint; placement = regular vertical stack in front window; joint policy = each dial has an independent CONTINUOUS rotation about the same horizontal axis.

## 组合数预审
Slot A(4) × Slot B(3) × Slot C(3) × dial_count samples(3) = 108 ≥ 10 ✓.

## 排除项(未来 compatibility matrix 素材)
- No blocked cells in this batch; all planned cells converged.
- dial_count applies only to `dial_stack_access`; keyed access candidates ignore `dial_count`.
- round_discus_body with full dial_stack_access was not directly sampled; template side should treat it as a compatibility question unless implemented with an appropriate front dial window.
