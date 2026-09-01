# Curtain / blind - template source map

pattern: mixed
parents:
- rec_horizontal-wooden-venetian-blind-a-wooden-headra_20260611_160824_216734_cc3eb44b <- picture/Curtain/blind/002.png (horizontal wooden venetian blind: wooden headrail box, dark wood slats on ladder tapes, heavier bottom rail, lift cords + tilt wand). Covers Slot A=horizontal_venetian, Slot B=parent_slat_count, Slot C=single_stack.
- rec_vertical-blinds-for-a-tall-window-a-long-extrude_20260611_160757_186286_2182787b <- picture/Curtain/blind/001.png (vertical blinds for a tall window: long extruded-aluminum headrail with carrier hooks, gray fabric vertical vanes side by side, control wand with chain at the left end; vanes tilt in unison and the vane set traverses along the headrail). Covers Slot A=vertical_vanes, Slot B=parent_vane_count, Slot C=single_traverse.

Window blind family with headrail/track, shade panel type, repeated slats or vanes, and traverse/lift
controls. Variants isolate shade topology, slat-count multiplicity, and vertical split/traverse policy.

## Slot 候选覆盖

### Slot A:shade panel topology
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| horizontal_venetian | rec_horizontal-wooden-venetian-blind-a-wooden-headra_20260611_160824_216734_cc3eb44b (parent) | slat loop, tilt/lift joints | horizontal venetian slats under headrail | converged |
| vertical_vanes | rec_vertical-blinds-for-a-tall-window-a-long-extrude_20260611_160757_186286_2182787b (parent) | vane loop, track/traverse | vertical hanging fabric vanes | converged |
| roller_shade | rec_blind_var_roller_shade | roller tube, sheet, bottom bar | roll-down fabric sheet on a top roller tube | converged |
| roman_folds | rec_blind_var_roman_folds | fold panel loop | stacked roman fabric folds | converged |
| cellular_honeycomb | rec_blind_var_cellular_honeycomb | honeycomb cell loop | cellular/honeycomb shade panel cells | converged |

### Slot B:slat / vane count
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| parent_count | parents | `slat_{i}` or `vane_{i}` loop | inherited slat/vane count and spacing | converged |
| slat_count_12 | rec_blind_var_slat_count_12 | `slat_1` through `slat_12` | horizontal blind with 12 slats | converged |
| slat_count_40 | rec_blind_var_slat_count_40 | `slat_1` through `slat_40` | dense horizontal blind with 40 slats | converged |

### Slot C:traverse / split policy
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| single_stack | parents | one slat/vane set under one headrail | one continuous blind stack | converged |
| center_split_vertical | rec_blind_var_center_split_vertical | left/right vane sets, traverse joints | vertical blind split into two traversing sets | converged |

## Multiplicity / Copy Logic
- count_param: `slat_count` for horizontal venetian candidates and `vane_count` for vertical candidates.
- N 样本已覆盖: slat_count {12, 40, parent}; vertical split side_count {2}.
- 模板建议 N_range: [12, 40] for horizontal slats; vertical vane count should be separately sampled before generalization.
- copied object / naming / placement / joint policy: repeated slats/vanes emitted with `slat_{i}` / `vane_{i}` names, regular spacing, and identical tilt/traverse joint policy.

## 组合数预审
Slot A(5) x Slot B(3) x Slot C(2) = 30 >= 10 ✓.

## 排除项(未来 compatibility matrix 素材)
- No blocked cells in this batch; all planned blind variants converged.
- slat_count applies to horizontal venetian candidates; roller/roman/cellular candidates use their own local fold/cell counts.
- center_split_vertical is only directly sampled with vertical vanes and should be compatibility-checked before applying to roller or cellular styles.
