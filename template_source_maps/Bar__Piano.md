# Bar / Piano - template source map

pattern: mixed
parents:
- rec_a-glossy-black-grand-piano-with-its-curved-wing-_20260605_132149_762624_be39da53 <- picture/Bar/Piano/001.png (black grand piano with curved case, hinged top lid, keyboard/fallboard, legs, pedals). Covers Slot A=grand_case, Slot B=standard_top_lid, Slot C=three_pedals.
- rec_a-glossy-black-upright-piano-standing-tall-again_20260605_132213_191152_c63fde85 <- picture/Bar/Piano/002.png (black upright piano cabinet with keyboard, fallboard, top lid, pedal box). Covers Slot A=upright_case, Slot B=upright_lid_fallboard, Slot C=three_pedals.

Piano family with cabinet/case body, keyboard cover/top lid mechanisms, pedals, and music-desk
accessory layer. Variants isolate body footprint, lid/fallboard articulation, pedal multiplicity,
and fold-down music support.

## Slot 候选覆盖

### Slot A:case / cabinet body
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| grand_case | rec_a-glossy-black-grand-piano-with-its-curved-wing-_20260605_132149_762624_be39da53 (parent) | curved case, lid joint | full grand-piano curved wing case | converged |
| upright_case | rec_a-glossy-black-upright-piano-standing-tall-again_20260605_132213_191152_c63fde85 (parent) | upright cabinet, fallboard, pedal box | tall vertical upright cabinet | converged |
| baby_grand_case | rec_piano_var_baby_grand_body | compact curved case, retained lid/keys | shorter baby-grand footprint | converged |
| spinet_upright_case | rec_piano_var_spinet_upright_body | compact upright cabinet, retained key/pedal joints | lower compact spinet upright body | converged |

### Slot B:lid / fallboard mechanism
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| standard_lid_fallboard | parents | top lid/fallboard joints | parent top lid and keyboard cover policy | converged |
| split_top_lid | rec_piano_var_split_top_lid | `lid_front`/`lid_rear` leaves + per-leaf hinge strips | two-leaf front/rear split hinged top lid | converged |
| sliding_fallboard | rec_piano_var_sliding_fallboard | fallboard prismatic slide | sliding fallboard cover over keyboard | converged |

### Slot C:pedal / music support layer
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| three_pedals | parents | pedal parts/joints | conventional three-pedal set | converged |
| four_pedals | rec_piano_var_four_pedals | `pedal_{i}` loop | four pedal multiplicity, emitted as looped copies | converged |
| fold_down_music_desk | rec_piano_var_fold_down_music_desk | music desk hinge | hinged fold-down sheet-music rest behind keyboard | converged |

## Multiplicity / Copy Logic
- count_param: `pedal_count`.
- N 样本已覆盖: {3, 4}.
- 模板建议 N_range: [2, 4] for piano pedals; wider pedal counts need additional console evidence.
- copied object / naming / placement / joint policy: pedal modules should be loop-emitted as `pedal_{i}` with regular lateral spacing and identical revolute/prismatic press policy.

## 组合数预审
Slot A(4) x Slot B(3) x Slot C(3) = 36 >= 10 ✓.

## 排除项(未来 compatibility matrix 素材)
- No blocked cells in this batch; all planned piano variants converged.
- four_pedals applies to pedal layer only; it should not force a case-body change.
- fold_down_music_desk is an accessory/control-surface candidate and should stay independent from case footprint except for mounting coordinates.
