# 0611 / manual_hand_drill — template source map

pattern: mixed
parents: `rec_picturex_0611__manual_hand_drill__001__png_16df0d5a84934a8899b1d92b650247dc` (`pictureY/0611/manual_hand_drill/001.png`)
canonical_baselines: none planned; add only if a selected origin fails the §11 readability contract during fork review.
underfilled_reason: none at P0; adaptive coverage follows the approved per-category slot vocabulary.

## Subcategory Contract

- core_identity: hand-cranked manual drill retaining a chuck, gear train, crank, and hand grip
- must_keep: defining use, picture identity, visible support interfaces, and at least one real non-fixed joint.
- must_not_become: electric drill, brace-only auger
- image_evidence: pictureY/0611/manual_hand_drill/001.png
- parent_evidence: rec_picturex_0611__manual_hand_drill__001__png_16df0d5a84934a8899b1d92b650247dc

## Slot Candidates

| slot | candidate | diversity_axis | source_type | record/evidence | key parts/joints/helpers | status |
|---|---|---|---|---|---|---|
| origin_design | manual_hand_drill_001 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__manual_hand_drill__001__png_16df0d5a84934a8899b1d92b650247dc` / `pictureY/0611/manual_hand_drill/001.png` | body, drive_gear, spindle, chuck_sleeve, crank, crank_grip, speed_lever, body_to_drive_gear, body_to_spindle, spindle_to_chuck_sleeve, drive_gear_to_crank, crank_to_grip, body_to_speed_lever, _x_cylinder | built ✓ |
| gear_train | single-pinion drive | ② | forked_anchor | `rec_0611_manual_hand_drill_var_gear_train_single_pinion_drive` from `rec_picturex_0611__manual_hand_drill__001__png_16df0d5a84934a8899b1d92b650247dc` | drive_gear_to_crank, body_to_drive_gear, drive_gear, drive gear thrust washer seats against side plate, drive gear shaft remains captured by frame, drive gear shaft passes through side-plate bearing, _build_crank_arm, crank_to_grip | built ✓ |
| gear_train | dual-pinion drive | ② | forked_anchor | `rec_0611_manual_hand_drill_var_gear_train_dual_pinion_drive` from `rec_picturex_0611__manual_hand_drill__001__png_16df0d5a84934a8899b1d92b650247dc` | drive_gear_to_crank, body_to_drive_gear, drive_gear, drive gear thrust washer seats against side plate, drive gear shaft remains captured by frame, drive gear shaft passes through side-plate bearing, _build_crank_arm, crank_to_grip | built ✓ |
| gear_train | enclosed bevel drive | ② | forked_anchor | `rec_0611_manual_hand_drill_var_gear_train_enclosed_bevel_drive` from `rec_picturex_0611__manual_hand_drill__001__png_16df0d5a84934a8899b1d92b650247dc` | drive_gear_to_crank, body_to_drive_gear, drive_gear, drive gear thrust washer seats against side plate, drive gear shaft remains captured by frame, drive gear shaft passes through side-plate bearing, _build_crank_arm, crank_to_grip | built ✓ |
| grip_form | pistol grip | ③ | forked_anchor | `rec_0611_manual_hand_drill_var_grip_form_pistol_grip` from `rec_picturex_0611__manual_hand_drill__001__png_16df0d5a84934a8899b1d92b650247dc` | crank_to_grip, crank_grip, _wood_handle_mesh, wood_grip, main_handle_end, main_handle, handle_ferrule, grip_end | built ✓ |
| grip_form | breast-plate grip | ③ | forked_anchor | `rec_0611_manual_hand_drill_var_grip_form_breast_plate_grip` from `rec_picturex_0611__manual_hand_drill__001__png_16df0d5a84934a8899b1d92b650247dc` | crank_to_grip, crank_grip, _wood_handle_mesh, wood_grip, main_handle_end, main_handle, handle_ferrule, grip_end | built ✓ |
| speed_selection | sliding two-speed selector | ② | forked_anchor | `rec_0611_manual_hand_drill_var_speed_selection_sliding_two_speed_sele` from `rec_picturex_0611__manual_hand_drill__001__png_16df0d5a84934a8899b1d92b650247dc` | body_to_speed_lever, speed_lever, selector_tip, selector_mount, selector_boss, selector_arm | built ✓ |
| speed_selection | reversible ratchet selector | ② | forked_anchor | `rec_0611_manual_hand_drill_var_speed_selection_reversible_ratchet_sel` from `rec_picturex_0611__manual_hand_drill__001__png_16df0d5a84934a8899b1d92b650247dc` | body_to_speed_lever, speed_lever, selector_tip, selector_mount, selector_boss, selector_arm | built ✓ |
| chuck | collet chuck | ② | forked_anchor | `rec_0611_manual_hand_drill_var_chuck_collet_chuck` from `rec_picturex_0611__manual_hand_drill__001__png_16df0d5a84934a8899b1d92b650247dc` | spindle_to_chuck_sleeve, chuck_sleeve, _chuck_nose_mesh, chuck_nose, chuck_backstop | built ✓ |

## Multiplicity / Copy Logic

- count_param: no strong repeated-part axis planned
- N samples: origins only
- suggested N_range: bounded by accepted source samples and downstream compile budget.
- copied object / naming / placement / joint policy: shared helper, `name_{i}`, regular placement, uniform joint policy; exact names resolve from accepted variants.

## Six-Axis Diversity Record

| axis | treatment | values / range / reason |
|---|---|---|
| ① skeleton / structural topology | source-backed | origin rows plus planned ① candidates |
| ② joint / mechanism type | source-backed | origin rows plus planned ② candidates |
| ③ primary form family | source-backed | origin rows plus planned ③ candidates |
| ④ surface decoration | record_only / world_knowledge_extrapolation | host-conformal seams, ribs, labels, bezels only |
| ⑤ proportion / size / travel | record_only | origin ranges plus modest safe companion tuning |
| ⑥ material / palette / finish | record_only | origin materials plus realistic companion colorways |

## Compatibility Probes

| probe | record_id | combined axes | risk tested | result |
|---|---|---|---|---|
| none at P0 | — | — | add only if cross-family interface review finds a real risk | — |

## Blocked / Excluded

- ④/⑤/⑥-only forks: excluded; these do not count as candidate anchors.
- neighbor categories (electric drill, brace-only auger): excluded.
- failed or unfit candidates will be appended with one-line reasons after 2–3 attempts.
