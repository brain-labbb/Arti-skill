# 0611 / manual_pipe_bender — template source map

pattern: mixed
parents: `rec_create-exactly-one-articulated-3d-articraft-asse_20260710_122307_880114_0d2fa0d0` (`pictureY/0611/manual_pipe_bender/003.png`), `rec_picturex_0611__manual_pipe_bender__004__png_75c4edd745e04e5f94a148bf3f55d5f1` (`pictureY/0611/manual_pipe_bender/004.png`), `rec_picturex_0611__manual_pipe_bender__005__png_388a055eab4842619094fb0bea63f2a0` (`pictureY/0611/manual_pipe_bender/005.png`), `rec_use-only-the-attached-reference-image-picturex-0_20260710_094545_791514_ca71ebd5` (`pictureY/0611/manual_pipe_bender/002.png`), `rec_use-the-attached-reference-image-as-the-authorit_20260710_093850_655349_8881bc1a` (`pictureY/0611/manual_pipe_bender/001.png`)
canonical_baselines: none planned; add only if a selected origin fails the §11 readability contract during fork review.
underfilled_reason: none at P0; adaptive coverage follows the approved per-category slot vocabulary.

## Subcategory Contract

- core_identity: manual pipe or conduit bender retaining a former and controlled bending motion
- must_keep: defining use, picture identity, visible support interfaces, and at least one real non-fixed joint.
- must_not_become: powered tube mill, pipe cutter
- image_evidence: pictureY/0611/manual_pipe_bender/003.png, pictureY/0611/manual_pipe_bender/004.png, pictureY/0611/manual_pipe_bender/005.png, pictureY/0611/manual_pipe_bender/002.png, pictureY/0611/manual_pipe_bender/001.png
- parent_evidence: rec_create-exactly-one-articulated-3d-articraft-asse_20260710_122307_880114_0d2fa0d0, rec_picturex_0611__manual_pipe_bender__004__png_75c4edd745e04e5f94a148bf3f55d5f1, rec_picturex_0611__manual_pipe_bender__005__png_388a055eab4842619094fb0bea63f2a0, rec_use-only-the-attached-reference-image-picturex-0_20260710_094545_791514_ca71ebd5, rec_use-the-attached-reference-image-as-the-authorit_20260710_093850_655349_8881bc1a

## Slot Candidates

| slot | candidate | diversity_axis | source_type | record/evidence | key parts/joints/helpers | status |
|---|---|---|---|---|---|---|
| origin_design | floor_manual_rotary_pipe_bender | ①/②/③ observed | origin_anchor | `rec_create-exactly-one-articulated-3d-articraft-asse_20260710_122307_880114_0d2fa0d0` / `pictureY/0611/manual_pipe_bender/003.png` | stand, bending_handle, toothed_arm, pressure_roller, handle_swing, ratchet_pivot, roller_spin, _slotted_floor_plate, _perforated_index_plate, _annular_hub, _follower_bracket, _toothed_ratchet_arm | built ✓ |
| origin_design | manual_pipe_bender_004 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__manual_pipe_bender__004__png_75c4edd745e04e5f94a148bf3f55d5f1` / `pictureY/0611/manual_pipe_bender/004.png` | frame, bending_die, main_lever, follower_shoe, pipe_cradle, f'guide_roller_{index}', f'adjustment_block_{index}', f'adjustment_screw_{index}', die_pivot, lever_pivot, follower_adjust, cradle_mount, f'roller_spin_{index}', f'block_slide_{index}' | built ✓ |
| origin_design | manual_pipe_bender_005 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__manual_pipe_bender__005__png_388a055eab4842619094fb0bea63f2a0` / `pictureY/0611/manual_pipe_bender/005.png` | base, bending_die, main_lever, follower_shoe, follower_roller, pipe_cradle, guide_roller, adjustment_screw, base_to_lever, base_to_die, lever_to_shoe, shoe_to_roller, base_to_cradle, cradle_to_roller | built ✓ |
| origin_design | compact_portable_hydraulic_pipe_bender | ①/②/③ observed | origin_anchor | `rec_use-only-the-attached-reference-image-picturex-0_20260710_094545_791514_ca71ebd5` / `pictureY/0611/manual_pipe_bender/002.png` | main_frame, ram_carriage, bending_former, pump_handle, workpiece, f'guide_{index}', frame_to_ram, frame_to_former, frame_to_handle, frame_to_workpiece, f'frame_to_guide_{index}', _rounded_box, _annular_disc, _former_shape | built ✓ |
| origin_design | compact_manual_pipe_bender | ①/②/③ observed | origin_anchor | `rec_use-the-attached-reference-image-as-the-authorit_20260710_093850_655349_8881bc1a` / `pictureY/0611/manual_pipe_bender/001.png` | frame, upper_handle, bending_former, follower_roller, stop_pin, spare_former, handle_pivot, former_rotation, follower_rotation, stop_adjustment, spare_mount, _ribbon_profile, _ribbon, _stepped_former | built ✓ |
| bender_topology | conduit shoe lever | ② | forked_anchor | `rec_0611_manual_pipe_bender_var_bender_topology_conduit_shoe_lever` from `rec_create-exactly-one-articulated-3d-articraft-asse_20260710_122307_880114_0d2fa0d0` | handle_swing, toothed_arm, bending_handle, _toothed_ratchet_arm, upper_handle_rail, toothed_arm_body, swinging upper handle clears fixed upper frame, swinging lower handle clears fixed mid frame | built ✓ |
| bender_topology | ratcheting crossbow | ② | forked_anchor | `rec_0611_manual_pipe_bender_var_bender_topology_ratcheting_crossbow` from `rec_picturex_0611__manual_pipe_bender__004__png_75c4edd745e04e5f94a148bf3f55d5f1` | manual_pipe_bender_004 | built ✓ |
| bender_topology | three-roll pyramid | ② | forked_anchor | `rec_0611_manual_pipe_bender_var_bender_topology_three_roll_pyramid` from `rec_picturex_0611__manual_pipe_bender__005__png_388a055eab4842619094fb0bea63f2a0` | manual_pipe_bender_005 | built ✓ |
| bender_topology | compact rotary-draw bench | ② | forked_anchor | `rec_0611_manual_pipe_bender_var_bender_topology_compact_rotary_draw_be` from `rec_use-only-the-attached-reference-image-picturex-0_20260710_094545_791514_ca71ebd5` | compact_portable_hydraulic_pipe_bender | built ✓ |
| bender_topology | geared ring roller | ② | forked_anchor | `rec_0611_manual_pipe_bender_var_bender_topology_geared_ring_roller` from `rec_use-the-attached-reference-image-as-the-authorit_20260710_093850_655349_8881bc1a` | follower_roller, roller_body, follower roller seats on moving handle arm, former_axle_head, former_axle, follower_axle_head, follower_axle, compact_manual_pipe_bender | built ✓ |
| bender_topology | hickey-style offset bender | ② | forked_anchor | `rec_0611_manual_pipe_bender_var_bender_topology_hickey_style_offset_be` from `rec_create-exactly-one-articulated-3d-articraft-asse_20260710_122307_880114_0d2fa0d0` | floor_manual_rotary_pipe_bender | built ✓ |
| drive | geared handwheel | ② | forked_anchor | `rec_0611_manual_pipe_bender_var_drive_geared_handwheel` from `rec_use-the-attached-reference-image-as-the-authorit_20260710_093850_655349_8881bc1a` | handle_pivot, upper_handle, follower roller seats on moving handle arm, handle cheek remains captured on pivot pin, former_axle_head, follower_axle_head | built ✓ |
| drive | screw-feed ram | ② | forked_anchor | `rec_0611_manual_pipe_bender_var_drive_screw_feed_ram` from `rec_create-exactly-one-articulated-3d-articraft-asse_20260710_122307_880114_0d2fa0d0` | handle_swing, toothed_arm, bending_handle, _toothed_ratchet_arm, upper_handle_rail, toothed_arm_body, swinging upper handle clears fixed upper frame, swinging lower handle clears fixed mid frame | built ✓ |
| drive | ratchet-sector drive | ② | forked_anchor | `rec_0611_manual_pipe_bender_var_drive_ratchet_sector_drive` from `rec_create-exactly-one-articulated-3d-articraft-asse_20260710_122307_880114_0d2fa0d0` | _toothed_ratchet_arm, ratchet_pivot, handle_swing, toothed_arm, bending_handle, ratchet arm remains captured on pivot pin, upper_handle_rail, toothed_arm_body | built ✓ |
| drive | chain-sprocket drive | ② | forked_anchor | `rec_0611_manual_pipe_bender_var_drive_chain_sprocket_drive` from `rec_use-the-attached-reference-image-as-the-authorit_20260710_093850_655349_8881bc1a` | handle_pivot, upper_handle, follower roller seats on moving handle arm, handle cheek remains captured on pivot pin, former_axle_head, follower_axle_head | built ✓ |
| former_count | 2-former turret | N | forked_anchor | `rec_0611_manual_pipe_bender_var_former_count_2_former_turret` from `rec_picturex_0611__manual_pipe_bender__004__png_75c4edd745e04e5f94a148bf3f55d5f1` | f'screw_turn_{index}', f'roller_spin_{index}', f'block_slide_{index}', f'guide_roller_{index}', f'adjustment_screw_{index}', f'adjustment_block_{index}', grooved_former, die_index_pin | built ✓ |
| former_count | 3-former turret | N | forked_anchor | `rec_0611_manual_pipe_bender_var_former_count_3_former_turret` from `rec_use-only-the-attached-reference-image-picturex-0_20260710_094545_791514_ca71ebd5` | frame_to_former, f'frame_to_guide_{index}', f'guide_{index}', bending_former, _former_shape, gray-green pipe seats against red former, former_die, former_crosshead | built ✓ |
| former_count | 5-former rack | N | forked_anchor | `rec_0611_manual_pipe_bender_var_former_count_5_former_rack` from `rec_picturex_0611__manual_pipe_bender__004__png_75c4edd745e04e5f94a148bf3f55d5f1` | f'screw_turn_{index}', f'roller_spin_{index}', f'block_slide_{index}', cradle_mount, pipe_cradle, frame, f'guide_roller_{index}', f'adjustment_screw_{index}' | built ✓ |
| guide_roller_count | 3 guide rollers | N | forked_anchor | `rec_0611_manual_pipe_bender_var_guide_roller_count_3_guide_rollers` from `rec_use-only-the-attached-reference-image-picturex-0_20260710_094545_791514_ca71ebd5` | f'frame_to_guide_{index}', f'guide_{index}', roller_ring, roller_cap, ram remains captured on the guide rods, guide_rod_1, guide_rod_0, former_bearing | built ✓ |
| mount | vise-mounted frame | ① | forked_anchor | `rec_0611_manual_pipe_bender_var_mount_vise_mounted_frame` from `rec_create-exactly-one-articulated-3d-articraft-asse_20260710_122307_880114_0d2fa0d0` | upper_frame_plate, swinging upper handle clears fixed upper frame, swinging lower handle clears fixed mid frame, mid_frame_plate, toothed_arm_body, roller_body, fixed_support_roller, slotted_base | built ✓ |
| mount | wheeled floor stand | ① | forked_anchor | `rec_0611_manual_pipe_bender_var_mount_wheeled_floor_stand` from `rec_create-exactly-one-articulated-3d-articraft-asse_20260710_122307_880114_0d2fa0d0` | stand, fixed_support_roller, _slotted_floor_plate, slotted_base, ratchet_mount, floor_manual_rotary_pipe_bender | built ✓ |
| former_profile | square-tube die | ③ | forked_anchor | `rec_0611_manual_pipe_bender_var_former_profile_square_tube_die` from `rec_picturex_0611__manual_pipe_bender__004__png_75c4edd745e04e5f94a148bf3f55d5f1` | die_pivot, bending_die, _build_die_shape, handle_tube, grooved_former, die_index_pin, bending die is captured on main pivot pin | built ✓ |
| former_profile | multi-radius stepped die | ③ | forked_anchor | `rec_0611_manual_pipe_bender_var_former_profile_multi_radius_stepped_di` from `rec_use-the-attached-reference-image-as-the-authorit_20260710_093850_655349_8881bc1a` | _stepped_former, former_rotation, spare_former, bending_former, _ribbon_profile, main former seats on curved steel frame, loose spare former is visibly separated, former_body | built ✓ |

## Multiplicity / Copy Logic

- count_param: former_count_count, guide_roller_count_count
- N samples: 2-former turret, 3-former turret, 5-former rack, 3 guide rollers
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
- neighbor categories (powered tube mill, pipe cutter): excluded.
- failed or unfit candidates will be appended with one-line reasons after 2–3 attempts.
