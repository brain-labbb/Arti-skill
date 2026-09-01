# 0611 / tv_wall_mount — template source map

pattern: mixed
parents: `rec_picturex_0611__tv_wall_mount__004__png_62c79a78a7ac4b8681822a3374714d9f` (`pictureY/0611/tv_wall_mount/004.png`), `rec_picturex_0611__tv_wall_mount__001__png_256af3716b30482fa15f69af5e2edcd4` (`pictureY/0611/tv_wall_mount/001.png`), `rec_picturex_0611__tv_wall_mount__002__png_742289b568674f468cc34207a1ecaffd` (`pictureY/0611/tv_wall_mount/002.png`), `rec_picturex_0611__tv_wall_mount__003__png_d5fa4b15696c407b9ae0e136f07b9080` (`pictureY/0611/tv_wall_mount/003.png`)
canonical_baselines: none planned; add only if a selected origin fails the §11 readability contract during fork review.
underfilled_reason: none at P0; adaptive coverage follows the approved per-category slot vocabulary.

## Subcategory Contract

- core_identity: wall-mounted television bracket retaining wall interface, VESA screen interface, and declared adjustment motion
- must_keep: defining use, picture identity, visible support interfaces, and at least one real non-fixed joint.
- must_not_become: monitor desk arm, floor TV stand
- image_evidence: pictureY/0611/tv_wall_mount/004.png, pictureY/0611/tv_wall_mount/001.png, pictureY/0611/tv_wall_mount/002.png, pictureY/0611/tv_wall_mount/003.png
- parent_evidence: rec_picturex_0611__tv_wall_mount__004__png_62c79a78a7ac4b8681822a3374714d9f, rec_picturex_0611__tv_wall_mount__001__png_256af3716b30482fa15f69af5e2edcd4, rec_picturex_0611__tv_wall_mount__002__png_742289b568674f468cc34207a1ecaffd, rec_picturex_0611__tv_wall_mount__003__png_d5fa4b15696c407b9ae0e136f07b9080

## Slot Candidates

| slot | candidate | diversity_axis | source_type | record/evidence | key parts/joints/helpers | status |
|---|---|---|---|---|---|---|
| origin_design | tv_wall_mount_004 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__tv_wall_mount__004__png_62c79a78a7ac4b8681822a3374714d9f` / `pictureY/0611/tv_wall_mount/004.png` | wall_plate, rear_arm, front_arm, tilt_head, f'rail_{index}', f'tilt_knob_{index}', wall_swivel, middle_swivel, screen_tilt, f'rail_slide_{index}', f'tilt_knob_turn_{index}', _box, _cylinder_z, _cylinder_y | built ✓ |
| origin_design | extended_full_motion_tv_wall_mount | ①/②/③ observed | origin_anchor | `rec_picturex_0611__tv_wall_mount__001__png_256af3716b30482fa15f69af5e2edcd4` / `pictureY/0611/tv_wall_mount/001.png` | wall_plate, shoulder_arm, forearm, head_yoke, tilt_plate, tilt_knob_0, tilt_knob_1, f'vesa_rail_{index}', wall_swivel, elbow_fold, head_swivel, screen_tilt, tilt_knob_turn_0, tilt_knob_turn_1 | built ✓ |
| origin_design | tv_wall_mount_002 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__tv_wall_mount__002__png_742289b568674f468cc34207a1ecaffd` / `pictureY/0611/tv_wall_mount/002.png` | wall_plate, swivel_post, lower_arm, upper_arm, wrist_block, tilt_yoke, vesa_plate, tilt_knob, base_swivel, shoulder_fold, elbow_fold, wrist_swivel, plate_tilt, plate_roll | built ✓ |
| origin_design | tv_wall_mount_003 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__tv_wall_mount__003__png_d5fa4b15696c407b9ae0e136f07b9080` / `pictureY/0611/tv_wall_mount/003.png` | wall_plate, shoulder_arm, forearm, vesa_plate, elbow_knob, tilt_knob, wall_to_shoulder, shoulder_to_forearm, forearm_to_vesa, forearm_to_elbow_knob, forearm_to_tilt_knob, _capsule_link, _wall_structure, _shoulder_body | built ✓ |
| arm_topology | fixed low-profile plate | ② | forked_anchor | `rec_0611_tv_wall_mount_var_arm_topology_fixed_low_profile_plate` from `rec_picturex_0611__tv_wall_mount__004__png_62c79a78a7ac4b8681822a3374714d9f` | rear_arm, front_arm, _rear_arm_shape, _front_arm_shape, _arm_shape, rear_arm_truss, front_arm_truss, wall_plate | built ✓ |
| arm_topology | tilt-only bracket | ② | forked_anchor | `rec_0611_tv_wall_mount_var_arm_topology_tilt_only_bracket` from `rec_picturex_0611__tv_wall_mount__001__png_256af3716b30482fa15f69af5e2edcd4` | shoulder_arm, shoulder_arm_shell, tilt_knob_turn_1, tilt_knob_turn_0, screen_tilt, tilt_plate, tilt_knob_1, tilt_knob_0 | built ✓ |
| arm_topology | single-arm full-motion | ② | forked_anchor | `rec_0611_tv_wall_mount_var_arm_topology_single_arm_full_motion` from `rec_picturex_0611__tv_wall_mount__002__png_742289b568674f468cc34207a1ecaffd` | upper_arm, lower_arm, _arm_cover, upper_arm_cover, lower_arm_cover, shoulder_fold, knob_turn, elbow_fold | built ✓ |
| arm_topology | dual-arm full-motion | ② | forked_anchor | `rec_0611_tv_wall_mount_var_arm_topology_dual_arm_full_motion` from `rec_picturex_0611__tv_wall_mount__003__png_d5fa4b15696c407b9ae0e136f07b9080` | shoulder_arm, _capsule_link, stacked elbow arms meet at washer | built ✓ |
| arm_topology | articulating frame | ② | forked_anchor | `rec_0611_tv_wall_mount_var_arm_topology_articulating_frame` from `rec_picturex_0611__tv_wall_mount__004__png_62c79a78a7ac4b8681822a3374714d9f` | rear_arm, front_arm, _rear_arm_shape, _front_arm_shape, _arm_shape, wall_plate_frame, rear_arm_truss, front_arm_truss | built ✓ |
| vesa_interface | crossed rails | ① | forked_anchor | `rec_0611_tv_wall_mount_var_vesa_interface_crossed_rails` from `rec_picturex_0611__tv_wall_mount__001__png_256af3716b30482fa15f69af5e2edcd4` | f'vesa_rail_{index}', f'plate_to_rail_{index}', _rail_shape, rail_shell | built ✓ |
| vesa_interface | four independent arms | ① | forked_anchor | `rec_0611_tv_wall_mount_var_vesa_interface_four_independent_arms` from `rec_picturex_0611__tv_wall_mount__003__png_d5fa4b15696c407b9ae0e136f07b9080` | forearm_to_vesa, vesa_plate, shoulder_arm, stacked elbow arms meet at washer, _vesa_plate, vesa_face | built ✓ |
| vesa_interface | sliding twin rails | ① | forked_anchor | `rec_0611_tv_wall_mount_var_vesa_interface_sliding_twin_rails` from `rec_picturex_0611__tv_wall_mount__004__png_62c79a78a7ac4b8681822a3374714d9f` | f'rail_slide_{index}', f'rail_{index}', _vesa_head_shape, _rail_shape, vesa_plate, rail_shell | built ✓ |
| vesa_interface | universal plate | ① | forked_anchor | `rec_0611_tv_wall_mount_var_vesa_interface_universal_plate` from `rec_picturex_0611__tv_wall_mount__002__png_742289b568674f468cc34207a1ecaffd` | vesa_plate, _vesa_plate_shape, plate_tilt, plate_roll, wall_plate, vesa_plate_shell, VESA plate is seated on the roll hub, plate_spine | built ✓ |
| height_adjustment | gas-spring vertical track | ② | forked_anchor | `rec_0611_tv_wall_mount_var_height_adjustment_gas_spring_vertical` from `rec_picturex_0611__tv_wall_mount__004__png_62c79a78a7ac4b8681822a3374714d9f` | f'tilt_knob_turn_{index}', f'rail_slide_{index}', f'tilt_knob_{index}', wall swivel bolt stays centered in rear collar, wall clevis captures rear collar vertically, middle swivel bolt stays centered in front collar, middle fork captures front collar vertically, knob_shaft | built ✓ |
| height_adjustment | toothed lift track | ② | forked_anchor | `rec_0611_tv_wall_mount_var_height_adjustment_toothed_lift_track` from `rec_picturex_0611__tv_wall_mount__004__png_62c79a78a7ac4b8681822a3374714d9f` | f'tilt_knob_turn_{index}', f'rail_slide_{index}', f'tilt_knob_{index}', wall swivel bolt stays centered in rear collar, wall clevis captures rear collar vertically, middle swivel bolt stays centered in front collar, middle fork captures front collar vertically, knob_shaft | built ✓ |
| height_adjustment | counterweighted lift | ② | forked_anchor | `rec_0611_tv_wall_mount_var_height_adjustment_counterweighted_lift` from `rec_picturex_0611__tv_wall_mount__004__png_62c79a78a7ac4b8681822a3374714d9f` | f'tilt_knob_turn_{index}', f'rail_slide_{index}', f'tilt_knob_{index}', wall swivel bolt stays centered in rear collar, wall clevis captures rear collar vertically, middle swivel bolt stays centered in front collar, middle fork captures front collar vertically, knob_shaft | built ✓ |
| screen_motion | tilt-swivel head | ② | forked_anchor | `rec_0611_tv_wall_mount_var_screen_motion_tilt_swivel_head` from `rec_picturex_0611__tv_wall_mount__001__png_256af3716b30482fa15f69af5e2edcd4` | head_swivel, head_yoke, _head_yoke_shape, extended_full_motion_tv_wall_mount, tilt_knob_turn_1, tilt_knob_turn_0, screen_tilt, head_yoke_shell | built ✓ |
| screen_motion | portrait-roll head | ② | forked_anchor | `rec_0611_tv_wall_mount_var_screen_motion_portrait_roll_head` from `rec_picturex_0611__tv_wall_mount__001__png_256af3716b30482fa15f69af5e2edcd4` | head_swivel, head_yoke, _head_yoke_shape, extended_full_motion_tv_wall_mount, head_yoke_shell, head_pin, tilt_knob_turn_1, tilt_knob_turn_0 | built ✓ |
| screen_motion | push-pull depth carriage | ② | forked_anchor | `rec_0611_tv_wall_mount_var_screen_motion_push_pull_depth_carriage` from `rec_picturex_0611__tv_wall_mount__001__png_256af3716b30482fa15f69af5e2edcd4` | extended_full_motion_tv_wall_mount, tilt_knob_turn_1, tilt_knob_turn_0, screen_tilt, elbow_fold, _tilt_carriage_shape, wall_pivot_pin, tilt_carriage | built ✓ |
| lock | pull-cord screen latch | ② | forked_anchor | `rec_0611_tv_wall_mount_var_lock_pull_cord_screen_latch` from `rec_picturex_0611__tv_wall_mount__003__png_d5fa4b15696c407b9ae0e136f07b9080` | tilt_lock_grip, elbow_lock_grip | built ✓ |

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
- neighbor categories (monitor desk arm, floor TV stand): excluded.
- failed or unfit candidates will be appended with one-line reasons after 2–3 attempts.
