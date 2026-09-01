# 0611 / tripod — template source map

pattern: mixed
parents: `rec_picturex_0611__tripod__001__png_aa32bb13f66c426592ea992cfd8b0968` (`pictureY/0611/tripod/001.png`), `rec_picturex_0611__tripod__002__png_e0249c65a8ef423e899cb4e318390384` (`pictureY/0611/tripod/002.png`), `rec_picturex_0611__tripod__003__png_e87f6574e5ab4934bd28f3b9649f8763` (`pictureY/0611/tripod/003.png`)
canonical_baselines: none planned; add only if a selected origin fails the §11 readability contract during fork review.
underfilled_reason: none at P0; adaptive coverage follows the approved per-category slot vocabulary.

## Subcategory Contract

- core_identity: three-legged equipment support retaining deployable legs and a mounting head
- must_keep: defining use, picture identity, visible support interfaces, and at least one real non-fixed joint.
- must_not_become: four-leg stand, monopod
- image_evidence: pictureY/0611/tripod/001.png, pictureY/0611/tripod/002.png, pictureY/0611/tripod/003.png
- parent_evidence: rec_picturex_0611__tripod__001__png_aa32bb13f66c426592ea992cfd8b0968, rec_picturex_0611__tripod__002__png_e0249c65a8ef423e899cb4e318390384, rec_picturex_0611__tripod__003__png_e87f6574e5ab4934bd28f3b9649f8763

## Slot Candidates

| slot | candidate | diversity_axis | source_type | record/evidence | key parts/joints/helpers | status |
|---|---|---|---|---|---|---|
| origin_design | pictureX_0611_tripod_001 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__tripod__001__png_aa32bb13f66c426592ea992cfd8b0968` / `pictureY/0611/tripod/001.png` | chassis, center_column, column_crank, pan_base, tilt_head, mounting_plate, pan_lock, tilt_lock, f'upper_leg_{index}', f'middle_leg_{index}', f'lower_leg_{index}', f'brace_{index}', f'{clamp_name}_lever_{index}', center_column_slide | built ✓ |
| origin_design | compact_camera_tripod_002 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__tripod__002__png_e0249c65a8ef423e899cb4e318390384` / `pictureY/0611/tripod/002.png` | hub, pan_body, tilt_head, pan_lock, mount_screw, f'upper_leg_{index}', f'lower_leg_{index}', f'clamp_lever_{index}', head_pan, head_tilt, pan_lock_turn, mount_screw_turn, f'leg_hinge_{index}', f'leg_extension_{index}' | built ✓ |
| origin_design | compact_threaded_tripod | ①/②/③ observed | origin_anchor | `rec_picturex_0611__tripod__003__png_e87f6574e5ab4934bd28f3b9649f8763` / `pictureY/0611/tripod/003.png` | hub, f'leg_{index}', f'hub_to_leg_{index}', _segment_origin | built ✓ |
| leg_stages | 1 stage | N | forked_anchor | `rec_0611_tripod_var_leg_stages_1_stage` from `rec_picturex_0611__tripod__001__png_aa32bb13f66c426592ea992cfd8b0968` | plate_slide, f'leg_hinge_{index}', center_column_slide, pan_base, f'upper_leg_{index}', f'middle_leg_{index}', f'lower_leg_{index}', crank_arm | built ✓ |
| leg_stages | 2 stages | N | forked_anchor | `rec_0611_tripod_var_leg_stages_2_stages` from `rec_picturex_0611__tripod__002__png_e0249c65a8ef423e899cb4e318390384` | f'leg_hinge_{index}', f'leg_extension_{index}', f'upper_leg_{index}', f'lower_leg_{index}' | built ✓ |
| leg_stages | 3 stages | N | forked_anchor | `rec_0611_tripod_var_leg_stages_3_stages` from `rec_picturex_0611__tripod__003__png_e87f6574e5ab4934bd28f3b9649f8763` | f'hub_to_leg_{index}', f'leg_{index}' | built ✓ |
| leg_stages | 4 stages | N | forked_anchor | `rec_0611_tripod_var_leg_stages_4_stages` from `rec_picturex_0611__tripod__001__png_aa32bb13f66c426592ea992cfd8b0968` | f'leg_hinge_{index}', pan_base, f'upper_leg_{index}', f'middle_leg_{index}', f'lower_leg_{index}', _leg_direction, plate_support | built ✓ |
| head | ball head | ② | forked_anchor | `rec_0611_tripod_var_head_ball_head` from `rec_picturex_0611__tripod__002__png_e0249c65a8ef423e899cb4e318390384` | head_tilt, head_pan, tilt_head, head_shell, head_seat, tilt_ball, tilt ball remains seated, tilt ball is centered in its socket | built ✓ |
| head | fluid video head | ② | forked_anchor | `rec_0611_tripod_var_head_fluid_video_head` from `rec_picturex_0611__tripod__001__png_aa32bb13f66c426592ea992cfd8b0968` | head_tilt, head_pan, tilt_head | built ✓ |
| head | geared three-way head | ② | forked_anchor | `rec_0611_tripod_var_head_geared_three_way_head` from `rec_picturex_0611__tripod__002__png_e0249c65a8ef423e899cb4e318390384` | head_tilt, head_pan, tilt_head, head_shell, head_seat | built ✓ |
| head | gimbal head | ② | forked_anchor | `rec_0611_tripod_var_head_gimbal_head` from `rec_picturex_0611__tripod__001__png_aa32bb13f66c426592ea992cfd8b0968` | head_tilt, head_pan, tilt_head | built ✓ |
| center_column | geared crank column | ② | forked_anchor | `rec_0611_tripod_var_center_column_geared_crank_column` from `rec_picturex_0611__tripod__001__png_aa32bb13f66c426592ea992cfd8b0968` | column_crank_turn, center_column_slide, column_crank, center_column, center column remains inserted through the guide, center column remains centered in its guide, pan_handle_shaft, crank_pivot | built ✓ |
| center_column | reversible horizontal boom column | ② | forked_anchor | `rec_0611_tripod_var_center_column_reversible_horizontal_bo` from `rec_picturex_0611__tripod__001__png_aa32bb13f66c426592ea992cfd8b0968` | center_column_slide, center_column, column_crank_turn, column_crank, center column remains inserted through the guide, center column remains centered in its guide, column_tube, column_top_cap | built ✓ |
| center_column | leveling-bowl column | ② | forked_anchor | `rec_0611_tripod_var_center_column_leveling_bowl_column` from `rec_picturex_0611__tripod__001__png_aa32bb13f66c426592ea992cfd8b0968` | center_column_slide, center_column, column_crank_turn, column_crank, center column remains inserted through the guide, center column remains centered in its guide, column_tube, column_top_cap | built ✓ |
| leg_topology | center-braced legs | ① | forked_anchor | `rec_0611_tripod_var_leg_topology_center_braced_legs` from `rec_picturex_0611__tripod__001__png_aa32bb13f66c426592ea992cfd8b0968` | f'leg_hinge_{index}', center_column_slide, pan_base, f'upper_leg_{index}', f'middle_leg_{index}', f'lower_leg_{index}', center_column, _leg_direction | built ✓ |
| leg_topology | independent-angle legs | ① | forked_anchor | `rec_0611_tripod_var_leg_topology_independent_angle_legs` from `rec_picturex_0611__tripod__003__png_e87f6574e5ab4934bd28f3b9649f8763` | f'hub_to_leg_{index}', f'leg_{index}' | built ✓ |
| leg_topology | flexible segmented legs | ① | forked_anchor | `rec_0611_tripod_var_leg_topology_flexible_segmented_legs` from `rec_picturex_0611__tripod__003__png_e87f6574e5ab4934bd28f3b9649f8763` | f'hub_to_leg_{index}', f'leg_{index}' | built ✓ |
| foot_interface | retractable spikes | ① | forked_anchor | `rec_0611_tripod_var_foot_interface_retractable_spikes` from `rec_picturex_0611__tripod__002__png_e0249c65a8ef423e899cb4e318390384` | _foot_mesh, rubber_foot, foot_socket | built ✓ |
| foot_interface | suction feet | ① | forked_anchor | `rec_0611_tripod_var_foot_interface_suction_feet` from `rec_picturex_0611__tripod__003__png_e87f6574e5ab4934bd28f3b9649f8763` | rubber_foot, foot_ferrule | built ✓ |

## Multiplicity / Copy Logic

- count_param: leg_stages_count
- N samples: 1 stage, 2 stages, 3 stages, 4 stages
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
- neighbor categories (four-leg stand, monopod): excluded.
- failed or unfit candidates will be appended with one-line reasons after 2–3 attempts.
