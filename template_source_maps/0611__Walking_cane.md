# 0611 / Walking_cane — template source map

pattern: mixed
parents: `rec_walking_cane__walking_cane__001_png_edc0b703c97d47f89cc9d6d0804dc3c7` (`pictureY/0611/Walking_cane/001.png`), `rec_walking_cane__walking_cane__002_png_94c2f346438b418ba9729696453aa20e` (`pictureY/0611/Walking_cane/002.png`), `rec_walking_cane__walking_cane__003_png_55ed776d6bbf4b6ebff9f55943ab9b18` (`pictureY/0611/Walking_cane/003.png`)
canonical_baselines: none planned; add only if a selected origin fails the §11 readability contract during fork review.
underfilled_reason: none at P0; adaptive coverage follows the approved per-category slot vocabulary.

## Subcategory Contract

- core_identity: real-world Walking cane retaining its defining use and articulation
- must_keep: defining use, picture identity, visible support interfaces, and at least one real non-fixed joint.
- must_not_become: neighbor category, decorative static prop
- image_evidence: pictureY/0611/Walking_cane/001.png, pictureY/0611/Walking_cane/002.png, pictureY/0611/Walking_cane/003.png
- parent_evidence: rec_walking_cane__walking_cane__001_png_edc0b703c97d47f89cc9d6d0804dc3c7, rec_walking_cane__walking_cane__002_png_94c2f346438b418ba9729696453aa20e, rec_walking_cane__walking_cane__003_png_55ed776d6bbf4b6ebff9f55943ab9b18

## Slot Candidates

| slot | candidate | diversity_axis | source_type | record/evidence | key parts/joints/helpers | status |
|---|---|---|---|---|---|---|
| origin_design | green_folding_quad_cane | ①/②/③ observed | origin_anchor | `rec_walking_cane__walking_cane__001_png_edc0b703c97d47f89cc9d6d0804dc3c7` / `pictureY/0611/Walking_cane/001.png` | handle_upper, middle_section, lower_section, upper_slide, lower_fold, dynamic_indexed_name, _hollow_ring, _strap_loop | built ✓ |
| origin_design | black_adjustable_walking_cane | ①/②/③ observed | origin_anchor | `rec_walking_cane__walking_cane__002_png_94c2f346438b418ba9729696453aa20e` / `pictureY/0611/Walking_cane/002.png` | upper_shaft, handle, wrist_strap, lower_shaft, adjustment_button, rubber_tip, alternate_base, upper_to_handle, handle_to_strap, upper_to_lower, lower_to_button, lower_to_tip, lower_to_alt_base, _tube_shell | built ✓ |
| origin_design | purple_foldable_walking_cane | ①/②/③ observed | origin_anchor | `rec_walking_cane__walking_cane__003_png_55ed776d6bbf4b6ebff9f55943ab9b18` / `pictureY/0611/Walking_cane/003.png` | handle_segment, shaft_1, shaft_2, tip_segment, fold_joint_0, fold_joint_1, fold_joint_2, _cylinder_x_origin, _add_tube_segment | built ✓ |
| handle_form | crook | ③ | forked_anchor | `rec_0611_walking_cane_var_handle_form_crook` from `rec_walking_cane__walking_cane__001_png_edc0b703c97d47f89cc9d6d0804dc3c7` | handle_upper, t_handle, handle_neck, quad_arm_y, quad_arm_x | planned |
| handle_form | derby offset | ③ | forked_anchor | `rec_0611_walking_cane_var_handle_form_derby_offset` from `rec_walking_cane__walking_cane__002_png_94c2f346438b418ba9729696453aa20e` | upper_to_handle, handle_to_strap, handle, wrist strap captured at handle eyelet, white_handle_bracket, vertical_handle_post, handle_screw, accessory_clip_arm | planned |
| ground_interface | tripod base | ① | forked_anchor | `rec_0611_walking_cane_var_ground_interface_tripod_base` from `rec_walking_cane__walking_cane__003_png_55ed776d6bbf4b6ebff9f55943ab9b18` | rubber_foot_pad | planned |
| ground_interface | quad base | ① | forked_anchor | `rec_0611_walking_cane_var_ground_interface_quad_base` from `rec_walking_cane__walking_cane__002_png_94c2f346438b418ba9729696453aa20e` | lower_to_alt_base, alternate_base, base_socket, alternate base accessory is clipped to shaft, rubber_tip_body | planned |
| shaft_count | 2 telescoping stages | N | forked_anchor | `rec_0611_walking_cane_var_shaft_count_2_telescoping_stages` from `rec_walking_cane__walking_cane__001_png_edc0b703c97d47f89cc9d6d0804dc3c7` | upper_shaft, telescoping_sleeve, quad_arm_y, quad_arm_x, extended telescoping section retains insertion | planned |
| shaft_count | 4 folding segments | N | forked_anchor | `rec_0611_walking_cane_var_shaft_count_4_folding_segments` from `rec_walking_cane__walking_cane__001_png_edc0b703c97d47f89cc9d6d0804dc3c7` | upper_shaft, quad_arm_y, quad_arm_x, green_folding_quad_cane | planned |
| secondary_motion | folding seat | ② | forked_anchor | `rec_0611_walking_cane_var_secondary_motion_folding_seat` from `rec_walking_cane__walking_cane__001_png_edc0b703c97d47f89cc9d6d0804dc3c7` | seat_carrier, seat_deck, seat_deploy_hinge, seat_support_leg_left/right, seat_leg_hinge_left/right, seat_brace_left/right, seat_foot_left/right; preserves upper_slide and lower_fold | built ✓ (`gpt-5.6-sol`, clean foreground compile) |

## Multiplicity / Copy Logic

- count_param: shaft_count_count
- N samples: 2 telescoping stages, 4 folding segments
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
- neighbor categories (neighbor category, decorative static prop): excluded.
- failed or unfit candidates will be appended with one-line reasons after 2–3 attempts.
