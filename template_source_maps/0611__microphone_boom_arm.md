# 0611 / microphone_boom_arm — template source map

pattern: mixed
parents: `rec_picturex_0611__microphone_boom_arm__001__png_40f72213fda046498d8240b2f25fc372` (`pictureY/0611/microphone_boom_arm/001.png`), `rec_picturex_0611__microphone_boom_arm__002__png_57c6553d3c90426287d7d05649c6d222` (`pictureY/0611/microphone_boom_arm/002.png`), `rec_picturex_0611__microphone_boom_arm__003__png_0696f40f5c9b438ca77c7ab59b5bf289` (`pictureY/0611/microphone_boom_arm/003.png`), `rec_picturex_0611__microphone_boom_arm__004__png_727d36bd44994666804b8c0ed0c101a8` (`pictureY/0611/microphone_boom_arm/004.png`)
canonical_baselines: none planned; add only if a selected origin fails the §11 readability contract during fork review.
underfilled_reason: none at P0; adaptive coverage follows the approved per-category slot vocabulary.

## Subcategory Contract

- core_identity: adjustable microphone boom arm retaining supported multi-axis positioning and a microphone interface
- must_keep: defining use, picture identity, visible support interfaces, and at least one real non-fixed joint.
- must_not_become: camera tripod, fixed microphone stand
- image_evidence: pictureY/0611/microphone_boom_arm/001.png, pictureY/0611/microphone_boom_arm/002.png, pictureY/0611/microphone_boom_arm/003.png, pictureY/0611/microphone_boom_arm/004.png
- parent_evidence: rec_picturex_0611__microphone_boom_arm__001__png_40f72213fda046498d8240b2f25fc372, rec_picturex_0611__microphone_boom_arm__002__png_57c6553d3c90426287d7d05649c6d222, rec_picturex_0611__microphone_boom_arm__003__png_0696f40f5c9b438ca77c7ab59b5bf289, rec_picturex_0611__microphone_boom_arm__004__png_727d36bd44994666804b8c0ed0c101a8

## Slot Candidates

| slot | candidate | diversity_axis | source_type | record/evidence | key parts/joints/helpers | status |
|---|---|---|---|---|---|---|
| origin_design | folded_black_microphone_boom_arm | ①/②/③ observed | origin_anchor | `rec_picturex_0611__microphone_boom_arm__001__png_40f72213fda046498d8240b2f25fc372` / `pictureY/0611/microphone_boom_arm/001.png` | desk_clamp, clamp_screw, swivel_post, lower_arm, upper_arm, shoulder_knob, elbow_knob, wrist_knob, mount_yoke, threaded_adapter, clamp_travel, base_yaw, shoulder_pitch, shoulder_knob_spin | built ✓ |
| origin_design | microphone_boom_arm_002 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__microphone_boom_arm__002__png_57c6553d3c90426287d7d05649c6d222` / `pictureY/0611/microphone_boom_arm/002.png` | desk_base, swivel, lower_arm, upper_arm, microphone_mount, shoulder_knob, elbow_knob, mount_knob, threaded_adapter, base_yaw, shoulder_pitch, elbow_pitch, mount_pitch, shoulder_knob_spin | built ✓ |
| origin_design | microphone_boom_arm_003 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__microphone_boom_arm__003__png_0696f40f5c9b438ca77c7ab59b5bf289` / `pictureY/0611/microphone_boom_arm/003.png` | desk_clamp, clamp_screw, base_swivel, lower_link, upper_link, mic_mount, microphone, clamp_adjust, base_yaw, lower_pitch, elbow_pitch, mount_pitch, microphone_tilt, _cylinder_y | built ✓ |
| origin_design | by_ba20_microphone_boom_arm | ①/②/③ observed | origin_anchor | `rec_picturex_0611__microphone_boom_arm__004__png_727d36bd44994666804b8c0ed0c101a8` / `pictureY/0611/microphone_boom_arm/004.png` | desk_clamp, clamp_screw, base_swivel, lower_arm, upper_arm, terminal, threaded_adapter, shock_mount, microphone, pop_filter, clamp_screw_slide, clamp_to_swivel, swivel_to_lower_arm, lower_to_upper_arm | built ✓ |
| arm_topology | low-profile horizontal boom | ① | forked_anchor | `rec_0611_microphone_boom_arm_var_arm_topology_low_profile_horizontal_bo` from `rec_picturex_0611__microphone_boom_arm__001__png_40f72213fda046498d8240b2f25fc372` | upper_arm, lower_arm, folded_black_microphone_boom_arm, upper_arm_shell, upper_arm_face, lower_arm_shell, lower_arm_face, _capsule_profile | built ✓ |
| arm_topology | tubular cantilever arm | ① | forked_anchor | `rec_0611_microphone_boom_arm_var_arm_topology_tubular_cantilever_arm` from `rec_picturex_0611__microphone_boom_arm__002__png_57c6553d3c90426287d7d05649c6d222` | upper_arm, lower_arm, microphone_boom_arm_002 | built ✓ |
| arm_topology | three-link articulated arm | ① | forked_anchor | `rec_0611_microphone_boom_arm_var_arm_topology_three_link_articulated_ar` from `rec_picturex_0611__microphone_boom_arm__003__png_0696f40f5c9b438ca77c7ab59b5bf289` | upper arm is captured at elbow, microphone_boom_arm_003, lower arm is captured in base fork, upper_link, lower_link, handle_end_1, handle_end_0, upper_jaw | built ✓ |
| arm_topology | parallelogram scissor arm | ① | forked_anchor | `rec_0611_microphone_boom_arm_var_arm_topology_parallelogram_scissor_arm` from `rec_picturex_0611__microphone_boom_arm__004__png_727d36bd44994666804b8c0ed0c101a8` | upper_arm_to_terminal, swivel_to_lower_arm, lower_to_upper_arm, upper_arm, lower_arm, _add_arm_rails, by_ba20_microphone_boom_arm, clamp_handle | built ✓ |
| arm_topology | wall-swing boom | ① | forked_anchor | `rec_0611_microphone_boom_arm_var_arm_topology_wall_swing_boom` from `rec_picturex_0611__microphone_boom_arm__001__png_40f72213fda046498d8240b2f25fc372` | upper_arm, lower_arm, folded_black_microphone_boom_arm, upper_arm_shell, upper_arm_face, lower_arm_shell, lower_arm_face, top_jaw | built ✓ |
| compensation | internal gas spring | ② | forked_anchor | `rec_0611_microphone_boom_arm_var_compensation_internal_gas_spring` from `rec_picturex_0611__microphone_boom_arm__002__png_57c6553d3c90426287d7d05649c6d222` | _spring_geometry, tension_spring_1, tension_spring_0 | built ✓ |
| compensation | cable counterweight | ② | forked_anchor | `rec_0611_microphone_boom_arm_var_compensation_cable_counterweight` from `rec_picturex_0611__microphone_boom_arm__003__png_0696f40f5c9b438ca77c7ab59b5bf289` | routed_cable, cable_anchor | built ✓ |
| compensation | torsion-spring hinge | ② | forked_anchor | `rec_0611_microphone_boom_arm_var_compensation_torsion_spring_hinge` from `rec_picturex_0611__microphone_boom_arm__002__png_57c6553d3c90426287d7d05649c6d222` | _spring_geometry, tension_spring_1, tension_spring_0, depth-staggered elbow members retain local hinge engagement | built ✓ |
| compensation | constant-force spring | ② | forked_anchor | `rec_0611_microphone_boom_arm_var_compensation_constant_force_spring` from `rec_picturex_0611__microphone_boom_arm__004__png_727d36bd44994666804b8c0ed0c101a8` | _add_tension_spring | built ✓ |
| base_mount | grommet mount | ② | forked_anchor | `rec_0611_microphone_boom_arm_var_base_mount_grommet_mount` from `rec_picturex_0611__microphone_boom_arm__002__png_57c6553d3c90426287d7d05649c6d222` | base_yaw, desk_base, _make_desk_base, base_shell, base_pad, base_foot, shoulder_support, mount_pitch | built ✓ |
| base_mount | wall plate | ② | forked_anchor | `rec_0611_microphone_boom_arm_var_base_mount_wall_plate` from `rec_picturex_0611__microphone_boom_arm__002__png_57c6553d3c90426287d7d05649c6d222` | base_yaw, desk_base, _make_desk_base, base_shell, base_pad, base_foot, shoulder_support, mount_pitch | built ✓ |
| base_mount | weighted desktop base | ② | forked_anchor | `rec_0611_microphone_boom_arm_var_base_mount_weighted_desktop_base` from `rec_picturex_0611__microphone_boom_arm__002__png_57c6553d3c90426287d7d05649c6d222` | base_yaw, desk_base, _make_desk_base, base_shell, base_pad, base_foot, shoulder_support, mount_pitch | built ✓ |
| base_mount | floor-stand base | ② | forked_anchor | `rec_0611_microphone_boom_arm_var_base_mount_floor_stand_base` from `rec_picturex_0611__microphone_boom_arm__002__png_57c6553d3c90426287d7d05649c6d222` | base_yaw, desk_base, _make_desk_base, shoulder_support, base_shell, base_pad, base_foot, mount_pitch | built ✓ |
| segment_count | single boom segment | N | forked_anchor | `rec_0611_microphone_boom_arm_var_segment_count_single_boom_segment` from `rec_picturex_0611__microphone_boom_arm__001__png_40f72213fda046498d8240b2f25fc372` | folded_black_microphone_boom_arm | built ✓ |
| segment_count | 3 boom segments | N | forked_anchor | `rec_0611_microphone_boom_arm_var_segment_count_3_boom_segments` from `rec_picturex_0611__microphone_boom_arm__003__png_0696f40f5c9b438ca77c7ab59b5bf289` | microphone_boom_arm_003 | built ✓ |

## Multiplicity / Copy Logic

- count_param: segment_count_count
- N samples: single boom segment, 3 boom segments
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
- neighbor categories (camera tripod, fixed microphone stand): excluded.
- failed or unfit candidates will be appended with one-line reasons after 2–3 attempts.
