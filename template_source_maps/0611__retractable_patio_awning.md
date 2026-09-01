# 0611 / retractable_patio_awning — template source map

pattern: mixed
parents: `rec_picturex_0611__retractable_patio_awning__001__png_3468bd36482f42c2941f222b1e69c057` (`pictureY/0611/retractable_patio_awning/001.png`), `rec_picturex_0611__retractable_patio_awning__002__png_50824a69ddb44d45ae217dae193e969a` (`pictureY/0611/retractable_patio_awning/002.png`)
canonical_baselines: none planned; add only if a selected origin fails the §11 readability contract during fork review.
underfilled_reason: none at P0; adaptive coverage follows the approved per-category slot vocabulary.

## Subcategory Contract

- core_identity: retractable outdoor canopy retaining supported fabric and a real extension mechanism
- must_keep: defining use, picture identity, visible support interfaces, and at least one real non-fixed joint.
- must_not_become: fixed pergola roof, ordinary patio umbrella
- image_evidence: pictureY/0611/retractable_patio_awning/001.png, pictureY/0611/retractable_patio_awning/002.png
- parent_evidence: rec_picturex_0611__retractable_patio_awning__001__png_3468bd36482f42c2941f222b1e69c057, rec_picturex_0611__retractable_patio_awning__002__png_50824a69ddb44d45ae217dae193e969a

## Slot Candidates

| slot | candidate | diversity_axis | source_type | record/evidence | key parts/joints/helpers | status |
|---|---|---|---|---|---|---|
| origin_design | retractable_patio_awning_001 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__retractable_patio_awning__001__png_3468bd36482f42c2941f222b1e69c057` / `pictureY/0611/retractable_patio_awning/001.png` | cassette, fabric_roller, front_bar, crank_handle, f'upper_arm_{index}', f'forearm_{index}', roller_rotation, front_bar_pitch, crank_rotation, f'shoulder_hinge_{index}', f'elbow_hinge_{index}', _fabric_geometry, _add_arm_segment | built ✓ |
| origin_design | freestanding_cantilever_patio_umbrella | ①/②/③ observed | origin_anchor | `rec_picturex_0611__retractable_patio_awning__002__png_50824a69ddb44d45ae217dae193e969a` / `pictureY/0611/retractable_patio_awning/002.png` | base_post, cantilever_arm, canopy, crank, arm_pivot, canopy_tilt, crank_rotation, _canopy_shell, _beam_pose, _add_beam | built ✓ |
| support_topology | wall cassette awning | ① | forked_anchor | `rec_0611_retractable_patio_awning_var_support_topology_wall_cassette_awning` from `rec_picturex_0611__retractable_patio_awning__001__png_3468bd36482f42c2941f222b1e69c057` | cassette, front_bar_body, rolled fabric stays inside cassette profile, retractable_patio_awning_001, front bar supports nearly the full fabric width, cassette_top, cassette_lip, cassette_back | built ✓ |
| support_topology | freestanding dual-post awning | ① | forked_anchor | `rec_0611_retractable_patio_awning_var_support_topology_freestanding_dual_pos` from `rec_picturex_0611__retractable_patio_awning__002__png_50824a69ddb44d45ae217dae193e969a` | base_post, support_post, base_slab, mast_mount_plate, freestanding_cantilever_patio_umbrella | built ✓ |
| support_topology | vertical drop awning | ① | forked_anchor | `rec_0611_retractable_patio_awning_var_support_topology_vertical_drop_awning` from `rec_picturex_0611__retractable_patio_awning__002__png_50824a69ddb44d45ae217dae193e969a` | base_post, support_post, base_slab, mast_mount_plate, hub_drop | built ✓ |
| support_topology | roof-mounted awning | ① | forked_anchor | `rec_0611_retractable_patio_awning_var_support_topology_roof_mounted_awning` from `rec_picturex_0611__retractable_patio_awning__002__png_50824a69ddb44d45ae217dae193e969a` | base_post, support_post, base_slab, mast_mount_plate | built ✓ |
| extension_mechanism | folding lateral arms | ② | forked_anchor | `rec_0611_retractable_patio_awning_var_extension_mechanism_folding_lateral_ar` from `rec_picturex_0611__retractable_patio_awning__001__png_3468bd36482f42c2941f222b1e69c057` | f'upper_arm_{index}', _add_arm_segment, f'shoulder_hinge_{index}', f'elbow_hinge_{index}', crank_handle, arm_beam, tension_link | built ✓ |
| extension_mechanism | telescoping arms | ② | forked_anchor | `rec_0611_retractable_patio_awning_var_extension_mechanism_telescoping_arms` from `rec_picturex_0611__retractable_patio_awning__001__png_3468bd36482f42c2941f222b1e69c057` | f'upper_arm_{index}', _add_arm_segment, f'shoulder_hinge_{index}', f'elbow_hinge_{index}', crank_handle, arm_beam, tension_link | built ✓ |
| extension_mechanism | scissor arms | ② | forked_anchor | `rec_0611_retractable_patio_awning_var_extension_mechanism_scissor_arms` from `rec_picturex_0611__retractable_patio_awning__001__png_3468bd36482f42c2941f222b1e69c057` | f'upper_arm_{index}', _add_arm_segment, f'shoulder_hinge_{index}', f'elbow_hinge_{index}', crank_handle, arm_beam, tension_link | built ✓ |
| extension_mechanism | guided side rails | ② | forked_anchor | `rec_0611_retractable_patio_awning_var_extension_mechanism_guided_side_rails` from `rec_picturex_0611__retractable_patio_awning__001__png_3468bd36482f42c2941f222b1e69c057` | f'shoulder_hinge_{index}', f'elbow_hinge_{index}', f'upper_arm_{index}', _add_arm_segment, tension_link, arm_beam | built ✓ |
| drive | chain-loop drive | ② | forked_anchor | `rec_0611_retractable_patio_awning_var_drive_chain_loop_drive` from `rec_picturex_0611__retractable_patio_awning__001__png_3468bd36482f42c2941f222b1e69c057` | manual crank seats in end drive, drive_housing, crank_rotation, f'upper_arm_{index}', crank_handle, _add_arm_segment, crank_spindle, crank_grip | built ✓ |
| drive | spring-rewind roller | ② | forked_anchor | `rec_0611_retractable_patio_awning_var_drive_spring_rewind_roller` from `rec_picturex_0611__retractable_patio_awning__001__png_3468bd36482f42c2941f222b1e69c057` | roller_rotation, fabric_roller, manual crank seats in end drive, drive_housing, roller_tube, roller_seam, roller_axle, crank_rotation | built ✓ |
| drive | gearbox crank | ② | forked_anchor | `rec_0611_retractable_patio_awning_var_drive_gearbox_crank` from `rec_picturex_0611__retractable_patio_awning__001__png_3468bd36482f42c2941f222b1e69c057` | manual crank seats in end drive, drive_housing, crank_rotation, f'upper_arm_{index}', crank_handle, _add_arm_segment, crank_spindle, crank_grip | built ✓ |
| arm_count | 2 support arms | N | forked_anchor | `rec_0611_retractable_patio_awning_var_arm_count_2_support_arms` from `rec_picturex_0611__retractable_patio_awning__002__png_50824a69ddb44d45ae217dae193e969a` | arm_pivot, cantilever_arm, base_post, support_post, crank_arm, base_slab, arm_pivot_pin, arm_bushing | built ✓ |
| arm_count | 3 support arms | N | forked_anchor | `rec_0611_retractable_patio_awning_var_arm_count_3_support_arms` from `rec_picturex_0611__retractable_patio_awning__002__png_50824a69ddb44d45ae217dae193e969a` | arm_pivot, cantilever_arm, base_post, support_post, crank_arm, base_slab, arm_pivot_pin, arm_bushing | built ✓ |
| arm_count | 4 support arms | N | forked_anchor | `rec_0611_retractable_patio_awning_var_arm_count_4_support_arms` from `rec_picturex_0611__retractable_patio_awning__002__png_50824a69ddb44d45ae217dae193e969a` | arm_pivot, cantilever_arm, base_post, support_post, crank_arm, base_slab, arm_pivot_pin, arm_bushing | built ✓ |

## Multiplicity / Copy Logic

- count_param: arm_count_count
- N samples: 2 support arms, 3 support arms, 4 support arms
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
- neighbor categories (fixed pergola roof, ordinary patio umbrella): excluded.
- failed or unfit candidates will be appended with one-line reasons after 2–3 attempts.
