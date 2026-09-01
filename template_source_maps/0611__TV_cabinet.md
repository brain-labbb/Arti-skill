# 0611 / TV_cabinet — template source map

pattern: mixed
parents: `rec_picturex_0611__tv_cabinet__007__png_5218617e80f34c2a80eaf361149c8e96` (`pictureY/0611/TV_cabinet/007.png`), `rec_picturex_0611__tv_cabinet__001__png_0de7f39b17dc4e05813ce61aeb47317c` (`pictureY/0611/TV_cabinet/001.png`), `rec_picturex_0611__tv_cabinet__002__png_9fdc892ea06942848430149a989cd758` (`pictureY/0611/TV_cabinet/002.png`), `rec_picturex_0611__tv_cabinet__003__png_1a4f362e3cda452089eefff24596931f` (`pictureY/0611/TV_cabinet/003.png`), `rec_picturex_0611__tv_cabinet__004__png_cf3db5cf4e0d441f939a096a0f07945b` (`pictureY/0611/TV_cabinet/004.png`), `rec_picturex_0611__tv_cabinet__005__png_48cdc4d5827749e5983f2671d3ffc3e3` (`pictureY/0611/TV_cabinet/005.png`), `rec_picturex_0611__tv_cabinet__006__png_a8744cd25a9543ea9b957bedff6f12fc` (`pictureY/0611/TV_cabinet/006.png`)
canonical_baselines: none planned; add only if a selected origin fails the §11 readability contract during fork review.
underfilled_reason: none at P0; adaptive coverage follows the approved per-category slot vocabulary.

## Subcategory Contract

- core_identity: storage TV cabinet with operable fronts or storage modules
- must_keep: defining use, picture identity, visible support interfaces, and at least one real non-fixed joint.
- must_not_become: plain table, open rack without cabinet function
- image_evidence: pictureY/0611/TV_cabinet/007.png, pictureY/0611/TV_cabinet/001.png, pictureY/0611/TV_cabinet/002.png, pictureY/0611/TV_cabinet/003.png, pictureY/0611/TV_cabinet/004.png, pictureY/0611/TV_cabinet/005.png, pictureY/0611/TV_cabinet/006.png
- parent_evidence: rec_picturex_0611__tv_cabinet__007__png_5218617e80f34c2a80eaf361149c8e96, rec_picturex_0611__tv_cabinet__001__png_0de7f39b17dc4e05813ce61aeb47317c, rec_picturex_0611__tv_cabinet__002__png_9fdc892ea06942848430149a989cd758, rec_picturex_0611__tv_cabinet__003__png_1a4f362e3cda452089eefff24596931f, rec_picturex_0611__tv_cabinet__004__png_cf3db5cf4e0d441f939a096a0f07945b, rec_picturex_0611__tv_cabinet__005__png_48cdc4d5827749e5983f2671d3ffc3e3, rec_picturex_0611__tv_cabinet__006__png_a8744cd25a9543ea9b957bedff6f12fc

## Slot Candidates

| slot | candidate | diversity_axis | source_type | record/evidence | key parts/joints/helpers | status |
|---|---|---|---|---|---|---|
| origin_design | tv_cabinet_007 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__tv_cabinet__007__png_5218617e80f34c2a80eaf361149c8e96` / `pictureY/0611/TV_cabinet/007.png` | carcass, dynamic_indexed_name, dynamic_indexed_name, _routed_front, _top_slab, _bun_foot_geometry, _bail_handle_geometry | built ✓ |
| origin_design | walnut_tv_cabinet_001 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__tv_cabinet__001__png_0de7f39b17dc4e05813ce61aeb47317c` / `pictureY/0611/TV_cabinet/001.png` | carcass, door_0, door_1, carcass_to_door_0, carcass_to_door_1, _rounded_box, _tapered_foot, _add_box, _build_door_visuals | built ✓ |
| origin_design | oak_four_door_tv_cabinet | ①/②/③ observed | origin_anchor | `rec_picturex_0611__tv_cabinet__002__png_9fdc892ea06942848430149a989cd758` / `pictureY/0611/TV_cabinet/002.png` | carcass, dynamic_indexed_name, dynamic_indexed_name, _add_door_grain, _add_concealed_hinges | built ✓ |
| origin_design | oak_tv_cabinet_003 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__tv_cabinet__003__png_1a4f362e3cda452089eefff24596931f` / `pictureY/0611/TV_cabinet/003.png` | dynamic_indexed_name, carcass, dynamic_indexed_name, _rounded_box, _tapered_leg, _add_pull, _add_front_grain, _add_drawer, _add_door | built ✓ |
| origin_design | walnut_tv_cabinet_004 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__tv_cabinet__004__png_cf3db5cf4e0d441f939a096a0f07945b` / `pictureY/0611/TV_cabinet/004.png` | dynamic_indexed_name, carcass, dynamic_indexed_name, _add_grain, _add_round_pull, _add_door, _add_drawer | built ✓ |
| origin_design | tv_cabinet_005 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__tv_cabinet__005__png_48cdc4d5827749e5983f2671d3ffc3e3` / `pictureY/0611/TV_cabinet/005.png` | cabinet, dynamic_indexed_name, dynamic_indexed_name, _rounded_box_mesh, _hinge_tube_mesh | built ✓ |
| origin_design | cherry_tv_cabinet_006 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__tv_cabinet__006__png_a8744cd25a9543ea9b957bedff6f12fc` / `pictureY/0611/TV_cabinet/006.png` | cabinet | built ✓ |
| closure | paired four-leaf bi-fold center doors | ② | forked_anchor | `rec_0611_tv_cabinet_var_closure_paired_bifold_doors` from `rec_picturex_0611__tv_cabinet__007__png_5218617e80f34c2a80eaf361149c8e96` | left_outer_door, left_inner_door, right_outer_door, right_inner_door; carcass-to-outer and outer-to-inner hinges | built ✓ |
| closure | drop-front media flap | ② | forked_anchor | `rec_0611_tv_cabinet_var_closure_drop_front_media_flap` from `rec_picturex_0611__tv_cabinet__001__png_0de7f39b17dc4e05813ce61aeb47317c` | carcass_to_door_1, carcass_to_door_0, door_1, door_0, _build_door_visuals, door_panel, closed doors retain narrow center reveal | planned |
| closure | tambour | ② | forked_anchor | `rec_0611_tv_cabinet_var_closure_tambour` from `rec_picturex_0611__tv_cabinet__002__png_9fdc892ea06942848430149a989cd758` | _add_door_grain, oak_four_door_tv_cabinet, door_panel, back_panel | planned |
| storage_count | 3 drawers | N | forked_anchor | `rec_0611_tv_cabinet_var_storage_count_3_drawers` from `rec_picturex_0611__tv_cabinet__003__png_1a4f362e3cda452089eefff24596931f` | _add_drawer, two photographed drawer fronts retain their narrow reveal, drawer_partition, drawer_bottom, drawer_back, drawer bank has equal aligned front widths | planned |
| storage_count | 3 open cubbies | N | forked_anchor | `rec_0611_tv_cabinet_var_storage_count_3_open_cubbies` from `rec_picturex_0611__tv_cabinet__004__png_cf3db5cf4e0d441f939a096a0f07945b` | _add_grain, _add_round_pull, _add_door, _add_drawer, dynamic_indexed_name | planned |
| storage_count | 3 doors | N | forked_anchor | `rec_0611_tv_cabinet_var_storage_count_3_doors` from `rec_picturex_0611__tv_cabinet__005__png_48cdc4d5827749e5983f2671d3ffc3e3` | door_panel, door_face, base_panel, back_panel | planned |
| support | floating wall mount | ① | forked_anchor | `rec_0611_tv_cabinet_var_support_floating_wall_mount` from `rec_picturex_0611__tv_cabinet__006__png_a8744cd25a9543ea9b957bedff6f12fc` | cabinet | planned |
| support | solid plinth | ① | forked_anchor | `rec_0611_tv_cabinet_var_support_solid_plinth` from `rec_picturex_0611__tv_cabinet__005__png_48cdc4d5827749e5983f2671d3ffc3e3` | base_panel, shelf_slab, shelf_light, recessed_plinth | planned |
| support | paired powder-coated steel sled end frames | ① | forked_anchor | `rec_0611_tv_cabinet_var_support_powder_coated_sled_base` from `rec_picturex_0611__tv_cabinet__003__png_1a4f362e3cda452089eefff24596931f` | sled_frame_0, sled_frame_1, bottom_panel, preserved drawer/door joints | built ✓ |
| body_form | corner | ③ | forked_anchor | `rec_0611_tv_cabinet_var_body_form_corner` from `rec_picturex_0611__tv_cabinet__007__png_5218617e80f34c2a80eaf361149c8e96` | carcass | planned |
| body_form | bowed front | ③ | forked_anchor | `rec_0611_tv_cabinet_var_body_form_bowed_front` from `rec_picturex_0611__tv_cabinet__002__png_9fdc892ea06942848430149a989cd758` | carcass, front_reveal | planned |

## Multiplicity / Copy Logic

- count_param: storage_count_count
- N samples: 3 drawers, 3 open cubbies, 3 doors
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
- neighbor categories (plain table, open rack without cabinet function): excluded.
- overlapping sliders: rejected by user; `rec_0611_tv_cabinet_var_closure_overlapping_sliders` and its materialization were deleted on 2026-07-11.
- swivel TV platform: rejected by user; `rec_0611_tv_cabinet_var_top_motion_swivel_tv_platform` and its materialization were deleted on 2026-07-11.
- failed or unfit candidates will be appended with one-line reasons after 2–3 attempts.
