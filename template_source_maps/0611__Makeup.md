# 0611 / Makeup — template source map

pattern: mixed
parents: `rec_picturex_0611__makeup__001__png_25a74d42e2cc47b8be52d1f0dd9e0807` (`pictureY/0611/Makeup/001.png`), `rec_use-the-attached-reference-image-as-the-primary-_20260710_093913_953497_b3fcb0ac` (`pictureY/0611/Makeup/002.png`)
canonical_baselines: none planned; add only if a selected origin fails the §11 readability contract during fork review.
underfilled_reason: none at P0; adaptive coverage follows the approved per-category slot vocabulary.

## Subcategory Contract

- core_identity: articulated makeup compact or palette retaining powder storage
- must_keep: defining use, picture identity, visible support interfaces, and at least one real non-fixed joint.
- must_not_become: jewelry box, empty cosmetic case
- image_evidence: pictureY/0611/Makeup/001.png, pictureY/0611/Makeup/002.png
- parent_evidence: rec_picturex_0611__makeup__001__png_25a74d42e2cc47b8be52d1f0dd9e0807, rec_use-the-attached-reference-image-as-the-primary-_20260710_093913_953497_b3fcb0ac

## Slot Candidates

| slot | candidate | diversity_axis | source_type | record/evidence | key parts/joints/helpers | status |
|---|---|---|---|---|---|---|
| origin_design | black_makeup_case_001 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__makeup__001__png_25a74d42e2cc47b8be52d1f0dd9e0807` / `pictureY/0611/Makeup/001.png` | case_base, mirror_lid, f'palette_{index}', base_to_lid, f'base_to_palette_{index}', _rounded_case_shell, _lid_frame, _palette_frame, _add_pan | built ✓ |
| origin_design | childrens_travel_makeup_vanity | ①/②/③ observed | origin_anchor | `rec_use-the-attached-reference-image-as-the-primary-_20260710_093913_953497_b3fcb0ac` / `pictureY/0611/Makeup/002.png` | case_body, lid, front_organizer, f'side_tray_{side_index}', case_to_lid, case_to_front_organizer, case_to_side_tray_0, case_to_side_tray_1, _rounded_box, _hollow_tray, _tube_x, _pocketed_plate | built ✓ |
| palette_topology | accordion tier carrier | ② | forked_anchor | `rec_0611_makeup_var_palette_topology_accordion_tier_carrie` from `rec_picturex_0611__makeup__001__png_25a74d42e2cc47b8be52d1f0dd9e0807` | f'base_to_palette_{index}', f'palette_{index}', _palette_frame, palette_frame, palette 1 rests on its corner pivot post, palette 1 pans remain inside the rim, palette 0 rests on its corner pivot post, palette 0 pans remain inside the rim | built ✓ |
| palette_topology | fan-out wing carrier | ② | forked_anchor | `rec_0611_makeup_var_palette_topology_fan_out_wing_carrier` from `rec_use-the-attached-reference-image-as-the-primary-_20260710_093913_953497_b3fcb0ac` | case_to_side_tray_1, case_to_side_tray_0, f'side_tray_{side_index}', _hollow_tray, side_tray_shell, front_tray_shell, central_palette_insert | built ✓ |
| palette_topology | pull-out drawer carrier | ② | forked_anchor | `rec_0611_makeup_var_palette_topology_pull_out_drawer_carri` from `rec_use-the-attached-reference-image-as-the-primary-_20260710_093913_953497_b3fcb0ac` | case_to_side_tray_1, case_to_side_tray_0, f'side_tray_{side_index}', _hollow_tray, side_tray_shell, front_tray_shell, side_slide_tail, front_slide_tail | built ✓ |
| palette_topology | flip-over double-sided palette leaf | ② | forked_anchor | `rec_0611_makeup_var_palette_topology_flip_over_double_sided_leaf` from `rec_use-the-attached-reference-image-as-the-primary-_20260710_093913_953497_b3fcb0ac` | flip_palette_leaf, case_to_flip_palette_leaf, flip_leaf_frame, palette_axle, upper_face_pan_*, reverse_face_pan_*, palette_pivot_socket_* | built ✓ |
| case_form | round compact | ③ | forked_anchor | `rec_0611_makeup_var_case_form_round_compact` from `rec_picturex_0611__makeup__001__png_25a74d42e2cc47b8be52d1f0dd9e0807` | case_base, _rounded_case_shell, case_shell, black_round_compact_002, _fan_sector, _palette_frame, _lid_frame, palette_frame, f'round_shadow_pan_{pan_index}' | built ✓; corrected curved fan trays and compact rear hinge |
| case_form | train-case tower | ③ | forked_anchor | `rec_0611_makeup_var_case_form_train_case_tower` from `rec_picturex_0611__makeup__001__png_25a74d42e2cc47b8be52d1f0dd9e0807` | case_base, _rounded_case_shell, lid hinge is supported by the rear case knuckles, case_shell, black_makeup_case_001, _palette_frame, _lid_frame, palette_frame | built ✓ |
| case_form | book-style folio | ③ | forked_anchor | `rec_0611_makeup_var_case_form_book_style_folio` from `rec_picturex_0611__makeup__001__png_25a74d42e2cc47b8be52d1f0dd9e0807` | case_base, _rounded_case_shell, lid hinge is supported by the rear case knuckles, case_shell, black_makeup_case_001, _palette_frame, _lid_frame, palette_frame | built ✓ |
| case_form | cylindrical vanity case | ③ | forked_anchor | `rec_0611_makeup_var_case_form_cylindrical_vanity_case` from `rec_use-the-attached-reference-image-as-the-primary-_20260710_093913_953497_b3fcb0ac` | case_to_side_tray_1, case_to_side_tray_0, case_to_lid, case_to_front_organizer, case_body, case_molded_shell, mirror_opening_frame, side_tray_shell | built ✓ |
| opening_motion | independent orthogonal pull-out organizers | ② observed | origin_anchor | `rec_use-the-attached-reference-image-as-the-primary-_20260710_093913_953497_b3fcb0ac` / `pictureY/0611/Makeup/002.png` | front_organizer, f'side_tray_{side_index}', case_to_front_organizer, case_to_side_tray_0, case_to_side_tray_1, front_slide_tail, side_slide_tail, front_runner_*, side_runner_* | built ✓ |
| opening_motion | telescoping side trays | ② | forked_anchor | `rec_0611_makeup_var_opening_motion_telescoping_side_trays` from `rec_use-the-attached-reference-image-as-the-primary-_20260710_093913_953497_b3fcb0ac` | case_to_side_tray_1, case_to_side_tray_0, f'side_tray_{side_index}', _hollow_tray, side_tray_shell, side_slide_tail, front_tray_shell, mirror_opening_frame | built ✓ |
| applicator_storage | flat front-organizer brush channels | ① observed | origin_anchor | `rec_use-the-attached-reference-image-as-the-primary-_20260710_093913_953497_b3fcb0ac` / `pictureY/0611/Makeup/002.png` | front_organizer, brush_channel_insert, brush_channel_*, front_tray_shell, _pocketed_plate | built ✓ |
| applicator_storage | fitted lid-mounted brush roll | ① | forked_anchor | `rec_0611_makeup_var_applicator_storage_lid_brush_roll` from `rec_use-the-attached-reference-image-as-the-primary-_20260710_093913_953497_b3fcb0ac` | lid, lid_brush_roll_panel, brush_sleeve_*, brush_handle_*, front_accessory_pocket | built ✓ |
| pan_module_interface | fixed round powder wells | ① observed | origin_anchor | `rec_use-the-attached-reference-image-as-the-primary-_20260710_093913_953497_b3fcb0ac` / `pictureY/0611/Makeup/002.png` | case_body, central_palette_insert, central_pan_*, _rounded_box | built ✓ |
| pan_module_interface | keyed snap-in rectangular tile grid | ① | forked_anchor | `rec_0611_makeup_var_pan_module_interface_snap_in_tile_grid` from `rec_use-the-attached-reference-image-as-the-primary-_20260710_093913_953497_b3fcb0ac` | case_body, snap_tile_grid_insert, tile_seat_*, snap_tile_*, _keyed_tile_seat, _keyed_snap_tile | built ✓ |
| palette_count | 3 palette trays | N | forked_anchor | `rec_0611_makeup_var_palette_count_3_palette_trays` from `rec_use-the-attached-reference-image-as-the-primary-_20260710_093913_953497_b3fcb0ac` | f'side_tray_{side_index}', case_to_side_tray_1, case_to_side_tray_0, _hollow_tray, side_tray_shell, front_tray_shell, central_palette_insert | built ✓ |
| palette_count | 5 palette trays | N | forked_anchor | `rec_0611_makeup_var_palette_count_5_palette_trays` from `rec_use-the-attached-reference-image-as-the-primary-_20260710_093913_953497_b3fcb0ac` | f'side_tray_{side_index}', case_to_side_tray_1, case_to_side_tray_0, _hollow_tray, side_tray_shell, front_tray_shell, central_palette_insert | built ✓ |
| palette_count | 7 palette trays | N | forked_anchor | `rec_0611_makeup_var_palette_count_7_palette_trays` from `rec_use-the-attached-reference-image-as-the-primary-_20260710_093913_953497_b3fcb0ac` | f'side_tray_{side_index}', case_to_side_tray_1, case_to_side_tray_0, _hollow_tray, side_tray_shell, front_tray_shell, central_palette_insert | built ✓ |
| closure | over-center clasp | ② | forked_anchor | `rec_0611_makeup_var_closure_over_center_clasp` from `rec_picturex_0611__makeup__001__png_25a74d42e2cc47b8be52d1f0dd9e0807` | base_to_lid, mirror_lid, lid_clasp, closed lid covers the central organizer footprint, _lid_frame, mirror is inset within the black lid frame, lid_hinge_barrel, lid_frame | built ✓ |

## Multiplicity / Copy Logic

- count_param: palette_count_count
- N samples: 3 palette trays, 5 palette trays, 7 palette trays
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
- neighbor categories (jewelry box, empty cosmetic case): excluded.
- failed or unfit candidates will be appended with one-line reasons after 2–3 attempts.
- rotating carousel carrier: removed after human review; the mechanism was not a convincing fit for the picture-bound travel makeup vanity.
- cantilever tier lift and bifold palette wing: removed after human review; replaced with better-isolated, interface-specific opening mechanisms.
- drop-front makeup workstation and fold-down side cosmetic shelves: removed after human review; further opening-motion variants were redundant, so coverage moved to applicator-storage and pan-module-interface topology slots while preserving the origin joint graph.
