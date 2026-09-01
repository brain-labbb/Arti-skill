# 0611 / Makeup1 — template source map

pattern: mixed
parents: `rec_picturex_0611__makeup1__001__png_1e56ed25897943d1bb05005bc41aca4e` (`pictureY/0611/Makeup1/001.png`), `rec_picturex_0611__makeup1__002__png_350d6df173ce4cc08935724fb35d43dc` (`pictureY/0611/Makeup1/002.png`), `rec_picturex_0611__makeup1__003__png_1e77f77007c24952b1f52a18542fa4d8` (`pictureY/0611/Makeup1/003.png`)
canonical_baselines: none planned; add only if a selected origin fails the §11 readability contract during fork review.
underfilled_reason: none at P0; adaptive coverage follows the approved per-category slot vocabulary.

## Subcategory Contract

- core_identity: articulated makeup compact or palette retaining powder storage
- must_keep: defining use, picture identity, visible support interfaces, and at least one real non-fixed joint.
- must_not_become: jewelry box, empty cosmetic case
- image_evidence: pictureY/0611/Makeup1/001.png, pictureY/0611/Makeup1/002.png, pictureY/0611/Makeup1/003.png
- parent_evidence: rec_picturex_0611__makeup1__001__png_1e56ed25897943d1bb05005bc41aca4e, rec_picturex_0611__makeup1__002__png_350d6df173ce4cc08935724fb35d43dc, rec_picturex_0611__makeup1__003__png_1e77f77007c24952b1f52a18542fa4d8

## Slot Candidates

| slot | candidate | diversity_axis | source_type | record/evidence | key parts/joints/helpers | status |
|---|---|---|---|---|---|---|
| origin_design | nine_pan_makeup_compact_001 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__makeup1__001__png_1e56ed25897943d1bb05005bc41aca4e` / `pictureY/0611/Makeup1/001.png` | palette, cover, cover_hinge, _rounded_plate, _x_cylinder, _make_case_shell, _make_hinge_mounts, _make_powder_pan, _make_lid_shell, _make_lid_inset | built ✓ |
| origin_design | ten_well_concealer_palette | ①/②/③ observed | origin_anchor | `rec_picturex_0611__makeup1__002__png_350d6df173ce4cc08935724fb35d43dc` / `pictureY/0611/Makeup1/002.png` | palette, cover, cover_hinge, _x_cylinder, _make_base_shell, _make_pan_insert, _make_lid_frame | built ✓ |
| origin_design | two_way_cake_compact | ①/②/③ observed | origin_anchor | `rec_picturex_0611__makeup1__003__png_1e77f77007c24952b1f52a18542fa4d8` / `pictureY/0611/Makeup1/003.png` | base, lid, latch, lid_hinge, latch_press, _shift_profile, _profile_at_z, _mesh | built ✓ |
| powder_layout | 4-quadrant | N | forked_anchor | `rec_0611_makeup1_var_powder_layout_4_quadrant` from `rec_picturex_0611__makeup1__001__png_1e56ed25897943d1bb05005bc41aca4e` | _make_powder_pan, closed cover seats just above the powder tray, palette, nine_pan_makeup_compact_001 | planned |
| powder_layout | 6-radial | N | forked_anchor | `rec_0611_makeup1_var_powder_layout_6_radial` from `rec_picturex_0611__makeup1__002__png_350d6df173ce4cc08935724fb35d43dc` | _make_pan_insert, palette, ten_well_concealer_palette, tray_shell, closed cover seats on the molded tray rim, closed cover aligns with the rectangular tray | planned |
| powder_layout | 12-well | N | forked_anchor | `rec_0611_makeup1_var_powder_layout_12_well` from `rec_picturex_0611__makeup1__003__png_1e77f77007c24952b1f52a18542fa4d8` | powder_cake, well_frame, closed lid seats just above raised well frame | planned |
| case_form | round puck | ③ | forked_anchor | `rec_0611_makeup1_var_case_form_round_puck` from `rec_picturex_0611__makeup1__001__png_1e56ed25897943d1bb05005bc41aca4e` | _make_case_shell, case_shell, _make_lid_shell, lid_shell | planned |
| case_form | elongated rectangle | ③ | forked_anchor | `rec_0611_makeup1_var_case_form_elongated_rectangle` from `rec_picturex_0611__makeup1__003__png_1e77f77007c24952b1f52a18542fa4d8` | well_frame, mirror_frame, closed lid seats just above raised well frame, base, base_shell, lid_shell, latch stem remains captured by front housing, base_hinge_pin | planned |
| opening | second hinged tray | ② | forked_anchor | `rec_0611_makeup1_var_opening_second_hinged_tray` from `rec_picturex_0611__makeup1__002__png_350d6df173ce4cc08935724fb35d43dc` | tray_shell, closed cover seats on the molded tray rim, closed cover aligns with the rectangular tray, _make_pan_insert | planned |
| opening | fan-out tray | ② | forked_anchor | `rec_0611_makeup1_var_opening_fan_out_tray` from `rec_picturex_0611__makeup1__002__png_350d6df173ce4cc08935724fb35d43dc` | tray_shell, closed cover seats on the molded tray rim, closed cover aligns with the rectangular tray, _make_pan_insert | planned |
| closure | push latch | ② | forked_anchor | `rec_0611_makeup1_var_closure_push_latch` from `rec_picturex_0611__makeup1__003__png_1e77f77007c24952b1f52a18542fa4d8` | lid_hinge, latch_press, lid, latch, closed lid covers compact footprint, lid_shell, lid_hinge_bridge, lid_hinge_barrel | planned |
| closure | sliding latch | ② | forked_anchor | `rec_0611_makeup1_var_closure_sliding_latch` from `rec_picturex_0611__makeup1__003__png_1e77f77007c24952b1f52a18542fa4d8` | lid_hinge, latch_press, lid, latch, closed lid covers compact footprint, lid_shell, lid_hinge_bridge, lid_hinge_barrel | planned |

## Multiplicity / Copy Logic

- count_param: powder_layout_count
- N samples: 4-quadrant, 6-radial, 12-well
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
