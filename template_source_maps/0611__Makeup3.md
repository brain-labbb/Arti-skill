# 0611 / Makeup3 — template source map

pattern: mixed
parents: `rec_picturex_0611__makeup3__002__png_4960c988351041a8a8bc5cc76140e8b6` (`pictureY/0611/Makeup3/002.png`), `rec_picturex_0611__makeup3__001__png_d432b592bfe24d18aefa827f7ae9c15e` (`pictureY/0611/Makeup3/001.png`)
canonical_baselines: none planned; add only if a selected origin fails the §11 readability contract during fork review.
underfilled_reason: none at P0; adaptive coverage follows the approved per-category slot vocabulary.

## Subcategory Contract

- core_identity: articulated makeup compact or palette retaining powder storage
- must_keep: defining use, picture identity, visible support interfaces, and at least one real non-fixed joint.
- must_not_become: jewelry box, empty cosmetic case
- image_evidence: pictureY/0611/Makeup3/002.png, pictureY/0611/Makeup3/001.png
- parent_evidence: rec_picturex_0611__makeup3__002__png_4960c988351041a8a8bc5cc76140e8b6, rec_picturex_0611__makeup3__001__png_d432b592bfe24d18aefa827f7ae9c15e

## Slot Candidates

| slot | candidate | diversity_axis | source_type | record/evidence | key parts/joints/helpers | status |
|---|---|---|---|---|---|---|
| origin_design | patterned_powder_compact | ①/②/③ observed | origin_anchor | `rec_picturex_0611__makeup3__002__png_4960c988351041a8a8bc5cc76140e8b6` / `pictureY/0611/Makeup3/002.png` | base, powder_insert, lid, latch_button, base_to_powder_insert, base_to_lid, base_to_latch_button, _base_wall, _powder_pan, _powder_cake, _lid_shell_closed, _label_plaque, _latch_button, _material_name | built ✓ |
| origin_design | four_petal_makeup_compact | ①/②/③ observed | origin_anchor | `rec_picturex_0611__makeup3__001__png_d432b592bfe24d18aefa827f7ae9c15e` / `pictureY/0611/Makeup3/001.png` | base, lid, clasp, base_to_lid, base_to_clasp, _petal_shape, _annular_tube_x, _rounded_button, _clover_motif, _center_medallion | built ✓ |
| case_form | round | ③ | forked_anchor | `rec_0611_makeup3_var_case_form_round` from `rec_picturex_0611__makeup3__002__png_4960c988351041a8a8bc5cc76140e8b6` | powder insert retained inside case footprint, closed lid covers compact body, base_to_powder_insert, base_to_lid, base_to_latch_button, base, _lid_shell_closed, _base_wall | planned |
| case_form | hexagonal | ③ | forked_anchor | `rec_0611_makeup3_var_case_form_hexagonal` from `rec_picturex_0611__makeup3__001__png_d432b592bfe24d18aefa827f7ae9c15e` | base_body, base_to_lid, base_to_clasp, base, lid_shell, lid barrel is carried by base hinge, clasp stays guided by front housing, clasp is captured by its housing slot | planned |
| case_form | rectangular | ③ | forked_anchor | `rec_0611_makeup3_var_case_form_rectangular` from `rec_picturex_0611__makeup3__002__png_4960c988351041a8a8bc5cc76140e8b6` | powder insert retained inside case footprint, closed lid covers compact body, base_to_powder_insert, base_to_lid, base_to_latch_button, base, _lid_shell_closed, _base_wall | planned |
| powder_layout | 2-well fitted semicircular sectors | N + ⑥ companion | forked_anchor | `rec_0611_makeup3_var_powder_layout_2_well` from `rec_picturex_0611__makeup3__002__png_4960c988351041a8a8bc5cc76140e8b6` | base_to_powder_insert, powder_insert, _powder_pan_2well, _half_well_cake, powder_cake, peach/rose-beige two-tone palette, 2-well cakes are fitted semicircular sectors, powder well visibly occupies the compact, powder insert retained inside case footprint, patterned_powder_compact | planned |
| powder_layout | 4-well | N | forked_anchor | `rec_0611_makeup3_var_powder_layout_4_well` from `rec_picturex_0611__makeup3__002__png_4960c988351041a8a8bc5cc76140e8b6` | base_to_powder_insert, powder_insert, _powder_pan, _powder_cake, powder_cake, powder well visibly occupies the compact, powder insert retained inside case footprint, patterned_powder_compact | planned |
| insert_motion | guided slide | ② | forked_anchor | `rec_0611_makeup3_var_insert_motion_guided_slide` from `rec_picturex_0611__makeup3__001__png_d432b592bfe24d18aefa827f7ae9c15e` | tray_surface, lid barrel is carried by base hinge, hinge_barrel, clasp stays guided by front housing, base_hinge_1, base_hinge_0 | planned |
| lid_module | fitted inner mirror | ① | forked_anchor | `rec_0611_makeup3_var_lid_module_fitted_inner_mirror` from `rec_picturex_0611__makeup3__001__png_d432b592bfe24d18aefa827f7ae9c15e` | base_to_lid, lid, lid_shell, lid barrel is carried by base hinge, closed lid seats on compact rim, closed lid covers the circular palette | planned |
| closure | toggle latch | ② | forked_anchor | `rec_0611_makeup3_var_closure_toggle_latch` from `rec_picturex_0611__makeup3__001__png_d432b592bfe24d18aefa827f7ae9c15e` | base_to_lid, base_to_clasp, lid, clasp, closed lid covers the circular palette, lid_shell, lid barrel is carried by base hinge, lacquer_panel | planned |

## Multiplicity / Copy Logic

- count_param: powder_layout_count
- N samples: 2-well, 4-well
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
