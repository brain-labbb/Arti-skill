# 0611 / Makeup2 — template source map

pattern: mixed
parents: `rec_picturex_0611__makeup2__001__png_88723ca63e414320b8fa80969891b63a` (`pictureY/0611/Makeup2/001.png`), `rec_picturex_0611__makeup2__002__png_e452c59a23e8409d9b7df37aa3754bfb` (`pictureY/0611/Makeup2/002.png`)
canonical_baselines: none planned; add only if a selected origin fails the §11 readability contract during fork review.
underfilled_reason: none at P0; adaptive coverage follows the approved per-category slot vocabulary.

## Subcategory Contract

- core_identity: articulated makeup compact or palette retaining powder storage
- must_keep: defining use, picture identity, visible support interfaces, and at least one real non-fixed joint.
- must_not_become: jewelry box, empty cosmetic case
- image_evidence: pictureY/0611/Makeup2/001.png, pictureY/0611/Makeup2/002.png
- parent_evidence: rec_picturex_0611__makeup2__001__png_88723ca63e414320b8fa80969891b63a, rec_picturex_0611__makeup2__002__png_e452c59a23e8409d9b7df37aa3754bfb

## Slot Candidates

| slot | candidate | diversity_axis | source_type | record/evidence | key parts/joints/helpers | status |
|---|---|---|---|---|---|---|
| origin_design | blue_dual_powder_compact | ①/②/③ observed | origin_anchor | `rec_picturex_0611__makeup2__001__png_88723ca63e414320b8fa80969891b63a` / `pictureY/0611/Makeup2/001.png` | base, powder_tray, mirrored_lid, base_to_powder_tray, base_to_mirrored_lid, _rounded_prism, _x_cylinder, _build_base_shell, _build_base_accent, _build_tray_plate, _build_tray_pan, _build_tray_powder, _build_lid_frame, _build_mirror | built ✓ |
| origin_design | champagne_four_pan_compact | ①/②/③ observed | origin_anchor | `rec_picturex_0611__makeup2__002__png_e452c59a23e8409d9b7df37aa3754bfb` / `pictureY/0611/Makeup2/002.png` | palette_base, mirror_lid, lid_hinge, _rounded_box, _base_shell, _pan_shape, _lid_shell, _mirror_shape | built ✓ |
| powder_layout | 2-well | N | forked_anchor | `rec_0611_makeup2_var_powder_layout_2_well` from `rec_picturex_0611__makeup2__001__png_88723ca63e414320b8fa80969891b63a` | base_to_powder_tray, powder_tray, _build_tray_powder, upper_powder, lower_powder, closed powder tray clears lower compartment, blue_dual_powder_compact, _build_tray_pan | converged |
| powder_layout | 6-well | N | forked_anchor | `rec_0611_makeup2_var_powder_layout_6_well` from `rec_picturex_0611__makeup2__002__png_e452c59a23e8409d9b7df37aa3754bfb` | palette_base, _pan_shape, closed lid seats on tray rim, closed lid aligns over compact tray, champagne_four_pan_compact | converged |
| powder_layout | 8-well | N | forked_anchor | `rec_0611_makeup2_var_powder_layout_8_well` from `rec_picturex_0611__makeup2__001__png_88723ca63e414320b8fa80969891b63a` | base_to_powder_tray, powder_tray, _build_tray_powder, upper_powder, lower_powder, closed powder tray clears lower compartment, blue_dual_powder_compact, _build_tray_pan | converged |
| case_form | round | ③ | forked_anchor | `rec_0611_makeup2_var_case_form_round` from `rec_picturex_0611__makeup2__001__png_88723ca63e414320b8fa80969891b63a` | _build_lid_frame, closed lid frame seats above base shell, _build_base_shell, lid_frame, base_to_powder_tray, base_to_mirrored_lid, base, base_shell | converged |
| case_form | clover | ③ | forked_anchor | `rec_0611_makeup2_var_case_form_clover` from `rec_picturex_0611__makeup2__001__png_88723ca63e414320b8fa80969891b63a` | _build_lid_frame, closed lid frame seats above base shell, _build_base_shell, lid_frame, base_to_powder_tray, base_to_mirrored_lid, base, base_shell | converged |
| tray_topology | fan-out carrier | ② | forked_anchor | `rec_0611_makeup2_var_tray_topology_fan_out_carrier` from `rec_picturex_0611__makeup2__002__png_e452c59a23e8409d9b7df37aa3754bfb` | closed lid seats on tray rim, closed lid aligns over compact tray | converged |
| tray_topology | double stacked carrier | ② | forked_anchor | `rec_0611_makeup2_var_tray_topology_double_stacked_carrier` from `rec_picturex_0611__makeup2__002__png_e452c59a23e8409d9b7df37aa3754bfb` | closed lid seats on tray rim, closed lid aligns over compact tray | converged |
| closure | push latch | ② | forked_anchor | `rec_0611_makeup2_var_closure_push_latch` from `rec_picturex_0611__makeup2__002__png_e452c59a23e8409d9b7df37aa3754bfb` | lid_hinge, mirror_lid, _lid_shell, lid_shell, lid_barrel, hinge pin remains captured by lid barrel, closed lid seats on tray rim, closed lid aligns over compact tray | converged |
| closure | over-center latch | ② | forked_anchor | `rec_0611_makeup2_var_closure_over_center_latch` from `rec_picturex_0611__makeup2__002__png_e452c59a23e8409d9b7df37aa3754bfb` | lid_hinge, mirror_lid, closed lid aligns over compact tray, _lid_shell, lid_shell, lid_barrel, hinge pin spans the rotating center barrel, hinge pin remains captured by lid barrel | converged |

## Multiplicity / Copy Logic

- count_param: powder_layout_count
- N samples: 2-well, 6-well, 8-well
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
