# 0611 / scanner — template source map

pattern: mixed
parents: `rec_picturex_0611__scanner__001__png_6482e4e89f5d440aa43d1bd988868677` (`pictureY/0611/scanner/001.png`), `rec_picturex_0611__scanner__002__png_cf96e810b13f4b2a98c94bad8cf7ca41` (`pictureY/0611/scanner/002.png`), `rec_picturex_0611__scanner__003__png_2b900b1e80bc4b40a9270371ad23f9ea` (`pictureY/0611/scanner/003.png`)
canonical_baselines: none planned; add only if a selected origin fails the §11 readability contract during fork review.
underfilled_reason: none at P0; adaptive coverage follows the approved per-category slot vocabulary.

## Subcategory Contract

- core_identity: real-world scanner retaining its defining use and articulation
- must_keep: defining use, picture identity, visible support interfaces, and at least one real non-fixed joint.
- must_not_become: neighbor category, decorative static prop
- image_evidence: pictureY/0611/scanner/001.png, pictureY/0611/scanner/002.png, pictureY/0611/scanner/003.png
- parent_evidence: rec_picturex_0611__scanner__001__png_6482e4e89f5d440aa43d1bd988868677, rec_picturex_0611__scanner__002__png_cf96e810b13f4b2a98c94bad8cf7ca41, rec_picturex_0611__scanner__003__png_2b900b1e80bc4b40a9270371ad23f9ea

## Slot Candidates

| slot | candidate | diversity_axis | source_type | record/evidence | key parts/joints/helpers | status |
|---|---|---|---|---|---|---|
| origin_design | flatbed_scanner_001 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__scanner__001__png_6482e4e89f5d440aa43d1bd988868677` / `pictureY/0611/scanner/001.png` | scanner_body, platen, control_panel, scan_button, status_lens, lid, body_to_platen, body_to_control_panel, panel_to_scan_button, body_to_status_lens, body_to_lid, _rounded_prism, _scanner_housing_shape, _lid_panel_shape | built ✓ |
| origin_design | flatbed_scanner_002 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__scanner__002__png_cf96e810b13f4b2a98c94bad8cf7ca41` / `pictureY/0611/scanner/002.png` | chassis, lid, lid_hinge, dynamic_indexed_name, _rounded_box, _housing_shape, _platen_bezel_shape, _control_deck_shape, _lid_panel_shape | built ✓ |
| origin_design | scanner_003 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__scanner__003__png_2b900b1e80bc4b40a9270371ad23f9ea` / `pictureY/0611/scanner/003.png` | base, lid, dynamic_indexed_name, base_to_lid, dynamic_indexed_name, _rounded_box, _base_shell_shape, _base_hinge_shape, _lid_hinge_shape | built ✓ |
| scanner_topology | sheet-fed | ① | forked_anchor | `rec_0611_scanner_var_scanner_topology_sheet_fed` from `rec_picturex_0611__scanner__001__png_6482e4e89f5d440aa43d1bd988868677` | scanner_body, _scanner_housing_shape, flatbed_scanner_001, closed lid seats on the scanner housing | planned |
| scanner_topology | book scanner | ① | forked_anchor | `rec_0611_scanner_var_scanner_topology_book_scanner` from `rec_picturex_0611__scanner__002__png_cf96e810b13f4b2a98c94bad8cf7ca41` | flatbed_scanner_002 | planned |
| feed | hinged ADF | ② | forked_anchor | `rec_0611_scanner_var_feed_hinged_adf` from `rec_picturex_0611__scanner__003__png_2b900b1e80bc4b40a9270371ad23f9ea` | base_to_lid, dynamic_indexed_name, base, lid, dynamic_indexed_name | planned |
| feed | duplex trays | ② | forked_anchor | `rec_0611_scanner_var_feed_duplex_trays` from `rec_picturex_0611__scanner__001__png_6482e4e89f5d440aa43d1bd988868677` | body_to_platen, body_to_control_panel, panel_to_scan_button, body_to_status_lens, body_to_lid | planned |
| scan_motion | visible prismatic scan bar | ② | forked_anchor | `rec_0611_scanner_var_scan_motion_visible_prismatic_scan_bar` from `rec_picturex_0611__scanner__002__png_cf96e810b13f4b2a98c94bad8cf7ca41` | lid_hinge, scan_light_strip, scan_button, scan_bed, inner scan bed is framed by the black platen bezel, glass is layered above the recessed scan bed | planned |
| lid | rising book hinge | ② | forked_anchor | `rec_0611_scanner_var_lid_rising_book_hinge` from `rec_picturex_0611__scanner__003__png_2b900b1e80bc4b40a9270371ad23f9ea` | _lid_hinge_shape, base_to_lid, lid, lid_hinge, _base_hinge_shape, lid_shell, lid_liner, hinge_mount | planned |
| control | tilting panel | ② | forked_anchor | `rec_0611_scanner_var_control_tilting_panel` from `rec_picturex_0611__scanner__001__png_6482e4e89f5d440aa43d1bd988868677` | body_to_control_panel, panel_to_scan_button, control_panel, _lid_panel_shape, lid_panel, _control_ring_shape, scan button is retained by its control surround, front control ring is seated on the housing | planned |
| body_form | portable slim | ③ | forked_anchor | `rec_0611_scanner_var_body_form_portable_slim` from `rec_picturex_0611__scanner__001__png_6482e4e89f5d440aa43d1bd988868677` | body_to_status_lens, body_to_platen, body_to_lid, body_to_control_panel, scanner_body, housing_shell, _scanner_housing_shape, front control ring is seated on the housing | planned |

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
- neighbor categories (neighbor category, decorative static prop): excluded.
- failed or unfit candidates will be appended with one-line reasons after 2–3 attempts.
