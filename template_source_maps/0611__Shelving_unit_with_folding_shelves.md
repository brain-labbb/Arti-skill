# 0611 / Shelving_unit_with_folding_shelves — template source map

pattern: mixed
parents: `rec_picturex_0611__shelving_unit_with_folding_shelves__001__png_e8b60115b93a454ea62704ad6bb82fd3` (`pictureY/0611/Shelving_unit_with_folding_shelves/001.png`), `rec_picturex_0611__shelving_unit_with_folding_shelves__002__png_dfef7f1783ab4ab1a8ac720fe4d09aed` (`pictureY/0611/Shelving_unit_with_folding_shelves/002.png`)
canonical_baselines: none planned; add only if a selected origin fails the §11 readability contract during fork review.
underfilled_reason: none at P0; adaptive coverage follows the approved per-category slot vocabulary.

## Subcategory Contract

- core_identity: Shelving unit with folding shelves retaining usable shelf support and adjustment or folding function
- must_keep: defining use, picture identity, visible support interfaces, and at least one real non-fixed joint.
- must_not_become: fixed cabinet, decorative wall panel
- image_evidence: pictureY/0611/Shelving_unit_with_folding_shelves/001.png, pictureY/0611/Shelving_unit_with_folding_shelves/002.png
- parent_evidence: rec_picturex_0611__shelving_unit_with_folding_shelves__001__png_e8b60115b93a454ea62704ad6bb82fd3, rec_picturex_0611__shelving_unit_with_folding_shelves__002__png_dfef7f1783ab4ab1a8ac720fe4d09aed

## Slot Candidates

| slot | candidate | diversity_axis | source_type | record/evidence | key parts/joints/helpers | status |
|---|---|---|---|---|---|---|
| origin_design | wall_mounted_folding_shelves | ①/②/③ observed | origin_anchor | `rec_picturex_0611__shelving_unit_with_folding_shelves__001__png_e8b60115b93a454ea62704ad6bb82fd3` / `pictureY/0611/Shelving_unit_with_folding_shelves/001.png` | wall_frame, dynamic_indexed_name, dynamic_indexed_name, _cylinder_between, _shelf_board_mesh | built ✓ |
| origin_design | folding_wall_shelf_002 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__shelving_unit_with_folding_shelves__002__png_dfef7f1783ab4ab1a8ac720fe4d09aed` / `pictureY/0611/Shelving_unit_with_folding_shelves/002.png` | wall_frame, dynamic_indexed_name, dynamic_indexed_name, _frame_mesh, _add_shelf_visuals, _add_arm_visuals | built ✓ |
| tier_count | 1 | N | forked_anchor | `rec_0611_shelving_unit_with_folding_she_var_tier_count_1` from `rec_picturex_0611__shelving_unit_with_folding_shelves__001__png_e8b60115b93a454ea62704ad6bb82fd3` | _shelf_board_mesh, two shelf tiers preserve the reference spacing, shelf_board, shelf footprints align in the open pose | planned |
| tier_count | 3 | N | forked_anchor | `rec_0611_shelving_unit_with_folding_she_var_tier_count_3` from `rec_picturex_0611__shelving_unit_with_folding_shelves__002__png_dfef7f1783ab4ab1a8ac720fe4d09aed` | _add_shelf_visuals, shelf_board, shelf tiers preserve reference spacing, folding_wall_shelf_002 | planned |
| tier_count | 4 | N | forked_anchor | `rec_0611_shelving_unit_with_folding_she_var_tier_count_4` from `rec_picturex_0611__shelving_unit_with_folding_shelves__001__png_e8b60115b93a454ea62704ad6bb82fd3` | _shelf_board_mesh, two shelf tiers preserve the reference spacing, shelf_board, shelf footprints align in the open pose | planned |
| column_height | tall (1.44 m) | ⑤ / N (height-driven) | forked_anchor | `rec_0611_shelving_unit_with_folding_she_var_column_height_tall` from `rec_picturex_0611__shelving_unit_with_folding_shelves__001__png_e8b60115b93a454ea62704ad6bb82fd3` | standard_web, standard_flange, RAIL_HEIGHT drives SHELF_LEVELS, shelf_board, shelf footprints align in the open pose | built ✓ |
| fold_motion | drop-down | ② | forked_anchor | `rec_0611_shelving_unit_with_folding_she_var_fold_motion_drop_down` from `rec_picturex_0611__shelving_unit_with_folding_shelves__002__png_dfef7f1783ab4ab1a8ac720fe4d09aed` | pivot_pin, pivot_boss | planned |
| fold_motion | fold-up | ② | forked_anchor | `rec_0611_shelving_unit_with_folding_she_var_fold_motion_fold_up` from `rec_picturex_0611__shelving_unit_with_folding_shelves__001__png_e8b60115b93a454ea62704ad6bb82fd3` | pivot_bushing | planned |
| fold_motion | concertina | ② | forked_anchor | `rec_0611_shelving_unit_with_folding_she_var_fold_motion_concertina` from `rec_picturex_0611__shelving_unit_with_folding_shelves__002__png_dfef7f1783ab4ab1a8ac720fe4d09aed` | pivot_pin, pivot_boss | planned |
| support | scissor bracket | ② | forked_anchor | `rec_0611_shelving_unit_with_folding_she_var_support_scissor_bracket` from `rec_picturex_0611__shelving_unit_with_folding_shelves__001__png_e8b60115b93a454ea62704ad6bb82fd3` | support_pad, wall_frame, _shelf_board_mesh, two shelf tiers preserve the reference spacing, shelf_board, shelf footprints align in the open pose | planned |
| support | articulated stay | ② | forked_anchor | `rec_0611_shelving_unit_with_folding_she_var_support_articulated_stay` from `rec_picturex_0611__shelving_unit_with_folding_shelves__002__png_dfef7f1783ab4ab1a8ac720fe4d09aed` | support_web, support_flange, wall_frame, _frame_mesh, _add_shelf_visuals, shelf_board, shelf tiers preserve reference spacing, folding_wall_shelf_002 | planned |
| support | chain stay | ② | forked_anchor | `rec_0611_shelving_unit_with_folding_she_var_support_chain_stay` from `rec_picturex_0611__shelving_unit_with_folding_shelves__001__png_e8b60115b93a454ea62704ad6bb82fd3` | support_pad, wall_frame, _shelf_board_mesh, two shelf tiers preserve the reference spacing, shelf_board, shelf footprints align in the open pose | planned |

## Multiplicity / Copy Logic

- count_param: tier_count_count
- N samples: 1, 3, 4
- suggested N_range: bounded by accepted source samples and downstream compile budget.
- copied object / naming / placement / joint policy: shared helper, `name_{i}`, regular placement, uniform joint policy; exact names resolve from accepted variants.
- height-driven multiplicity (`column_height` axis): `RAIL_HEIGHT` is the driver; `TIER_COUNT = max(1, int((RAIL_HEIGHT - LEVEL_START - TOP_MARGIN) // LEVEL_STEP) + 1)` derives the tier count so taller columns carry more shelf boards at fixed `LEVEL_STEP`. `LEVEL_STEP` independently sets board-to-board distance. This makes tier_count a *consequence* of geometry rather than a free integer, guaranteeing no tier overruns the standard.

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
- neighbor categories (fixed cabinet, decorative wall panel): excluded.
- failed or unfit candidates will be appended with one-line reasons after 2–3 attempts.
