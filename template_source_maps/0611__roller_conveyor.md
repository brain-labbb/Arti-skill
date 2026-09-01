# 0611 / roller_conveyor — template source map

pattern: mixed
parents: `rec_picturex_0611__roller_conveyor__001__png_2bd58ebac63f4af6b9b728a17a7202a8` (`pictureY/0611/roller_conveyor/001.png`), `rec_picturex_0611__roller_conveyor__002__png_683f31d003644d4cb05f495eeafb0ce5` (`pictureY/0611/roller_conveyor/002.png`)
canonical_baselines: none planned; add only if a selected origin fails the §11 readability contract during fork review.
underfilled_reason: none at P0; adaptive coverage follows the approved per-category slot vocabulary.

## Subcategory Contract

- core_identity: real-world roller conveyor retaining its defining use and articulation
- must_keep: defining use, picture identity, visible support interfaces, and at least one real non-fixed joint.
- must_not_become: neighbor category, decorative static prop
- image_evidence: pictureY/0611/roller_conveyor/001.png, pictureY/0611/roller_conveyor/002.png
- parent_evidence: rec_picturex_0611__roller_conveyor__001__png_2bd58ebac63f4af6b9b728a17a7202a8, rec_picturex_0611__roller_conveyor__002__png_683f31d003644d4cb05f495eeafb0ce5

## Slot Candidates

| slot | candidate | diversity_axis | source_type | record/evidence | key parts/joints/helpers | status |
|---|---|---|---|---|---|---|
| origin_design | eight_roller_conveyor | ①/②/③ observed | origin_anchor | `rec_picturex_0611__roller_conveyor__001__png_2bd58ebac63f4af6b9b728a17a7202a8` / `pictureY/0611/roller_conveyor/001.png` | frame, dynamic_indexed_name, dynamic_indexed_name, _bearing_ring_mesh, _add_frame_visuals, _add_roller_visuals | built ✓ |
| origin_design | roller_conveyor_002 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__roller_conveyor__002__png_683f31d003644d4cb05f495eeafb0ce5` / `pictureY/0611/roller_conveyor/002.png` | frame, dynamic_indexed_name, dynamic_indexed_name, _material_name, _add_square_tube_leg, _add_diagonal_brace | built ✓ |
| roller_count | 5 | N | forked_anchor | `rec_0611_roller_conveyor_var_roller_count_5` from `rec_picturex_0611__roller_conveyor__001__png_2bd58ebac63f4af6b9b728a17a7202a8` | _add_roller_visuals, roller_shell, eight_roller_conveyor, _bearing_ring_mesh, axle | planned |
| roller_count | 12 | N | forked_anchor | `rec_0611_roller_conveyor_var_roller_count_12` from `rec_picturex_0611__roller_conveyor__002__png_683f31d003644d4cb05f495eeafb0ce5` | roller_conveyor_002, last roller lies within conveyor frame, last roller axle seats in bearing, first roller lies within conveyor frame, first roller axle seats in bearing, axle, adjacent rollers share the same conveyor width | planned |
| roller_count | 18 | N | forked_anchor | `rec_0611_roller_conveyor_var_roller_count_18` from `rec_picturex_0611__roller_conveyor__001__png_2bd58ebac63f4af6b9b728a17a7202a8` | _add_roller_visuals, roller_shell, eight_roller_conveyor, _bearing_ring_mesh, axle | planned |
| frame_form | curved arc | ③ | forked_anchor | `rec_0611_roller_conveyor_var_frame_form_curved_arc` from `rec_picturex_0611__roller_conveyor__002__png_683f31d003644d4cb05f495eeafb0ce5` | frame, last roller lies within conveyor frame, first roller lies within conveyor frame | planned |
| frame_form | expanding straight | ③ | forked_anchor | `rec_0611_roller_conveyor_var_frame_form_expanding_straight` from `rec_picturex_0611__roller_conveyor__001__png_2bd58ebac63f4af6b9b728a17a7202a8` | frame, _add_frame_visuals | planned |
| support | folding legs | ② | forked_anchor | `rec_0611_roller_conveyor_var_support_folding_legs` from `rec_picturex_0611__roller_conveyor__002__png_683f31d003644d4cb05f495eeafb0ce5` | frame, last roller lies within conveyor frame, first roller lies within conveyor frame, _add_square_tube_leg | planned |
| support | scissor base | ② | forked_anchor | `rec_0611_roller_conveyor_var_support_scissor_base` from `rec_picturex_0611__roller_conveyor__002__png_683f31d003644d4cb05f495eeafb0ce5` | frame, last roller lies within conveyor frame, first roller lies within conveyor frame, _add_square_tube_leg | planned |
| roller_layout | split rollers | ① | forked_anchor | `rec_0611_roller_conveyor_var_roller_layout_split_rollers` from `rec_picturex_0611__roller_conveyor__002__png_683f31d003644d4cb05f495eeafb0ce5` | roller_conveyor_002, last roller lies within conveyor frame, last roller axle seats in bearing, first roller lies within conveyor frame, first roller axle seats in bearing, adjacent rollers share the same conveyor width, axle | planned |
| roller_layout | staggered rollers | ① | forked_anchor | `rec_0611_roller_conveyor_var_roller_layout_staggered_rollers` from `rec_picturex_0611__roller_conveyor__002__png_683f31d003644d4cb05f495eeafb0ce5` | roller_conveyor_002, last roller lies within conveyor frame, last roller axle seats in bearing, first roller lies within conveyor frame, first roller axle seats in bearing, adjacent rollers share the same conveyor width, axle | planned |
| height | telescoping legs | ② | forked_anchor | `rec_0611_roller_conveyor_var_height_telescoping_legs` from `rec_picturex_0611__roller_conveyor__002__png_683f31d003644d4cb05f495eeafb0ce5` | _add_square_tube_leg | planned |

## Multiplicity / Copy Logic

- count_param: roller_count_count
- N samples: 5, 12, 18
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
