# 0611 / manual_grain_mill — template source map

pattern: mixed
parents: `rec_picturex_0611__manual_grain_mill__001__png_c673835d54464d63a16b129708043f07` (`pictureY/0611/manual_grain_mill/001.png`), `rec_picturex_0611__manual_grain_mill__002__png_224660b70da84424944362aef1f266ef` (`pictureY/0611/manual_grain_mill/002.png`)
canonical_baselines: none planned; add only if a selected origin fails the §11 readability contract during fork review.
underfilled_reason: none at P0; adaptive coverage follows the approved per-category slot vocabulary.

## Subcategory Contract

- core_identity: manual manual grain mill retaining a hand-driven grinding path
- must_keep: defining use, picture identity, visible support interfaces, and at least one real non-fixed joint.
- must_not_become: electric grinder, decorative container
- image_evidence: pictureY/0611/manual_grain_mill/001.png, pictureY/0611/manual_grain_mill/002.png
- parent_evidence: rec_picturex_0611__manual_grain_mill__001__png_c673835d54464d63a16b129708043f07, rec_picturex_0611__manual_grain_mill__002__png_224660b70da84424944362aef1f266ef

## Slot Candidates

| slot | candidate | diversity_axis | source_type | record/evidence | key parts/joints/helpers | status |
|---|---|---|---|---|---|---|
| origin_design | stone_hand_grain_mill | ①/②/③ observed | origin_anchor | `rec_picturex_0611__manual_grain_mill__001__png_c673835d54464d63a16b129708043f07` / `pictureY/0611/manual_grain_mill/001.png` | stand, runner, crank_grip, grinding_shaft, crank_spin, _lower_quern_shape, _runner_stone_shape, _wood_grip_shape | built ✓ |
| origin_design | manual_grain_mill_002 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__manual_grain_mill__002__png_224660b70da84424944362aef1f266ef` / `pictureY/0611/manual_grain_mill/002.png` | mill_body, grinding_shaft, crank_handle, shaft_revolve, handle_revolve, _mill_body_shape, _grinding_wheel_shape | built ✓ |
| mill_topology | table-clamped burr | ① | forked_anchor | `rec_0611_manual_grain_mill_var_mill_topology_table_clamped_burr` from `rec_picturex_0611__manual_grain_mill__001__png_c673835d54464d63a16b129708043f07` | stone_hand_grain_mill | planned |
| mill_topology | hopper quern | ① | forked_anchor | `rec_0611_manual_grain_mill_var_mill_topology_hopper_quern` from `rec_picturex_0611__manual_grain_mill__002__png_224660b70da84424944362aef1f266ef` | mill_body, _mill_body_shape, manual_grain_mill_002, hopper_body | planned |
| hopper_form | box | ③ | forked_anchor | `rec_0611_manual_grain_mill_var_hopper_form_box` from `rec_picturex_0611__manual_grain_mill__002__png_224660b70da84424944362aef1f266ef` | hopper_body | planned |
| hopper_form | conical funnel | ③ | forked_anchor | `rec_0611_manual_grain_mill_var_hopper_form_conical_funnel` from `rec_picturex_0611__manual_grain_mill__002__png_224660b70da84424944362aef1f266ef` | hopper_body | planned |
| drive | side crank | ① | forked_anchor | `rec_0611_manual_grain_mill_var_drive_side_crank` from `rec_picturex_0611__manual_grain_mill__001__png_c673835d54464d63a16b129708043f07` | grinding_shaft, crank_spin, crank_grip, crank_arm, hand grip is retained on crank washer, crank_socket | planned |
| drive | spoked hand wheel | ① | forked_anchor | `rec_0611_manual_grain_mill_var_drive_spoked_hand_wheel` from `rec_picturex_0611__manual_grain_mill__002__png_224660b70da84424944362aef1f266ef` | _grinding_wheel_shape, crank collar seats against the wheel face, wheel_disc, wheel occupies the central grinding chamber, shaft_revolve, handle_revolve, grinding_shaft, crank_handle | planned |
| gap_adjustment | indexed collar | ② | forked_anchor | `rec_0611_manual_grain_mill_var_gap_adjustment_indexed_collar` from `rec_picturex_0611__manual_grain_mill__002__png_224660b70da84424944362aef1f266ef` | handle_collar, crank collar seats against the wheel face | planned |
| grinding_count | dual runner stones | N | forked_anchor | `rec_0611_manual_grain_mill_var_grinding_count_dual_runner_stones` from `rec_picturex_0611__manual_grain_mill__001__png_c673835d54464d63a16b129708043f07` | _runner_stone_shape, runner seats on lower grinding stone, grinding_shaft, runner, runner_stone, stone_hand_grain_mill, runner stays seated after quarter turn, runner remains inside lower quern footprint | planned |

## Multiplicity / Copy Logic

- count_param: grinding_count_count
- N samples: dual runner stones
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
- neighbor categories (electric grinder, decorative container): excluded.
- failed or unfit candidates will be appended with one-line reasons after 2–3 attempts.
