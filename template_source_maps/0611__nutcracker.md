# 0611 / nutcracker — template source map

pattern: mixed
parents: `rec_picturex_0611__nutcracker__001__png_72ea48d3b6864ac1bf2864aab225ca4d` (`pictureY/0611/nutcracker/001.png`), `rec_picturex_0611__nutcracker__002__png_ff76a040583c4f96bdf4d50be4fa4f16` (`pictureY/0611/nutcracker/002.png`)
canonical_baselines: none planned; add only if a selected origin fails the §11 readability contract during fork review.
underfilled_reason: none at P0; adaptive coverage follows the approved per-category slot vocabulary.

## Subcategory Contract

- core_identity: real-world nutcracker retaining its defining use and articulation
- must_keep: defining use, picture identity, visible support interfaces, and at least one real non-fixed joint.
- must_not_become: neighbor category, decorative static prop
- image_evidence: pictureY/0611/nutcracker/001.png, pictureY/0611/nutcracker/002.png
- parent_evidence: rec_picturex_0611__nutcracker__001__png_72ea48d3b6864ac1bf2864aab225ca4d, rec_picturex_0611__nutcracker__002__png_ff76a040583c4f96bdf4d50be4fa4f16

## Slot Candidates

| slot | candidate | diversity_axis | source_type | record/evidence | key parts/joints/helpers | status |
|---|---|---|---|---|---|---|
| origin_design | walnut_handle_nutcracker | ①/②/③ observed | origin_anchor | `rec_picturex_0611__nutcracker__001__png_72ea48d3b6864ac1bf2864aab225ca4d` / `pictureY/0611/nutcracker/001.png` | pivot_pin, dynamic_indexed_name, pivot_to_arm_0, pivot_to_arm_1, _arm_metal_shape, _grip_shape, _wood_handle_shape | built ✓ |
| origin_design | polished_hinged_nutcracker_002 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__nutcracker__002__png_ff76a040583c4f96bdf4d50be4fa4f16` / `pictureY/0611/nutcracker/002.png` | pivot, long_lever, short_lever, pivot_to_long_lever, pivot_to_short_lever, _rotated, _lever_body, _jaw_teeth, _handle_grooves, _pivot_pin | built ✓ |
| jaw_form | deep cup | ③ | forked_anchor | `rec_0611_nutcracker_var_jaw_form_deep_cup` from `rec_picturex_0611__nutcracker__001__png_72ea48d3b6864ac1bf2864aab225ca4d` | image pose preserves the visible open jaw cup, _wood_handle_shape, pivot_to_arm_1, pivot_to_arm_0, wood_handle, walnut_handle_nutcracker, _grip_shape, _arm_metal_shape | planned |
| jaw_form | tapered serrated cone | ③ | forked_anchor | `rec_0611_nutcracker_var_jaw_form_tapered_serrated_cone` from `rec_picturex_0611__nutcracker__002__png_ff76a040583c4f96bdf4d50be4fa4f16` | _jaw_teeth, short_jaw_teeth, long_jaw_teeth, _handle_grooves, open serrated jaws preserve the reference nut gap, handle_grooves, short_arm_body, short lever is retained by pivot head | planned |
| mechanism | table lever | ② | forked_anchor | `rec_0611_nutcracker_var_mechanism_table_lever` from `rec_picturex_0611__nutcracker__002__png_ff76a040583c4f96bdf4d50be4fa4f16` | pivot_to_short_lever, pivot_to_long_lever, short_lever, pivot, long_lever, stacked stamped levers clear each other through the pivot, short_arm_body, short lever is retained by pivot head | planned |
| mechanism | screw press | ② | forked_anchor | `rec_0611_nutcracker_var_mechanism_screw_press` from `rec_picturex_0611__nutcracker__002__png_ff76a040583c4f96bdf4d50be4fa4f16` | pivot_to_short_lever, pivot_to_long_lever, pivot, _pivot_pin, stacked stamped levers clear each other through the pivot, short_arm_body, short lever is retained by pivot head, short lever eye surrounds shared pivot | planned |
| mechanism | compound link | ② | forked_anchor | `rec_0611_nutcracker_var_mechanism_compound_link` from `rec_picturex_0611__nutcracker__002__png_ff76a040583c4f96bdf4d50be4fa4f16` | pivot_to_short_lever, pivot_to_long_lever, pivot, _pivot_pin, stacked stamped levers clear each other through the pivot, short_arm_body, short lever is retained by pivot head, short lever eye surrounds shared pivot | planned |
| return | torsion spring | ② | forked_anchor | `rec_0611_nutcracker_var_return_torsion_spring` from `rec_picturex_0611__nutcracker__001__png_72ea48d3b6864ac1bf2864aab225ca4d` | pivot_to_arm_0, pivot_to_arm_1, pivot_pin, dynamic_indexed_name, _arm_metal_shape | planned |
| handle | long curved | ③ | forked_anchor | `rec_0611_nutcracker_var_handle_long_curved` from `rec_picturex_0611__nutcracker__002__png_ff76a040583c4f96bdf4d50be4fa4f16` | _handle_grooves, pivot_to_long_lever, long_lever, handle_grooves, _jaw_teeth, pivot_to_short_lever, long_jaw_teeth, long_arm_body | planned |
| handle | ring handle | ③ | forked_anchor | `rec_0611_nutcracker_var_handle_ring_handle` from `rec_picturex_0611__nutcracker__001__png_72ea48d3b6864ac1bf2864aab225ca4d` | _wood_handle_shape, pivot_to_arm_1, pivot_to_arm_0, wood_handle, walnut_handle_nutcracker, _arm_metal_shape, outer pivot cap seats against arm 1, outer pivot cap seats against arm 0 | planned |
| capacity | indexed jaw stop | ② | forked_anchor | `rec_0611_nutcracker_var_capacity_indexed_jaw_stop` from `rec_picturex_0611__nutcracker__001__png_72ea48d3b6864ac1bf2864aab225ca4d` | image pose preserves the visible open jaw cup, _wood_handle_shape, pivot_to_arm_1, pivot_to_arm_0, dynamic_indexed_name, wood_handle, walnut_handle_nutcracker, _grip_shape | planned |

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
