# 0611 / trombone — template source map

pattern: mixed
parents: `rec_picturex_0611__trombone__001__png_513fd46a62334ef0903072ada85d6386` (`pictureY/0611/trombone/001.png`), `rec_picturex_0611__trombone__002__png_f0513a9763434e3597111cf52ab02504` (`pictureY/0611/trombone/002.png`)
canonical_baselines: none planned; add only if a selected origin fails the §11 readability contract during fork review.
underfilled_reason: none at P0; adaptive coverage follows the approved per-category slot vocabulary.

## Subcategory Contract

- core_identity: brass trombone retaining bell, mouthpiece air path, and hand slide or established trombone valve layout
- must_keep: defining use, picture identity, visible support interfaces, and at least one real non-fixed joint.
- must_not_become: trumpet, euphonium
- image_evidence: pictureY/0611/trombone/001.png, pictureY/0611/trombone/002.png
- parent_evidence: rec_picturex_0611__trombone__001__png_513fd46a62334ef0903072ada85d6386, rec_picturex_0611__trombone__002__png_f0513a9763434e3597111cf52ab02504

## Slot Candidates

| slot | candidate | diversity_axis | source_type | record/evidence | key parts/joints/helpers | status |
|---|---|---|---|---|---|---|
| origin_design | tenor_trombone | ①/②/③ observed | origin_anchor | `rec_picturex_0611__trombone__001__png_513fd46a62334ef0903072ada85d6386` / `pictureY/0611/trombone/001.png` | body, outer_slide, tuning_slide, outer_slide_travel, tuning_slide_travel, _x_tube, _z_bar, _bell_flare_mesh, _mouthpiece_mesh, _gooseneck_mesh, _slide_bow_mesh, _tuning_crook_mesh, _material_name | built ✓ |
| origin_design | slender_tenor_trombone | ①/②/③ observed | origin_anchor | `rec_picturex_0611__trombone__002__png_f0513a9763434e3597111cf52ab02504` / `pictureY/0611/trombone/002.png` | bell_section, hand_slide, hand_slide_travel, _annular_tube_x, _annular_u_path, _annular_rear_bow, _ring_x | built ✓ |
| body_family | bass trombone with F attachment | ② | forked_anchor | `rec_0611_trombone_var_body_family_bass_trombone_with_f_attac` from `rec_picturex_0611__trombone__001__png_513fd46a62334ef0903072ada85d6386` | body, tenor_trombone | built ✓ |
| body_family | valve trombone | ② | forked_anchor | `rec_0611_trombone_var_body_family_valve_trombone` from `rec_picturex_0611__trombone__002__png_f0513a9763434e3597111cf52ab02504` | slender_tenor_trombone | built ✓ |
| body_family | compact soprano trombone | ② | forked_anchor | `rec_0611_trombone_var_body_family_compact_soprano_trombone` from `rec_picturex_0611__trombone__001__png_513fd46a62334ef0903072ada85d6386` | body, tenor_trombone | built ✓ |
| body_family | contrabass double-slide trombone | ② | forked_anchor | `rec_0611_trombone_var_body_family_contrabass_double_slide_tr` from `rec_picturex_0611__trombone__001__png_513fd46a62334ef0903072ada85d6386` | body, tuning_slide_travel, outer_slide_travel, tuning_slide, outer_slide, _slide_bow_mesh, tuning slide is seated on the loop legs at rest, tenor_trombone | built ✓ |
| attachment | single rotary trigger | ② | forked_anchor | `rec_0611_trombone_var_attachment_single_rotary_trigger` from `rec_picturex_0611__trombone__002__png_f0513a9763434e3597111cf52ab02504` | hand_slide_travel, bell_section, hand_slide, _annular_tube_x, _annular_u_path | built ✓ |
| attachment | dual rotary triggers | ② | forked_anchor | `rec_0611_trombone_var_attachment_dual_rotary_triggers` from `rec_picturex_0611__trombone__002__png_f0513a9763434e3597111cf52ab02504` | hand_slide_travel, bell_section, hand_slide, _annular_tube_x, _annular_u_path | built ✓ |
| attachment | tuning-in-slide valve | ② | forked_anchor | `rec_0611_trombone_var_attachment_tuning_in_slide_valve` from `rec_picturex_0611__trombone__001__png_513fd46a62334ef0903072ada85d6386` | tuning_slide_travel, tuning_slide, outer_slide_travel, outer_slide, tuning slide is seated on the loop legs at rest, extended tuning slide stays seated on the loop legs, _tuning_crook_mesh, _slide_bow_mesh | built ✓ |
| water_key_count | 1 water key | N | forked_anchor | `rec_0611_trombone_var_water_key_count_1_water_key` from `rec_picturex_0611__trombone__002__png_f0513a9763434e3597111cf52ab02504` | _annular_tube_x, _annular_u_path, _annular_rear_bow, _ring_x, bell_section | built ✓ |
| water_key_count | 2 water keys | N | forked_anchor | `rec_0611_trombone_var_water_key_count_2_water_keys` from `rec_picturex_0611__trombone__002__png_f0513a9763434e3597111cf52ab02504` | _annular_tube_x, _annular_u_path, _annular_rear_bow, _ring_x, bell_section | built ✓ |

## Multiplicity / Copy Logic

- count_param: water_key_count_count
- N samples: 1 water key, 2 water keys
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
- neighbor categories (trumpet, euphonium): excluded.
- failed or unfit candidates will be appended with one-line reasons after 2–3 attempts.
