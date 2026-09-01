# 0611 / Music_stand1 — template source map

pattern: mixed
parents: `rec_picturex_0611__music_stand1__001__png_d2462452f2844af9b48bfde226a5fd22` (`pictureY/0611/Music_stand1/001.png`), `rec_picturex_0611__music_stand1__002__png_f0e5c91b55d54a39aa056cc73ac35fe7` (`pictureY/0611/Music_stand1/002.png`), `rec_picturex_0611__music_stand1__003__png_bea9d2286d784399bf2ecfe83ba56fa2` (`pictureY/0611/Music_stand1/003.png`)
canonical_baselines: none planned; add only if a selected origin fails the §11 readability contract during fork review.
underfilled_reason: none at P0; adaptive coverage follows the approved per-category slot vocabulary.

## Subcategory Contract

- core_identity: adjustable music stand that supports sheet music
- must_keep: defining use, picture identity, visible support interfaces, and at least one real non-fixed joint.
- must_not_become: lectern, microphone stand
- image_evidence: pictureY/0611/Music_stand1/001.png, pictureY/0611/Music_stand1/002.png, pictureY/0611/Music_stand1/003.png
- parent_evidence: rec_picturex_0611__music_stand1__001__png_d2462452f2844af9b48bfde226a5fd22, rec_picturex_0611__music_stand1__002__png_f0e5c91b55d54a39aa056cc73ac35fe7, rec_picturex_0611__music_stand1__003__png_bea9d2286d784399bf2ecfe83ba56fa2

## Slot Candidates

| slot | candidate | diversity_axis | source_type | record/evidence | key parts/joints/helpers | status |
|---|---|---|---|---|---|---|
| origin_design | music_stand_001 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__music_stand1__001__png_d2462452f2844af9b48bfde226a5fd22` / `pictureY/0611/Music_stand1/001.png` | base, mid_post, upper_post, lower_lock, upper_lock, desk, lower_height, upper_height, lower_lock_turn, upper_lock_turn, desk_tilt, _shift_profile, _annular_tube, _desk_plate_geometry | built ✓ |
| origin_design | lightweight_wire_music_stand | ①/②/③ observed | origin_anchor | `rec_picturex_0611__music_stand1__002__png_f0e5c91b55d54a39aa056cc73ac35fe7` / `pictureY/0611/Music_stand1/002.png` | column_base, telescoping_column, music_rest, dynamic_indexed_name, height_slide, rest_tilt, dynamic_indexed_name, _beam_between, _tube_mesh | built ✓ |
| origin_design | music_stand_003 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__music_stand1__003__png_bea9d2286d784399bf2ecfe83ba56fa2` / `pictureY/0611/Music_stand1/003.png` | base, mid_post, upper_post, desk, lower_lock, upper_lock, tilt_lock, dynamic_indexed_name, height_lower, height_upper, desk_tilt, lower_lock_turn, upper_lock_turn, tilt_lock_turn | built ✓ |
| desk_form | solid tray | ① | forked_anchor | `rec_0611_music_stand1_var_desk_form_solid_tray` from `rec_picturex_0611__music_stand1__001__png_d2462452f2844af9b48bfde226a5fd22` | desk_tilt, desk, _desk_plate_geometry, upper stage remains inserted at full height, lower stage remains inserted at full height, desk_plate | planned |
| desk_form | split folding leaves | ① | forked_anchor | `rec_0611_music_stand1_var_desk_form_split_folding_leaves` from `rec_picturex_0611__music_stand1__002__png_f0e5c91b55d54a39aa056cc73ac35fe7` | column_base, telescoping_column, music_rest, dynamic_indexed_name, _beam_between | planned |
| base | round weighted base | ① | forked_anchor | `rec_0611_music_stand1_var_base_round_weighted_base` from `rec_picturex_0611__music_stand1__003__png_bea9d2286d784399bf2ecfe83ba56fa2` | base, leg_tube, foot | planned |
| base | folding tripod | ① | forked_anchor | `rec_0611_music_stand1_var_base_folding_tripod` from `rec_picturex_0611__music_stand1__002__png_f0e5c91b55d54a39aa056cc73ac35fe7` | column_base, tripod_hub | planned |
| height_stages | single stage | N | forked_anchor | `rec_0611_music_stand1_var_height_stages_single_stage` from `rec_picturex_0611__music_stand1__003__png_bea9d2286d784399bf2ecfe83ba56fa2` | upper stage remains centered when collapsed, height_upper, height_lower, upper_post, mid_post, upper tube retained in middle post, upper tube centered in middle post, middle tube retained in lower post | planned |
| height_stages | three stages | N | forked_anchor | `rec_0611_music_stand1_var_height_stages_three_stages` from `rec_picturex_0611__music_stand1__001__png_d2462452f2844af9b48bfde226a5fd22` | upper_height, lower_height, upper stage remains inserted at full height, lower stage remains inserted at full height | planned |
| retention | pivoting page clips | ② | forked_anchor | `rec_0611_music_stand1_var_retention_pivoting_page_clips` from `rec_picturex_0611__music_stand1__001__png_d2462452f2844af9b48bfde226a5fd22` | lower_height, upper_height, lower_lock_turn, upper_lock_turn, desk_tilt | planned |

## Multiplicity / Copy Logic

- count_param: height_stages_count
- N samples: single stage, three stages
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
- neighbor categories (lectern, microphone stand): excluded.
- failed or unfit candidates will be appended with one-line reasons after 2–3 attempts.
