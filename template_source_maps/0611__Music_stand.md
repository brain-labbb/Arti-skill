# 0611 / Music_stand — template source map

pattern: mixed
parents: `rec_picturex_0611__music_stand__003__png_ba84304f87af408b86ed03dbf0841561` (`pictureY/0611/Music_stand/003.png`), `rec_picturex_0611__music_stand__001__png_33f7c765337e454fb69849d9bc1214dc` (`pictureY/0611/Music_stand/001.png`), `rec_picturex_0611__music_stand__002__png_67f259c9d314425da2f3bc1d28aadc9f` (`pictureY/0611/Music_stand/002.png`)
canonical_baselines: none planned; add only if a selected origin fails the §11 readability contract during fork review.
underfilled_reason: none at P0; adaptive coverage follows the approved per-category slot vocabulary.

## Subcategory Contract

- core_identity: adjustable music stand that supports sheet music
- must_keep: defining use, picture identity, visible support interfaces, and at least one real non-fixed joint.
- must_not_become: lectern, microphone stand
- image_evidence: pictureY/0611/Music_stand/003.png, pictureY/0611/Music_stand/001.png, pictureY/0611/Music_stand/002.png
- parent_evidence: rec_picturex_0611__music_stand__003__png_ba84304f87af408b86ed03dbf0841561, rec_picturex_0611__music_stand__001__png_33f7c765337e454fb69849d9bc1214dc, rec_picturex_0611__music_stand__002__png_67f259c9d314425da2f3bc1d28aadc9f

## Slot Candidates

| slot | candidate | diversity_axis | source_type | record/evidence | key parts/joints/helpers | status |
|---|---|---|---|---|---|---|
| origin_design | music_stand_003 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__music_stand__003__png_ba84304f87af408b86ed03dbf0841561` / `pictureY/0611/Music_stand/003.png` | base, height_knob, shaft, desk, tilt_knob, dynamic_indexed_name, height_knob_spin, shaft_slide, desk_tilt, tilt_knob_spin, dynamic_indexed_name, _hollow_tube, _desk_tray | built ✓ |
| origin_design | desktop_book_music_stand_001 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__music_stand__001__png_33f7c765337e454fb69849d9bc1214dc` / `pictureY/0611/Music_stand/001.png` | base, rotation_lock, turntable, lift_carriage, desk, dynamic_indexed_name, base_to_rotation_lock, base_to_turntable, turntable_to_lift, lift_to_desk, dynamic_indexed_name, _rounded_box, _rounded_frame, _hollow_guide | built ✓ |
| origin_design | desktop_folding_music_stand_002 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__music_stand__002__png_67f259c9d314425da2f3bc1d28aadc9f` / `pictureY/0611/Music_stand/002.png` | base, lower_yoke, upper_yoke, desk, dynamic_indexed_name, base_to_lower_yoke, lower_to_upper_yoke, upper_yoke_to_desk, dynamic_indexed_name, _rounded_box, _base_shape, _rectangular_tube, _lower_frame_shape, _fold_barrel_shape | built ✓ |
| desk_form | perforated panel | ① | forked_anchor | `rec_0611_music_stand_var_desk_form_perforated_panel` from `rec_picturex_0611__music_stand__003__png_ba84304f87af408b86ed03dbf0841561` | desk_tilt, desk, _desk_tray | planned |
| desk_form | wire frame | ① | forked_anchor | `rec_0611_music_stand_var_desk_form_wire_frame` from `rec_picturex_0611__music_stand__001__png_33f7c765337e454fb69849d9bc1214dc` | _rounded_frame, wood_frame, tilt axle passes through lower frame rail, lift_to_desk, base_to_turntable, base_to_rotation_lock, desk, base | planned |
| desk_form | folding-wing desk | ① | forked_anchor | `rec_0611_music_stand_var_desk_form_folding_wing_desk` from `rec_picturex_0611__music_stand__002__png_67f259c9d314425da2f3bc1d28aadc9f` | upper_yoke_to_desk, desk, _desk_shape, _desk_hinge_socket_shape, wood_desk, desktop_folding_music_stand_002, desk_hinge_socket, desk_hinge_barrel | planned |
| base | folding tripod | ② | forked_anchor | `rec_0611_music_stand_var_base_folding_tripod` from `rec_picturex_0611__music_stand__002__png_67f259c9d314425da2f3bc1d28aadc9f` | base_to_lower_yoke, base, _base_shape, base_hinge_pin, base_frame, base pin is captured by fold barrel, base hinge has retained pin engagement, _lower_frame_shape | planned |
| base | four-leg base | ② | forked_anchor | `rec_0611_music_stand_var_base_four_leg_base` from `rec_picturex_0611__music_stand__003__png_ba84304f87af408b86ed03dbf0841561` | base, base_hub, tilt_knob_body, leg_tube, height_knob_body, height clamp shoe supports the telescoping shaft, foot | planned |
| height_stages | three stages | N | forked_anchor | `rec_0611_music_stand_var_height_stages_three_stages` from `rec_picturex_0611__music_stand__001__png_33f7c765337e454fb69849d9bc1214dc` | lift shaft is deeply inserted at reference height | planned |
| page_retention | paired swing arms | ② | forked_anchor | `rec_0611_music_stand_var_page_retention_paired_swing_arms` from `rec_picturex_0611__music_stand__001__png_33f7c765337e454fb69849d9bc1214dc` | clip_arm | planned |
| page_retention | paired clips | ② | forked_anchor | `rec_0611_music_stand_var_page_retention_paired_clips` from `rec_picturex_0611__music_stand__001__png_33f7c765337e454fb69849d9bc1214dc` | clip_tip, clip_pivot, clip_neck, clip_arm | planned |

## Multiplicity / Copy Logic

- count_param: height_stages_count
- N samples: three stages
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
