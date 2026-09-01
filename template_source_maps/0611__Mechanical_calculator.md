# 0611 / Mechanical_calculator — template source map

pattern: mixed
parents: `rec_picturex_0611__mechanical_calculator__001__png_3c15e16013b5410bb59e4a17fa83e3fb` (`pictureY/0611/Mechanical_calculator/001.png`), `rec_picturex_0611__mechanical_calculator__002__png_bfb3ab073a6b4a5a8e31a0dc5139ecbe` (`pictureY/0611/Mechanical_calculator/002.png`), `rec_picturex_0611__mechanical_calculator__003__png_6c29e006127e48e78c56f9b8f384a8da` (`pictureY/0611/Mechanical_calculator/003.png`)
canonical_baselines: none planned; add only if a selected origin fails the §11 readability contract during fork review.
underfilled_reason: none at P0; adaptive coverage follows the approved per-category slot vocabulary.

## Subcategory Contract

- core_identity: hand-operated mechanical calculator with visible keys, register, and mechanical actuation
- must_keep: defining use, picture identity, visible support interfaces, and at least one real non-fixed joint.
- must_not_become: electronic calculator, typewriter
- image_evidence: pictureY/0611/Mechanical_calculator/001.png, pictureY/0611/Mechanical_calculator/002.png, pictureY/0611/Mechanical_calculator/003.png
- parent_evidence: rec_picturex_0611__mechanical_calculator__001__png_3c15e16013b5410bb59e4a17fa83e3fb, rec_picturex_0611__mechanical_calculator__002__png_bfb3ab073a6b4a5a8e31a0dc5139ecbe, rec_picturex_0611__mechanical_calculator__003__png_6c29e006127e48e78c56f9b8f384a8da

## Slot Candidates

| slot | candidate | diversity_axis | source_type | record/evidence | key parts/joints/helpers | status |
|---|---|---|---|---|---|---|
| origin_design | mechanical_calculator_001 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__mechanical_calculator__001__png_3c15e16013b5410bb59e4a17fa83e3fb` / `pictureY/0611/Mechanical_calculator/001.png` | frame, paper_roll, crank, f'function_key_{index}', f'slider_{index}', f'number_wheel_{index}', f'carry_wheel_{index}', f'key_{row}_{column}', paper_feed, crank_turn, f'function_press_{index}', f'slider_travel_{index}', f'number_index_{index}', f'carry_step_{index}' | built ✓ |
| origin_design | mechanical_calculator_002 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__mechanical_calculator__002__png_bfb3ab073a6b4a5a8e31a0dc5139ecbe` / `pictureY/0611/Mechanical_calculator/002.png` | housing, carry_rack, crank, grip, f'wheel_{wheel_index}', f'slider_{slider_index}', f'key_{row}_{column}', carry_rack_shift, crank_turn, grip_spin, f'wheel_{wheel_index}_spin', f'slider_{slider_index}_shift', f'key_{row}_{column}_press', _housing_shell | built ✓ |
| origin_design | mechanical_calculator_003 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__mechanical_calculator__003__png_6c29e006127e48e78c56f9b8f384a8da` / `pictureY/0611/Mechanical_calculator/003.png` | housing, crank, crank_grip, f'wheel_{index}', f'slider_{index}', f'key_{row}_{column}', housing_to_crank, crank_to_grip, f'housing_to_wheel_{index}', f'housing_to_slider_{index}', f'housing_to_key_{row}_{column}', _add_quad, _wedge_box | built ✓ |
| calculator_topology | pinwheel calculator | ② | forked_anchor | `rec_0611_mechanical_calculator_var_calculator_topology_pinwheel_calculato` from `rec_picturex_0611__mechanical_calculator__001__png_3c15e16013b5410bb59e4a17fa83e3fb` | _calculator_housing_mesh, mechanical_calculator_001 | built ✓ |
| calculator_topology | stepped-drum calculator | ② | forked_anchor | `rec_0611_mechanical_calculator_var_calculator_topology_stepped_drum_calcu` from `rec_picturex_0611__mechanical_calculator__002__png_bfb3ab073a6b4a5a8e31a0dc5139ecbe` | number_drum, mechanical_calculator_002 | built ✓ |
| calculator_topology | direct-key comptometer | ② | forked_anchor | `rec_0611_mechanical_calculator_var_calculator_topology_direct_key_comptom` from `rec_picturex_0611__mechanical_calculator__003__png_6c29e006127e48e78c56f9b8f384a8da` | f'housing_to_key_{row}_{column}', f'key_{row}_{column}', representative_key_is_seated, mechanical_calculator_003, key_stem, key_cap | built ✓ |
| calculator_topology | lever adding machine | ② | forked_anchor | `rec_0611_mechanical_calculator_var_calculator_topology_lever_adding_machi` from `rec_picturex_0611__mechanical_calculator__001__png_3c15e16013b5410bb59e4a17fa83e3fb` | _calculator_housing_mesh, mechanical_calculator_001, crank_arm | built ✓ |
| calculator_topology | rotary-dial calculator | ② | forked_anchor | `rec_0611_mechanical_calculator_var_calculator_topology_rotary_dial_calcul` from `rec_picturex_0611__mechanical_calculator__002__png_bfb3ab073a6b4a5a8e31a0dc5139ecbe` | mechanical_calculator_002 | built ✓ |
| register_form | exposed pinwheel bank | ③ | forked_anchor | `rec_0611_mechanical_calculator_var_register_form_exposed_pinwheel_bank` from `rec_picturex_0611__mechanical_calculator__003__png_6c29e006127e48e78c56f9b8f384a8da` | register_tower | built ✓ |
| register_form | enclosed window bank | ③ | forked_anchor | `rec_0611_mechanical_calculator_var_register_form_enclosed_window_bank` from `rec_picturex_0611__mechanical_calculator__003__png_6c29e006127e48e78c56f9b8f384a8da` | register_tower | built ✓ |
| register_form | traveling carriage register | ③ | forked_anchor | `rec_0611_mechanical_calculator_var_register_form_traveling_carriage_regis` from `rec_picturex_0611__mechanical_calculator__003__png_6c29e006127e48e78c56f9b8f384a8da` | register_tower, traveling_carriage_register_has_wide_overhanging_body | built ✓; selector sliders and guide tracks removed during human review |
| key_matrix | 10-key keypad | N | forked_anchor | `rec_0611_mechanical_calculator_var_key_matrix_10_key_keypad` from `rec_picturex_0611__mechanical_calculator__001__png_3c15e16013b5410bb59e4a17fa83e3fb` | f'key_press_{row}_{column}', f'key_{row}_{column}', f'function_key_{index}', key_support, key_stem, key_legend, key_cap, function_key | built ✓ |
| key_matrix | 50-key keyboard | N | forked_anchor | `rec_0611_mechanical_calculator_var_key_matrix_50_key_keyboard` from `rec_picturex_0611__mechanical_calculator__002__png_bfb3ab073a6b4a5a8e31a0dc5139ecbe` | f'key_{row}_{column}_press', f'key_{row}_{column}', rear key stem seats on key plate, keyboard_cover, key_plate, front key stem seats on key plate | built ✓ |
| key_matrix | 90-key full keyboard | N | forked_anchor | `rec_0611_mechanical_calculator_var_key_matrix_90_key_full_keyboard` from `rec_picturex_0611__mechanical_calculator__002__png_bfb3ab073a6b4a5a8e31a0dc5139ecbe` | f'key_{row}_{column}_press', f'key_{row}_{column}', rear key stem seats on key plate, keyboard_cover, key_plate, front key stem seats on key plate | built ✓ |
| drive | vertical pull lever | ② | forked_anchor | `rec_0611_mechanical_calculator_var_drive_vertical_pull_lever` from `rec_picturex_0611__mechanical_calculator__001__png_3c15e16013b5410bb59e4a17fa83e3fb` | crank_turn, crank_arm, crank, crank_shaft, crank_grip, crank_bearing | built ✓ |
| drive | folding side crank | ② | forked_anchor | `rec_0611_mechanical_calculator_var_drive_folding_side_crank` from `rec_picturex_0611__mechanical_calculator__002__png_bfb3ab073a6b4a5a8e31a0dc5139ecbe` | crank_turn, crank shaft passes through side bearing, crank, _crank_tube_mesh, handgrip seats on crank end pin, display_side_1, display_side_0, crank_tube | built ✓ |
| drive | front reciprocating lever | ② | forked_anchor | `rec_0611_mechanical_calculator_var_drive_front_reciprocating_lever` from `rec_picturex_0611__mechanical_calculator__001__png_3c15e16013b5410bb59e4a17fa83e3fb` | crank_turn, crank_arm, crank, crank_shaft, front_trim, front_lip, crank_grip, crank_bearing | built ✓ |
| clearing_motion | carriage reset lever | ② | forked_anchor | `rec_0611_mechanical_calculator_var_clearing_motion_carriage_reset_lever` from `rec_picturex_0611__mechanical_calculator__003__png_6c29e006127e48e78c56f9b8f384a8da` | mechanism_floor, crank_arm | built ✓; selector sliders and guide tracks removed during human review |
| clearing_motion | rotating zeroing knob | ② | forked_anchor | `rec_0611_mechanical_calculator_var_clearing_motion_rotating_zeroing_knob` from `rec_picturex_0611__mechanical_calculator__003__png_6c29e006127e48e78c56f9b8f384a8da` | mechanism_floor | built ✓ |

## Multiplicity / Copy Logic

- count_param: key_matrix_count
- N samples: 10-key keypad, 50-key keyboard, 90-key full keyboard
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
- neighbor categories (electronic calculator, typewriter): excluded.
- failed or unfit candidates will be appended with one-line reasons after 2–3 attempts.
