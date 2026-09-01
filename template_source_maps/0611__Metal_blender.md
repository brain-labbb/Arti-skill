# 0611 / Metal_blender — template source map

pattern: mixed
parents: `rec_picturex_0611__metal_blender__001__png_5eda8fb0cd7845e4a6db2e727f235d1e` (`pictureY/0611/Metal_blender/001.png`)
canonical_baselines: none planned; add only if a selected origin fails the §11 readability contract during fork review.
underfilled_reason: none at P0; adaptive coverage follows the approved per-category slot vocabulary.

## Subcategory Contract

- core_identity: manual metal bending brake or forming machine; Blender is a source-name typo for bender
- must_keep: defining use, picture identity, visible support interfaces, and at least one real non-fixed joint.
- must_not_become: kitchen blender, powered stamping press
- image_evidence: pictureY/0611/Metal_blender/001.png
- parent_evidence: rec_picturex_0611__metal_blender__001__png_5eda8fb0cd7845e4a6db2e727f235d1e

## Slot Candidates

| slot | candidate | diversity_axis | source_type | record/evidence | key parts/joints/helpers | status |
|---|---|---|---|---|---|---|
| origin_design | fb4_bench_bending_brake | ①/②/③ observed | origin_anchor | `rec_picturex_0611__metal_blender__001__png_5eda8fb0cd7845e4a6db2e727f235d1e` / `pictureY/0611/Metal_blender/001.png` | frame, clamp_lever, feed_handwheel, forming_crank, clamp_lever_pivot, feed_handwheel_spin, forming_crank_pivot, _mesh, _annular_tube_x, _hex_prism_x | built ✓ |
| forming_topology | V-die press brake | ② | forked_anchor | `rec_0611_metal_blender_var_forming_topology_v_die_press_brake` from `rec_picturex_0611__metal_blender__001__png_5eda8fb0cd7845e4a6db2e727f235d1e` | forming_crank_pivot, forming_crank, forming_bearing, forming shaft retains insertion in collar, forming shaft remains captured by bearing, fixed_die, fb4_bench_bending_brake | built ✓ |
| forming_topology | three-roll slip roller | ② | forked_anchor | `rec_0611_metal_blender_var_forming_topology_three_roll_slip_rolle` from `rec_picturex_0611__metal_blender__001__png_5eda8fb0cd7845e4a6db2e727f235d1e` | wheel_hub, forming_crank_pivot, forming_crank, forming_bearing, forming shaft remains captured by bearing, forming shaft retains insertion in collar, feed_bearing, feed screw remains captured by bearing | built ✓ |
| forming_topology | bead roller | ② | forked_anchor | `rec_0611_metal_blender_var_forming_topology_bead_roller` from `rec_picturex_0611__metal_blender__001__png_5eda8fb0cd7845e4a6db2e727f235d1e` | wheel_hub, forming_crank_pivot, forming_crank, forming_bearing, forming shaft remains captured by bearing, forming shaft retains insertion in collar, feed_bearing, feed screw remains captured by bearing | built ✓ |
| forming_topology | box-and-pan finger brake | ② | forked_anchor | `rec_0611_metal_blender_var_forming_topology_box_and_pan_finger_br` from `rec_picturex_0611__metal_blender__001__png_5eda8fb0cd7845e4a6db2e727f235d1e` | forming_crank_pivot, forming_crank, forming_bearing, forming shaft retains insertion in collar, forming shaft remains captured by bearing, fb4_bench_bending_brake | built ✓ |
| clamping | eccentric cam clamp | ② | forked_anchor | `rec_0611_metal_blender_var_clamping_eccentric_cam_clamp` from `rec_picturex_0611__metal_blender__001__png_5eda8fb0cd7845e4a6db2e727f235d1e` | clamp_lever_pivot, clamp_lever, clamp_pivot_bracket, clamp pivot retains bearing insertion, clamp pivot remains captured in bracket | built ✓ |
| clamping | screw-beam clamp | ② | forked_anchor | `rec_0611_metal_blender_var_clamping_screw_beam_clamp` from `rec_picturex_0611__metal_blender__001__png_5eda8fb0cd7845e4a6db2e727f235d1e` | clamp_lever_pivot, clamp_lever, screw_core, feed screw retains insertion in collar, feed screw remains captured by bearing, clamp_pivot_bracket, clamp pivot retains bearing insertion, clamp pivot remains captured in bracket | built ✓ |
| feed_motion | rack handwheel feed | ② | forked_anchor | `rec_0611_metal_blender_var_feed_motion_rack_handwheel_feed` from `rec_picturex_0611__metal_blender__001__png_5eda8fb0cd7845e4a6db2e727f235d1e` | feed_handwheel_spin, feed_handwheel, forming_crank_pivot, clamp_lever_pivot, frame, pivot_pin, feed_bearing, feed screw retains insertion in collar | built ✓ |
| feed_motion | lead-screw carriage feed | ② | forked_anchor | `rec_0611_metal_blender_var_feed_motion_lead_screw_carriage_feed` from `rec_picturex_0611__metal_blender__001__png_5eda8fb0cd7845e4a6db2e727f235d1e` | feed_handwheel_spin, forming_crank_pivot, clamp_lever_pivot, feed_handwheel, feed screw retains insertion in collar, feed screw remains captured by bearing, screw_core, pivot_pin | built ✓ |
| die_profile | round bead dies | ③ | forked_anchor | `rec_0611_metal_blender_var_die_profile_round_bead_dies` from `rec_picturex_0611__metal_blender__001__png_5eda8fb0cd7845e4a6db2e727f235d1e` | fixed_die | built ✓ |
| die_profile | sharp V dies | ③ | forked_anchor | `rec_0611_metal_blender_var_die_profile_sharp_v_dies` from `rec_picturex_0611__metal_blender__001__png_5eda8fb0cd7845e4a6db2e727f235d1e` | fixed_die | built ✓ |

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
- neighbor categories (kitchen blender, powered stamping press): excluded.
- failed or unfit candidates will be appended with one-line reasons after 2–3 attempts.
