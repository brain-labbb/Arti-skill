# 0611 / sliding_turnstile — template source map

pattern: mixed
parents: `rec_picturex_0611__sliding_turnstile__004__png_8fae6907fc244ca7a6eca695e85859a5` (`pictureY/0611/sliding_turnstile/004.png`), `rec_picturex_0611__sliding_turnstile__001__png_772b0dd0d42d49c69b0b1b82171e31de` (`pictureY/0611/sliding_turnstile/001.png`), `rec_picturex_0611__sliding_turnstile__002__png_a2915edfe41b4d8a9471e7ba1aa92f0e` (`pictureY/0611/sliding_turnstile/002.png`), `rec_picturex_0611__sliding_turnstile__003__png_2a8af54613b6444f808d94553318b240` (`pictureY/0611/sliding_turnstile/003.png`)
canonical_baselines: none planned; add only if a selected origin fails the §11 readability contract during fork review.
underfilled_reason: none at P0; adaptive coverage follows the approved per-category slot vocabulary.

## Subcategory Contract

- core_identity: pedestrian access-control gate with moving barriers
- must_keep: defining use, picture identity, visible support interfaces, and at least one real non-fixed joint.
- must_not_become: tripod turnstile, ordinary fence
- image_evidence: pictureY/0611/sliding_turnstile/004.png, pictureY/0611/sliding_turnstile/001.png, pictureY/0611/sliding_turnstile/002.png, pictureY/0611/sliding_turnstile/003.png
- parent_evidence: rec_picturex_0611__sliding_turnstile__004__png_8fae6907fc244ca7a6eca695e85859a5, rec_picturex_0611__sliding_turnstile__001__png_772b0dd0d42d49c69b0b1b82171e31de, rec_picturex_0611__sliding_turnstile__002__png_a2915edfe41b4d8a9471e7ba1aa92f0e, rec_picturex_0611__sliding_turnstile__003__png_2a8af54613b6444f808d94553318b240

## Category-wide Base Normalization

- Human-approved 2026-07-12: all 4 origins and 13 variants replace exposed rail, H-frame, spine, channel, crossbar, or narrow-beam foundations with flat backing plates.
- Plate vocabulary is intentionally varied across records: compact/deep, graphite/satin/brushed/bright-metal, accessible-lane, modular-bank, radiused four-lane, and lane-count-specific footprints.
- Individual cabinet feet, module bridges, part trees, joint graphs, motion limits, and each variant's primary axis remain unchanged.
- Every edited record's `run_tests()` asserts its named plate and rejects the former rail/beam visual names; all 17 records compile cleanly.

## Slot Candidates

| slot | candidate | diversity_axis | source_type | record/evidence | key parts/joints/helpers | status |
|---|---|---|---|---|---|---|
| origin_design | dual_lane_glass_speed_gate | ①/②/③ observed | origin_anchor | `rec_picturex_0611__sliding_turnstile__004__png_8fae6907fc244ca7a6eca695e85859a5` / `pictureY/0611/sliding_turnstile/004.png` | floor_anchor, dynamic_indexed_name, dynamic_indexed_name, _housing_shape, _top_cap_shape, _glass_panel_shape, _add_housing_visuals, _add_gate_panel | built ✓ |
| origin_design | five_pedestal_speed_gate | ①/②/③ observed | origin_anchor | `rec_picturex_0611__sliding_turnstile__001__png_772b0dd0d42d49c69b0b1b82171e31de` / `pictureY/0611/sliding_turnstile/001.png` | mounting_frame, dynamic_indexed_name, dynamic_indexed_name, _mounting_plate_shape, _pedestal_shell_shape, _glass_panel_shape | built ✓ |
| origin_design | sliding_turnstile_002 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__sliding_turnstile__002__png_a2915edfe41b4d8a9471e7ba1aa92f0e` / `pictureY/0611/sliding_turnstile/002.png` | pedestal_frame, _capsule, _slotted_cap, _reader_cap, _glass_panel, _add_panel_visuals | built ✓ |
| origin_design | sliding_turnstile_003 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__sliding_turnstile__003__png_2a8af54613b6444f808d94553318b240` / `pictureY/0611/sliding_turnstile/003.png` | foundation, dynamic_indexed_name, dynamic_indexed_name, _capsule, _pedestal_shell, _top_cap, _trim_ring, _panel_shape, _sensor_pad | built ✓ |
| panel_count | single | N | forked_anchor | `rec_0611_sliding_turnstile_var_panel_count_single` from `rec_picturex_0611__sliding_turnstile__004__png_8fae6907fc244ca7a6eca695e85859a5` | _glass_panel_shape, _add_gate_panel, service_panel, glass_panel, barrier_3, barrier_2, barrier_1, barrier_0 | planned |
| lane_support_topology | shared center-divider bank | ① | origin_anchor | `rec_picturex_0611__sliding_turnstile__001__png_772b0dd0d42d49c69b0b1b82171e31de` | mounting_frame, pedestal_{i}, frame_to_pedestal_{i} | built ✓ |
| lane_support_topology | paired independent lane modules | ① | forked_anchor | `rec_0611_sliding_turnstile_var_panel_count_paired` from `rec_picturex_0611__sliding_turnstile__001__png_772b0dd0d42d49c69b0b1b82171e31de` | lane_module_{i}, mounting_bridge_{i}, left/right_pedestal_{i}, left/right_barrier_{i} | built ✓ |
| barrier_planform | chamfered rectangular leaf | ③ | origin_anchor | `rec_picturex_0611__sliding_turnstile__002__png_a2915edfe41b4d8a9471e7ba1aa92f0e` | _glass_panel, glass_panel, leading_edge | built ✓ |
| barrier_planform | sloped-shoulder trapezoidal wing | ③ | forked_anchor | `rec_0611_sliding_turnstile_var_barrier_planform_sloped_shoulder` from `rec_picturex_0611__sliding_turnstile__002__png_a2915edfe41b4d8a9471e7ba1aa92f0e` | sloped_shoulder_glass, root_spine, lower_guide | built ✓ (`gpt-5.6-sol`, high; human-review refill) |
| panel_construction | frameless transparent wing | ③ | origin_anchor | `rec_picturex_0611__sliding_turnstile__004__png_8fae6907fc244ca7a6eca695e85859a5` | _glass_panel_shape, glass_panel, carrier_rail | built ✓ |
| barrier_support_topology | full-height hinge mullion with bearings concealed inside the housing | ① | forked_anchor | `rec_0611_sliding_turnstile_var_barrier_support_full_height_hinge_column` from `rec_picturex_0611__sliding_turnstile__004__png_8fae6907fc244ca7a6eca695e85859a5` | hinge_mullion, mullion_glass_channel, housing_shell, top_cap | rebuilt ✓ (`gpt-5.6-sol`, high; 2026-07-12 human-review refill, exposed upper/lower bearing blocks removed) |
| panel_form | waist-high glass | ③ | forked_anchor | `rec_0611_sliding_turnstile_var_panel_form_waist_high_glass` from `rec_picturex_0611__sliding_turnstile__003__png_2a8af54613b6444f808d94553318b240` | _panel_shape, lane 1 glass leaves form the closed barrier, lane 0 glass leaves form the closed barrier, glass | planned |
| panel_form | full-height glass | ③ | forked_anchor | `rec_0611_sliding_turnstile_var_panel_form_full_height_glass` from `rec_picturex_0611__sliding_turnstile__003__png_2a8af54613b6444f808d94553318b240` | _panel_shape, lane 1 glass leaves form the closed barrier, lane 0 glass leaves form the closed barrier, glass | planned |
| motion_family | vertical-axis swing wing | ② | origin_anchor | `rec_picturex_0611__sliding_turnstile__004__png_8fae6907fc244ca7a6eca695e85859a5` | _add_gate_panel, pivot_barrel, housing_to_barrier revolute joints | built ✓ |
| motion_family | horizontal retracting leaf | ② | origin_anchor | `rec_picturex_0611__sliding_turnstile__003__png_2a8af54613b6444f808d94553318b240` | gate_*_slide, drive_shoe, side_guide_* | built ✓ |
| reader_module | flush top sensor pad | ① | origin_anchor | `rec_picturex_0611__sliding_turnstile__003__png_2a8af54613b6444f808d94553318b240` | _sensor_pad, top_sensor, front_slot | built ✓ |
| reader_module | raised biometric terminal | ① | forked_anchor | `rec_0611_sliding_turnstile_var_reader_module_raised_biometric` from `rec_picturex_0611__sliding_turnstile__003__png_2a8af54613b6444f808d94553318b240` | reader_plinth, reader_body, biometric_screen, camera_aperture, status_light | built ✓ (`gpt-5.6-sol`, high) |
| lane_layout | equal-width twin lanes | ① | origin_anchor | `rec_picturex_0611__sliding_turnstile__004__png_8fae6907fc244ca7a6eca695e85859a5` | HOUSING_X, housing_0..2, barrier_0..3 | built ✓ |
| lane_layout | standard plus accessible wide lane | ① | forked_anchor | `rec_0611_sliding_turnstile_var_lane_layout_accessible_wide` from `rec_picturex_0611__sliding_turnstile__004__png_8fae6907fc244ca7a6eca695e85859a5` | asymmetric HOUSING_X, length-parameterized glass panels, four swing joints | built ✓ (`gpt-5.6-sol`, high) |
| lane_count | 1 | N | forked_anchor | `rec_0611_sliding_turnstile_var_lane_count_1` from `rec_picturex_0611__sliding_turnstile__004__png_8fae6907fc244ca7a6eca695e85859a5` | second lane glass leaves retain a narrow center gap, first lane glass leaves retain a narrow center gap, dual_lane_glass_speed_gate, barrier_1 | planned |
| lane_count | 2 | N | forked_anchor | `rec_0611_sliding_turnstile_var_lane_count_2` from `rec_picturex_0611__sliding_turnstile__004__png_8fae6907fc244ca7a6eca695e85859a5` | second lane glass leaves retain a narrow center gap, first lane glass leaves retain a narrow center gap, dual_lane_glass_speed_gate, barrier_2 | planned |
| lane_count | 4 | N | forked_anchor | `rec_0611_sliding_turnstile_var_lane_count_4` from `rec_picturex_0611__sliding_turnstile__003__png_2a8af54613b6444f808d94553318b240` | lane 1 glass leaves form the closed barrier, lane 1 clears when both leaves retract, lane 0 glass leaves form the closed barrier, lane 0 clears when both leaves retract | planned |
| pedestal_form | round | ③ | forked_anchor | `rec_0611_sliding_turnstile_var_pedestal_form_round` from `rec_picturex_0611__sliding_turnstile__003__png_2a8af54613b6444f808d94553318b240` | _pedestal_shell, lane 1 glass leaves form the closed barrier, lane 0 glass leaves form the closed barrier | planned |
| pedestal_form | slim capsule | ③ | forked_anchor | `rec_0611_sliding_turnstile_var_pedestal_form_slim_capsule` from `rec_picturex_0611__sliding_turnstile__003__png_2a8af54613b6444f808d94553318b240` | _pedestal_shell, _capsule, lane 1 glass leaves form the closed barrier, lane 0 glass leaves form the closed barrier | planned |

## Multiplicity / Copy Logic

- count_param: lane_count_count, panel_count_count, lane_module_count
- N samples: single and opposed-pair leaves; 1, 2, and 4 lanes; shared-bank and 4 paired independent lane modules
- suggested N_range: bounded by accepted source samples and downstream compile budget.
- copied object / naming / placement / joint policy: shared helper, `name_{i}`, regular placement, uniform joint policy; exact names resolve from accepted variants.

## Six-Axis Diversity Record

| axis | treatment | values / range / reason |
|---|---|---|
| ① skeleton / structural topology | source-backed | shared center-divider bank; paired independent lane modules; equal/mixed-width lane layouts; flush/raised reader modules; short carrier/full-height hinge-column support |
| ② joint / mechanism type | source-backed | vertical-axis swing wings and direct horizontal retracting leaves from origins |
| ③ primary form family | source-backed | rectangular/sloped-shoulder planar boundaries; frameless panel construction; round/slim pedestal forms |
| ④ surface decoration | record_only / world_knowledge_extrapolation | host-conformal seams, ribs, labels, bezels; backing-plate finish/edge treatment only |
| ⑤ proportion / size / travel | record_only | origin ranges plus modest safe companion tuning |
| ⑥ material / palette / finish | record_only | origin materials plus realistic companion colorways |

## Compatibility Probes

| probe | record_id | combined axes | risk tested | result |
|---|---|---|---|---|
| none at P0 | — | — | add only if cross-family interface review finds a real risk | — |

## Blocked / Excluded

- ④/⑤/⑥-only forks: excluded; these do not count as candidate anchors.
- neighbor categories (tripod turnstile, ordinary fence): excluded.
- single-stage single-rail conversion: rejected at human variant review; removed in favor of mainstream visible product slots.
- nested two-panel cassette: rejected at human variant review; removed as unnecessarily complex barrier segmentation.
- vertical-drop glass mechanism: rejected at human variant review; removed as an unrepresentative mechanism for this pool.
- two-stage telescopic carrier: rejected at human variant review; removed as over-complex relative to the origin vocabulary.
- tapered fan flap: rejected at human variant review; removed because the sector-like silhouette did not read as a convincing commercial speed-gate wing.
- four-sided perimeter sash: rejected at human variant review; removed because the over-framed leaf weakened the speed-gate identity.
- failed or unfit candidates will be appended with one-line reasons after 2–3 attempts.
