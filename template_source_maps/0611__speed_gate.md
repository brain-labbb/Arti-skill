# 0611 / speed_gate — template source map

pattern: mixed
parents: `rec_picturex_0611__speed_gate__001__png_54981d2cd9e84316b2a4769f9ba6a943` (`pictureY/0611/speed_gate/001.png`), `rec_picturex_0611__speed_gate__002__png_0a454d17b7944c38a11fa734e37dfd49` (`pictureY/0611/speed_gate/002.png`)
canonical_baselines: none planned; add only if a selected origin fails the §11 readability contract during fork review.
underfilled_reason: none at P0; adaptive coverage follows the approved per-category slot vocabulary.

## Subcategory Contract

- core_identity: pedestrian access-control gate with moving barriers
- must_keep: defining use, picture identity, visible support interfaces, and at least one real non-fixed joint.
- must_not_become: tripod turnstile, ordinary fence
- image_evidence: pictureY/0611/speed_gate/001.png, pictureY/0611/speed_gate/002.png
- parent_evidence: rec_picturex_0611__speed_gate__001__png_54981d2cd9e84316b2a4769f9ba6a943, rec_picturex_0611__speed_gate__002__png_0a454d17b7944c38a11fa734e37dfd49

## Slot Candidates

| slot | candidate | diversity_axis | source_type | record/evidence | key parts/joints/helpers | status |
|---|---|---|---|---|---|---|
| origin_design | speed_gate_001 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__speed_gate__001__png_54981d2cd9e84316b2a4769f9ba6a943` / `pictureY/0611/speed_gate/001.png` | lane_frame, dynamic_indexed_name, _hex_profile, _tapered_pedestal, _hex_prism | built ✓ |
| origin_design | speed_gate_002 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__speed_gate__002__png_0a454d17b7944c38a11fa734e37dfd49` / `pictureY/0611/speed_gate/002.png` | mounting_frame, dynamic_indexed_name, dynamic_indexed_name, _rounded_box, _pedestal_shell, _top_cap, _reader_pad, _screen_housing, _wing_panel, _mounting_frame | built ✓ |
| wing_count | single | N | forked_anchor | `rec_0611_speed_gate_var_wing_count_single` from `rec_picturex_0611__speed_gate__001__png_54981d2cd9e84316b2a4769f9ba6a943` | right lane wings preserve the narrow closed seam, right lane glass leaves share the reference wing height, left lane wings preserve the narrow closed seam, left lane glass leaves share the reference wing height, glass_panel | planned |
| wing_count | paired | N | forked_anchor | `rec_0611_speed_gate_var_wing_count_paired` from `rec_picturex_0611__speed_gate__002__png_0a454d17b7944c38a11fa734e37dfd49` | _wing_panel, acrylic_panel | planned |
| wing_count | four-wing lane | N | forked_anchor | `rec_0611_speed_gate_var_wing_count_four_wing_lane` from `rec_picturex_0611__speed_gate__001__png_54981d2cd9e84316b2a4769f9ba6a943` | lane_frame, right lane wings preserve the narrow closed seam, right lane glass leaves share the reference wing height, left lane wings preserve the narrow closed seam, left lane glass leaves share the reference wing height, glass_panel | planned |
| wing_form | waist-high | ③ | forked_anchor | `rec_0611_speed_gate_var_wing_form_waist_high` from `rec_picturex_0611__speed_gate__002__png_0a454d17b7944c38a11fa734e37dfd49` | _wing_panel, acrylic_panel | planned |
| wing_form | full-height | ③ | forked_anchor | `rec_0611_speed_gate_var_wing_form_full_height` from `rec_picturex_0611__speed_gate__001__png_54981d2cd9e84316b2a4769f9ba6a943` | right lane glass leaves share the reference wing height, left lane glass leaves share the reference wing height, right lane wings preserve the narrow closed seam, left lane wings preserve the narrow closed seam, glass_panel | planned |
| wing_form | curved glass | ③ | forked_anchor | `rec_0611_speed_gate_var_wing_form_curved_glass` from `rec_picturex_0611__speed_gate__001__png_54981d2cd9e84316b2a4769f9ba6a943` | right lane glass leaves share the reference wing height, left lane glass leaves share the reference wing height, glass_panel, glass_clamp, right lane wings preserve the narrow closed seam, left lane wings preserve the narrow closed seam | planned |
| gate_motion | swing | ② | forked_anchor | `rec_0611_speed_gate_var_gate_motion_swing` from `rec_picturex_0611__speed_gate__001__png_54981d2cd9e84316b2a4769f9ba6a943` | speed_gate_001, pivot_bar | planned |
| gate_motion | vertical retract | ② | forked_anchor | `rec_0611_speed_gate_var_gate_motion_vertical_retract` from `rec_picturex_0611__speed_gate__001__png_54981d2cd9e84316b2a4769f9ba6a943` | speed_gate_001, pivot_bar | planned |
| gate_motion | telescoping retract | ② | forked_anchor | `rec_0611_speed_gate_var_gate_motion_telescoping_retract` from `rec_picturex_0611__speed_gate__001__png_54981d2cd9e84316b2a4769f9ba6a943` | speed_gate_001, pivot_bar | planned |
| lane_count | 1 | N | forked_anchor | `rec_0611_speed_gate_var_lane_count_1` from `rec_picturex_0611__speed_gate__002__png_0a454d17b7944c38a11fa734e37dfd49` | lane 1 closed leaves retain anti pinch gap, lane 0 closed leaves retain anti pinch gap | planned |
| lane_count | 2 | N | forked_anchor | `rec_0611_speed_gate_var_lane_count_2` from `rec_picturex_0611__speed_gate__002__png_0a454d17b7944c38a11fa734e37dfd49` | lane 1 closed leaves retain anti pinch gap, lane 0 closed leaves retain anti pinch gap | planned |
| lane_count | 3 | N | forked_anchor | `rec_0611_speed_gate_var_lane_count_3` from `rec_picturex_0611__speed_gate__002__png_0a454d17b7944c38a11fa734e37dfd49` | lane 1 closed leaves retain anti pinch gap, lane 0 closed leaves retain anti pinch gap | planned |
| pedestal_form | slim rounded | ③ | forked_anchor | `rec_0611_speed_gate_var_pedestal_form_slim_rounded` from `rec_picturex_0611__speed_gate__002__png_0a454d17b7944c38a11fa734e37dfd49` | _rounded_box, _pedestal_shell, three_base_plates | planned |

## Multiplicity / Copy Logic

- count_param: lane_count_count, wing_count_count
- N samples: single, paired, four-wing lane, 1, 2, 3
- suggested N_range: bounded by accepted source samples and downstream compile budget.
- copied object / naming / placement / joint policy: shared helper, `name_{i}`, regular placement, uniform joint policy; exact names resolve from accepted variants.

## Six-Axis Diversity Record

| axis | treatment | values / range / reason |
|---|---|---|
| ① skeleton / structural topology | source-backed | origin rows plus planned ① candidates |
| ② joint / mechanism type | source-backed | origin rows plus planned ② candidates |
| ③ primary form family | source-backed | origin rows plus planned ③ candidates |
| ④ surface decoration | record_only / world_knowledge_extrapolation | host-conformal seams, ribs, labels, bezels, and structurally safe floor-plate perforations only |
| ⑤ proportion / size / travel | record_only | origin ranges plus modest safe companion tuning |
| ⑥ material / palette / finish | record_only | origin materials plus realistic companion colorways |

## Floor Plate Correction (in-place; no new records)

All 15 existing origins/variants now use one continuous floor plate instead of separate foot plates plus frame/tie rails. These treatments are axis-local surface construction and do **not** add a new variant slot. Every plate is sized from `PEDESTAL_X` with a conservative cabinet/wing margin; perforations are loop-generated only in gaps between adjacent pedestal footprints.

| record | floor plate treatment |
|---|---|
| `rec_picturex_0611__speed_gate__001__png_54981d2cd9e84316b2a4769f9ba6a943` | rounded solid plate |
| `rec_picturex_0611__speed_gate__002__png_0a454d17b7944c38a11fa734e37dfd49` | chamfered plate + round holes |
| `rec_0611_speed_gate_var_wing_count_single` | capsule plate + long slots |
| `rec_0611_speed_gate_var_wing_count_paired` | octagonal plate + diamond holes |
| `rec_0611_speed_gate_var_wing_count_four_wing_lane` | trapezoid plate + hex holes |
| `rec_0611_speed_gate_var_wing_form_waist_high` | rounded plate + long slots |
| `rec_0611_speed_gate_var_wing_form_full_height` | chamfered plate + diamond holes |
| `rec_0611_speed_gate_var_wing_form_curved_glass` | capsule plate + round holes |
| `rec_0611_speed_gate_var_gate_motion_swing` | octagonal plate + long slots |
| `rec_0611_speed_gate_var_gate_motion_vertical_retract` | rounded plate + hex holes |
| `rec_0611_speed_gate_var_gate_motion_telescoping_retract` | trapezoid plate + round holes |
| `rec_0611_speed_gate_var_lane_count_1` | capsule plate + diamond holes |
| `rec_0611_speed_gate_var_lane_count_2` | chamfered plate + long slots |
| `rec_0611_speed_gate_var_lane_count_3` | rounded plate + round holes |
| `rec_0611_speed_gate_var_pedestal_form_slim_rounded` | capsule plate + hex holes |

## Compatibility Probes

| probe | record_id | combined axes | risk tested | result |
|---|---|---|---|---|
| none at P0 | — | — | add only if cross-family interface review finds a real risk | — |

## Blocked / Excluded

- ④/⑤/⑥-only forks: excluded; these do not count as candidate anchors.
- neighbor categories (tripod turnstile, ordinary fence): excluded.
- failed or unfit candidates will be appended with one-line reasons after 2–3 attempts.
