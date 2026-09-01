# 0611 / tube_cutter — template source map

pattern: mixed
parents: `rec_picturex_0611__tube_cutter__002__png_20317f26aa01480f9571cba45f69fada` (`pictureY/0611/tube_cutter/002.png`), `rec_picturex_0611__tube_cutter__003__png_8079a52cd8014a2181f7ab795e321303` (`pictureY/0611/tube_cutter/003.png`), `rec_picturex_0611__tube_cutter__001__png_2cdbda5a722043988b55f5c37497d26c` (`pictureY/0611/tube_cutter/001.png`)
canonical_baselines: none planned; add only if a selected origin fails the §11 readability contract during fork review.
underfilled_reason: none at P0; adaptive coverage follows the approved per-category slot vocabulary.

## Subcategory Contract

- core_identity: real-world tube cutter retaining its defining use and articulation
- must_keep: defining use, picture identity, visible support interfaces, and at least one real non-fixed joint.
- must_not_become: neighbor category, decorative static prop
- image_evidence: pictureY/0611/tube_cutter/002.png, pictureY/0611/tube_cutter/003.png, pictureY/0611/tube_cutter/001.png
- parent_evidence: rec_picturex_0611__tube_cutter__002__png_20317f26aa01480f9571cba45f69fada, rec_picturex_0611__tube_cutter__003__png_8079a52cd8014a2181f7ab795e321303, rec_picturex_0611__tube_cutter__001__png_2cdbda5a722043988b55f5c37497d26c

## Slot Candidates

| slot | candidate | diversity_axis | source_type | record/evidence | key parts/joints/helpers | status |
|---|---|---|---|---|---|---|
| origin_design | compact_c_frame_tube_cutter | ①/②/③ observed | origin_anchor | `rec_picturex_0611__tube_cutter__002__png_20317f26aa01480f9571cba45f69fada` / `pictureY/0611/tube_cutter/002.png` | frame, cutter_carriage, cutting_wheel, adjustment_screw, dynamic_indexed_name, adjustment_screw_spin, cutter_carriage_slide, cutting_wheel_spin, dynamic_indexed_name, _frame_shape, _side_plate_shape, _carriage_shape | built ✓ |
| origin_design | blue_c_frame_tube_cutter | ①/②/③ observed | origin_anchor | `rec_picturex_0611__tube_cutter__003__png_8079a52cd8014a2181f7ab795e321303` / `pictureY/0611/tube_cutter/003.png` | frame, carriage, cutter_wheel, knob, dynamic_indexed_name, carriage_slide, cutter_spin, guide_roller_0_spin, guide_roller_1_spin, knob_turn, _y_cylinder, _c_ring, _build_frame_body, _build_side_plate | built ✓ |
| origin_design | compact_handheld_tube_cutter | ①/②/③ observed | origin_anchor | `rec_picturex_0611__tube_cutter__001__png_2cdbda5a722043988b55f5c37497d26c` / `pictureY/0611/tube_cutter/001.png` | frame, adjustment_knob, cutter_carriage, cutting_wheel, dynamic_indexed_name, adjustment_knob_spin, cutter_feed, cutting_wheel_spin, dynamic_indexed_name, _soften, _frame_shape, _collar_shape, _side_grip_shape, _carriage_shape | built ✓ |
| roller_count | 3 | N | forked_anchor | `rec_0611_tube_cutter_var_roller_count_3` from `rec_picturex_0611__tube_cutter__002__png_20317f26aa01480f9571cba45f69fada` | cutting_wheel_spin, cutting_wheel, source-open throat remains clear between roller and cutter, roller_hub, roller_flange_1, roller_flange_0, roller_body, cutting wheel is opposed beneath the guide roller pair | planned |
| roller_count | 4 | N | forked_anchor | `rec_0611_tube_cutter_var_roller_count_4` from `rec_picturex_0611__tube_cutter__003__png_8079a52cd8014a2181f7ab795e321303` | guide_roller_1_spin, guide_roller_0_spin, _build_roller, cutter_wheel, _build_cutting_wheel, roller_tread, roller_pin, cutter overlaps second guide roller footprint | planned |
| feed | ratchet | ② | forked_anchor | `rec_0611_tube_cutter_var_feed_ratchet` from `rec_picturex_0611__tube_cutter__001__png_2cdbda5a722043988b55f5c37497d26c` | cutter_feed, feed_screw | planned |
| feed | quick release | ② | forked_anchor | `rec_0611_tube_cutter_var_feed_quick_release` from `rec_picturex_0611__tube_cutter__003__png_8079a52cd8014a2181f7ab795e321303` | closed feed narrows circular throat | planned |
| frame | open C-frame | ① | forked_anchor | `rec_0611_tube_cutter_var_frame_open_c_frame` from `rec_picturex_0611__tube_cutter__002__png_20317f26aa01480f9571cba45f69fada` | frame, _frame_shape, compact_c_frame_tube_cutter, frame_body, roller_body, carriage_body, source-open throat remains clear between roller and cutter | planned |
| frame | chain cutter | ① | forked_anchor | `rec_0611_tube_cutter_var_frame_chain_cutter` from `rec_picturex_0611__tube_cutter__001__png_2cdbda5a722043988b55f5c37497d26c` | frame, _frame_shape, carriage stem retains deep insertion in frame, c_frame, cutter_feed, cutter_carriage, cutter_pin_cap, cutter carriage is supported by handle guide bushing | planned |
| secondary | fold-out reamer | ② | forked_anchor | `rec_0611_tube_cutter_var_secondary_fold_out_reamer` from `rec_picturex_0611__tube_cutter__002__png_20317f26aa01480f9571cba45f69fada` | adjustment_screw_spin, cutter_carriage_slide, cutting_wheel_spin, dynamic_indexed_name, frame | planned |
| wheel_module | quick-change cutter wheel | ① | forked_anchor | `rec_0611_tube_cutter_var_wheel_module_quick_change_cutter_wheel` from `rec_picturex_0611__tube_cutter__001__png_2cdbda5a722043988b55f5c37497d26c` | cutting_wheel_spin, cutting_wheel, _guide_roller_shape, wheel_hub, cutting wheel opposes upper guide roller, cutting wheel opposes lower guide roller, cutting wheel is captured on carriage axle, guide roller spacing matches compact throat | planned |

## Multiplicity / Copy Logic

- count_param: roller_count_count
- N samples: 3, 4
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
