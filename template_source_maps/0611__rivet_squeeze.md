# 0611 / rivet_squeeze — template source map

pattern: mixed
parents: `rec_picturex_0611__rivet_squeeze__002__png_7a78c071827541ed822cff3fb676aa83` (`pictureY/0611/rivet_squeeze/002.png`), `rec_picturex_0611__rivet_squeeze__003__png_7c15724dd1fc4e4a9c3ba1c34794454c` (`pictureY/0611/rivet_squeeze/003.png`), `rec_picturex_0611__rivet_squeeze__001__png_2ba6198e3af4438abc2a873af2483586` (`pictureY/0611/rivet_squeeze/001.png`)
canonical_baselines: none planned; add only if a selected origin fails the §11 readability contract during fork review.
underfilled_reason: none at P0; adaptive coverage follows the approved per-category slot vocabulary.

## Subcategory Contract

- core_identity: hand rivet squeezer retaining opposed rivet sets and a force-multiplying squeeze mechanism
- must_keep: defining use, picture identity, visible support interfaces, and at least one real non-fixed joint.
- must_not_become: blind-rivet puller, hole punch
- image_evidence: pictureY/0611/rivet_squeeze/002.png, pictureY/0611/rivet_squeeze/003.png, pictureY/0611/rivet_squeeze/001.png
- parent_evidence: rec_picturex_0611__rivet_squeeze__002__png_7a78c071827541ed822cff3fb676aa83, rec_picturex_0611__rivet_squeeze__003__png_7c15724dd1fc4e4a9c3ba1c34794454c, rec_picturex_0611__rivet_squeeze__001__png_2ba6198e3af4438abc2a873af2483586

## Slot Candidates

| slot | candidate | diversity_axis | source_type | record/evidence | key parts/joints/helpers | status |
|---|---|---|---|---|---|---|
| origin_design | reference_002_compound_rivet_squeezer | ①/②/③ observed | origin_anchor | `rec_picturex_0611__rivet_squeeze__002__png_7a78c071827541ed822cff3fb676aa83` / `pictureY/0611/rivet_squeeze/002.png` | frame, fixed_handle, squeeze_lever, compound_link, squeeze_ram, anvil_screw, frame_to_fixed_handle, frame_to_squeeze_lever, lever_to_compound_link, frame_to_squeeze_ram, frame_to_anvil_screw, _side_extrusion, _y_cylinder, _frame_shape | built ✓ |
| origin_design | pictureX_0611_rivet_squeeze_003 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__rivet_squeeze__003__png_7c15724dd1fc4e4a9c3ba1c34794454c` / `pictureY/0611/rivet_squeeze/003.png` | frame, lever, nosepiece, latch, frame_to_lever, frame_to_nosepiece, frame_to_latch, _front_plate, _axis_x_tube, _nosepiece_shape, _material_name | built ✓ |
| origin_design | hand_rivet_squeezer_001 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__rivet_squeeze__001__png_2ba6198e3af4438abc2a873af2483586` / `pictureY/0611/rivet_squeeze/001.png` | frame, moving_handle, ram, adjustment_die, grip_latch, frame_to_handle, frame_to_ram, frame_to_adjustment_die, handle_to_latch, _plate_from_profile, _frame_shape, _moving_handle_shape, _fixed_grip_shape, _moving_grip_shape | built ✓ |
| yoke_frame | deep C-yoke | ③ | forked_anchor | `rec_0611_rivet_squeeze_var_yoke_frame_deep_c_yoke` from `rec_picturex_0611__rivet_squeeze__002__png_7a78c071827541ed822cff3fb676aa83` | frame_to_squeeze_ram, frame_to_squeeze_lever, frame_to_fixed_handle, frame_to_anvil_screw, frame, _frame_shape, forged_frame, squeeze_lever_body | built ✓ |
| yoke_frame | alligator jaw | ③ | forked_anchor | `rec_0611_rivet_squeeze_var_yoke_frame_alligator_jaw` from `rec_picturex_0611__rivet_squeeze__003__png_7c15724dd1fc4e4a9c3ba1c34794454c` | frame_to_nosepiece, frame_to_lever, frame_to_latch, frame, jaw_fastener, body_shell, moving_grip, fixed_grip | built ✓ |
| yoke_frame | compact straight head | ③ | forked_anchor | `rec_0611_rivet_squeeze_var_yoke_frame_compact_straight_head` from `rec_picturex_0611__rivet_squeeze__001__png_2ba6198e3af4438abc2a873af2483586` | frame_to_ram, frame_to_handle, frame_to_adjustment_die, frame, _frame_shape, frame_shell | built ✓ |
| squeeze_mechanism | compound toggle | ② | forked_anchor | `rec_0611_rivet_squeeze_var_squeeze_mechanism_compound_toggle` from `rec_picturex_0611__rivet_squeeze__002__png_7a78c071827541ed822cff3fb676aa83` | lever_to_compound_link, compound_link, frame_to_squeeze_ram, frame_to_squeeze_lever, compound link is retained by pivot pin, squeeze_ram, squeeze_lever, squeeze lever is captured on main pivot | built ✓ |
| squeeze_mechanism | eccentric cam lever | ② | forked_anchor | `rec_0611_rivet_squeeze_var_squeeze_mechanism_eccentric_cam_lever` from `rec_picturex_0611__rivet_squeeze__002__png_7a78c071827541ed822cff3fb676aa83` | lever_to_compound_link, frame_to_squeeze_lever, squeeze_lever, squeeze lever is captured on main pivot, frame_to_squeeze_ram, frame_to_fixed_handle, squeeze_ram, fixed_handle | built ✓ |
| squeeze_mechanism | screw press | ② | forked_anchor | `rec_0611_rivet_squeeze_var_squeeze_mechanism_screw_press` from `rec_picturex_0611__rivet_squeeze__002__png_7a78c071827541ed822cff3fb676aa83` | lever_to_compound_link, frame_to_squeeze_ram, frame_to_squeeze_lever, frame_to_anvil_screw, squeeze_ram, squeeze_lever, compound_link, anvil_screw | built ✓ |
| head_module | quick-change yoke | ② | forked_anchor | `rec_0611_rivet_squeeze_var_head_module_quick_change_yoke` from `rec_picturex_0611__rivet_squeeze__003__png_7c15724dd1fc4e4a9c3ba1c34794454c` | jaw_fastener | built ✓ |
| head_module | rotating set holder | ② | forked_anchor | `rec_0611_rivet_squeeze_var_head_module_rotating_set_holder` from `rec_picturex_0611__rivet_squeeze__001__png_2ba6198e3af4438abc2a873af2483586` | upper_die_holder, lower_die_holder | built ✓ |
| return | torsion handle spring | ② | forked_anchor | `rec_0611_rivet_squeeze_var_return_torsion_handle_spring` from `rec_picturex_0611__rivet_squeeze__001__png_2ba6198e3af4438abc2a873af2483586` | handle_to_latch, frame_to_handle, moving_handle, _moving_handle_shape, handle_forging, grip_latch, return_spring, closed handles retain a narrow non-intersecting grip gap | built ✓ |
| return | leaf return spring | ② | forked_anchor | `rec_0611_rivet_squeeze_var_return_leaf_return_spring` from `rec_picturex_0611__rivet_squeeze__001__png_2ba6198e3af4438abc2a873af2483586` | return_spring | built ✓ |

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
- neighbor categories (blind-rivet puller, hole punch): excluded.
- failed or unfit candidates will be appended with one-line reasons after 2–3 attempts.
