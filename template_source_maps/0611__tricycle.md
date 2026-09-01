# 0611 / tricycle — template source map

pattern: mixed
parents: `rec_picturex_0611__tricycle__001__png_8cefebf42a304137bc2ad69ee47c5f91` (`pictureY/0611/tricycle/001.png`)
canonical_baselines: none planned; add only if a selected origin fails the §11 readability contract during fork review.
underfilled_reason: none at P0; adaptive coverage follows the approved per-category slot vocabulary.

## Subcategory Contract

- core_identity: three-wheeled pedal cycle retaining one front or rear wheel pair, steering, seat, and human drive
- must_keep: defining use, picture identity, visible support interfaces, and at least one real non-fixed joint.
- must_not_become: bicycle, four-wheel pedal cart
- image_evidence: pictureY/0611/tricycle/001.png
- parent_evidence: rec_picturex_0611__tricycle__001__png_8cefebf42a304137bc2ad69ee47c5f91

## Slot Candidates

| slot | candidate | diversity_axis | source_type | record/evidence | key parts/joints/helpers | status |
|---|---|---|---|---|---|---|
| origin_design | pictureX_0611_tricycle_001 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__tricycle__001__png_8cefebf42a304137bc2ad69ee47c5f91` / `pictureY/0611/tricycle/001.png` | chassis, cargo_basket, saddle, steering, front_wheel, rear_wheel_0, rear_wheel_1, pedal_crank, pedal_0, pedal_1, basket_mount, saddle_mount, steering_joint, front_wheel_spin | built ✓ |
| rear_module | open rear cargo basket | ① | origin_anchor | `rec_picturex_0611__tricycle__001__png_8cefebf42a304137bc2ad69ee47c5f91` / `pictureY/0611/tricycle/001.png` | cargo_basket, basket_shell, basket_rim, basket_front_reinforcement, cargo_deck, basket_support_a, basket_support_b, basket_mount | built ✓ |
| rear_module | rear two-person passenger bench | ①/N | forked_anchor | `rec_0611_tricycle_var_rear_module_passenger_bench` from `rec_picturex_0611__tricycle__001__png_8cefebf42a304137bc2ad69ee47c5f91` | rear_passenger_bench, PASSENGER_COUNT, passenger_cushion_0..1, side_rail_0..1, support_0..1, footrest_0..1, footrest_support_0..1, passenger_module_mount | built ✓ (`gpt-5.6-sol`, high; human-review refill) |
| frame_topology | low-step child frame | ① | forked_anchor | `rec_0611_tricycle_var_frame_topology_low_step_child_frame` from `rec_picturex_0611__tricycle__001__png_8cefebf42a304137bc2ad69ee47c5f91` | frame_badge_blank, pedal_body, chassis, saddle_base | built ✓ |
| frame_topology | delta adult frame | ① | forked_anchor | `rec_0611_tricycle_var_frame_topology_delta_adult_frame` from `rec_picturex_0611__tricycle__001__png_8cefebf42a304137bc2ad69ee47c5f91` | frame_badge_blank, pedal_body, chassis, saddle_base | built ✓ |
| frame_topology | tadpole two-front frame | ① | forked_anchor | `rec_0611_tricycle_var_frame_topology_tadpole_two_front_frame` from `rec_picturex_0611__tricycle__001__png_8cefebf42a304137bc2ad69ee47c5f91` | frame_badge_blank, pedal_body, front_wheel_spin, front_wheel, chassis, saddle_base, front_fender, front_basket_rim | built ✓ |
| frame_topology | front cargo-box frame | ① | forked_anchor | `rec_0611_tricycle_var_frame_topology_front_cargo_box_frame` from `rec_picturex_0611__tricycle__001__png_8cefebf42a304137bc2ad69ee47c5f91` | frame_badge_blank, pedal_body, front_wheel_spin, front_wheel, chassis, cargo_basket, saddle_base, front_fender | built ✓ |
| frame_topology | folding commuter frame | ① | forked_anchor | `rec_0611_tricycle_var_frame_topology_folding_commuter_frame` from `rec_picturex_0611__tricycle__001__png_8cefebf42a304137bc2ad69ee47c5f91` | frame_badge_blank, pedal_body, chassis, saddle_base | built ✓ |
| frame_topology | drift-trike frame | ① | forked_anchor | `rec_0611_tricycle_var_frame_topology_drift_trike_frame` from `rec_picturex_0611__tricycle__001__png_8cefebf42a304137bc2ad69ee47c5f91` | frame_badge_blank, pedal_body, chassis, saddle_base | built ✓ |
| drive | front-wheel direct pedals | ② | forked_anchor | `rec_0611_tricycle_var_drive_front_wheel_direct_pedals` from `rec_picturex_0611__tricycle__001__png_8cefebf42a304137bc2ad69ee47c5f91` | front_wheel_spin, front_wheel, rear_wheel_1_spin, rear_wheel_0_spin, rear_wheel_1, rear_wheel_0, _wheel_visuals, pedal_crank | built ✓ |
| drive | mid-drive freewheel | ② | forked_anchor | `rec_0611_tricycle_var_drive_mid_drive_freewheel` from `rec_picturex_0611__tricycle__001__png_8cefebf42a304137bc2ad69ee47c5f91` | crank_joint, pedal_crank, upper_head_collar, steering bearing remains centered in head tube, lower_head_collar, head_tube, crank_spindle, crank spindle stays centered in front hub | built ✓ |
| drive | shaft drive | ② | forked_anchor | `rec_0611_tricycle_var_drive_shaft_drive` from `rec_picturex_0611__tricycle__001__png_8cefebf42a304137bc2ad69ee47c5f91` | crank_joint, pedal_crank, upper_head_collar, steering bearing remains centered in head tube, lower_head_collar, head_tube, crank_spindle, crank spindle stays centered in front hub | built ✓ |
| steering | direct fork steering | ② | forked_anchor | `rec_0611_tricycle_var_steering_direct_fork_steering` from `rec_picturex_0611__tricycle__001__png_8cefebf42a304137bc2ad69ee47c5f91` | steering_joint, steering, steering_stem, steering_bearing, steering bearing remains centered in head tube, steering bearing remains axially captured, fork_crown | built ✓ |
| steering | linkage steering | ② | forked_anchor | `rec_0611_tricycle_var_steering_linkage_steering` from `rec_picturex_0611__tricycle__001__png_8cefebf42a304137bc2ad69ee47c5f91` | steering_joint, steering, steering_stem, steering_bearing, steering bearing remains centered in head tube, steering bearing remains axially captured | built ✓ |
| steering | lean steering | ② | forked_anchor | `rec_0611_tricycle_var_steering_lean_steering` from `rec_picturex_0611__tricycle__001__png_8cefebf42a304137bc2ad69ee47c5f91` | steering_joint, steering, steering_stem, steering_bearing, steering bearing remains centered in head tube, steering bearing remains axially captured | built ✓ |
| seat_count | 2 seats | N | forked_anchor | `rec_0611_tricycle_var_seat_count_2_seats` from `rec_picturex_0611__tricycle__001__png_8cefebf42a304137bc2ad69ee47c5f91` | seat_post_sleeve, seat_post | built ✓ |
| seat_count | 3 tandem seats | N | forked_anchor | `rec_0611_tricycle_var_seat_count_3_tandem_seats` from `rec_picturex_0611__tricycle__001__png_8cefebf42a304137bc2ad69ee47c5f91` | seat_post_sleeve, seat_post | built ✓ |

## Multiplicity / Copy Logic

- count_param: seat_count_count
- N samples: 2 seats, 3 tandem seats
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

- rear-chain drive / `rec_0611_tricycle_var_drive_rear_chain_drive`: rejected during human variant review and deleted; the drive slot remains covered by three retained candidates, and the fork budget was reassigned to the missing `rear_module` slot.
- ④/⑤/⑥-only forks: excluded; these do not count as candidate anchors.
- neighbor categories (bicycle, four-wheel pedal cart): excluded.
- failed or unfit candidates will be appended with one-line reasons after 2–3 attempts.
