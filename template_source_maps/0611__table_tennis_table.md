# 0611 / table_tennis_table — template source map

pattern: mixed
parents: `rec_picturex_0611__table_tennis_table__001__png_2da4a8e86cf243529ce2a808f66b0f35` (`pictureY/0611/table_tennis_table/001.png`), `rec_picturex_0611__table_tennis_table__002__png_d06cc47a12244303ba0dde10b808a04e` (`pictureY/0611/table_tennis_table/002.png`), `rec_picturex_0611__table_tennis_table__003__png_cc149fb9317941819ab725725ad68830` (`pictureY/0611/table_tennis_table/003.png`), `rec_picturex_0611__table_tennis_table__004__png_b2d0ede5a27e4257881a91122d62c32d` (`pictureY/0611/table_tennis_table/004.png`)
canonical_baselines: none planned; add only if a selected origin fails the §11 readability contract during fork review.
underfilled_reason: none at P0; adaptive coverage follows the approved per-category slot vocabulary.

## Subcategory Contract

- core_identity: real-world table tennis table retaining its defining use and articulation
- must_keep: defining use, picture identity, visible support interfaces, and at least one real non-fixed joint.
- must_not_become: neighbor category, decorative static prop
- image_evidence: pictureY/0611/table_tennis_table/001.png, pictureY/0611/table_tennis_table/002.png, pictureY/0611/table_tennis_table/003.png, pictureY/0611/table_tennis_table/004.png
- parent_evidence: rec_picturex_0611__table_tennis_table__001__png_2da4a8e86cf243529ce2a808f66b0f35, rec_picturex_0611__table_tennis_table__002__png_d06cc47a12244303ba0dde10b808a04e, rec_picturex_0611__table_tennis_table__003__png_cc149fb9317941819ab725725ad68830, rec_picturex_0611__table_tennis_table__004__png_b2d0ede5a27e4257881a91122d62c32d

## Slot Candidates

| slot | candidate | diversity_axis | source_type | record/evidence | key parts/joints/helpers | status |
|---|---|---|---|---|---|---|
| origin_design | portable_table_tennis_table_001 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__table_tennis_table__001__png_2da4a8e86cf243529ce2a808f66b0f35` / `pictureY/0611/table_tennis_table/001.png` | chassis, dynamic_indexed_name, chassis_to_half_0, chassis_to_half_1, dynamic_indexed_name | built ✓ |
| origin_design | portable_table_tennis_table_002 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__table_tennis_table__002__png_d06cc47a12244303ba0dde10b808a04e` / `pictureY/0611/table_tennis_table/002.png` | chassis, table_half_0, table_half_1, dynamic_indexed_name, half_hinge_0, half_hinge_1, dynamic_indexed_name, _segment_visual, _add_table_half, _add_leg, _add_brace | built ✓ |
| origin_design | challenger_rollaway_table_tennis_003 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__table_tennis_table__003__png_cc149fb9317941819ab725725ad68830` / `pictureY/0611/table_tennis_table/003.png` | chassis, near_half, far_half, near_half_lift, far_half_lift, HINGE_AXIS_X, hinge_pin_0_0..1_1, hinge_knuckle_0..1, hinge_leaf_0..1, caster_0..3, caster_0..3_swivel, _segment, _add_flat_half, _add_upright_half, _add_caster | rebuilt and seed-approved ✓ (`gpt-5.6-sol`, high); hinge sweep repaired |
| origin_design | compact_folding_table_tennis_table_004 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__table_tennis_table__004__png_b2d0ede5a27e4257881a91122d62c32d` / `pictureY/0611/table_tennis_table/004.png` | west_top, east_top, center_fold, leg_0..leg_3, leg_0_fold..leg_3_fold, _add_tabletop_half, _add_leg_assembly | rebuilt ✓ (`gpt-5.6-sol`, high) |
| top_topology | rigid one-piece | ① | forked_anchor | `rec_0611_table_tennis_table_var_top_topology_rigid_one_piece` from `rec_picturex_0611__table_tennis_table__001__png_2da4a8e86cf243529ce2a808f66b0f35` | net_top_tape | built ✓ (existing anchor) |
| top_topology | four-panel | ① | forked_anchor | `rec_0611_table_tennis_table_var_top_topology_four_panel` from `rec_picturex_0611__table_tennis_table__002__png_d06cc47a12244303ba0dde10b808a04e` | playing_panel, net_panel | built ✓ (existing anchor) |
| fold_motion | synchronized dual lift | ② | forked_anchor | `rec_0611_table_tennis_table_var_fold_motion_synchronized_dual_lift` from `rec_picturex_0611__table_tennis_table__003__png_cc149fb9317941819ab725725ad68830` | synchronization_shaft, synchronized_dual_lift, near_half_lift/far_half_lift Mimic, cross_shaft, crank_hub_0..1, near_lift_link_0..1, far_lift_link_0..1 | rebuilt ✓ (`gpt-5.6-sol`, high) from approved repaired seed |
| fold_motion | playback half | ② | forked_anchor | `rec_0611_table_tennis_table_var_fold_motion_playback_half` from `rec_picturex_0611__table_tennis_table__004__png_b2d0ede5a27e4257881a91122d62c32d` | center_fold, east_top, playback_receiver_0..1, playback_stay_0..1 | rebuilt ✓ (`gpt-5.6-sol`, high) |
| support_topology | scissor X-frames | ① | forked_anchor | `rec_0611_table_tennis_table_var_support_scissor_x_frames` from `rec_picturex_0611__table_tennis_table__003__png_cc149fb9317941819ab725725ad68830` | scissor_frame_0..1, scissor_cross_0..1, scissor_frame_0..1_fold, scissor_frame_0..1_pivot, _add_scissor_x_frame | rebuilt ✓ (`gpt-5.6-sol`, high) from approved repaired seed |
| caster_count | 6 | N | forked_anchor | `rec_0611_table_tennis_table_var_caster_count_6` from `rec_picturex_0611__table_tennis_table__003__png_cc149fb9317941819ab725725ad68830` | CASTER_COUNT, caster_0..5, caster_0..5_swivel, _add_caster, _add_caster_socket | built ✓ (`gpt-5.6-sol`, high) from approved repaired seed |
| caster_count | 4 | N | forked_anchor | `rec_0611_table_tennis_table_var_caster_count_4` from `rec_picturex_0611__table_tennis_table__001__png_2da4a8e86cf243529ce2a808f66b0f35` | wheel_hub | built ✓ (existing anchor) |
| caster_count | 8 | N | forked_anchor | `rec_0611_table_tennis_table_var_caster_count_8` from `rec_picturex_0611__table_tennis_table__002__png_d06cc47a12244303ba0dde10b808a04e` | wheel_tire, wheel_hub | built ✓ (existing anchor) |
| support_topology | fixed outdoor pedestal | ① | forked_anchor | `rec_0611_table_tennis_table_var_support_fixed_outdoor_pedestal` from `rec_picturex_0611__table_tennis_table__003__png_cc149fb9317941819ab725725ad68830` | chassis, pedestal_0..1_footplate, pedestal_0..1_pier, saddle_tie, hinge_crossbeam, _add_ground_pedestal | rebuilt ✓ (`gpt-5.6-sol`, high) from approved repaired seed |
| support_topology | paired U-frames | ①/N | forked_anchor | `rec_0611_table_tennis_table_var_support_paired_u_frames` from `rec_picturex_0611__table_tennis_table__004__png_b2d0ede5a27e4257881a91122d62c32d` | u_frame_0, u_frame_1, u_frame_0_fold, u_frame_1_fold, _add_u_frame | rebuilt ✓ (`gpt-5.6-sol`, high) |

## Multiplicity / Copy Logic

- count_param: `CASTER_COUNT` / existing-anchor caster count; paired support-family count
- N samples: 4, 6, 8 casters; paired support families from origin anchors
- suggested N_range: bounded by accepted source samples and downstream compile budget.
- copied object / naming / placement / joint policy: caster via `_add_caster` + `_add_caster_socket`, `caster_{i}` / `caster_{i}_swivel`, regular x-stations × symmetric y-sides, one continuous swivel per copy; paired support families use indexed shared helpers and uniform pivot policy.

## Six-Axis Diversity Record

| axis | treatment | values / range / reason |
|---|---|---|
| ① skeleton / structural topology | source-backed | rollaway central chassis, paired U-frames, paired scissor X-frames, fixed twin outdoor pedestals; rigid one-piece and four-panel top anchors |
| ② joint / mechanism type | source-backed | independent half lifts, synchronized dual lift with Mimic followers, center fold/playback mechanisms |
| ③ primary form family | source-backed | full-regulation rollaway table, portable table, compact folding table |
| ④ surface decoration | record_only / world_knowledge_extrapolation | white boundary lines, apron seams, safety labels and host-conformal hardware markings only |
| ⑤ proportion / size / travel | record_only | regulation 2.740 × 1.525 × 0.760 m parent; 90° half-lift travel; compact both-up storage envelope; modest safe tube/tire variation |
| ⑥ material / palette / finish | record_only | tournament blue/white top with dark powder-coated steel and red safety hardware; galvanized steel/concrete outdoor finish |

## Compatibility Probes

| probe | record_id | combined axes | risk tested | result |
|---|---|---|---|---|
| none at P0 | — | — | add only if cross-family interface review finds a real risk | — |

## Blocked / Excluded

- ④/⑤/⑥-only forks: excluded; these do not count as candidate anchors.
- neighbor categories (neighbor category, decorative static prop): excluded.
- failed or unfit candidates will be appended with one-line reasons after 2–3 attempts.
