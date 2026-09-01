# 0611 / sailboat_winch_with_pawl_and_handle — template source map

pattern: mixed
parents: `rec_picturex_0611__sailboat_winch_with_pawl_and_handle__003__png_8e642a4b246e4f8bbe68ce130afb5c2b` (`pictureY/0611/sailboat_winch_with_pawl_and_handle/003.png`), `rec_picturex_0611__sailboat_winch_with_pawl_and_handle__001__png_c33f82e11c604776b2a2110808781683` (`pictureY/0611/sailboat_winch_with_pawl_and_handle/001.png`), `rec_picturex_0611__sailboat_winch_with_pawl_and_handle__002__png_c4d5af90f90240ac9ff42989781e53d6` (`pictureY/0611/sailboat_winch_with_pawl_and_handle/002.png`)
canonical_baselines: none planned; add only if a selected origin fails the §11 readability contract during fork review.
underfilled_reason: none at P0; adaptive coverage follows the approved per-category slot vocabulary.

## Subcategory Contract

- core_identity: manual line winch retaining drum or capstan, pawl, and removable or integral handle
- must_keep: defining use, picture identity, visible support interfaces, and at least one real non-fixed joint.
- must_not_become: electric windlass, plain rope pulley
- image_evidence: pictureY/0611/sailboat_winch_with_pawl_and_handle/003.png, pictureY/0611/sailboat_winch_with_pawl_and_handle/001.png, pictureY/0611/sailboat_winch_with_pawl_and_handle/002.png
- parent_evidence: rec_picturex_0611__sailboat_winch_with_pawl_and_handle__003__png_8e642a4b246e4f8bbe68ce130afb5c2b, rec_picturex_0611__sailboat_winch_with_pawl_and_handle__001__png_c33f82e11c604776b2a2110808781683, rec_picturex_0611__sailboat_winch_with_pawl_and_handle__002__png_c4d5af90f90240ac9ff42989781e53d6

## Slot Candidates

| slot | candidate | diversity_axis | source_type | record/evidence | key parts/joints/helpers | status |
|---|---|---|---|---|---|---|
| origin_design | source_003_manual_ratchet_winch | ①/②/③ observed | origin_anchor | `rec_picturex_0611__sailboat_winch_with_pawl_and_handle__003__png_8e642a4b246e4f8bbe68ce130afb5c2b` / `pictureY/0611/sailboat_winch_with_pawl_and_handle/003.png` | frame, drum, drive_handle, ratchet_pawl, handle_grip, frame_to_drum, frame_to_drive_handle, frame_to_ratchet_pawl, handle_to_grip, _mesh, _tube_z, _cylinder_z, _build_frame_shape, _build_drum_shape | built ✓ |
| origin_design | hand_strap_winch | ①/②/③ observed | origin_anchor | `rec_picturex_0611__sailboat_winch_with_pawl_and_handle__001__png_c33f82e11c604776b2a2110808781683` / `pictureY/0611/sailboat_winch_with_pawl_and_handle/001.png` | frame, toothed_drum, strap_spool, crank, pawl, line_hardware, hook, drum_rotation, spool_rotation, crank_rotation, pawl_pivot, line_mount, hook_mount, _profile_plate | built ✓ |
| origin_design | manual_ratchet_winch_002 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__sailboat_winch_with_pawl_and_handle__002__png_c4d5af90f90240ac9ff42989781e53d6` / `pictureY/0611/sailboat_winch_with_pawl_and_handle/002.png` | frame, drum, pawl, handle, grip, frame_to_drum, frame_to_pawl, drum_to_handle, handle_to_grip, _axis_x_cylinder, _axis_x_annulus, _front_cheek, _rear_pedestal, _drum_core | built ✓ |
| winch_topology | vertical capstan deck winch | ① | forked_anchor | `rec_0611_sailboat_winch_with_pawl_and_h_var_winch_topology_vertical_capstan_deck_w` from `rec_picturex_0611__sailboat_winch_with_pawl_and_handle__003__png_8e642a4b246e4f8bbe68ce130afb5c2b` | source_003_manual_ratchet_winch | built ✓ |
| winch_topology | self-tailing deck winch | ① | forked_anchor | `rec_0611_sailboat_winch_with_pawl_and_h_var_winch_topology_self_tailing_deck_winch` from `rec_picturex_0611__sailboat_winch_with_pawl_and_handle__001__png_c33f82e11c604776b2a2110808781683` | hand_strap_winch | built ✓ |
| winch_topology | enclosed horizontal reel | ① | forked_anchor | `rec_0611_sailboat_winch_with_pawl_and_h_var_winch_topology_enclosed_horizontal_ree` from `rec_picturex_0611__sailboat_winch_with_pawl_and_handle__002__png_c4d5af90f90240ac9ff42989781e53d6` | manual_ratchet_winch_002 | built ✓ |
| winch_topology | open-cheek reel | ① | forked_anchor | `rec_0611_sailboat_winch_with_pawl_and_h_var_winch_topology_open_cheek_reel` from `rec_picturex_0611__sailboat_winch_with_pawl_and_handle__003__png_8e642a4b246e4f8bbe68ce130afb5c2b` | source_003_manual_ratchet_winch, drum remains between frame cheeks | built ✓ |
| gear_stages | direct drive | ① | forked_anchor | `rec_0611_sailboat_winch_with_pawl_and_h_var_gear_stages_direct_drive` from `rec_picturex_0611__sailboat_winch_with_pawl_and_handle__003__png_8e642a4b246e4f8bbe68ce130afb5c2b` | frame_to_drive_handle, drive_handle, drive_shaft, drive_pinion, handle_to_grip, handle_grip, crank_arm, rotating grip is retained on crank axle | built ✓ |
| gear_stages | single reduction | N | forked_anchor | `rec_0611_sailboat_winch_with_pawl_and_h_var_gear_stages_single_reduction` from `rec_picturex_0611__sailboat_winch_with_pawl_and_handle__003__png_8e642a4b246e4f8bbe68ce130afb5c2b` | reduction gears are closely meshed without overlap, ratchet_gear | built ✓ |
| gear_stages | two-speed reduction | ① | forked_anchor | `rec_0611_sailboat_winch_with_pawl_and_h_var_gear_stages_two_speed_reduction` from `rec_picturex_0611__sailboat_winch_with_pawl_and_handle__003__png_8e642a4b246e4f8bbe68ce130afb5c2b` | reduction gears are closely meshed without overlap, ratchet_gear | built ✓ |
| pawl_count | single pawl | N | forked_anchor | `rec_0611_sailboat_winch_with_pawl_and_h_var_pawl_count_single_pawl` from `rec_picturex_0611__sailboat_winch_with_pawl_and_handle__001__png_c33f82e11c604776b2a2110808781683` | pawl_pivot, pawl, pawl_washer, pawl_pivot_bracket, pawl_pin, pawl_body | built ✓ |
| pawl_count | dual pawl | N | forked_anchor | `rec_0611_sailboat_winch_with_pawl_and_h_var_pawl_count_dual_pawl` from `rec_picturex_0611__sailboat_winch_with_pawl_and_handle__002__png_c4d5af90f90240ac9ff42989781e53d6` | frame_to_pawl, pawl, _pawl_body, pawl_washer, pawl_pin, pawl_body, pawl tip rests against a ratchet tooth, pawl seats against its retaining washer | built ✓ |
| pawl_count | triple pawl | N | forked_anchor | `rec_0611_sailboat_winch_with_pawl_and_h_var_pawl_count_triple_pawl` from `rec_picturex_0611__sailboat_winch_with_pawl_and_handle__001__png_c33f82e11c604776b2a2110808781683` | pawl_pivot, pawl, pawl_washer, pawl_pivot_bracket, pawl_pin, pawl_body | built ✓ |
| handle | removable socket handle | ② | forked_anchor | `rec_0611_sailboat_winch_with_pawl_and_h_var_handle_removable_socket_handle` from `rec_picturex_0611__sailboat_winch_with_pawl_and_handle__002__png_c4d5af90f90240ac9ff42989781e53d6` | handle_to_grip, drum_to_handle, handle, _handle_nut, _handle_arm, removable handle retains deep socket insertion, withdrawn handle still has guided socket engagement, square handle stem is centered in spindle socket | built ✓ |
| handle | folding double-grip crank | ② | forked_anchor | `rec_0611_sailboat_winch_with_pawl_and_h_var_handle_folding_double_grip_crank` from `rec_picturex_0611__sailboat_winch_with_pawl_and_handle__002__png_c4d5af90f90240ac9ff42989781e53d6` | handle_to_grip, drum_to_handle, handle, _handle_nut, _handle_arm, withdrawn handle still has guided socket engagement, square handle stem is centered in spindle socket, removable handle retains deep socket insertion | built ✓ |
| handle | ratcheting handle | ② | forked_anchor | `rec_0611_sailboat_winch_with_pawl_and_h_var_handle_ratcheting_handle` from `rec_picturex_0611__sailboat_winch_with_pawl_and_handle__002__png_c4d5af90f90240ac9ff42989781e53d6` | handle_to_grip, drum_to_handle, handle, _handle_nut, _handle_arm, withdrawn handle still has guided socket engagement, square handle stem is centered in spindle socket, removable handle retains deep socket insertion | built ✓ |

## Multiplicity / Copy Logic

- count_param: gear_stages_count, pawl_count_count
- N samples: single reduction, single pawl, dual pawl, triple pawl
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
- neighbor categories (electric windlass, plain rope pulley): excluded.
- failed or unfit candidates will be appended with one-line reasons after 2–3 attempts.
