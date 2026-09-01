# 0611 / turnbuckle — template source map

pattern: mixed
parents: `rec_picturex_0611__turnbuckle__004__png_1cebe68f2a7d4b5080da47f9d2a8754f` (`pictureY/0611/turnbuckle/004.png`), `rec_picturex_0611__turnbuckle__001__png_689bc9e644844763a8ada4d8d9cd895c` (`pictureY/0611/turnbuckle/001.png`), `rec_picturex_0611__turnbuckle__002__png_5eeeedffab0d4787b0fdb48f25f4e307` (`pictureY/0611/turnbuckle/002.png`), `rec_picturex_0611__turnbuckle__003__png_f334640b8817451f89c2694fb3384dfd` (`pictureY/0611/turnbuckle/003.png`)
canonical_baselines: none planned; add only if a selected origin fails the §11 readability contract during fork review.
underfilled_reason: none at P0; adaptive coverage follows the approved per-category slot vocabulary.

## Subcategory Contract

- core_identity: real-world turnbuckle retaining its defining use and articulation
- must_keep: defining use, picture identity, visible support interfaces, and at least one real non-fixed joint.
- must_not_become: neighbor category, decorative static prop
- image_evidence: pictureY/0611/turnbuckle/004.png, pictureY/0611/turnbuckle/001.png, pictureY/0611/turnbuckle/002.png, pictureY/0611/turnbuckle/003.png
- parent_evidence: rec_picturex_0611__turnbuckle__004__png_1cebe68f2a7d4b5080da47f9d2a8754f, rec_picturex_0611__turnbuckle__001__png_689bc9e644844763a8ada4d8d9cd895c, rec_picturex_0611__turnbuckle__002__png_5eeeedffab0d4787b0fdb48f25f4e307, rec_picturex_0611__turnbuckle__003__png_f334640b8817451f89c2694fb3384dfd

## Slot Candidates

| slot | candidate | diversity_axis | source_type | record/evidence | key parts/joints/helpers | status |
|---|---|---|---|---|---|---|
| origin_design | hook_eye_turnbuckle_004 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__turnbuckle__004__png_1cebe68f2a7d4b5080da47f9d2a8754f` / `pictureY/0611/turnbuckle/004.png` | thread_bridge, barrel, hook_rod, eye_rod, barrel_rotation, hook_adjustment, eye_adjustment, _x_cylinder, _x_tube, _fuse, _threaded_rod, _tube_along_polyline, _build_core, _build_barrel | built ✓ |
| origin_design | eye_eye_turnbuckle_001 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__turnbuckle__001__png_689bc9e644844763a8ada4d8d9cd895c` / `pictureY/0611/turnbuckle/001.png` | left_fitting, barrel, right_fitting, barrel_rotation, right_thread_travel, _x_cylinder, _threaded_shank, _eye_fitting, _open_turnbuckle_barrel | built ✓ |
| origin_design | jaw_eye_turnbuckle_002 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__turnbuckle__002__png_5eeeedffab0d4787b0fdb48f25f4e307` / `pictureY/0611/turnbuckle/002.png` | spindle, barrel, fork_rod, eye_rod, clevis_pin, spindle_to_barrel, spindle_to_fork, spindle_to_eye, fork_to_pin, _cylinder_x, _cylinder_y, _cone_x, _threaded_rod, _barrel_shape | built ✓ |
| origin_design | jaw_jaw_turnbuckle_003 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__turnbuckle__003__png_f334640b8817451f89c2694fb3384dfd` / `pictureY/0611/turnbuckle/003.png` | jaw_fitting_0, barrel, jaw_fitting_1, clevis_pin_0, clevis_pin_1, barrel_adjust, rod_adjust, pin_spin_0, pin_spin_1, _mesh, _x_cylinder, _y_cylinder, _box, _build_open_barrel | built ✓ |
| end_topology | hook-hook | ① | forked_anchor | `rec_0611_turnbuckle_var_end_topology_hook_hook` from `rec_picturex_0611__turnbuckle__004__png_1cebe68f2a7d4b5080da47f9d2a8754f` | hook_adjustment, hook_rod, _build_hook_rod, hook_shoulder, hook_rod_core, hook_fitting, hook_eye_turnbuckle_004, hook_engagement | planned |
| end_topology | hook-jaw | ① | forked_anchor | `rec_0611_turnbuckle_var_end_topology_hook_jaw` from `rec_picturex_0611__turnbuckle__001__png_689bc9e644844763a8ada4d8d9cd895c` | left_fitting, barrel, right_fitting, _x_cylinder, _threaded_shank | planned |
| barrel_form | closed cylinder | ③ | forked_anchor | `rec_0611_turnbuckle_var_barrel_form_closed_cylinder` from `rec_picturex_0611__turnbuckle__002__png_5eeeedffab0d4787b0fdb48f25f4e307` | spindle_to_barrel, barrel, _cylinder_y, _cylinder_x, _barrel_shape, fork threaded rod remains engaged in barrel, eye threaded rod remains engaged in barrel, barrel_shell | planned |
| barrel_form | forged oval | ③ | forked_anchor | `rec_0611_turnbuckle_var_barrel_form_forged_oval` from `rec_picturex_0611__turnbuckle__003__png_f334640b8817451f89c2694fb3384dfd` | barrel_adjust, barrel, _build_open_barrel, right threaded shank seats in barrel socket, left threaded shank seats in barrel socket | planned |
| lock | paired lock nuts | N | forked_anchor | `rec_0611_turnbuckle_var_lock_paired_lock_nuts` from `rec_picturex_0611__turnbuckle__004__png_1cebe68f2a7d4b5080da47f9d2a8754f` | hook_adjustment, eye_adjustment | planned |
| end_motion | swivel ends | ② | forked_anchor | `rec_0611_turnbuckle_var_end_motion_swivel_ends` from `rec_picturex_0611__turnbuckle__003__png_f334640b8817451f89c2694fb3384dfd` | pin_spin_1, pin_spin_0, right rod remains threaded at full adjustment | planned |
| pin | quick-release clevis pin | ② | forked_anchor | `rec_0611_turnbuckle_var_pin_quick_release_clevis_pin` from `rec_picturex_0611__turnbuckle__002__png_5eeeedffab0d4787b0fdb48f25f4e307` | clevis_pin, fork_to_pin, clevis pin is captured by the fork, _pin_shape, pin_hardware | planned |

## Multiplicity / Copy Logic

- count_param: lock_count
- N samples: paired lock nuts
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
