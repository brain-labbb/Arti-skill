# 0611 / overhead_projector_with_articulating_mirror — template source map

pattern: mixed
parents: `rec_picturex_0611__overhead_projector_with_articulating_mirror__001__png_05cea393e58442418c6e510b3f7de420` (`pictureY/0611/overhead_projector_with_articulating_mirror/001.png`)
canonical_baselines: none planned; add only if a selected origin fails the §11 readability contract during fork review.
underfilled_reason: none at P0; adaptive coverage follows the approved per-category slot vocabulary.

## Subcategory Contract

- core_identity: overhead transparency projector retaining illuminated stage, projection head, and articulating mirror
- must_keep: defining use, picture identity, visible support interfaces, and at least one real non-fixed joint.
- must_not_become: digital projector, document camera
- image_evidence: pictureY/0611/overhead_projector_with_articulating_mirror/001.png
- parent_evidence: rec_picturex_0611__overhead_projector_with_articulating_mirror__001__png_05cea393e58442418c6e510b3f7de420

## Slot Candidates

| slot | candidate | diversity_axis | source_type | record/evidence | key parts/joints/helpers | status |
|---|---|---|---|---|---|---|
| origin_design | overhead_projector_001 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__overhead_projector_with_articulating_mirror__001__png_05cea393e58442418c6e510b3f7de420` / `pictureY/0611/overhead_projector_with_articulating_mirror/001.png` | projector_body, lift_post, projection_head, mirror, power_switch, lock_knob, height_adjust, head_tilt, mirror_tilt, switch_rock, lock_knob_turn, _projector_body_shape, _stage_frame_shape, _post_sleeve_shape | built ✓ |
| post_topology | single telescoping mast | ① | forked_anchor | `rec_0611_overhead_projector_with_articu_var_post_topology_single_telescoping_mast` from `rec_picturex_0611__overhead_projector_with_articulating_mirror__001__png_05cea393e58442418c6e510b3f7de420` | lift_post, _stage_frame_shape, _post_sleeve_shape, stage_frame, raised mast remains retained in the sleeve, post_sleeve, post_bracket, mast lock knob is mounted on its shaft | built ✓ |
| post_topology | twin-column mast | ① | forked_anchor | `rec_0611_overhead_projector_with_articu_var_post_topology_twin_column_mast` from `rec_picturex_0611__overhead_projector_with_articulating_mirror__001__png_05cea393e58442418c6e510b3f7de420` | lift_post, _stage_frame_shape, _post_sleeve_shape, stage_frame, raised mast remains retained in the sleeve, post_sleeve, post_bracket, mast lock knob is mounted on its shaft | built ✓ |
| post_topology | folding cantilever mast | ① | forked_anchor | `rec_0611_overhead_projector_with_articu_var_post_topology_folding_cantilever_mast` from `rec_picturex_0611__overhead_projector_with_articulating_mirror__001__png_05cea393e58442418c6e510b3f7de420` | lift_post, folding head is captured by the post fork, _stage_frame_shape, _post_sleeve_shape, stage_frame, raised mast remains retained in the sleeve, post_sleeve, post_bracket | built ✓ |
| mirror_motion | dual-hinge mirror | ② | forked_anchor | `rec_0611_overhead_projector_with_articu_var_mirror_motion_dual_hinge_mirror` from `rec_picturex_0611__overhead_projector_with_articulating_mirror__001__png_05cea393e58442418c6e510b3f7de420` | mirror_tilt, lock_knob_turn, mirror, mirror pivot is carried by the yoke, _stage_frame_shape, stage_frame, pivot_drum, pivot_boss_1 | built ✓ |
| mirror_motion | fold-flat mirror | ② | forked_anchor | `rec_0611_overhead_projector_with_articu_var_mirror_motion_fold_flat_mirror` from `rec_picturex_0611__overhead_projector_with_articulating_mirror__001__png_05cea393e58442418c6e510b3f7de420` | mirror_tilt, lock_knob_turn, mirror, mirror pivot is carried by the yoke, _stage_frame_shape, stage_frame, pivot_drum, pivot_boss_1 | built ✓ |
| mirror_motion | swiveling mirror yoke | ② | forked_anchor | `rec_0611_overhead_projector_with_articu_var_mirror_motion_swiveling_mirror_yoke` from `rec_picturex_0611__overhead_projector_with_articulating_mirror__001__png_05cea393e58442418c6e510b3f7de420` | mirror_tilt, mirror pivot is carried by the yoke, lock_knob_turn, mirror, _stage_frame_shape, yoke_socket_1, yoke_socket_0, stage_frame | built ✓ |
| head_form | round lamp head | ③ | forked_anchor | `rec_0611_overhead_projector_with_articu_var_head_form_round_lamp_head` from `rec_picturex_0611__overhead_projector_with_articulating_mirror__001__png_05cea393e58442418c6e510b3f7de420` | head_tilt, projection_head, projection_head_shell, head_fork_1, head_fork_0, head_crossbar, head_arm, folding head is captured by the post fork | built ✓ |
| head_form | rectangular enclosed hood | ③ | forked_anchor | `rec_0611_overhead_projector_with_articu_var_head_form_rectangular_enclosed_hood` from `rec_picturex_0611__overhead_projector_with_articulating_mirror__001__png_05cea393e58442418c6e510b3f7de420` | head_tilt, projection_head, projection_head_shell, head_fork_1, head_fork_0, head_crossbar, head_arm, folding head is captured by the post fork | built ✓ |
| focus | rack-and-pinion head focus | ② | forked_anchor | `rec_0611_overhead_projector_with_articu_var_focus_rack_and_pinion_head_focus` from `rec_picturex_0611__overhead_projector_with_articulating_mirror__001__png_05cea393e58442418c6e510b3f7de420` | head_tilt, projection_head, projection_head_shell, head_fork_1, head_fork_0, head_crossbar, head_arm, folding head is captured by the post fork | built ✓ |
| focus | sliding lens-barrel focus | ② | forked_anchor | `rec_0611_overhead_projector_with_articu_var_focus_sliding_lens_barrel_focus` from `rec_picturex_0611__overhead_projector_with_articulating_mirror__001__png_05cea393e58442418c6e510b3f7de420` | projection_lens, lens_ring, fresnel_lens, head_tilt, projection_head, projection_head_shell, head_fork_1, head_fork_0 | built ✓ |

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
- neighbor categories (digital projector, document camera): excluded.
- failed or unfit candidates will be appended with one-line reasons after 2–3 attempts.
