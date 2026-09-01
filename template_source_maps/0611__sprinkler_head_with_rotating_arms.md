# 0611 / sprinkler_head_with_rotating_arms — template source map

pattern: mixed
parents: `rec_picturex_0611__sprinkler_head_with_rotating_arms__001__png_7ff2244508c64596895614e834e21aae` (`pictureY/0611/sprinkler_head_with_rotating_arms/001.png`), `rec_picturex_0611__sprinkler_head_with_rotating_arms__002__png_70341112add34cc997a9ecd7802603c4` (`pictureY/0611/sprinkler_head_with_rotating_arms/002.png`), `rec_picturex_0611__sprinkler_head_with_rotating_arms__003__png_c3c083e861614cb0a0f31263560e13d9` (`pictureY/0611/sprinkler_head_with_rotating_arms/003.png`)
canonical_baselines: none planned; add only if a selected origin fails the §11 readability contract during fork review.
underfilled_reason: none at P0; adaptive coverage follows the approved per-category slot vocabulary.

## Subcategory Contract

- core_identity: real-world sprinkler head with rotating arms retaining its defining use and articulation
- must_keep: defining use, picture identity, visible support interfaces, and at least one real non-fixed joint.
- must_not_become: neighbor category, decorative static prop
- image_evidence: pictureY/0611/sprinkler_head_with_rotating_arms/001.png, pictureY/0611/sprinkler_head_with_rotating_arms/002.png, pictureY/0611/sprinkler_head_with_rotating_arms/003.png
- parent_evidence: rec_picturex_0611__sprinkler_head_with_rotating_arms__001__png_7ff2244508c64596895614e834e21aae, rec_picturex_0611__sprinkler_head_with_rotating_arms__002__png_70341112add34cc997a9ecd7802603c4, rec_picturex_0611__sprinkler_head_with_rotating_arms__003__png_c3c083e861614cb0a0f31263560e13d9

## Slot Candidates

| slot | candidate | diversity_axis | source_type | record/evidence | key parts/joints/helpers | status |
|---|---|---|---|---|---|---|
| origin_design | three_arm_rotating_sprinkler | ①/②/③ observed | origin_anchor | `rec_picturex_0611__sprinkler_head_with_rotating_arms__001__png_7ff2244508c64596895614e834e21aae` / `pictureY/0611/sprinkler_head_with_rotating_arms/001.png` | base, rotor, dynamic_indexed_name, base_to_rotor, dynamic_indexed_name, _cylinder_x, _cylinder_y, _base_shell, _side_connector, _radial_rib, _hub_shell, _arm_tube, _nozzle_shell, _orange_insert | built ✓ |
| origin_design | three_arm_brass_rotating_sprinkler | ①/②/③ observed | origin_anchor | `rec_picturex_0611__sprinkler_head_with_rotating_arms__002__png_70341112add34cc997a9ecd7802603c4` / `pictureY/0611/sprinkler_head_with_rotating_arms/002.png` | connector, arm_assembly, dynamic_indexed_name, connector_to_arm_assembly, dynamic_indexed_name, _rotate_z, _unit, _frame_for_local_z, _aabb_center | built ✓ |
| origin_design | sprinkler_head_003 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__sprinkler_head_with_rotating_arms__003__png_c3c083e861614cb0a0f31263560e13d9` / `pictureY/0611/sprinkler_head_with_rotating_arms/003.png` | base, rotor, dynamic_indexed_name, rotor_spin, dynamic_indexed_name, _midpoint, _distance, _cylinder_origin_between, _rotate_z, _make_base_casting, _make_hose_connector, _make_adjustment_ring, _make_wheel_tire, _make_wheel_insert | built ✓ |
| arm_count | 2 | N | forked_anchor | `rec_0611_sprinkler_head_with_rotating_a_var_arm_count_2` from `rec_picturex_0611__sprinkler_head_with_rotating_arms__001__png_7ff2244508c64596895614e834e21aae` | _arm_tube, three_arm_rotating_sprinkler, rotor hub retains deep engagement on the shaft, central shaft remains centered in the rotating hub, bearing_shaft | planned |
| arm_count | 4 | N | forked_anchor | `rec_0611_sprinkler_head_with_rotating_a_var_arm_count_4` from `rec_picturex_0611__sprinkler_head_with_rotating_arms__002__png_70341112add34cc997a9ecd7802603c4` | connector_to_arm_assembly, arm_assembly, three_arm_brass_rotating_sprinkler, index_rib, central_shaft | planned |
| arm_form | curved S | ③ | forked_anchor | `rec_0611_sprinkler_head_with_rotating_a_var_arm_form_curved_s` from `rec_picturex_0611__sprinkler_head_with_rotating_arms__003__png_c3c083e861614cb0a0f31263560e13d9` | knurled collar captures central shaft | planned |
| arm_form | straight radial | ③ | forked_anchor | `rec_0611_sprinkler_head_with_rotating_a_var_arm_form_straight_radial` from `rec_picturex_0611__sprinkler_head_with_rotating_arms__001__png_7ff2244508c64596895614e834e21aae` | _arm_tube, three_arm_rotating_sprinkler, _radial_rib, rotor hub retains deep engagement on the shaft, central shaft remains centered in the rotating hub, bearing_shaft | planned |
| base | tripod | ① | forked_anchor | `rec_0611_sprinkler_head_with_rotating_a_var_base_tripod` from `rec_picturex_0611__sprinkler_head_with_rotating_arms__003__png_c3c083e861614cb0a0f31263560e13d9` | base, _make_base_casting, base_casting | planned |
| base | ground spike | ① | forked_anchor | `rec_0611_sprinkler_head_with_rotating_a_var_base_ground_spike` from `rec_picturex_0611__sprinkler_head_with_rotating_arms__001__png_7ff2244508c64596895614e834e21aae` | base_to_rotor, base, _base_shell, perforated_base | planned |
| nozzle_count | 2 per arm | N | forked_anchor | `rec_0611_sprinkler_head_with_rotating_a_var_nozzle_count_2_per_arm` from `rec_picturex_0611__sprinkler_head_with_rotating_arms__001__png_7ff2244508c64596895614e834e21aae` | _arm_tube, three_arm_rotating_sprinkler, _nozzle_shell, rotor hub retains deep engagement on the shaft, nozzle_shell, central shaft remains centered in the rotating hub, bearing_shaft | planned |
| nozzle_motion | pivoting jet | ② | forked_anchor | `rec_0611_sprinkler_head_with_rotating_a_var_nozzle_motion_pivoting_jet` from `rec_picturex_0611__sprinkler_head_with_rotating_arms__003__png_c3c083e861614cb0a0f31263560e13d9` | rotor_spin, _make_adjustment_ring, _add_nozzle, adjustment_ring | planned |

## Multiplicity / Copy Logic

- count_param: arm_count_count, nozzle_count_count
- N samples: 2, 4, 2 per arm
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
