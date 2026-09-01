# 0611 / worm_gear_and_wheel_assembly — template source map

pattern: mixed
parents: `rec_worm_gear_and_wheel_assembly__worm_gear_and_wheel_assembly__001_png_c097f6455b834d47b95478b26e163c36` (`pictureY/0611/worm_gear_and_wheel_assembly/001.png`), `rec_worm_gear_and_wheel_assembly__worm_gear_and_wheel_assembly__002_png_d1bae7f8a0f645068588a092bf260924` (`pictureY/0611/worm_gear_and_wheel_assembly/002.png`)
canonical_baselines: none planned; add only if a selected origin fails the §11 readability contract during fork review.
underfilled_reason: none at P0; adaptive coverage follows the approved per-category slot vocabulary.

## Subcategory Contract

- core_identity: meshing worm-and-wheel transmission retaining perpendicular shafts and real rotational joints
- must_keep: defining use, picture identity, visible support interfaces, and at least one real non-fixed joint.
- must_not_become: spur gear pair, decorative gear display
- image_evidence: pictureY/0611/worm_gear_and_wheel_assembly/001.png, pictureY/0611/worm_gear_and_wheel_assembly/002.png
- parent_evidence: rec_worm_gear_and_wheel_assembly__worm_gear_and_wheel_assembly__001_png_c097f6455b834d47b95478b26e163c36, rec_worm_gear_and_wheel_assembly__worm_gear_and_wheel_assembly__002_png_d1bae7f8a0f645068588a092bf260924

## Slot Candidates

| slot | candidate | diversity_axis | source_type | record/evidence | key parts/joints/helpers | status |
|---|---|---|---|---|---|---|
| origin_design | worm_gear_and_wheel_assembly | ①/②/③ observed | origin_anchor | `rec_worm_gear_and_wheel_assembly__worm_gear_and_wheel_assembly__001_png_c097f6455b834d47b95478b26e163c36` / `pictureY/0611/worm_gear_and_wheel_assembly/001.png` | support, worm_shaft, worm_wheel, support_to_worm, support_to_wheel, _circle_profile, _toothed_wheel_profile, _helical_thread_geometry | built ✓ |
| origin_design | worm_gear_and_wheel_assembly | ①/②/③ observed | origin_anchor | `rec_worm_gear_and_wheel_assembly__worm_gear_and_wheel_assembly__002_png_d1bae7f8a0f645068588a092bf260924` / `pictureY/0611/worm_gear_and_wheel_assembly/002.png` | carrier, wheel, worm, carrier_to_wheel, carrier_to_worm, _shape_aabb_size, _tube_along_z, _tube_along_x, _tube_along_y, _keyed_hub | built ✓ |
| housing_topology | open pillow-block frame | ① | forked_anchor | `rec_0611_worm_gear_and_wheel_assembly_var_housing_topology_open_pillow_block_fra` from `rec_worm_gear_and_wheel_assembly__worm_gear_and_wheel_assembly__001_png_c097f6455b834d47b95478b26e163c36` | support_to_worm, support_to_wheel, support, bearing face supports the bored brass hub | built ✓ |
| housing_topology | enclosed gearbox | ① | forked_anchor | `rec_0611_worm_gear_and_wheel_assembly_var_housing_topology_enclosed_gearbox` from `rec_worm_gear_and_wheel_assembly__worm_gear_and_wheel_assembly__002_png_d1bae7f8a0f645068588a092bf260924` | carrier, wheel, worm, _shape_aabb_size, _tube_along_z | built ✓ |
| housing_topology | adjustable split housing | ① | forked_anchor | `rec_0611_worm_gear_and_wheel_assembly_var_housing_topology_adjustable_split_hous` from `rec_worm_gear_and_wheel_assembly__worm_gear_and_wheel_assembly__001_png_c097f6455b834d47b95478b26e163c36` | support, worm_shaft, worm_wheel, _circle_profile, _toothed_wheel_profile | built ✓ |
| worm_starts | 1-start worm | N | forked_anchor | `rec_0611_worm_gear_and_wheel_assembly_var_worm_starts_1_start_worm` from `rec_worm_gear_and_wheel_assembly__worm_gear_and_wheel_assembly__002_png_d1bae7f8a0f645068588a092bf260924` | carrier_to_worm, worm, worm_gear_and_wheel_assembly, worm thread is tangent above wheel teeth, worm and wheel teeth overlap in meshing footprint, rear_strut_1 | built ✓ |
| worm_starts | 2-start worm | N | forked_anchor | `rec_0611_worm_gear_and_wheel_assembly_var_worm_starts_2_start_worm` from `rec_worm_gear_and_wheel_assembly__worm_gear_and_wheel_assembly__001_png_c097f6455b834d47b95478b26e163c36` | support_to_worm, worm_wheel, worm_shaft, worm_gear_and_wheel_assembly, worm thread is seated at the wheel tooth tips, worm thread crosses the wheel tooth face, meshing overlap remains when worm spins | built ✓ |
| worm_starts | 4-start worm | N | forked_anchor | `rec_0611_worm_gear_and_wheel_assembly_var_worm_starts_4_start_worm` from `rec_worm_gear_and_wheel_assembly__worm_gear_and_wheel_assembly__002_png_d1bae7f8a0f645068588a092bf260924` | carrier_to_worm, worm, worm_gear_and_wheel_assembly, worm thread is tangent above wheel teeth, worm and wheel teeth overlap in meshing footprint | built ✓ |
| mesh_orientation | top-mesh worm | ③ | forked_anchor | `rec_0611_worm_gear_and_wheel_assembly_var_mesh_orientation_top_mesh_worm` from `rec_worm_gear_and_wheel_assembly__worm_gear_and_wheel_assembly__002_png_d1bae7f8a0f645068588a092bf260924` | carrier_to_worm, worm, worm_gear_and_wheel_assembly, worm thread is tangent above wheel teeth, worm and wheel teeth overlap in meshing footprint, top_bearing_bridge | built ✓ |
| mesh_orientation | vertical-shaft worm | ③ | forked_anchor | `rec_0611_worm_gear_and_wheel_assembly_var_mesh_orientation_vertical_shaft_worm` from `rec_worm_gear_and_wheel_assembly__worm_gear_and_wheel_assembly__001_png_c097f6455b834d47b95478b26e163c36` | worm_shaft, support_to_worm, worm_wheel, worm_gear_and_wheel_assembly, worm thread is seated at the wheel tooth tips, worm thread crosses the wheel tooth face, meshing overlap remains when worm spins, chamfered_shaft | built ✓ |
| backlash_adjustment | eccentric bearing carrier | ② | forked_anchor | `rec_0611_worm_gear_and_wheel_assembly_var_backlash_adjustment_eccentric_bearing` from `rec_worm_gear_and_wheel_assembly__worm_gear_and_wheel_assembly__002_png_d1bae7f8a0f645068588a092bf260924` | carrier_to_worm, carrier_to_wheel, carrier, wheel_bearing_collar, top_bearing_bridge, lower_bearing_bridge, axle_thrust_collar | built ✓ |
| backlash_adjustment | sliding worm carriage | ② | forked_anchor | `rec_0611_worm_gear_and_wheel_assembly_var_backlash_adjustment_sliding_worm_carri` from `rec_worm_gear_and_wheel_assembly__worm_gear_and_wheel_assembly__002_png_d1bae7f8a0f645068588a092bf260924` | carrier_to_worm, worm, worm_gear_and_wheel_assembly, worm thread is tangent above wheel teeth, worm and wheel teeth overlap in meshing footprint, wheel_bearing_collar, axle_thrust_collar | built ✓ |

## Multiplicity / Copy Logic

- count_param: worm_starts_count
- N samples: 1-start worm, 2-start worm, 4-start worm
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
- neighbor categories (spur gear pair, decorative gear display): excluded.
- failed or unfit candidates will be appended with one-line reasons after 2–3 attempts.
