# 0611 / Wine_rack — template source map

pattern: mixed
parents: `rec_wine_rack__wine_rack__001_png_3489ff2270b645d89354800da6b888f0` (`pictureY/0611/Wine_rack/001.png`)
canonical_baselines: none planned; add only if a selected origin fails the §11 readability contract during fork review.
underfilled_reason: none at P0; adaptive coverage follows the approved per-category slot vocabulary.

## Subcategory Contract

- core_identity: wine rack that physically supports multiple bottles
- must_keep: defining use, picture identity, visible support interfaces, and at least one real non-fixed joint.
- must_not_become: wine cabinet, bottle crate
- image_evidence: pictureY/0611/Wine_rack/001.png
- parent_evidence: rec_wine_rack__wine_rack__001_png_3489ff2270b645d89354800da6b888f0

## Slot Candidates

| slot | candidate | diversity_axis | source_type | record/evidence | key parts/joints/helpers | status |
|---|---|---|---|---|---|---|
| origin_design | compact_angled_wine_rack | ①/②/③ observed | origin_anchor | `rec_wine_rack__wine_rack__001_png_3489ff2270b645d89354800da6b888f0` / `pictureY/0611/Wine_rack/001.png` | rack_frame, dynamic_indexed_name, dynamic_indexed_name, _oriented_box, _build_wood_frame, _build_shoulder, _neck_cradle_path | built ✓ |
| bottle_count | 3 | N | forked_anchor | `rec_0611_wine_rack_var_bottle_count_3` from `rec_wine_rack__wine_rack__001_png_3489ff2270b645d89354800da6b888f0` | rack_frame, sliding bottle stays laterally inside its groove, extended bottle remains supported by the rack, extended bottle neck remains longitudinally retained, extended bottle neck remains centered in its U-cradle, compact_angled_wine_rack, _neck_cradle_path | planned |
| bottle_count | 6 | N | forked_anchor | `rec_0611_wine_rack_var_bottle_count_6` from `rec_wine_rack__wine_rack__001_png_3489ff2270b645d89354800da6b888f0` | rack_frame, sliding bottle stays laterally inside its groove, extended bottle remains supported by the rack, extended bottle neck remains longitudinally retained, extended bottle neck remains centered in its U-cradle, compact_angled_wine_rack, _neck_cradle_path | planned |
| bottle_count | 9 | N | forked_anchor | `rec_0611_wine_rack_var_bottle_count_9` from `rec_wine_rack__wine_rack__001_png_3489ff2270b645d89354800da6b888f0` | rack_frame, sliding bottle stays laterally inside its groove, extended bottle remains supported by the rack, extended bottle neck remains longitudinally retained, extended bottle neck remains centered in its U-cradle, compact_angled_wine_rack, _neck_cradle_path | planned |
| rack_topology | honeycomb | ① | forked_anchor | `rec_0611_wine_rack_var_rack_topology_honeycomb` from `rec_wine_rack__wine_rack__001_png_3489ff2270b645d89354800da6b888f0` | rack_frame, extended bottle remains supported by the rack, compact_angled_wine_rack, sliding bottle stays laterally inside its groove, extended bottle neck remains longitudinally retained, extended bottle neck remains centered in its U-cradle, _neck_cradle_path, _build_wood_frame | converged |
| rack_topology | diagonal X-grid | ① | forked_anchor | `rec_0611_wine_rack_var_rack_topology_diagonal_x_grid` from `rec_wine_rack__wine_rack__001_png_3489ff2270b645d89354800da6b888f0` | rack_frame, extended bottle remains supported by the rack, compact_angled_wine_rack, sliding bottle stays laterally inside its groove, extended bottle neck remains longitudinally retained, extended bottle neck remains centered in its U-cradle, _neck_cradle_path, _build_wood_frame | planned |
| rack_topology | vertical tower | ① | forked_anchor | `rec_0611_wine_rack_var_rack_topology_vertical_tower` from `rec_wine_rack__wine_rack__001_png_3489ff2270b645d89354800da6b888f0` | rack_frame, extended bottle remains supported by the rack, compact_angled_wine_rack, sliding bottle stays laterally inside its groove, extended bottle neck remains longitudinally retained, extended bottle neck remains centered in its U-cradle, _neck_cradle_path, _build_wood_frame | planned |
| mount | wall mounted | ① | forked_anchor | `rec_0611_wine_rack_var_mount_wall_mounted` from `rec_wine_rack__wine_rack__001_png_3489ff2270b645d89354800da6b888f0` | base_heel | planned |
| mount | countertop | ① | forked_anchor | `rec_0611_wine_rack_var_mount_countertop` from `rec_wine_rack__wine_rack__001_png_3489ff2270b645d89354800da6b888f0` | base_heel | planned |
| expansion | rotating carousel | ② | forked_anchor | `rec_0611_wine_rack_var_expansion_rotating_carousel` from `rec_wine_rack__wine_rack__001_png_3489ff2270b645d89354800da6b888f0` | rack_frame, base_pedestal, rack_carousel_pivot (REVOLUTE, vertical axis), sliding bottle stays laterally inside its groove, extended bottle remains supported by the rack, compact_angled_wine_rack, _neck_cradle_path, _build_wood_frame | planned |
| orientation | upright presentation | ③ | forked_anchor | `rec_0611_wine_rack_var_orientation_upright_presentation` from `rec_wine_rack__wine_rack__001_png_3489ff2270b645d89354800da6b888f0` | rack_frame, dynamic_indexed_name, _oriented_box, _build_wood_frame, _build_shoulder | planned |

## Multiplicity / Copy Logic

- count_param: bottle_count_count
- N samples: 3, 6, 9
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
- neighbor categories (wine cabinet, bottle crate): excluded.
- failed or unfit candidates will be appended with one-line reasons after 2–3 attempts.
