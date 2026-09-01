<!--
subcategory_contract:
  category: Camping_Outdoor Gear
  subcategory: Folding camp table
  core_identity: A portable, foldable/collapsible camp table with a raised rectangular work surface on foldable or telescoping legs.
  must_keep: [raised horizontal work surface, at least one real fold/telescope/collapse articulation on the legs or top, four-corner leg support, packs down small]
  must_not_become: [fixed patio/picnic table, workbench, folding chair/stool, camp cot, serving cart with wheels, shelving unit]
  image_evidence:
    - 002.png (origin A): compact square roll-top table, mottled-gray aluminum slat deck, black perimeter/underside rails, four straight tubular folding legs, front+rear diagonal X-braces, packs into a carry bag.
    - 001.png (origin B): larger rectangular roll-top table, black aluminum slats, telescoping height-adjustable legs with thumb-lock collars, X/scissor braces, orange-bound black fabric+mesh under-table storage hammock.
  parent_evidence:
    - A: parts tabletop, leg_0..3, brace_0..3; joints tabletop_to_leg_i (REVOLUTE fold), tabletop_to_brace_i (REVOLUTE); helpers _slat_mesh, _capsule_x_mesh, _brace_mesh; SLAT_COUNT=9; rubber_foot, hinge_socket_i, upper_collar.
    - B: parts tabletop_frame, lower_leg_0..3, front_brace_0/1, rear_brace_0/1; joints table_to_lower_leg_i (PRISMATIC telescope), table_to_*_brace (REVOLUTE); helpers _cylinder_between, _add_leg_sleeve, _add_lower_leg, _add_brace; slat_count=13; pocket_bottom + front/rear_mesh_panel storage; leg_sleeve_i, foot_pad_i.
-->

# Camping_Outdoor Gear / Folding camp table — template source map
pattern: mixed (parallel_children: independent leg + brace children off one tabletop root; multiplicity: looped slats)
parents:
- rec_camping_outdoor_gear__folding_camp_table_1a04de6b29ee4143b411d8805aa338b9 (origin A) — picture/Camping_Outdoor Gear/Folding camp table/002.png
- rec_camping_outdoor_gear__folding_camp_table_e67ceec6b1ae4b31a958e16fdae9b35c (origin B) — picture/Camping_Outdoor Gear/Folding camp table/001.png
canonical_baselines: none
underfilled_reason: none (12 candidate anchors within the normal 12–18 budget)

## Slot Candidates
| slot | candidate | diversity_axis | source_type | record/evidence | key parts/joints | status |
|---|---|---|---|---|---|---|
| surface_construction | roll-top aluminum slats | ③ | origin_anchor | origin A & B (SLAT_COUNT / slat_count loops) | tabletop / tabletop_frame, slat_{i} | converged |
| surface_construction | solid one-piece rigid panel top | ③ | forked_anchor | rec_folding_camp_table_var_solid_panel_top (parent A) | tabletop panel replacing slat loop | converged |
| surface_construction | taut fabric / mesh membrane top | ③ | forked_anchor | rec_folding_camp_table_var_mesh_fabric_top (parent A) | fabric panel on perimeter rails | converged |
| leg_style / support | straight vertical folding tubular legs | ① | origin_anchor | origin A | leg_i straight_tube, tabletop_to_leg_i | converged |
| leg_style / support | telescoping height-adjustable legs | ① | origin_anchor | origin B | lower_leg_i, table_to_lower_leg_i (PRISMATIC) | converged |
| leg_style / support | outward-splayed A-frame legs | ① | forked_anchor | rec_folding_camp_table_var_splayed_aframe_legs (parent A) | angled leg_i, rubber_foot | converged |
| leg_style / support | crossed X / trestle leg pairs | ① | forked_anchor | rec_folding_camp_table_var_x_cross_legs (parent A) | two X leg pairs, top revolute pivots | converged |
| fold_mechanism | revolute swing-fold legs | ② | origin_anchor | origin A | tabletop_to_leg_i (REVOLUTE) | converged |
| fold_mechanism | prismatic telescoping legs | ② | origin_anchor | origin B | table_to_lower_leg_i (PRISMATIC) | converged |
| fold_mechanism | central bi-fold / suitcase top hinge | ② | forked_anchor | rec_folding_camp_table_var_bifold_center_hinge (parent A) | tabletop_left/right, tabletop_left_to_right (REVOLUTE) | converged |
| fold_mechanism | scissor / accordion collapsing base | ② | forked_anchor | rec_folding_camp_table_var_accordion_scissor_base (parent A) | scissor linkage members, central revolute pivots | converged |
| top_topology | fold-down drop-leaf side extension | ① | forked_anchor | rec_folding_camp_table_var_drop_leaf_extension (parent A) | drop_leaf, tabletop_to_drop_leaf (REVOLUTE) | converged |
| under_table_storage | none (bare frame) | ③ | origin_anchor | origin A | — | converged |
| under_table_storage | soft fabric+mesh hammock pocket | ③ | origin_anchor | origin B | pocket_bottom, front/rear_mesh_panel | converged |
| under_table_storage | rigid slatted lower shelf tier | ③ | forked_anchor | rec_folding_camp_table_var_rigid_lower_shelf (parent B) | lower shelf deck between legs | converged |
| slat_multiplicity | N=9 slats | N | origin_anchor | origin A (SLAT_COUNT=9) | slat_{i} loop | converged |
| slat_multiplicity | N=13 slats | N | origin_anchor | origin B (slat_count=13) | table_slat_{i} loop | converged |
| slat_multiplicity | N=6 wide coarse slats | N | forked_anchor | rec_folding_camp_table_var_slats_6 (parent A) | slat_{i} loop @ SLAT_COUNT=6 | converged |
| slat_multiplicity | N=20 narrow fine slats | N | forked_anchor | rec_folding_camp_table_var_slats_20 (parent B) | table_slat_{i} loop @ slat_count=20 | converged |

## Multiplicity / Copy Logic
- count_param: SLAT_COUNT (origin A) / slat_count (origin B)
- N samples: 6 (forked A), 9 (origin A), 13 (origin B), 20 (forked B)
- suggested N_range: 5–24 slats (depends on TABLE_X/depth and slat width)
- copied object: one aluminum slat visual (slat_{i} / table_slat_{i}); naming: indexed slat_{i}; placement: even spacing across the top with fixed SLAT_GAP; joint policy: slats are rigid members of the tabletop part (no per-slat joint). Secondary repeated set = 4 legs + their braces (leg count held at 4; not varied as a multiplicity axis because 3-leg camp tables are unstable/uncommon).

## Six-Axis Diversity Record
| axis | treatment | values / range / reason |
|---|---|---|
| ① skeleton / structural topology | source-backed | straight legs (A), telescoping legs (B), splayed A-frame, X-cross trestle, drop-leaf extension |
| ② joint / mechanism type | source-backed | revolute swing-fold legs (A), prismatic telescope (B), central bi-fold top hinge, scissor/accordion base; brace hinges revolute throughout |
| ③ primary form family | source-backed | surface: roll-top slats (A,B) / solid panel / fabric-mesh top; storage: none (A) / fabric hammock (B) / rigid shelf |
| ④ surface decoration | record_only / world_knowledge_extrapolation | screw-head rows, oval inserts, corner connectors, small edge logo mark (companion only; not a standalone variant) |
| ⑤ proportion / size / travel | record_only | compact ~0.80x0.54m square (A) vs ~1.06x0.65m rectangle (B); telescope travel 0–0.16m; ride-along on splayed/x-cross/drop-leaf/mesh forks |
| ⑥ material / palette / finish | record_only | origins are gray/black monotone; companion colorways offered: wood-grain/bamboo deck, coyote-tan/olive frame, two-tone half panels |

## Compatibility Probes
| probe | record_id | combined axes | risk tested | result |
|---|---|---|---|---|
| (none planned) | — | — | single-axis forks kept isolated; no risky cross-slot combos required | n/a |

## Blocked / Excluded
- Leg-count multiplicity (3 vs 4 vs 6 legs): excluded — 4-leg support is the near-universal stable form; 3-leg camp tables are uncommon/unstable, so not a useful template N axis.
- Wheeled serving-cart lower tier: excluded — drifts to camp kitchen/cart neighbor category.
- Pure colorway / material-only variants (tan frame, bamboo top): not standalone anchors (⑥ audit only); offered as companion variations on structural forks.
- Tilt/drafting top: excluded — not a real folding-camp-table form; drifts to workbench/drafting neighbor.
