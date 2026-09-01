<!--
subcategory_contract:
  category: Emergency Equipment
  subcategory: Fire escape ladder
  core_identity: a deployable/climbable ladder used for emergency egress from a building
  must_keep: [two parallel side members with regularly spaced horizontal rungs, a mounting/anchor interface to a building, at least one real deploy joint (slide/fold/swing/hook pivot) or clear deploy path]
  must_not_become: [step stool / stepladder A-frame, scaffold or work tower, balcony railing / guardrail / handrail, rope-only or cargo net, fire-truck aerial ladder]
  image_evidence: [001/002 show a galvanized exterior wall-mounted ladder, cylindrical safety cage of horizontal hoops + vertical ribs, distributed wall standoff brackets with anchor plates, a front sliding lower drop-section with yellow/red release sleeves, arrows indicating up/down travel]
  parent_evidence: [origin A: upper_ladder+lower_ladder, rail_0/1, rung_{i} loop, cage_hoop_{i}+cage_bar_{i}, standoff/wall_plate array, PRISMATIC ladder_to_lower, release_latch; origin B: fixed_ladder+lower_ladder+safety_gate, fixed_side_rail_{s}, fixed_rung_{i}=range(14), cage_hoop_{i}+cage_vertical_rib, wall_standoff+wall_anchor_plate, PRISMATIC fixed_to_lower_ladder + REVOLUTE fixed_to_safety_gate, yellow_release_sleeve/rubber_foot]
-->

# Emergency Equipment / Fire escape ladder — template source map
pattern: mixed (parallel_children + multiplicity + linear_chain of forked families)
parents:
- rec_emergency_equipment__fire_escape_ladder_20b875b6a2eb4af79b8c9f7f8c71a939 (origin A) — picture/Emergency Equipment/Fire escape ladder/002.png — wall_fire_escape_ladder_cage: rigid caged ladder + PRISMATIC drop section (no gate)
- rec_emergency_equipment__fire_escape_ladder_e6a5530c1933414f9994ed542b5e0784 (origin B) — picture/Emergency Equipment/Fire escape ladder/001.png — exterior_fire_escape_ladder_with_cage: rigid caged ladder + PRISMATIC drop section + REVOLUTE safety gate
canonical_baselines: none
underfilled_reason: none (14 candidate anchors = 2 origins + 12 forks, within normal 12–18)

Both origins occupy the same grid cell: ③ rigid-fixed-cage steel wall ladder with a ② PRISMATIC lower drop-section. Origin B additionally source-backs a ② REVOLUTE hinge (safety gate). Family A (portable roll-up/chain/strap window escape ladder) is absent from origins and is forked into source-backed anchors. Each candidate is forked from the nearest origin.

## Slot Candidates
| slot | candidate | diversity_axis | source_type | record/evidence | key parts/joints | status |
|---|---|---|---|---|---|---|
| ladder_family (③ primary form family) | rigid fixed caged wall ladder | ③ | origin_anchor | origins A & B | fixed_side_rail/rail_{s}, rung_{i}, cage_hoop_{i} | on-grid (origin A & B) |
| ladder_family | portable roll-up chain-link escape ladder (hooks over sill) | ③ | forked_anchor | fork from A | chain_link_{s}_{i}, rung_{i}, sill_hook + sill_hook_pivot (REVOLUTE) | converged (rec_fire_escape_ladder_var_chain_rollup) |
| ladder_family | multi-section folding/articulated escape ladder | ③/② | forked_anchor | fork from B | ladder_seg_{n}, seg{n}_rung_{i}, fold_hinge_{n} (REVOLUTE) | converged (rec_fire_escape_ladder_var_folding_articulated) |
| ladder_family | multi-stage telescoping/collapsible escape ladder | ② | forked_anchor | fork from A | stage_{n}, stage{n}_rung_{i}, stageN_to_stageM (PRISMATIC) | converged (rec_fire_escape_ladder_var_telescoping_multistage) |
| side_member_type (① skeleton/topology) | rigid round steel rails | ① | origin_anchor | origins A & B | rail_0/1, fixed_side_rail_{s} | on-grid |
| side_member_type | flexible chain-link side members | ① | forked_anchor | via chain_rollup | chain_link_{s}_{i} | converged (covered by chain_rollup) |
| side_member_type | flexible woven webbing/strap side members | ① | forked_anchor | fork from A | side_strap_{s} / strap_segment_{i}, rung_{i} | converged (rec_fire_escape_ladder_var_strap_webbing) |
| anchor_hook_interface (② joint/mechanism) | bolted wall standoff + anchor plates | ② | origin_anchor | origins A & B | standoff_{b}_{s}, wall_plate/wall_anchor_plate | on-grid |
| anchor_hook_interface | over-the-windowsill folding hook | ② | forked_anchor | fork from A | sill_hook_arm, sill_hook_finger_{s}, sill_hook_pivot (REVOLUTE) | converged (rec_fire_escape_ladder_var_windowsill_hook) |
| anchor_hook_interface | roof/parapet hook-over bracket | ② | forked_anchor | fork from B | parapet_hook_arm, parapet_hook_leg_{s}, parapet_hook_pivot (REVOLUTE) | converged (rec_fire_escape_ladder_var_parapet_roof_hook) |
| guard_system (② guard topology) | full cylindrical safety cage | ② | origin_anchor | origins A & B | cage_hoop_{i}, cage_bar_{i}/cage_vertical_rib | on-grid |
| guard_system | cageless open ladder + top grab-rail | ② | forked_anchor | fork from B | grab_rail_{s}, extended fixed_side_rail | converged (rec_fire_escape_ladder_var_no_cage_grabrail) |
| guard_system | central fall-arrest guide rail + sliding shuttle | ② | forked_anchor | fork from A | arrest_rail, arrest_bracket_{i}, arrest_shuttle, shuttle_to_rail (PRISMATIC) | converged (rec_fire_escape_ladder_var_fall_arrest_rail) |
| lower_access_mechanism (② joint) | PRISMATIC sliding drop-section | ② | origin_anchor | origins A & B | lower_ladder, ladder_to_lower/fixed_to_lower_ladder (PRISMATIC) | on-grid |
| lower_access_mechanism | REVOLUTE counterweighted swing-down drop-section | ② | forked_anchor | fork from A | lower_ladder, lower_swing_hinge (REVOLUTE), swing_pin/sleeve, counterweight_arm | converged (rec_fire_escape_ladder_var_hinged_drop_section) |
| rung_count_multiplicity (N) | N=4 rungs | N | forked_anchor | fork from B | fixed_rung_{i} via range(4) | converged (rec_fire_escape_ladder_var_rungs_n4) |
| rung_count_multiplicity (N) | N=8 rungs | N | forked_anchor | fork from B | fixed_rung_{i} via range(8) | converged (rec_fire_escape_ladder_var_rungs_n8) |
| rung_count_multiplicity (N) | N=12 rungs | N | forked_anchor | fork from B | fixed_rung_{i} via range(12) | converged (rec_fire_escape_ladder_var_rungs_n12) |

## Multiplicity / Copy Logic
- count_param: rung count of the climbing ladder run (origin B: `rung_zs = [1.62 + 0.32 * i for i in range(14)]`; origin A: while-loop `rung_z += 0.30`).
- N samples (source-backed via forks): 4, 8, 12 (origins already source-back 14 and ~17).
- suggested N_range: 3–20 rungs (single-storey stub to multi-storey run).
- copied object: single horizontal rung cylinder emitted by `_cyl_x`.
- naming: stable indexed `fixed_rung_{i}` / `rung_{i}` (and per-segment `seg{n}_rung_{i}`, per-stage `stage{n}_rung_{i}` in family variants).
- placement: even vertical pitch along z (~0.30–0.32 m), rails/cage/standoffs scale to bracket the run.
- joint policy: rungs are welded fixed (no per-rung joint); multiplicity fork changes only the rung count, not body family/joints/anchor.
- secondary multiplicity (record_only, not forked separately): cage hoop count `cage_hoop_{i}` and standoff-level count also loop-driven; expose but do not enumerate to avoid padding.

## Six-Axis Diversity Record
| axis | treatment | values / range / reason |
|---|---|---|
| ① skeleton / structural topology | source-backed | rigid round steel rails (origin) vs flexible chain-link side members vs flexible webbing/strap side members; two-parallel-members + rungs invariant preserved |
| ② joint / mechanism type | source-backed | PRISMATIC drop (origin A&B), REVOLUTE hinge (origin B gate; forked swing drop + fold hinges), PRISMATIC multi-stage telescope, PRISMATIC fall-arrest shuttle, REVOLUTE hook pivots (sill/parapet) |
| ③ primary form family | source-backed | rigid fixed caged wall ladder (origin) vs portable roll-up chain ladder vs folding articulated ladder vs telescoping collapsible ladder |
| ④ surface decoration | record_only / world_knowledge_extrapolation | safety-yellow/red release sleeves & latch tabs (origin), up/down travel arrows, hazard striping on release sleeves — host-conformal only, no dedicated variant |
| ⑤ proportion / size / travel | record_only | rung pitch ~0.30–0.32; rung count 3–20; slide travel (origin lower≈0..0.95–1.20); cage radius ~0.43–0.58; standoff depth ~0.18–0.39; rides along as companion on rung-N and hook forks |
| ⑥ material / palette / finish | record_only | galvanized_steel / shadowed_steel base; safety_yellow, red_latch accents; black rubber feet; companion colorways: anodized-aluminium, safety-orange rails, zinc vs black-oxide chain, painted-red brackets — palette only |

## Compatibility Probes
| probe | record_id | combined axes | risk tested | result |
|---|---|---|---|---|
| (none planned) | — | — | family × guard combos (e.g. cage on a chain roll-up ladder) are physically incoherent and excluded rather than probed | n/a |

## Blocked / Excluded
- cylindrical cage on flexible chain/strap or folding/telescoping families: incoherent (cages exist only on rigid fixed ladders); families are forked cageless — recorded as gated, not probed.
- rope-only / single-line descent device: excluded, not a climbable two-rail ladder (neighbor category, drifts out of subcategory).
- ground-supported leaning extension ladder / stepladder A-frame: excluded as neighbor category (not building-egress mounted).
- separate cage-hoop-count and standoff-count multiplicity variants: not forked (would pad); copy logic recorded under Multiplicity instead.
