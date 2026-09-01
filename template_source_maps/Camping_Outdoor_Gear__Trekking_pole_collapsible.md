<!--
subcategory_contract:
  category: Camping_Outdoor Gear
  subcategory: Trekking pole collapsible
  core_identity: a hand-held collapsible trekking/hiking pole whose shaft packs down via a collapse mechanism (telescoping sections OR folding Z-fold sections), with a top grip, wrist strap, a ground tip, and a removable basket
  must_keep: [grip/handle at top, collapsible segmented shaft, ground contact tip, at least one real non-fixed collapse joint (prismatic telescope or revolute fold)]
  must_not_become: [tent pole, ski pole, walking cane/crutch, monopod/tripod, avalanche probe, fishing rod]
  image_evidence:
    - 002.png (origin A): single pole folded into a compact 3-tube bundle, cork ergonomic handle with palm hook, black flip-lock levers, wrist webbing + accessory carabiner, plus a kit of interchangeable baskets (snow + trekking), rubber feet and paw tips, carry bag
    - 001.png (origin B): a pair of poles, telescoping segmented shafts (white upper / carbon lower), cork handles, black foam sub-grips, flip clamps, wrist straps, small baskets and pointed tips
  parent_evidence:
    - origin A model: folding Z-fold via REVOLUTE upper_to_middle_fold + middle_to_lower_fold, PRISMATIC lower_to_tip_slide, flip locks upper_lock/lower_lock, _add_fold_hinge_hardware, _add_lock_lever, _add_basket, _ergonomic_handle_mesh cork handle, palm_hook, carbide_point + rubber_tip
    - origin B model: telescoping flick-lock, PRISMATIC upper_to_mid_{i} + mid_to_lower_{i}, REVOLUTE flick levers upper_clamp_hinge_{i}/lower_clamp_hinge_{i}, loop over POLE_X for a pair of poles, _make_cork_handle_mesh, _make_foam_grip_mesh, _make_tip_mesh, _make_basket_mesh
-->

# Camping_Outdoor Gear / Trekking pole collapsible — template source map
pattern: mixed (linear_chain collapse stages + multiplicity on section count; origin B also parallel_children pair of poles)
parents:
- rec_camping_outdoor_gear__trekking_pole_collapsible_306eae8f686b4c709cddeb5eb80aa07f (origin A) — picture/Camping_Outdoor Gear/Trekking pole collapsible/002.png — single folding Z-fold pole
- rec_camping_outdoor_gear__trekking_pole_collapsible_da9d6d602d774a8f9b9822d3f5c7a5e4 (origin B) — picture/Camping_Outdoor Gear/Trekking pole collapsible/001.png — pair of telescoping flick-lock poles
canonical_baselines: none
underfilled_reason: none (12 candidate anchors within simple→normal budget)

## Slot Candidates
| slot | candidate | diversity_axis | source_type | record/evidence | key parts/joints | status |
|---|---|---|---|---|---|---|
| collapse_mechanism | folding Z-fold sections (+ telescoping tip) | ② joint/mechanism | origin_anchor | origin A | upper_section/middle_section/lower_section, upper_to_middle_fold, middle_to_lower_fold (REVOLUTE), lower_to_tip_slide (PRISMATIC) | converged (on-grid) |
| collapse_mechanism | telescoping flick-lock sections | ② joint/mechanism | origin_anchor | origin B | mid_stage_{i}/lower_stage_{i}, upper_to_mid_{i}/mid_to_lower_{i} (PRISMATIC), upper_clamp_hinge_{i}/lower_clamp_hinge_{i} (REVOLUTE) | converged (on-grid) |
| collapse_mechanism | telescoping twist-lock (expander) | ② joint/mechanism | forked_anchor | rec_trekking_pole_collapsible_var_twist_lock (from B) | replace flick levers/hinges with CONTINUOUS twist collars; keep PRISMATIC telescope | converged |
| collapse_mechanism | pure Z-fold tent-pole (no telescope) | ② joint/mechanism | forked_anchor | rec_trekking_pole_collapsible_var_pure_fold_tentpole (from A) | replace lower_to_tip_slide PRISMATIC with a 3rd REVOLUTE fold; shock-cord proxy | converged |
| section_multiplicity | N=2 telescoping sections | N multiplicity | forked_anchor | rec_trekking_pole_collapsible_var_n2_telescope (from B) | one PRISMATIC joint + one flick lever per pole | converged |
| section_multiplicity | N=3 telescoping sections | N multiplicity | origin_anchor | origin B | upper_to_mid_{i} + mid_to_lower_{i} | converged (on-grid) |
| section_multiplicity | N=4 telescoping sections | N multiplicity | forked_anchor | rec_trekking_pole_collapsible_var_n4_telescope (from B) | add mid2_stage_{i} + PRISMATIC joint + flick lever, loop-built | converged |
| section_multiplicity | N=3 folding sections | N multiplicity | origin_anchor | origin A | upper/middle/lower fold segments | converged (on-grid) |
| section_multiplicity | N=4 folding sections | N multiplicity | forked_anchor | rec_trekking_pole_collapsible_var_n4_fold (from A) | add fold tube segment + REVOLUTE fold, loop-built | converged |
| section_multiplicity | N=5 folding sections | N multiplicity | forked_anchor | rec_trekking_pole_collapsible_var_n5_fold (from A) | add two fold tube segments + REVOLUTE folds, loop-built | converged |
| grip_type | cork ergonomic handle | ③ form family | origin_anchor | origin A + B | cork_handle / _ergonomic_handle_mesh / _make_cork_handle_mesh, palm_hook, top_cap | converged (on-grid) |
| grip_type | EVA foam straight grip | ③ form family | forked_anchor | rec_trekking_pole_collapsible_var_foam_grip (from B) | replace cork lathe handle with long foam grip, merge foam sub-grip | converged |
| grip_type | T-crossbar cane handle | ③ form family | forked_anchor | rec_trekking_pole_collapsible_var_t_handle (from A) | replace cork handle/palm_hook/top_cap with perpendicular T grip (static) | converged |
| tip_and_basket | carbide point + small trekking basket | ③ form family | origin_anchor | origin A + B | carbide_point/carbide_tip, basket_ring, _add_basket/_make_basket_mesh | converged (on-grid) |
| tip_and_basket | large snow basket (radial spokes) | ③ form family | forked_anchor | rec_trekking_pole_collapsible_var_snow_basket (from B) | enlarge basket torus + loop-emitted basket_spoke_{k} | converged |
| tip_and_basket | rubber walking-foot (paw) tip | ③ form family | forked_anchor | rec_trekking_pole_collapsible_var_rubber_foot_tip (from A) | replace carbide_point spike with angled rubber foot over inner_ferrule | converged |

## Multiplicity / Copy Logic
- count_param: number of collapse sections per pole (telescoping stages OR fold segments)
- N samples:
  - telescoping (from origin B): N=2 (var_n2_telescope), N=3 (origin B), N=4 (var_n4_telescope)
  - folding (from origin A): N=3 (origin A), N=4 (var_n4_fold), N=5 (var_n5_fold)
- suggested N_range: telescoping 2–4; folding 3–6
- copied object: a shaft-section part (telescoping: tube + sleeve + clamp collar + flick lever; folding: tube + ferrule + hinge hardware + connector bridge)
- naming: indexed stage/segment names emitted from a loop, e.g. mid_stage_{i} / lower_stage_{i} and fold segment_{k}; joints upper_to_mid_{i} / *_fold_{k}
- placement rule: telescoping = linear coaxial nested chain by decreasing radius (PRISMATIC z-axis); folding = alternating Z-fold chain by REVOLUTE joints with alternating swing sign, parallel-when-folded
- joint policy: one PRISMATIC per telescoping stage (+ one REVOLUTE flick lever) OR one REVOLUTE per fold joint; keep exactly the collapse joint added/removed per N step
- secondary multiplicity (record_only, not the N axis): pole-count kit (origin A = 1 pole, origin B = 2 poles via POLE_X loop); basket spoke count (origin A = 6 radial spokes loop, origin B = 2 cross spokes, snow-basket fork ~8 spokes)

## Six-Axis Diversity Record
| axis | treatment | values / range / reason |
|---|---|---|
| ① skeleton / structural topology | source-backed | linear collapse chain (grip → nested/folded sections → tip); origin B adds parallel pair-of-poles topology; kept across all variants |
| ② joint / mechanism type | source-backed | telescoping flick-lock (B, PRISMATIC + REVOLUTE lever), folding Z-fold (A, REVOLUTE + PRISMATIC tip), telescoping twist-lock (fork, CONTINUOUS collar), pure fold (fork) |
| ③ primary form family | source-backed | grip family (cork / EVA foam / T-crossbar cane); tip family (carbide point / rubber walking foot); basket family (small trekking / large snow) |
| ④ surface decoration | record_only / world_knowledge_extrapolation | carbon-look label strips (pole_{i}_carbon_label), etched shaft graphics, branded strap tags/webbing; host-conformal only, no dedicated variant |
| ⑤ proportion / size / travel | record_only | deployed length ~1.1–1.3 m; folded bundle length shrinks with more sections; telescoping travel per stage; may ride as companion (T-handle shorter, Z-fold shorter pack) |
| ⑥ material / palette / finish | record_only | origins are black/graphite/cork monotone; ride-along colorways: blue anodized (twist_lock), red/orange snow basket, accent-band foam grip |

## Compatibility Probes
| probe | record_id | combined axes | risk tested | result |
|---|---|---|---|---|
| twist-lock + N=4 telescoping | rec_trekking_pole_collapsible_var_probe_twist_n4 | ② twist-lock mechanism + N=4 sections | nested twist-collar / sleeve-mouth clearance and coaxial retention through 4 stages | converged |

## Blocked / Excluded
- pole-count-only variant (1 vs 2 poles): excluded as standalone — it is kit/packaging multiplicity, not a per-pole structural axis; recorded as record_only across origins.
- ski pole / avalanche probe / walking cane substitutions: excluded as neighbor-category drift (must keep trekking handle + collapsible segmented shaft + basket + tip).
- carry bag and standalone carabiner accessory: excluded as packaging/background (origin A already models a token strap+carabiner attached to the handle; not a separate variant).
- ④/⑤/⑥-only changes: excluded as standalone variants per rules; recorded above and attached as companion variations where natural.
