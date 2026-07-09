<!--
subcategory_contract:
  category: Camping_Outdoor Gear
  subcategory: Camp chair
  core_identity: a portable, foldable/packable outdoor seat with a fabric (or sling) seating surface carried on a collapsible tube/pole frame
  must_keep: [portable folding/collapsible frame, fabric or sling seating surface, at least one real non-fixed folding/reclining/swivel joint]
  must_not_become: [rigid indoor dining/office chair, gas-lift task chair, patio glider bench, camp cot with no chair mode, hammock, picnic table]
  image_evidence:
    - "002.png (origin A): lounge/recliner camp chair with fabric sling seat+back, hinged fabric arm rests, cup holder, head bolster, reclining footrest extension, crossed scissor tube legs, rubber pad feet"
    - "001.png (origin B): high-back padded quad-fold armchair, black/gray oxford fabric with orange piping, fixed padded arms, cup holder ring, mesh side pocket, X-scissor braces, four plastic pad feet"
  parent_evidence:
    - "origin A (folding_lounge_camp_chair): parts base_frame (single bent perimeter/back tube scaffold), seat_panel, back_panel, footrest_panel, left/right_armrest, left/right_arm_support, left/right_cross_leg_0/1, cup_holder, head_pillow, feet; joints base_to_footrest/base_to_*_armrest/base_to_*_arm_support/base_to_*_cross_leg_* are REVOLUTE; helpers _fabric_panel_geometry, _straight_tube, _loop_tube, _edge_loop"
    - "origin B (folding_camping_chair): single chair_frame part holds all tube/fabric visuals; children front_cross_brace, side_cross_brace_0, side_cross_brace_1 with REVOLUTE joints frame_to_front_cross/frame_to_side_cross_0/frame_to_side_cross_1; helpers _tube_pose, _add_tube, _add_ball, _add_box"
-->

# Camping_Outdoor Gear / Camp chair — template source map
pattern: mixed (parallel_children scissor/recline/swivel braces + multiplicity for leg-count and seat-count)
parents:
- rec_camping_outdoor_gear__camp_chair_da47f30a91aa4278b213ea8f9ebf9b93 (origin A, lounge/recliner) — picture/Camping_Outdoor Gear/Camp chair/002.png
- rec_camping-outdoor-gear-camp-chair-001-png-use-the-_20260706_151429_784260_ff85d60d (origin B, high-back quad-fold armchair) — picture/Camping_Outdoor Gear/Camp chair/001.png
canonical_baselines: none
underfilled_reason: none (normal richness; 15 candidate anchors within 12–18 budget)

## Slot Candidates
| slot | candidate | diversity_axis | source_type | record/evidence | key parts/joints | status |
|---|---|---|---|---|---|---|
| frame_skeleton / support_base | quad X-brace tubular armchair | ① | origin_anchor | origin B chair_frame + front/side cross braces | chair_frame, front_cross_brace, side_cross_brace_0/1, frame_to_*_cross | converged (origin) |
| frame_skeleton / support_base | scissor-leg lounge frame (single bent-tube scaffold) | ① | origin_anchor | origin A base_frame + 4 cross legs | base_frame, left/right_cross_leg_0/1, base_to_*_cross_leg_* | converged (origin) |
| frame_skeleton / support_base | director's-chair rectangular side-fold frame w/ rigid arms | ① | forked_anchor | rec_camp_chair_var_director_frame (from B) | chair_frame, side_cross_brace_0/1, frame_to_side_cross_0/1 | converged |
| frame_skeleton / support_base | low pole-and-hub backpacking frame | ① | forked_anchor | rec_camp_chair_var_low_pole_hub (from B) | chair_frame, front_cross_brace, frame_to_front_cross | converged |
| frame_skeleton / support_base | backless X-frame folding stool | ③/① | forked_anchor | rec_camp_chair_var_xframe_stool (from B) | chair_frame, seat_gray_center, all three scissor braces | converged |
| frame_skeleton / support_base | rocker/glider curved-runner base | ① | forked_anchor | rec_camp_chair_var_rocker_base (from B) | chair_frame feet -> rocker runners, scissor braces kept | converged |
| frame_skeleton / support_base | butterfly crossed-loop sling frame | ① | forked_anchor | rec_camp_chair_var_butterfly_sling (from B) | chair_frame, seat_gray_center, front_cross_brace | converged |
| frame_skeleton / support_base | tall bar-height frame + foot rail | ① | forked_anchor | rec_camp_chair_var_bar_height (from B) | chair_frame legs elongated + added foot rail, scissor braces | converged |
| body_form (seat/back envelope) | high-back padded armchair sling | ③ | origin_anchor | origin B | seat_gray_center, back_gray_center | converged (origin) |
| body_form (seat/back envelope) | lounge w/ reclining footrest | ③ | origin_anchor | origin A | seat_panel, back_panel, footrest_panel | converged (origin) |
| body_form (seat/back envelope) | round moon/saucer bucket sling | ③ | forked_anchor | rec_camp_chair_var_moon_saucer (from B) | chair_frame round rim + bowl sling, scissor braces | converged |
| body_form (seat/back envelope) | flat chaise/cot lounger (near-coplanar) | ③ | forked_anchor | rec_camp_chair_var_flat_cot_lounger (from A) | seat_panel, back_panel, footrest_panel, base_to_footrest | converged |
| opening_or_motion (joint) | reclining footrest hinge | ② | origin_anchor | origin A base_to_footrest REVOLUTE | footrest_panel, base_to_footrest | converged (origin) |
| opening_or_motion (joint) | hinged fabric armrests | ② | origin_anchor | origin A base_to_*_armrest REVOLUTE | left/right_armrest, base_to_*_armrest | converged (origin) |
| opening_or_motion (joint) | adjustable back-recline hinge | ② | forked_anchor | rec_camp_chair_var_recline_back (from B) | new back_recliner child + frame_to_recline_back REVOLUTE | converged |
| opening_or_motion (joint) | 360-degree swivel seat | ② | forked_anchor | rec_camp_chair_var_swivel_seat (from B) | new base_to_swivel CONTINUOUS + seat_swivel child | converged |
| opening_or_motion (joint) | zero-gravity recline-lock pivot | ② | forked_anchor | rec_camp_chair_var_zero_gravity (from A) | new base_to_recline_pivot REVOLUTE, base_to_footrest kept | converged |
| multiplicity: leg count | tripod (N=3) vs quad (N=4) support | N | forked_anchor | rec_camp_chair_var_tripod_stool (from B) | radial leg_{i} loop + apex fold pivot | converged |
| multiplicity: seat count | double/triple bench (N=2/3 seat cells) | N | forked_anchor | rec_camp_chair_var_bench_multi (from B) | seat_{i} cell loop, one scissor brace per bay | converged |

## Multiplicity / Copy Logic
- count_param: (a) leg count for tripod/quad stool base; (b) seat_cell count for the folding bench/loveseat
- N samples: legs N=3 (tripod fork), N=4 (both origins); seat_cells N=1 (origins), N=2 (loveseat fork), N=3 (triple bench fork)
- suggested N_range: legs 3–4 (free-standing minimum 3); seat_cells 1–3 (2–3 for benches)
- copied object / naming / placement / joint policy:
  - legs: copied object = leg+foot; naming leg_{i}; placement radial at 360/N deg from a shared apex ring; joint = single revolute fold at apex
  - seat_cells: copied object = seat sling + back panel + divider leg; naming seat_{i}; placement tiled along +Y at fixed seat pitch; joint = one scissor fold brace per bay, mirrored outer side braces preserved
  - both origins already emit the 4 scissor legs / 3 braces via loops (origin A scissor_specs loop, origin B per-brace parts); template extraction should reuse loop-based copy

## Six-Axis Diversity Record
| axis | treatment | values / range / reason |
|---|---|---|
| ① skeleton / structural topology | source-backed | quad X-brace (B), scissor-leg lounge (A), director side-fold, low pole-hub, X-frame stool, rocker-runner base, butterfly crossed-loop, tall bar-height frame |
| ② joint / mechanism type | source-backed | footrest-lift hinge (A), hinged fabric arms (A), quad/side scissor folds (both), back-recline hinge, 360-deg swivel (continuous), zero-gravity recline-lock; all REVOLUTE/CONTINUOUS with MotionLimits |
| ③ primary form family | source-backed | high-back padded armchair (B), footrest lounge (A), backless stool, round moon/saucer bucket, flat chaise/cot lounger |
| ④ surface decoration | record_only / world_knowledge_extrapolation | orange piping + white contrast stitching + brand logo patch (B), black edge piping + light stitch rails + orange accent bands (A); host-conformal only |
| ⑤ proportion / size / travel | record_only | seat height standard vs bar-height vs low backpacking; compact tripod vs wide bench; upright vs reclined vs flat-bed travel; kids-compact scale (recorded, not forked alone) |
| ⑥ material / palette / finish | record_only + companion | origins: black/gray oxford + orange (B), dark-brown + orange (A), powder-coated gray steel tube; companion colorways rode along: tan/olive canvas, blue/red ripstop, teal/navy bowl, green/camo, hardwood arms |

## Compatibility Probes
| probe | record_id | combined axes | risk tested | result |
|---|---|---|---|---|
| full recliner (back-recline + footrest-lift) | rec_camp_chair_var_probe_full_recliner (from A) | ② back-recline hinge + ② footrest-lift hinge | self-collision/clearance between back_panel/head_pillow, arm rests, raised footrest; single scissor base stability across full pose envelope | converged |

## Blocked / Excluded
- footrest-add fork on origin B: excluded as a standalone anchor — the reclining footrest is already an origin_anchor on origin A (would duplicate an existing candidate).
- hinged-armrest fork: excluded as standalone — already an origin_anchor on origin A (base_to_*_armrest REVOLUTE).
- pure colorway / stitching / logo variants: excluded as standalone (⑥/④ audit-only); ride along as companion variations.
- kids-compact / oversized-only variants: excluded as standalone (⑤ proportion audit-only).
- wheeled / gas-lift / five-star-base / powered-recliner chairs: blocked (neighbor category drift out of Camp chair).
