# 0611 / rivet_squeeze1 — template source map
pattern: parallel_children (symmetric two-handle squeeze) + swappable head/nose/bottle/grip modules + spare-tip multiplicity
parents: rec_use-the-attached-reference-image-as-the-primary-_20260713_122536_177242_f3961d86 | picture: pictureY/0611/rivet_squeeze1/001.png
canonical_baselines: (none — the origin seed is the on-grid baseline for every slot)
underfilled_reason: (none — 7 slots, 15 source-backed candidate anchors across seed + 13 variants)

subcategory_contract:
  category: 0611
  subcategory: rivet_squeeze1
  core_identity: a hand-powered two-handle lever rivet / rivet-nut setter that squeezes two handles about a central head to pull a mandrel through a top nosepiece, with a spent-mandrel catch bottle under the head
  must_keep: [central head with a top mandrel-pull nosepiece stack, two lever handles on a symmetric revolute squeeze (>=1 non-fixed joint), hand-powered actuation, under-head spent-mandrel bottle]
  must_not_become: [pneumatic/battery power rivet gun, bench/press riveter, generic pliers/crimper/wire-stripper]
  image_evidence: [two lever handles forming a wide V with rubber grips, a coloured cast head, a vertical top nose stack (hex nut + knurled collar + square mandrel tip), a translucent white catch bottle under the head, spare nose tips on a handle; colourways across the 12-tool reference sheet: red/orange/blue/green/yellow/teal/black]
  parent_evidence: [parts head/handle_0/handle_1; head visuals head_spine + red_head_body cover + two diagonal side_plate_left/right links seated in slots + solid lower_housing + recessed cover/housing hex screws + pull_rod; nose stack nose_washer+knurled_collar+hex_lock_nut+mandrel_tip; collection_bottle+bottle_adapter; spare_tip_rack+spare_tip_{0..2} (N=3); handle metal_arm+pivot_lug+grip_core+grip_overmold(+_back)+pivot_cap; joints head_to_handle_0 (revolute) + head_to_handle_1 (revolute, mimic of _0) = symmetric squeeze]

## Slot Candidates
| slot | candidate | diversity_axis | source_type | record/evidence | key parts/joints | status |
|---|---|---|---|---|---|---|
| handle_topology | wide_v (baseline) | ① skeleton | origin_anchor | origin seed | handle_0/1 metal_arm two-segment wide V; head_to_handle_0/1 (2× revolute mimic) | converged |
| handle_topology | long_straight | ① skeleton | forked_anchor | rec_rivet_squeeze1_var_handle_long_straight | long near-parallel straight metal_arm; 2× revolute squeeze kept | converged |
| handle_topology | compact_short | ① skeleton | forked_anchor | rec_rivet_squeeze1_var_handle_compact_short | short stubby steeply-angled metal_arm; 2× revolute squeeze kept | converged |
| handle_topology | cranked_offset | ① skeleton | forked_anchor | rec_rivet_squeeze1_var_handle_cranked_offset | dog-leg cranked metal_arm (blended knuckles); spare_tip_rack flush on arm; 2× revolute | converged |
| head_form | enclosed_cast (baseline) | ③ primary form | origin_anchor | origin seed | red_head_body cover + side_plate links + solid lower_housing (enclosed static toggle) | converged |
| head_form | inline_steel_linkage | ③ primary form | forked_anchor | rec_rivet_squeeze1_var_head_inline_steel_linkage | open spaced steel side-plates exposing the polished toggle linkage | converged |
| head_form | squared_cast | ③ primary form | forked_anchor | rec_rivet_squeeze1_var_head_squared_cast | rectangular boxy chamfered cast head block | converged |
| head_form | round_barrel | ③ primary form | forked_anchor | rec_rivet_squeeze1_var_head_round_barrel | rounded cylindrical barrel head (revolved) with side pivot bosses | converged |
| nosepiece_form | single_stack (baseline) | ③ form | origin_anchor | origin seed | nose_washer+knurled_collar+hex_lock_nut+mandrel_tip inline on head | converged |
| nosepiece_form | long_mandrel_stem | ③ form | forked_anchor | rec_rivet_squeeze1_var_nosepiece_long_mandrel_stem | tall exposed threaded mandrel stem + extended chuck above the head | converged |
| return_mechanism | none_static (baseline) | ② mechanism | origin_anchor | origin seed | enclosed static toggle, no exposed return spring | converged |
| return_mechanism | two_handle_return_coils | ② mechanism | forked_anchor | rec_rivet_squeeze1_var_mechanism_return_spring | one helical return coil per handle (visual on each handle, swings with it) + L anchor wire to pivot; 2× revolute kept | converged |
| grip_construction | two_tone_overmold (baseline) | ④ grip module | origin_anchor | origin seed | grip_core (black) + grip_overmold/_back (red rubber panels) | converged |
| grip_construction | single_ribbed_sleeve | ④ grip module | forked_anchor | rec_rivet_squeeze1_var_grip_single_ribbed_sleeve | one moulded ribbed rubber sleeve per handle (no two-tone panels) | converged |
| grip_construction | closed_loop_ring | ④ grip module | forked_anchor | rec_rivet_squeeze1_var_grip_closed_loop_ring | closed D-ring loop grip at each arm end | converged |
| collection_bottle | compact_underslung (baseline) | ④ module | origin_anchor | origin seed | collection_bottle + bottle_adapter straight below the head | converged |
| collection_bottle | large_canister | ④ module | forked_anchor | rec_rivet_squeeze1_var_bottle_large_canister | taller/larger cylindrical screw-on catch canister on same adapter | converged |
| collection_bottle | angled_side_mount | ④ module | forked_anchor | rec_rivet_squeeze1_var_bottle_angled_side_mount | bottle mounted on a diagonal stub off one side of the head | converged |
| spare_tip_multiplicity | rack_N3 (baseline) | N | origin_anchor | origin seed | spare_tip_rack + spare_tip_{0..2} loop, N=3 | converged |
| spare_tip_multiplicity | magazine_N6 | N | forked_anchor | rec_rivet_squeeze1_var_tip_magazine_six | onboard indexed magazine, tip_{i} loop range(6), N=6 | converged |

## Multiplicity / Copy Logic
- count_param: spare_tip count on the onboard holder (spare_tip_{i} / tip_{i})
- N samples: {3 (seed rack), 6 (magazine)}
- suggested N_range: 3–8
- copied object: one nose-tip solid; naming: spare_tip_{i} (seed) / tip_{i} (magazine); shared geometry helper
- placement: seed = linear row on a rack beam on handle_0; magazine = evenly spaced row/arc seated in a magazine block on handle_0
- joint policy: fixed decorations (host visuals), not separate joints
- secondary multiplicity: recessed head screws (cover/housing_front|rear_screw_{0,1}) and brand_mark_{0..2} are loop-emitted host decorations (④), not a candidate-anchor N axis

## Six-Axis Diversity Record
| axis | treatment | values / range / reason |
|---|---|---|
| ① skeleton / structural topology | source-backed | handle_topology: wide_v (origin), long_straight, compact_short, cranked_offset (all keep the 2× revolute symmetric squeeze) |
| ② joint / mechanism type | source-backed | symmetric revolute squeeze (origin, mimic pair) is fixed identity; return_mechanism module adds two per-handle return coils (visual, swings with handle) — no new powered joint |
| ③ primary form family | source-backed | head_form: enclosed_cast (origin), inline_steel_linkage, squared_cast, round_barrel; nosepiece_form: single_stack (origin), long_mandrel_stem |
| ④ surface decoration / modules | record_only / world_knowledge_extrapolation | grip_construction (overmold/single-sleeve/closed-loop), collection_bottle (compact/large/angled), recessed hex screws, branded_plate+brand_mark, knurled_collar knurling — host-conformal, non-structural |
| ⑤ proportion / size / travel | record_only | arm length (short compact ↔ long straight), squeeze travel (revolute upper ≈ 0.35 rad), nose stem length, bottle height |
| ⑥ material / palette / finish | record_only | 13 distinct real colourways observed across the variant pool + seed → palette_style vocabulary: red (seed), black, orange, steel/silver, green, blue, yellow, teal, purple, off-white, bronze, chrome, gunmetal; grips two-tone rubber or plain black; translucent white bottle; dark recessed steel screws |

## Compatibility Probes
| probe | record_id | combined axes | risk tested | result |
|---|---|---|---|---|
| (none) | — | — | — | — |

## Blocked / Excluded
- battery/electric/pneumatic power rivet gun: out of subcategory (power tool), not hand-powered
- bench/press-mounted riveter: out of subcategory (not handheld)
- nose turret / rotating-swivel-head / lazy-tong-concertina / hydraulic-cylinder / right-angle-head / compound-bell-crank: earlier exploratory forks, deleted by author as off-reference (the 12-tool reference sheet shows only two-handle lever setters); intentionally NOT in the current pool
- single origin only (001.png); all 13 variants fork from it — no sibling origins to account for
