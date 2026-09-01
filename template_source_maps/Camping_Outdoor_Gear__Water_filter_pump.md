<!--
subcategory_contract:
  category: Camping_Outdoor Gear
  subcategory: Water filter pump
  core_identity: a handheld, hand-powered water filter pump that forces source water through a filter cartridge via a reciprocating/driven hand actuator
  must_keep:
    - a filter body/cartridge (the thing water is pushed through)
    - a hand actuator with a real non-fixed joint (piston/plunger, lever, crank, or squeeze bulb) that drives the pump
    - dirty-water intake and clean-water outlet interfaces (hose barbs / ports)
  must_not_become:
    - gravity/squeeze straw filter with no pump actuator (e.g. LifeStraw / Sawyer squeeze)
    - electric/USB powered pump
    - generic hand tire pump / bike pump / car jack / lab filtration rig / plain water bottle
  image_evidence:
    - 001.png: olive twin-barrel housing (tall pump cylinder fused to shorter filter cylinder), T-bar paddle plunger reciprocating out the top, looped clear intake hose, output hose to a glass
    - 002.png: single olive cylinder with top manifold + green T-handle plunger, clear/smoky hoses to a clean cup and a brown-water sample cup, black rubber grip panel
  parent_evidence:
    - origin1 (72ff3145): twin fused cylinders, PRISMATIC body_to_handle piston + REVOLUTE body_to_filter_cap twist cap, 5 grip ribs, intake hose+prefilter, output hose+clip
    - origin2 (1c2f03adc): single cylinder, PRISMATIC plunger, two FIXED cups, two PRISMATIC detachable hose ports, 5 finger-scallop grip reliefs
    - origin3 (4b72f2e9): parallel filter cartridge + offset pump sleeve joined by 2 molded bridges, single PRISMATIC plunger, side outlet hose
-->

# Camping_Outdoor Gear / Water filter pump — template source map
pattern: mixed (linear_chain actuator + parallel_children ports/cups + multiplicity ribs/webs/legs)
parents:
- origin1 = rec_a-handheld-backpacking-water-microfilter-hand-pu_20260708_160830_693858_72ff3145 (picture/Camping_Outdoor Gear/Water filter pump/001.png) — twin-barrel, piston + twist cap
- origin2 = rec_camping_outdoor_gear__water_filter_pump_1c2f03adc4a5434c99f27df29d239b97 (picture/Camping_Outdoor Gear/Water filter pump/002.png) — single cylinder, plunger + detachable ports + cups
- origin3 = rec_camping_outdoor_gear__water_filter_pump_4b72f2e9740148e2b07ac2a2dfaa6a66 (picture/Camping_Outdoor Gear/Water filter pump/001.png) — parallel filter+sleeve, single plunger
canonical_baselines: none
underfilled_reason: none (normal budget met with coverage-first candidates)
note: origin1 and origin3 both reference 001.png but are two distinct authored originals (twin-barrel piston vs. parallel-sleeve plunger); both accounted on-grid.

## Slot Candidates
| slot | candidate | diversity_axis | source_type | record/evidence | key parts/joints | status |
|---|---|---|---|---|---|---|
| pump_actuation | piston push-pull plunger (T-bar paddle) | ② | origin_anchor | origin1 | pump_handle / body_to_handle PRISMATIC | on-grid |
| pump_actuation | piston push-pull plunger (T-handle) | ② | origin_anchor | origin2, origin3 | plunger(_handle) / body_to_plunger PRISMATIC | on-grid |
| pump_actuation | pivoting lever pump handle | ② | forked_anchor | rec_water_filter_pump_var_lever_pump (parent origin3) | plunger / body_to_plunger -> REVOLUTE lever | converged |
| pump_actuation | rotary hand-crank | ② | forked_anchor | rec_water_filter_pump_var_rotary_crank (parent origin2) | crank / body_to_plunger -> CONTINUOUS | converged |
| pump_actuation | squeeze-bulb / diaphragm | ② | forked_anchor | rec_water_filter_pump_var_squeeze_bulb (parent origin2) | squeeze_bulb / short-travel PRISMATIC | converged |
| filter_body_form | twin fused barrels (pump+filter side by side) | ③ | origin_anchor | origin1 | housing_pump, housing_filter | on-grid |
| filter_body_form | single integrated cylinder (pump internal) | ③ | origin_anchor | origin2 | cylindrical_body, plunger_sleeve | on-grid |
| filter_body_form | parallel filter + offset pump sleeve (bridged) | ③ | origin_anchor | origin3 | filter_body, pump_sleeve, lower/upper_bridge | on-grid |
| filter_body_form | inline coaxial pump-over-filter column | ③ | forked_anchor | rec_water_filter_pump_var_inline_coaxial (parent origin3) | filter_body + coaxial barrel | converged |
| filter_body_form | bottle-top threaded mount | ③ | forked_anchor | rec_water_filter_pump_var_bottle_top (parent origin2) | threaded collar + FIXED mounted bottle | converged |
| hose_port_config | fixed intake hose + output hose | ③ | origin_anchor | origin1 | intake_hose/intake_prefilter, output_hose/output_clip | on-grid |
| hose_port_config | detachable prismatic ports + cups | ②/③ | origin_anchor | origin2 | outlet_port, intake_port, body_to_*_port PRISMATIC | on-grid |
| hose_port_config | single side outlet hose | ③ | origin_anchor | origin3 | outlet_barb, curved_clear_hose, hose_end_fitting | on-grid |
| hose_port_config | two-stage inline pre-filter canister on intake | ① | forked_anchor | rec_water_filter_pump_var_inline_prefilter (parent origin1) | inline canister splicing intake_hose | converged |
| support_base | flat base pad / base cap | ① | origin_anchor | origin1/2/3 | base pad, rounded_bottom_cap, base_cap | on-grid |
| support_base | folding tripod stand (3 legs) | ① / N | forked_anchor | rec_water_filter_pump_var_tripod_stand (parent origin3) | leg_0..2 / base_to_leg_i REVOLUTE | converged |
| handle_grip_form | closed D-ring / loop pull handle | ③ | forked_anchor | rec_water_filter_pump_var_ring_pull_handle (parent origin1) | pump_handle ring grip on handle_rod | converged |

## Multiplicity / Copy Logic
- grip ribs (origin1): count_param = len(RIB_ANGLES_DEG); N samples = 3 (var_grip_ribs_sparse), 5 (origin1), 10 (var_grip_ribs_dense); suggested N_range 3–12; copied_object = body_rib_i; naming body_rib_{i}; placement = even angular spacing over grip arc; joint_policy = static (no joints).
- molded bridge webs (origin3): count_param = number of bridge boxes; N samples = 2 (origin3), 4 (var_bridge_webs_multi); suggested N_range 2–5; copied_object = bridge_web_i; naming bridge_web_{i}; placement = even Z stacking between the two columns; joint_policy = static.
- tripod legs (var_tripod_stand): count_param = leg count; N sample = 3; suggested N_range 3–4; copied_object = leg_i; naming leg_{i}; placement = radial 120°; joint_policy = one REVOLUTE fold joint per leg.
- finger scallops (origin2 green_grip_reliefs, 5) and detachable ports (origin2, 2) are additional record_only repeated patterns available for template sampling.

## Six-Axis Diversity Record
| axis | treatment | values / range / reason |
|---|---|---|
| ① skeleton / structural topology | source-backed | actuator subtree, inline pre-filter canister (added intake part), tripod base subtree with N=3 hinged legs |
| ② joint / mechanism type | source-backed | PRISMATIC piston (origins) -> REVOLUTE lever, CONTINUOUS crank, short-travel squeeze; plus REVOLUTE twist cap (origin1) and PRISMATIC detachable ports (origin2) |
| ③ primary form family | source-backed | twin-barrel / single cylinder / parallel-sleeve (origins) + inline coaxial column + bottle-top mount; grip form: T-bar / T-handle / ring-pull |
| ④ surface decoration | record_only / world_knowledge_extrapolation | molded grip ribs, black rubber grip panel + finger scallops, fluted/ribbed cap knurl, front recess/label panel — host-conformal only |
| ⑤ proportion / size / travel | record_only | body H ~0.15–0.26 m; pump stroke 0.075–0.085 m; cap twist 0.75 rad; port detach 0.030 m — may ride as companion only |
| ⑥ material / palette / finish | record_only | olive/dark-olive plastic, black rubber grips, clear/smoky flexible hose, grey fittings, blue connector, stainless rod; origins are olive-monotone so colorway rides as companion on several forks |

## Compatibility Probes
| probe | record_id | combined axes | risk tested | result |
|---|---|---|---|---|
| lever pump on twin-barrel body | rec_water_filter_pump_probe_lever_twin_barrel | ② lever x ③ twin-barrel | lever swing arc vs. neighboring filter cylinder + twist-cap envelope | converged |

## Blocked / Excluded
- gravity-hang / squeeze straw with no pump actuator: excluded — drops the core pump mechanism, becomes a neighbor subcategory (gravity/straw filter).
- electric/USB inline pump: excluded — not hand-powered, out of subcategory intent.
- pump_actuation candidate values combine freely with any ③ body form in principle, but lever-on-twin-barrel is gated behind the compatibility_probe above due to swing clearance.
