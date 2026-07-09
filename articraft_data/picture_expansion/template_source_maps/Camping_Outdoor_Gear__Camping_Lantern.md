<!--
subcategory_contract:
  category: Camping_Outdoor Gear
  subcategory: Camping Lantern
  core_identity: a portable self-contained area light (battery/fuel LED) with a light-emitting body plus a way to carry/hang or stand it at a campsite
  must_keep:
    - a diffusing/glowing light body (chamber, globe, cage+diffuser, panel)
    - a carry/hang or support feature (bail, hook, base, legs, stake)
    - at least one real non-fixed joint (fold-up handle/bail, fold-out legs, telescoping/twist collapse)
  must_not_become:
    - handheld flashlight/torch (end-emitting, no area diffuser + hang/stand)
    - fixed ceiling/pendant light fixture or table lamp (not portable campsite gear)
    - oil lamp with a live wick burner, tiki torch, or chemical glow-stick
    - non-lighting cylinder (thermos, speaker, water bottle)
  image_evidence:
    - 001.png: compact black/graphite collapsible LED canister; ribbed lower base; clear chamber with LED strip; fold-up U-shaped wire bail; telescoping pull-up collapse
    - 002.png: cylindrical wire-guard cage around a warm glowing diffuser column; olive/black vented base and top control block; carry bail; three fold-out tripod legs
  parent_evidence:
    - rec ...9f7633a6: parts lower_base/upper_lantern/wire_handle; joints base_to_lantern_slide (PRISMATIC telescope), lantern_to_handle_hinge (REVOLUTE bail); clear_chamber, led_board, ribbed base_shell + foot_lug ring
    - rec ...c2bd28cb: parts lantern_body/carry_bail/leg_0..2; joints body_to_bail (REVOLUTE), body_to_leg_i (REVOLUTE x3); _lantern_cage_mesh (8 vertical guard bars), glowing_diffuser/led_column, vented_base, top_vent_cap, top_control_block
-->

# Camping_Outdoor Gear / Camping Lantern — template source map
pattern: mixed (linear_chain collapse + parallel_children handle/legs + multiplicity legs/cage-bars)
parents:
- rec_camping_outdoor_gear__camping_lantern_9f7633a6fc3d4d8e82070ee08dd075ac (picture/Camping_Outdoor Gear/Camping Lantern/001.png) — collapsible LED canister lantern
- rec_camping_outdoor_gear__camping_lantern_c2bd28cbafb94fb18d81093fd26983a4 (picture/Camping_Outdoor Gear/Camping Lantern/002.png) — hanging tripod wire-cage lantern
canonical_baselines: none
underfilled_reason: none (14 candidate anchors, within normal 12–18)

## Slot Candidates
| slot | candidate | diversity_axis | source_type | record/evidence | key parts/joints | status |
|---|---|---|---|---|---|---|
| body_form | clear cylindrical light chamber | ③ | origin_anchor | rec ...9f7633a6 clear_chamber/led_board | upper_lantern, clear_chamber | origin |
| body_form | cylindrical wire-cage + diffuser | ③ | origin_anchor | rec ...c2bd28cb _lantern_cage_mesh, glowing_diffuser | lantern_body, cylindrical_wire_cage | origin |
| body_form | barn/hurricane bulged glass globe | ③ | forked_anchor | rec_camping_lantern_var_barn_globe_body (from ...c2bd28cb) | lantern_body globe, carry_bail | converged |
| body_form | spherical orb diffuser globe | ③ | forked_anchor | rec_camping_lantern_var_orb_globe_body (from ...9f7633a6) | upper_lantern orb, led_board | converged |
| body_form | rectangular box/panel fixture | ③ | forked_anchor | rec_camping_lantern_var_box_fixture_body (from ...c2bd28cb) | lantern_body box, carry_bail | converged |
| body_form | flat disc/puck panel | ③ | forked_anchor | rec_camping_lantern_var_panel_puck_body (from ...9f7633a6) | upper_lantern puck, led_board | converged |
| body_form | slim tall tube/stick column | ③ | forked_anchor | rec_camping_lantern_var_stick_tube_body (from ...9f7633a6) | upper_lantern column, clear_chamber | converged |
| support_or_base | flat sitting canister base | ① | origin_anchor | rec ...9f7633a6 base_shell/foot_lug | lower_base | origin |
| support_or_base | fold-out tripod legs (3) | ① | origin_anchor | rec ...c2bd28cb leg_0..2 | leg_i, body_to_leg_i | origin |
| support_or_base | single central ground stake | ① | forked_anchor | rec_camping_lantern_var_ground_stake_base (from ...9f7633a6) | lower_base spike | converged |
| support_or_base | magnetic disc base + fold-out clip | ① | forked_anchor | rec_camping_lantern_var_magnetic_clip_base (from ...9f7633a6) | lower_base puck, clip hinge | converged |
| opening_or_motion | telescoping prismatic collapse | ② | origin_anchor | rec ...9f7633a6 base_to_lantern_slide | skirt_sleeve, base_to_lantern_slide | origin |
| opening_or_motion | fixed rigid body (no collapse) | ② | origin_anchor | rec ...c2bd28cb (rigid cage) | lantern_body | origin |
| opening_or_motion | twist-lock threaded base cap | ② | forked_anchor | rec_camping_lantern_var_twist_base_cap (from ...c2bd28cb) | base_cap, twist revolute/continuous | converged |
| opening_or_motion | accordion/bellows squeeze collapse | ② | forked_anchor | rec_camping_lantern_var_accordion_collapse (from ...9f7633a6) | bellows wall, base_to_lantern_slide | converged |
| opening_or_motion | hinged swing-open guard door | ② | forked_anchor | rec_camping_lantern_var_hinged_door (from ...c2bd28cb) | hinged_door, door revolute | converged |
| handle_or_grip | fold-up U-shaped carry bail | ② | origin_anchor | rec ...9f7633a6 wire_bail / rec ...c2bd28cb carry_bail | wire_handle/carry_bail, hinge | origin |
| handle_or_grip | single fold-out top hook | ② | forked_anchor | rec_camping_lantern_var_fixed_top_hook (from ...c2bd28cb) | top hook, body_to_hook hinge | converged |
| handle_or_grip | side-pivoting carry loop | ② | forked_anchor | rec_camping_lantern_var_side_carry_loop (from ...9f7633a6) | side loop, relocated hinge | converged |
| multiplicity (legs) | 4 fold-out legs (quadpod) | N | forked_anchor | rec_camping_lantern_var_legs_quadpod (from ...c2bd28cb) | leg_i, body_to_leg_i | converged |
| multiplicity (cage bars) | 12 vertical guard bars | N | forked_anchor | rec_camping_lantern_var_cage_bars_n12 (from ...c2bd28cb) | cylindrical_wire_cage, _lantern_cage_mesh | converged |

## Multiplicity / Copy Logic
- count_param:
  - legs: leg-count loop `for i in range(3)` in build_object_model (rec ...c2bd28cb), driving leg_i parts + body_to_leg_i joints + leg_socket_i
  - cage bars: `for bar in range(8)` guard-bar count inside `_lantern_cage_mesh` (rec ...c2bd28cb)
- N samples:
  - legs: 3 (origin) → 4 (var_legs_quadpod)
  - cage bars: 8 (origin) → 12 (var_cage_bars_n12)
- suggested N_range: legs 3–4 (physical stability limit; >4 rare); cage bars 6–16
- copied object / naming / placement / joint policy:
  - legs: copied object = fold-out leg (leg_i wire + hinge_pin); naming leg_i / leg_socket_i; placement radial even spacing TAU*i/N + phase offset; joint policy one body_to_leg_i REVOLUTE per leg (same axis/limits)
  - cage bars: copied object = one vertical cage-bar column; naming indexed within lattice; placement even angular spacing n*bar/N; joint policy static (no joint)

## Six-Axis Diversity Record
| axis | treatment | values / range / reason |
|---|---|---|
| ① skeleton / structural topology | source-backed | flat sitting base (o1), fold-out tripod legs (o2); forked: single ground stake, magnetic disc + clip |
| ② joint / mechanism type | source-backed | telescoping prismatic (o1), fixed rigid (o2), fold-up bail REVOLUTE (o1/o2); forked: twist-lock cap, accordion bellows, hinged door, single fold-out hook, side carry loop |
| ③ primary form family | source-backed | clear cylinder chamber (o1), wire-cage + diffuser (o2); forked: barn globe, orb globe, box/panel fixture, disc puck, slim tube/stick |
| ④ surface decoration | record_only / world_knowledge_extrapolation | grip ribs, castellated foot lugs, cap tabs, brand stroke, LED lens dot arrays; companion: latch knob, rubber grip sleeve, vertical fluting (host-conformal only) |
| ⑤ proportion / size / travel | record_only | canister H~0.17m R~0.064m; slide travel 0.070m; companion proportions: squat box, flat puck, tall slim stick, compact mini |
| ⑥ material / palette / finish | record_only | origins color-monotone (satin black / graphite / olive / blackened steel + warm LED, smoked clear); companion colorways: vintage brass/copper, frosted white, sand/tan, anodized accent, translucent silicone |

## Compatibility Probes
| probe | record_id | combined axes | risk tested | result |
|---|---|---|---|---|
| (none planned) | — | — | — | — |

## Blocked / Excluded
- live-wick oil lamp / gas-mantle burner: drifts to fuel-burning lamp neighbor; excluded (must_not_become).
- string-light / festoon bulb strand: not a single self-contained area lantern; excluded.
- solar-panel garden path light: crosses into fixed garden fixture; excluded (companion solar-panel decoration only if host-conformal, not a structural fork).
- legs N>4 / cage bars as solid panels: excluded as padding / body-family drift.
