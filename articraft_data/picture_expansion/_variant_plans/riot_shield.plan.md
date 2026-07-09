# Variant Plan — Law Enforcement_Protective Gear / Riot shield

slug: `riot_shield` · pattern: **mixed** (linear panel chain via fold hinge(s) + parallel handle/viewport children) · richness band: **simple (8 anchors)**

## subcategory_contract
```yaml
subcategory_contract:
  category: Law Enforcement_Protective Gear
  subcategory: Riot shield
  core_identity: A hand-carried (or self-standing) body-shielding barrier used by police/security for crowd control; a broad rigid or soft-armor panel presented between the officer and a threat.
  must_keep:
    - a broad protective shield panel (the shielding surface) larger than the body's torso footprint
    - a way to hold or stand it (carry-handle / forearm grip, or self-standing A-frame)
    - either real folding/opening articulation OR an explicitly static single rigid shield (static_only allowed)
  must_not_become: [ballistic vest / body armor plate, medieval tower shield or buckler prop, tactical backpack, tent/panel display board]
  image_evidence:
    - 001: matte-black rigid molded polymer bi-fold, POLICE lettering, angled top deflector bend + forward kick flare, steel clamp hinge brackets, perforated vision window, free-standing A-frame
    - 002: gray/dark soft ballistic-fabric two-panel G-FOLD, edge binding, black logo patch, cast-aluminum carry-handle bracket with open grip window + 4 dome bolts, webbing strap, grommets, fabric fold-sleeve hinge, A-frame tent rest pose
  parent_evidence:
    - A(4985a0ef): parts front_panel/rear_panel; revolute fold_hinge; _front_panel_solid/_rear_panel_solid; PerforatedPanelGeometry vision_mesh; clamp brackets + hinge_pin_*; ribs x3
    - B(d961ac50): parts front_panel/carry_handle/rear_panel; revolute panel_fold; FIXED panel_to_handle; _rounded_plate/_grip_frame/_add_binding; grommets x3; 4 dome bolts
```

## Slot / Candidate grid
| slot | candidates | source |
|---|---|---|
| body_form (③) | rigid_molded_polymer_panel(A) · soft_ballistic_fabric_panel(B) · curved_polycarbonate_shell(fork) · round_convex_shell(fork) | origins + 2 forks |
| panel_topology (①/N) | two_panel_bifold(A,B) · three_panel_trifold(fork n3) · four_panel_quadfold(fork n4) · single_monolithic(curved/round forks) | origins + forks |
| opening_or_motion (②) | fold_hinge revolute — fabric-roll(B)/pin-bracket(A) · gun_port shutter_hinge revolute(fork) · static none(curved/round forks) | origins + fork |
| handle_or_grip | bolt_on_carry_handle_bracket(B) · forearm_cradle+grip_bar(fork) · free_standing_none(A) | origins + fork |
| support_or_base | A_frame_self_standing_rear_panel(A,B) · handheld_no_base(curved/round forks) | origins + forks |
| vision_port | perforated_vision_window(A) · hinged_gun_port_shutter(fork) · integral_clear_body(curved fork) · none(B) | origins + fork |

Every supported slot reaches >=2 structurally distinct candidates.

## Six-Axis Diversity Audit
| axis | status | values |
|---|---|---|
| ① skeleton/topology | candidate-anchor (source-backed) | two-panel bifold(A,B); three-panel(n3); four-panel(n4); single monolithic (curved/round); handle sub-tree forearm-cradle+grip-bar(grip_forearm) |
| ② joint/mechanism | candidate-anchor (source-backed) | revolute fold hinge — fabric fold-sleeve(B) / pin+clamp-bracket(A); revolute gun-port shutter_hinge(gunport) |
| ③ primary form family | candidate-anchor (source-backed) | rigid bent polymer plate(A); soft fabric slab(B); curved concave polycarbonate shell(form_curved); round convex dished disc(form_round) |
| ④ surface decoration | record_only / world_knowledge_extrapolation | POLICE lettering(A), G-FOLD logo patch + white bar(B), stiffening ribs, perforation pattern; extrapolate reflective stripes, unit numbers, agency crest (host-conformal only) |
| ⑤ proportion/size/travel | record_only | panel W 0.50-0.55, H 0.55; wall t 0.012(rigid)/0.030(soft); fold travel ~ -0.62..2.29 rad(B), stow -0.50..0.10(A); curved shell ~0.9x0.6; round Ø~0.6; shutter 0..~1.4 rad |
| ⑥ material/palette | record_only | matte-black polymer, gray/dark ballistic fabric, clear/smoke polycarbonate, cast aluminum, steel brackets, gunmetal pins; palettes black / gray / olive-tan tactical |

①②③ + N are source-backed via origins and converged forks. ④⑤⑥ are record_only / companion — never standalone variants and not counted toward the budget.

## Multiplicity / Copy Logic
- count_param: `n_panels` (folding-panel chain)
- N samples: **2** (origins A,B) · **3** (fork n3) · **4** (fork n4)
- suggested N_range: [2, 5]
- copied object: ballistic panel slab + edge binding (`_add_binding`) + fabric fold sleeve; naming `panel_{idx}`, `fold_hinge_{idx}`
- placement: chain along the fold axis at panel top edges; accordion alternating fold sense so sections collapse into a stack
- joint policy: exactly one revolute fold hinge between each consecutive panel pair, all sharing the fold-line axis; handle stays on panel_0
- (secondary repeated features — grommets x3, ribs x3, corner bolts x4 — stay parametric/`record_only`, not fork-worthy)

## Budget decision
Richness band **simple (8-12)**; target the **low end at 8 candidate anchors** (2 origins + 6 forks). `underfilled_reason`: riot shields have a genuinely limited structural vocabulary (a broad panel + optional fold + a hold + an optional viewport); beyond form-family, fold multiplicity, viewport mechanism and grip topology the honest candidates run out, so we cover breadth at the low end rather than pad with ④/⑤/⑥, scale, or color. No compatibility probes needed (no risky interface combinations).

Candidate anchors (8): A(origin), B(origin), form_curved, form_round, mechanism_gunport, grip_forearm, n3, n4.
Forks emitted: **6**.

## Variant Cards
```yaml
- variant_id: rec_riot_shield_var_form_curved
  source_type: forked_anchor
  parent_record_id: rec_folding-two-panel-soft-armor-riot-shield-g-fold-_20260708_144344_902932_d961ac50
  positioning: {product_archetype: classic curved polycarbonate handheld riot shield, why_same_subcategory: broad body-shielding barrier held by an officer}
  primary_axis: {slot: body_form, diversity_axis: ③, target_candidate: curved single-panel concave shell}
  structural_delta:
    change: [replace flat two-panel fold assembly with one vertically-curved shell, remove rear_panel + panel_fold, keep bolt-on handle, integral clear viewport band]
    keep_parts: [front_panel, carry_handle, mount_plate, grip_frame, bolt_shank_*, bolt_dome_*, panel_to_handle]
    joint_policy: static_only (single rigid piece; only the fixed handle mount)
    interface_policy: handle bolted flush to convex outer face
  multiplicity: {applies: false, target_n: null, copied_object: null, placement_rule: none}
  companion_variations: {allowed_④⑤⑥: [clear/smoke palette, edge trim, molded ribs], forbidden: [fold hinge, second panel, category drift]}
  acceptance_focus: [curved shell renders as a shield, handle mounted flush, no fold joint present, static_only honored]

- variant_id: rec_riot_shield_var_form_round
  source_type: forked_anchor
  parent_record_id: rec_folding-two-panel-soft-armor-riot-shield-g-fold-_20260708_144344_902932_d961ac50
  positioning: {product_archetype: round convex bubble control shield with central grip, why_same_subcategory: hand-carried body-shielding barrier}
  primary_axis: {slot: body_form, diversity_axis: ③, target_candidate: circular convex dished disc}
  structural_delta:
    change: [replace rectangular panels with one round dished disc, remove rear_panel + panel_fold, center boss grip]
    keep_parts: [front_panel, carry_handle, mount_plate, grip_frame, bolt_shank_*, bolt_dome_*, panel_to_handle]
    joint_policy: static_only (rigid disc; only fixed central grip mount)
    interface_policy: grip mounted at disc center on concave rear
  multiplicity: {applies: false, target_n: null, copied_object: null, placement_rule: none}
  companion_variations: {allowed_④⑤⑥: [rim band, unit ring, matte/clear palette], forbidden: [fold hinge, second panel, buckler/round-shield prop]}
  acceptance_focus: [round convex disc reads as shield, central grip supported, static_only honored]

- variant_id: rec_riot_shield_var_mechanism_gunport
  source_type: forked_anchor
  parent_record_id: rec_black-tactical-bi-fold-riot-shield-standing-as-a_20260708_144725_562096_4985a0ef
  positioning: {product_archetype: tactical shield with openable vision/gun port, why_same_subcategory: same bifold barrier body, adds a viewport shutter}
  primary_axis: {slot: opening_or_motion, diversity_axis: ②, target_candidate: hinged gun-port shutter (revolute)}
  structural_delta:
    change: [replace fixed perforated vision_mesh with a hinged viewport_shutter flap over the window; add shutter_hinge revolute at window top]
    keep_parts: [front_panel, front_panel_body, police_lettering, bracket_*_cheek, hinge_pin_*, rear_panel, rear_panel_body, window cutout, fold_hinge]
    joint_policy: add exactly one revolute (shutter_hinge, ~0..1.4 rad); preserve existing fold_hinge
    interface_policy: shutter pivots on window top edge, seats into a slim frame lip when closed
  multiplicity: {applies: false, target_n: null, copied_object: null, placement_rule: none}
  companion_variations: {allowed_④⑤⑥: [latch nub, port chamfer, gunmetal palette], forbidden: [body form change, fold topology change, add handle]}
  acceptance_focus: [shutter opens/closes on revolute, closed flap seats over window, fold_hinge intact]

- variant_id: rec_riot_shield_var_grip_forearm
  source_type: forked_anchor
  parent_record_id: rec_folding-two-panel-soft-armor-riot-shield-g-fold-_20260708_144344_902932_d961ac50
  positioning: {product_archetype: deployed handheld shield with forearm cradle + grip bar, why_same_subcategory: same folding panel body, different hold}
  primary_axis: {slot: handle_or_grip, diversity_axis: ①, target_candidate: forearm-cradle cuff + horizontal grip bar}
  structural_delta:
    change: [replace grip_frame open loop with a forearm_cradle cuff and a separate grip_bar on standoffs, both on mount_plate]
    keep_parts: [front_panel, front_slab, logo_patch, side_binding_*, rear_panel, panel_fold, carry_handle, mount_plate, bolt_*, panel_to_handle]
    joint_policy: preserve panel_fold; handle stays FIXED-mounted (no new joint)
    interface_policy: cuff + grip bar bolted to mount_plate via the 4 existing bolts
  multiplicity: {applies: false, target_n: null, copied_object: null, placement_rule: none}
  companion_variations: {allowed_④⑤⑥: [padded cuff liner, webbing texture, rubber grip], forbidden: [body form change, fold change, handle articulation]}
  acceptance_focus: [two-point grip reads correctly, mounted flush, fold_hinge intact]

- variant_id: rec_riot_shield_var_n3
  source_type: forked_anchor
  parent_record_id: rec_folding-two-panel-soft-armor-riot-shield-g-fold-_20260708_144344_902932_d961ac50
  positioning: {product_archetype: tri-fold collapsible riot shield, why_same_subcategory: same ballistic panel + fold mechanism, three sections}
  primary_axis: {slot: multiplicity, diversity_axis: N, target_candidate: n_panels=3}
  structural_delta:
    change: [loop-emit panel_0..panel_2 via shared helper; chain fold_hinge_0, fold_hinge_1 accordion]
    keep_parts: [front_panel/panel_0, carry_handle, panel_to_handle, _add_binding, side_binding_*, hinge_sleeve, panel_fold family]
    joint_policy: one revolute fold hinge between each consecutive panel pair, shared fold axis
    interface_policy: panels chained at top edges; handle on panel_0
  multiplicity: {applies: true, target_n: 3, copied_object: panel slab + binding + fold sleeve, placement_rule: chain}
  companion_variations: {allowed_④⑤⑥: [per-panel binding color], forbidden: [handle change, body form change, hand-written panels]}
  acceptance_focus: [3 loop-emitted panels, 2 revolute folds, collapses to stack]

- variant_id: rec_riot_shield_var_n4
  source_type: forked_anchor
  parent_record_id: rec_folding-two-panel-soft-armor-riot-shield-g-fold-_20260708_144344_902932_d961ac50
  positioning: {product_archetype: quad-fold collapsible riot shield, why_same_subcategory: same ballistic panel + fold mechanism, four sections}
  primary_axis: {slot: multiplicity, diversity_axis: N, target_candidate: n_panels=4}
  structural_delta:
    change: [loop-emit panel_0..panel_3 via shared helper; chain fold_hinge_0..fold_hinge_2 accordion]
    keep_parts: [front_panel/panel_0, carry_handle, panel_to_handle, _add_binding, side_binding_*, hinge_sleeve, panel_fold family]
    joint_policy: one revolute fold hinge between each consecutive panel pair, shared fold axis
    interface_policy: panels chained at top edges; handle on panel_0
  multiplicity: {applies: true, target_n: 4, copied_object: panel slab + binding + fold sleeve, placement_rule: chain}
  companion_variations: {allowed_④⑤⑥: [per-panel binding color], forbidden: [handle change, body form change, hand-written panels]}
  acceptance_focus: [4 loop-emitted panels, 3 revolute folds, collapses flat]
```

## Blocked / Excluded
- flat single-panel handheld shield: excluded — origins are already flat panels; dropping the fold adds no new ③ form family (curved/round cover the novel handheld forms).
- fold-out prop-stand leg / deployable kickstand: excluded — origins already self-stand via the A-frame rear panel; no closest non-fork parent cleanly supports it.
- folding/stowable handle (② on handle): excluded as thin/padding; grip topology already covered by grip_forearm.
- n_panels=5+ : excluded — N{2,3,4} already exposes the copy logic.
- compatibility probes: none — no risky cross-slot interface combination identified.
```
