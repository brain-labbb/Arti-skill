# Textiles_Fabric / Snap_button fastener — variant plan

slug `snap_button_fastener` · pattern **mixed** (single-root hand tool with one pivot + one guided ram; plus standalone two-half snap fastener as a ③ form-family branch). No meaningful multiplicity/copy-loop.

## subcategory_contract
```yaml
subcategory_contract:
  category: Textiles_Fabric
  subcategory: Snap_button fastener
  core_identity: >
    A snap-button (press-stud) fastener system: either the hand tool that presses/sets
    a snap-button onto fabric, or the two-half snap fastener itself (male stud + female
    socket) that snaps together. Metal die/cap/socket interface is the constant.
  must_keep:
    - a snap die/cap/socket interface (the actual snap-button halves)
    - a real actuation that closes/engages the two mating halves (revolute squeeze,
      prismatic ram/punch, or prismatic snap engagement)
    - hand-scale, fabric-hardware identity
  must_not_become:
    - sew-through Button
    - Rivet / Grommet / Eyelet-only tool
    - Magnetic snap, Buckle, Zipper
    - generic pliers, bench vise, C-clamp, hole punch, or drill/arbor press with no snap die
  image_evidence:
    - 001: chrome scissor plier with red vinyl grips; box of open-ring 5-prong silver snaps;
      four exploded snap parts (open ring, dome socket, ball stud, open ring); snaps set on yellow fabric
    - 002: mint-teal handheld C-frame press with return spring + interchangeable dies (B4/P3/P5/P8);
      trays of white plastic / silver / gold / gunmetal / brass ball-and-socket dome snaps
  parent_evidence:
    - A (dc888daf): two-bar scissor plier; frame_arm + lever_arm on one REVOLUTE handle_pivot;
      upper_die + anvil_post carry snap_socket / snap_stud halves; direct jaw-closure press
    - B (8d3803f8): handheld C-frame press; body (C-frame + lower_handle + return_spring + base_die)
      + lever (REVOLUTE body_to_lever) drives plunger (PRISMATIC body_to_plunger) through a steel
      guide_sleeve onto the base die; snap_cap over die_post work gap
```

## Origins (full reconciliation, 2/2 anchored)
| id | pic | built form | grid role |
|---|---|---|---|
| A `rec_hand-held-snap-button-fastener-pliers-a-chrome-s_20260708_092630_506687_dc888daf` | 001 | chrome scissor plier, red grips; direct jaw press | skeleton=scissor_plier / drive=direct_revolute / fastener=on-tool |
| B `rec_a-mint-teal-handheld-snap-fastener-press-pliers-_20260708_092112_065428_8d3803f8` | 002 | mint-teal handheld C-frame press; lever→guided ram + return spring | skeleton=handheld_c_frame / drive=lever_cam_prismatic_ram / fastener=on-tool |

## Slot / candidate grid
### Slot A — frame_skeleton / body (① / ③)
| candidate | diversity_axis | source_type | record/evidence | status |
|---|---|---|---|---|
| scissor_plier (open two-bar frame) | ① | origin_anchor | A | converged |
| handheld_c_frame_press | ①③ | origin_anchor | B | converged |
| bench_mounted_column_press | ① | forked_anchor | rec_snap_button_fastener_var_bench_press (from B) | planned |
| two_piece_anvil_punch_setter | ① | forked_anchor | rec_snap_button_fastener_var_punch_setter (from B) | planned |

### Slot B — drive_mechanism (②)
| candidate | diversity_axis | source_type | record/evidence | status |
|---|---|---|---|---|
| direct_jaw_press (single revolute) | ② | origin_anchor | A `handle_pivot` | converged |
| lever_cam_driven_prismatic_ram | ② | origin_anchor | B `body_to_lever`+`body_to_plunger` | converged |
| rotary_screw_arbor_ram (continuous screw → prismatic ram) | ② | forked_anchor | rec_snap_button_fastener_var_screw_press (from B) | planned |
| coaxial_punch_stroke (prismatic punch into anvil) | ② | forked_anchor | rec_snap_button_fastener_var_punch_setter (from B) | planned |

### Slot C — fastener_form_family (③) — the snap-button product itself
| candidate | diversity_axis | source_type | record/evidence | status |
|---|---|---|---|---|
| integrated on-tool snap halves | ③ | origin_anchor | A (cap/socket/prong/stud) + B (snap_cap/socket) | converged |
| standalone open-ring prong press-stud | ③ | forked_anchor | rec_snap_button_fastener_var_fastener_ring (from A) | planned |
| standalone ball-and-socket dome press-stud | ③ | forked_anchor | rec_snap_button_fastener_var_fastener_ball (from B) | planned |

## Six-Axis Diversity Audit
| axis | treatment | values / range / reason |
|---|---|---|
| ① skeleton / topology | source-backed (origin + forked_anchor) | scissor_plier(A) / handheld_c_frame(B) / bench_column_press(fork) / anvil+punch two-piece(fork) |
| ② joint / mechanism | source-backed (origin + forked_anchor) | direct revolute jaw press(A) / revolute lever + prismatic guided ram(B) / continuous screw + prismatic ram(fork) / prismatic punch stroke(fork); snap fasteners use prismatic snap engage |
| ③ primary form family | source-backed (origin + forked_anchor) | setting-tool body (planar plier frame vs volumetric C-frame vs bench block) vs the standalone press-stud fastener (open-ring prong vs ball-and-socket dome) |
| ④ surface decoration | record_only / world_knowledge_extrapolation | die stamp marks (B4/P3/P5/P8), decorative socket ring engraving/star pattern, dome pearl/textured caps; host-conformal only, no dedicated variant |
| ⑤ proportion / size / travel | record_only | plier length ~0.15 m; squeeze 0.15 rad (A) / 0.21 rad (B); ram stroke ~0.005 m; snap Ø ~10–15 mm; bench press scales larger |
| ⑥ material / palette / finish | record_only (companion) | chrome steel + red vinyl grip(A); mint-teal molded body + steel + brass(B); snap finishes: white plastic / nickel / silver / gold / gunmetal / antique brass |

## Multiplicity / Copy Logic
- **Not applicable / thin.** No repeated homogeneous loop-parts on a single setting tool (pivot ears ×2 in B are a fixed symmetric pair, not a copy-loop). The interchangeable die sets (B4/P3/P5/P8) are alternatives loaded one at a time, not simultaneous copies on the object.
- count_param: none; N samples: none; N_range: n/a; copied_object/naming/placement/joint_policy: n/a.

## Budget decision
- Richness band: **simple** (single-purpose hand tool + a small two-part fastener; limited genuine structural vocabulary).
- Candidate anchors total = **7** (2 origins + 5 forks): scissor_plier, handheld_c_frame_press, bench_press, punch_setter, screw_press, fastener_ring, fastener_ball.
- Fork jobs emitted = **5**.
- `underfilled_reason`: honest structural vocabulary for a snap-fastener setting tool + press-stud is thin. All press-tool variants share the same die/cap/socket interface; beyond the four skeleton/mechanism archetypes and the two fastener form-families there is no non-padding structural axis, and there is no true multiplicity/copy-loop. Targeting the low end of the simple band rather than inventing filler (extra die stamps, colors, sizes = ④/⑤/⑥ only).

## Variant cards
```yaml
- variant_card:
    variant_id: rec_snap_button_fastener_var_bench_press
    source_type: forked_anchor
    parent_record_id: rec_a-mint-teal-handheld-snap-fastener-press-pliers-_20260708_092112_065428_8d3803f8
    positioning: {product_archetype: KAM-style desktop snap/grommet hand-press machine, why_same_subcategory: still presses snap-button halves via lever→ram onto a die, just bench-mounted}
    primary_axis: {slot: frame_skeleton, diversity_axis: ①, target_candidate: bench_mounted_column_press}
    structural_delta:
      change: [replace two handheld handles with flat base plate + fixed vertical column, mount pivot ears on column top, base die stack under ram on the base plate]
      keep_parts: [body, lever, plunger, guide_sleeve, base_die, die_post, snap_socket, ram_shaft, nylon_tip, cap_die, snap_cap, return_spring, body_to_lever, body_to_plunger]
      joint_policy: preserve body_to_lever revolute + body_to_plunger prismatic
      interface_policy: ram runs in guide_sleeve, cap_die presses snap_cap onto die_post
    multiplicity: {applies: false, target_n: null, copied_object: null, placement_rule: none}
    companion_variations: {allowed_④⑤⑥: [cast-iron/painted-steel base palette, bench-scale proportion], forbidden: [arbor/drill press drift, screw-drive swap]}
    acceptance_focus: [base plate stable and non-floating, ram still closes cap onto die at full squeeze]

- variant_card:
    variant_id: rec_snap_button_fastener_var_punch_setter
    source_type: forked_anchor
    parent_record_id: rec_a-mint-teal-handheld-snap-fastener-press-pliers-_20260708_092112_065428_8d3803f8
    positioning: {product_archetype: base anvil + setter punch hand fixing tool from a snap kit, why_same_subcategory: still sets a snap-button by pressing a punch into a die/anvil}
    primary_axis: {slot: frame_skeleton, diversity_axis: ①, target_candidate: two_piece_anvil_punch_setter}
    structural_delta:
      change: [drop C-frame/lever/spring, keep stout anvil base carrying base_die/die_post/snap_socket, keep punch as prismatic plunger travelling into the anvil]
      keep_parts: [base_die, die_post, snap_socket, plunger, ram_shaft, nylon_tip, cap_die, snap_cap, body_to_plunger]
      joint_policy: keep single prismatic body_to_plunger as the only articulation
      interface_policy: punch guided coaxially into anvil recess so nothing floats
    multiplicity: {applies: false, target_n: null, copied_object: null, placement_rule: none}
    companion_variations: {allowed_④⑤⑥: [plain zinc/steel finish], forbidden: [hole punch/awl/hammer drift]}
    acceptance_focus: [punch and anvil coaxial, prismatic stroke compresses snap, no floating punch]

- variant_card:
    variant_id: rec_snap_button_fastener_var_screw_press
    source_type: forked_anchor
    parent_record_id: rec_a-mint-teal-handheld-snap-fastener-press-pliers-_20260708_092112_065428_8d3803f8
    positioning: {product_archetype: screw/arbor snap-and-eyelet setter twisted by a top handle, why_same_subcategory: still presses snap halves onto a die, drive is a screw instead of a squeeze lever}
    primary_axis: {slot: drive_mechanism, diversity_axis: ②, target_candidate: rotary_screw_arbor_ram}
    structural_delta:
      change: [remove lever/press_pad/pivot ears/spring, add rotary screw_handle (continuous joint about ram axis) whose threaded shaft advances the prismatic ram down]
      keep_parts: [body, front_column, rear_column, guide_sleeve, plunger, ram_shaft, nylon_tip, cap_die, snap_cap, base_die, die_post, snap_socket, body_to_plunger]
      joint_policy: replace lever-cam drive with continuous screw + prismatic ram (two coupled joints)
      interface_policy: ram stays guided in guide_sleeve; screw shaft coaxial with ram
    multiplicity: {applies: false, target_n: null, copied_object: null, placement_rule: none}
    companion_variations: {allowed_④⑤⑥: [black-phenolic/steel knob finish], forbidden: [C-clamp/vise/drill-press drift, bench-base skeleton swap]}
    acceptance_focus: [screw rotation visibly advances ram, cap closes on die, no bundled skeleton change]

- variant_card:
    variant_id: rec_snap_button_fastener_var_fastener_ring
    source_type: forked_anchor
    parent_record_id: rec_hand-held-snap-button-fastener-pliers-a-chrome-s_20260708_092630_506687_dc888daf
    positioning: {product_archetype: open-ring 5-prong press-stud installed on fabric, why_same_subcategory: it IS the snap-button fastener (the named object), two mating halves that snap}
    primary_axis: {slot: fastener_form_family, diversity_axis: ③, target_candidate: standalone_open_ring_prong_snap}
    structural_delta:
      change: [delete tool frame/lever/grips/dies, build female cap (cap_disc + spring/prong ring) + male stud (prong_ring eyelet + stud_post + stud_head) on two fabric swatches]
      keep_parts: [snap_socket, cap_disc, socket_ring, snap_stud, prong_ring, stud_post, stud_head]
      joint_policy: add one prismatic snap-engage joint (stud into ring socket) as the real non-fixed articulation
      interface_policy: coaxial stud head travels into/out of the ring socket
    multiplicity: {applies: false, target_n: null, copied_object: null, placement_rule: none}
    companion_variations: {allowed_④⑤⑥: [fabric swatch color, nickel/silver metal], forbidden: [sew-through button, rivet, grommet, magnetic snap]}
    acceptance_focus: [two halves coaxial, prismatic snap engage/disengage, no plier remnants]

- variant_card:
    variant_id: rec_snap_button_fastener_var_fastener_ball
    source_type: forked_anchor
    parent_record_id: rec_a-mint-teal-handheld-snap-fastener-press-pliers-_20260708_092112_065428_8d3803f8
    positioning: {product_archetype: domed ball-and-socket press-stud installed on fabric, why_same_subcategory: it IS the snap-button fastener, two mating halves that snap}
    primary_axis: {slot: fastener_form_family, diversity_axis: ③, target_candidate: standalone_ball_socket_dome_snap}
    structural_delta:
      change: [delete press frame/lever/ram/spring/dies, build female cap (domed snap_cap + S-spring socket_ring) + male ball stud (die_post ball + base rivet) on two fabric swatches]
      keep_parts: [snap_cap, snap_socket, cap_die, die_post]
      joint_policy: add one prismatic snap-engage joint (ball into socket) as the real non-fixed articulation
      interface_policy: coaxial ball seats into/releases from socket
    multiplicity: {applies: false, target_n: null, copied_object: null, placement_rule: none}
    companion_variations: {allowed_④⑤⑥: [brass/gunmetal/nickel dome, fabric swatch color], forbidden: [sew-through button, rivet, grommet, magnetic snap]}
    acceptance_focus: [ball-and-socket coaxial, prismatic snap engage/disengage, no press remnants]
```

## Blocked / Excluded
- Interchangeable-die multiplicity sweep (B4/P3/P5/P8): excluded — dies are loaded one at a time, not simultaneous copies; no copy-loop, would be ④-only padding.
- Grip material / snap finish / snap size variants: ④/⑤/⑥ only — recorded, never standalone forks.
- Standalone hammer / awl / hole-punch tool: blocked — neighbor category, not a snap-button setter.
