<!--
subcategory_contract:
  category: Emergency Equipment
  subcategory: Emergency exit door
  core_identity: a building egress door leaf that swings on hinges and is opened from
    the inside by panic/exit hardware for emergency escape
  must_keep:
    - at least one hinged (revolute) door leaf that swings open in a fixed frame
    - panic/exit hardware (touch bar, crossbar, paddle, or vertical-rod device)
    - "exit / push to open" signage identity
  must_not_become:
    - ordinary interior passage door or entry door with a plain lever/knob only
    - gate, turnstile, roller/overhead shutter, window, or curtain wall
  image_evidence:
    - 001.png: pair of gray steel leaves, two horizontal push bars, green "Push bar to
      open" labels, exposed vertical locking rod on left leaf, one leaf swung open
    - 002.png: close-up of a horizontal brushed-metal touch bar with green "push to open
      during emergency only" label, center latch block, hand pressing the bar
  parent_evidence:
    - e8695a: paired near_leaf/far_leaf on revolute hinges (mimic), articulated horizontal
      near_push_bar (PRISMATIC) driving center latch_bolt (PRISMATIC mimic); helpers
      _add_leaf_panel, _add_fixed_panic_hardware
    - 26c10e: loop-based add_door_leaf/add_push_bar/add_exit_sign, door_0/door_1 on
      independent revolute hinges, two prismatic push bars, static vertical_rod +
      rod_clamp_0..3 + rod_top_latch/rod_bottom_latch on door_0
-->

# Emergency Equipment / Emergency exit door — template source map
pattern: mixed (parallel_children leaves + linear_chain bar->latch mechanism + multiplicity on leaves)
parents:
- rec_emergency_equipment__emergency_exit_door_e8695a6c2b8d4e0895742ce991243255 (origin_anchor) — picture/Emergency Equipment/Emergency exit door/002.png
- rec_emergency_equipment__emergency_exit_door_26c10e7e51b0421fa63b57854f167f20 (origin_anchor) — picture/Emergency Equipment/Emergency exit door/001.png
canonical_baselines: none
underfilled_reason: none (12 forks planned + 2 origin anchors = 13 candidate anchors, within normal 12-18)

## Slot Candidates
| slot | candidate | diversity_axis | source_type | record/evidence | key parts/joints | status |
|---|---|---|---|---|---|---|
| leaf_config / multiplicity | double equal leaves (N=2) | N | origin_anchor | both origins | door_0/door_1, near_leaf/far_leaf, frame_to_door_0/1 (REVOLUTE) | converged (origin) |
| leaf_config / multiplicity | single active leaf (N=1) | N | forked_anchor | rec_emergency_exit_door_var_single_leaf (from 26c10e) | add_door_leaf, frame_to_door_0 (REVOLUTE), door_0_to_push_bar_0 (PRISMATIC) | converged |
| leaf_config / multiplicity | leaf-and-a-half (unequal active+inactive) | N | forked_anchor | rec_emergency_exit_door_var_leaf_and_half (from 26c10e) | add_door_leaf (per-leaf width), flush bolts, frame_to_door_0/1 (REVOLUTE) | converged |
| frame / opening topology | single leaf + fixed glazed sidelite | (1) | forked_anchor | rec_emergency_exit_door_var_sidelite (from 26c10e) | frame, static sidelite panel, frame_to_door_0 (REVOLUTE) | converged |
| frame / opening topology | fixed glazed transom above header | (1) | forked_anchor | rec_emergency_exit_door_var_transom (from 26c10e) | jamb_0/1, header, static transom panel | converged |
| frame / opening topology | overhead self-closer with articulated arm | (1)/(2) | forked_anchor | rec_emergency_exit_door_var_overhead_closer (from e8695a) | frame/top_jamb, near_leaf, closer_arm (REVOLUTE), frame_to_near_leaf | converged |
| vision panel / body form | solid leaf, no lite | (3) | origin_anchor | both origins | door_slab, near_door_skin | converged (origin) |
| vision panel / body form | small vision lite (wired glass) | (3) | forked_anchor | rec_emergency_exit_door_var_vision_lite (from 26c10e) | door_slab + glazed aperture module | converged |
| vision panel / body form | tall narrow full-height vision slot | (3) | forked_anchor | rec_emergency_exit_door_var_narrow_vision (from 26c10e) | door_slab + vertical glazed strip | converged |
| vision panel / body form | fully glazed stile-and-rail leaf | (3) | forked_anchor | rec_emergency_exit_door_var_full_glazed (from 26c10e) | glass infill in hinge_stile/meeting_stile/top_rail/bottom_rail | converged |
| panic hardware | horizontal touch/push bar (prismatic) | (2)/(3) | origin_anchor | both origins | near_push_bar/push_bar_0, near_leaf_to_push_bar (PRISMATIC) | converged (origin) |
| panic hardware | surface vertical-rod exit device (bar drives rods) | (2) | forked_anchor | rec_emergency_exit_door_var_vertical_rod (from 26c10e) | vertical_rod->top/bottom rod bolts, door_0_to_top_rod/bottom_rod (PRISMATIC mimic) | converged |
| panic hardware | pivoting push-paddle / crash paddle (revolute) | (2) | forked_anchor | rec_emergency_exit_door_var_crash_paddle (from e8695a) | paddle, near_leaf_to_push_bar (REVOLUTE), near_leaf_to_latch_bolt (PRISMATIC) | converged |
| panic hardware | recessed / flush concealed touch bar | (3) | forked_anchor | rec_emergency_exit_door_var_recessed_bar (from e8695a) | near_push_bar in mortised pocket, near_leaf_to_push_bar (PRISMATIC) | converged |
| probe | glazed leaf + paddle device | probe | compatibility_probe | rec_emergency_exit_door_var_probe_glazed_paddle (from e8695a) | glass infill + revolute paddle + latch_bolt | converged |

## Multiplicity / Copy Logic
- count_param: number of add_door_leaf calls (leaf count) / number of add_push_bar calls; secondary: rod_clamp_0..3 loop count on the vertical-rod device
- N samples: N=2 (both origins, equal pair), N=1 (single_leaf), N=2-unequal (leaf_and_half)
- suggested N_range: 1-2 active leaves (real exit-door doors are single or paired; >2 leaves is not this subcategory)
- copied object / naming / placement / joint policy: copied object = door leaf via add_door_leaf(name, side) with mirrored side=+1/-1 about the center seam; indexed naming door_0/door_1 and push_bar_0/push_bar_1; placement = symmetric left/right of frame centerline; joint policy = one independent REVOLUTE hinge per leaf (origin 26c10e non-mimicked; origin e8695a uses a mimic pair) plus one PRISMATIC push bar per active leaf

## Six-Axis Diversity Record
| axis | treatment | values / range / reason |
|---|---|---|
| (1) skeleton / structural topology | source-backed | frame+2 leaves (origins); + single-leaf frame; + leaf+fixed sidelite; + transom-extended frame; + overhead closer linkage |
| (2) joint / mechanism type | source-backed | REVOLUTE leaf hinge (all) + PRISMATIC touch bar->latch mimic (origins); forks add REVOLUTE paddle, PRISMATIC vertical-rod bolts, REVOLUTE closer arm |
| (3) primary form family | source-backed | solid steel slab leaf (origins); + small vision lite, narrow full-height lite, fully glazed stile-and-rail leaf; + recessed vs surface bar form |
| (4) surface decoration | record_only / world_knowledge_extrapolation | green "push bar to open" / "push to open during emergency only" labels, raised white text strokes, kick plate, sheet-metal recess trims, hinge barrels; wired vs clear glazing pattern as companion |
| (5) proportion / size / travel | record_only | leaf 0.84x2.03 m (26c10e) / ~0.60x1.42 m panel (e8695a); door_thickness 0.045; bar travel 0.015-0.024 m; latch travel to ~0.050 m; hinge swing 0-1.05 to 1.55 rad |
| (6) material / palette / finish | record_only | galvanized/satin gray leaf, dark frame gray, brushed aluminum hardware, emergency green + white signage, black gap shadow; companion colorways = fire-door red, stainless, anodized/tinted glass |

## Compatibility Probes
| probe | record_id | combined axes | risk tested | result |
|---|---|---|---|---|
| glazed leaf + paddle device | rec_emergency_exit_door_var_probe_glazed_paddle | (3) glazed body family + (2) revolute paddle | paddle bracket + latch mounting depth vs thin glass leaf; bracket vs glazing stops; latch-to-strike alignment | converged |

## Blocked / Excluded
- sliding / roller / overhead-shutter "exit": excluded — not a hinged egress leaf; drifts out of subcategory (must_keep swinging leaf).
- revolving door / turnstile egress: excluded — different mechanism family, not an exit door leaf.
- knob/lever-only passage door (no panic hardware): excluded — loses the exit-door identity (must_keep panic/exit hardware).
- N>2 leaves: excluded — real emergency exit doors are single or paired leaves; higher counts are not source-backed for this subcategory.
