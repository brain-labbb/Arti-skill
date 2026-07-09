# Fork-variant planning — 9 subcategories (proto-specs)

Date: 2026-06-23. Per `FORK_VARIANTS.md §2` (axis/slot decomposition → planned variant list).
These are **pre-fork proto-specs**, not finished source maps. Run forks per `VARIANT_PIPELINE.md`,
then deliver real source maps to `template_source_maps/<大类>__<小类>.md`.

## Status of the 原始资产 (originals) track
Every listed subcategory **already has ≥1 finalized parent** (all workbench-only). So no new original
is *required* to start forking. Candidate NEW originals worth building (image-dependent, do NOT fork these):
- **Equipment/Game console** — full standing pedestal arcade cabinet (legs/kickplate + coin door + overhead marquee box). Distinct enough it should be a new original, not a fork of the tabletop wedge.
- **Equipment/LED work light** — dedicated tripod jobsite light and clamp/hook pocket flood (only 1 image today; forks approximate them).
- **Light/Latern** — railroad/cage lantern + flat-panel candle lantern, IF their fork diffs come out too large to stay clean.
- **Equipment/Control panel** — rotary-knob / analog-gauge / mushroom-e-stop panels have no photographed anchor; build originals if imagery arrives.
- **Equipment/Power switch** — knife/open-blade switch family (would be a multi-axis rewrite as a fork).
- **Parts/quick release clamp** — cam-only clamp with integral welded nut (removes the adjuster joint).
- Pipeline / Watermill / Pump2 — no new original needed; existing parents cover the photographed forms.

## Totals
| 小类 | parents | pattern | planned variants | Π×N gate |
|---|---|---|---|---|
| Equipment/Pipeline | 3 | mixed | 10 | 500 ✓ |
| Equipment/Game console | 1 | multiplicity | 8 | 48 ✓ |
| Equipment/Control panel | 3 | mixed | 10 | 320 ✓ |
| Equipment/LED work light | 1 | mixed | 10 | 135 ✓ |
| Light/Latern | 1 | mixed | 12 | 192 ✓ |
| Machinery/Watermill | 1 | multiplicity | 8 | 81 ✓ |
| Parts/quick release clamp | 1 | parallel_children | 6 | 27 ✓ |
| Equipment/Power switch | 2 | mixed | 8 | 72 ✓ |
| Equipment/Pump2 | 1 | parallel_children | 9 | 64 ✓ |
| **TOTAL** | | | **81 forks** | |

---

## Parent record IDs (fork from these — never from a variant)
- Pipeline: `...pipe_20260609_180104..._d58aec8c` (gate-valve), `..._6c9b7c6f` (hydrant), `..._b30771ae` (globe)
- Game console: `...game_20260609_180045..._a5689b50`
- Control panel: `...cont_20260609_180035..._c28c270c` (P1 rod), `..._647d2061` (P2 rail), `..._ab3b9f65` (P3 conduit)
- LED work light: `...led-_20260609_180048..._f7e038e0`
- Light/Latern: `...lantern_20260610_081109..._5da4cb46`
- Watermill: `...watermill_20260610_081149..._afe3e6a1`
- Quick release clamp: `...quick-release-seat-clamp-i_20260610_085231..._1b80e476`
- Power switch: `...powe_20260609_180112..._621bac5e` (pendant), `...wall_20260609_154028..._5b4ad2d8` (wall plate)
- Pump2: `...pump_20260609_180115..._621823e2`

Slot rationale + excluded axes are in the per-subcategory sections of the conversation/agent reports;
the planned-variants tables below are the actionable fork list (append the §4 fixed suffix at fork time).

---

## Equipment / Pipeline — 10 variants (slots: body_form ×5, operator ×5, port ×5, outlet_count N{0,1,2,3})
All 3 parents loop-emit repeats (bolts/spokes/chain) — no §4 rewrite needed.
| record_id | parent | axis = value |
|---|---|---|
| rec_pipeline_var_body_straightgate | d58aec8c | body = straight-through gate body (no elbow) |
| rec_pipeline_var_body_anglevalve | d58aec8c | body = 90° angle valve |
| rec_pipeline_var_operator_3spoke_globe | b30771ae | operator = 3-spoke handwheel |
| rec_pipeline_var_operator_5spoke | b30771ae | operator = 5-spoke handwheel |
| rec_pipeline_var_operator_lever | b30771ae | operator = quarter-turn lever (revolute) |
| rec_pipeline_var_operator_teebar | d58aec8c | operator = crossed tee-bar handle |
| rec_pipeline_var_port_socketweld | d58aec8c | port = plain socket-weld ends |
| rec_pipeline_var_port_unionnut | b30771ae | port = union-nut couplings |
| rec_pipeline_var_outlet_1 | 6c9b7c6f | outlet_count = 1 (loop N=1) |
| rec_pipeline_var_outlet_3 | 6c9b7c6f | outlet_count = 3 (loop N=3) |

## Equipment / Game console — 8 variants (slots: body ×4, control ×4, joystick_count N{1,2,4})
Parent buttons/keypads use semantic-corner names, NOT range(n): station-count variants MUST rewrite to `for i in range(n)` `station_{i}`.
| record_id | parent | axis = value |
|---|---|---|
| rec_game_console_var_body_upright_box | a5689b50 | body = straight upright box cabinet |
| rec_game_console_var_body_cocktail | a5689b50 | body = cocktail flat-top table |
| rec_game_console_var_body_bartop_crown | a5689b50 | body = bartop + curved marquee crown |
| rec_game_console_var_ctrl_trackball | a5689b50 | control = trackball (revolute spin) |
| rec_game_console_var_ctrl_spinner | a5689b50 | control = spinner knob (revolute Z) |
| rec_game_console_var_ctrl_slider | a5689b50 | control = linear slider (prismatic) |
| rec_game_console_var_stations_x2 | a5689b50 | joystick_count N=2 (loop) |
| rec_game_console_var_stations_x4 | a5689b50 | joystick_count N=4 (loop) |

## Equipment / Control panel — 10 variants (slots: mount ×4, control ×5, readout ×4, button_count N{2,3,4,6})
P1/P2 buttons are hand-written tuples: multiplicity variants fork P3 (clean range(4)) or must demand loop rewrite.
| record_id | parent | axis = value |
|---|---|---|
| rec_control_panel_var_mountA_backplate | 647d2061 | mount = flush wall back-plate |
| rec_control_panel_var_ctrlB_rotaryknob | 647d2061 | control = rotary knob/dial |
| rec_control_panel_var_ctrlB_togglebank | 647d2061 | control = toggle/rocker bank |
| rec_control_panel_var_ctrlB_mushroom | c28c270c | control = mushroom e-stop |
| rec_control_panel_var_readC_gauge | 647d2061 | readout = analog round gauge |
| rec_control_panel_var_readC_lcdrow | c28c270c | readout = LCD + LED row + vent slots |
| rec_control_panel_var_ctrlB_pushbtn_railrotary | ab3b9f65 | control = push-buttons only (drop rotary) |
| rec_control_panel_var_N3_buttons | ab3b9f65 | button_count N=3 |
| rec_control_panel_var_N6_buttons | ab3b9f65 | button_count N=6 |
| rec_control_panel_var_mountA_railN | ab3b9f65 | mount = twin-rail clamp |

## Equipment / LED work light — 10 variants (slots: mount ×5, head ×3, panel ×3, led_count N{15,40,88})
Parent LED array already loop-emitted (no rewrite). Only 1 image — candidates held to real work-light forms.
| record_id | parent | axis = value |
|---|---|---|
| rec_led_work_light_var_mount_aframe | f7e038e0 | mount = folding A-frame stand |
| rec_led_work_light_var_mount_tripod | f7e038e0 | mount = tripod mast |
| rec_led_work_light_var_mount_hook | f7e038e0 | mount = fold-out hang hook |
| rec_led_work_light_var_head_tiltpan | f7e038e0 | head = tilt+pan yoke (2 revolutes) |
| rec_led_work_light_var_head_telescope | f7e038e0 | head = telescoping mast + tilt (prismatic+revolute) |
| rec_led_work_light_var_panel_cob_round | f7e038e0 | panel = round COB disc |
| rec_led_work_light_var_panel_dual | f7e038e0 | panel = dual side-by-side flood bar |
| rec_led_work_light_var_leds_sparse | f7e038e0 | led_count = 3×5 |
| rec_led_work_light_var_leds_dense | f7e038e0 | led_count = 8×11 |

## Light / Latern — 12 variants (slots: type ×4, cap ×4, carry ×4, guard_count N{2,6,10})
Parent's 2 guard tubes are hand-mirrored (not loop): N=2 variant rewrites them as `guard_member_{i}` loop (=§4 fix).
| record_id | parent | axis = value |
|---|---|---|
| rec_lantern_var_typeA_railroad_cage | 5da4cb46 | type = railroad/cage lantern |
| rec_lantern_var_typeA_candle_panel | 5da4cb46 | type = flat-panel candle lantern |
| rec_lantern_var_typeA_tubular_coldblast | 5da4cb46 | type = tubular cold-blast barn lantern |
| rec_lantern_var_topB_flat_pierced_crown | 5da4cb46 | cap = flat pierced vent crown |
| rec_lantern_var_topB_conical_louver | 5da4cb46 | cap = tall conical louvered chimney |
| rec_lantern_var_topB_peaked_roof | 5da4cb46 | cap = peaked roof/pagoda |
| rec_lantern_var_carryC_top_ring | 5da4cb46 | carry = fixed top swivel ring |
| rec_lantern_var_carryC_folding_strap | 5da4cb46 | carry = folding side strap |
| rec_lantern_var_carryC_hook_hanger | 5da4cb46 | carry = swivel hook hanger |
| rec_lantern_var_guardN_2_loop | 5da4cb46 | guard_count N=2 (loop rewrite) |
| rec_lantern_var_guardN_6 | 5da4cb46 | guard_count N=6 |
| rec_lantern_var_guardN_10 | 5da4cb46 | guard_count N=10 |

## Machinery / Watermill — 8 variants (slots: wheel_type ×3, mount ×3, spokes ×3, paddle_count N{8,12,16})
Parent paddles already loop-emitted; all paddles FIXED to wheel, one CONTINUOUS hub spin.
| record_id | parent | axis = value |
|---|---|---|
| rec_watermill_var_wheeltype_overshot | afe3e6a1 | wheel = enclosed buckets (overshot) |
| rec_watermill_var_wheeltype_breastshot | afe3e6a1 | wheel = angled scoop vanes (breastshot) |
| rec_watermill_var_mount_millhouse | afe3e6a1 | mount = mill-house wall |
| rec_watermill_var_mount_sluice | afe3e6a1 | mount = sluice/channel |
| rec_watermill_var_spokes_clasparm | afe3e6a1 | spokes = clasp-arm/compass-arm |
| rec_watermill_var_spokes_solidweb | afe3e6a1 | spokes = solid web disc |
| rec_watermill_var_paddles_n12 | afe3e6a1 | paddle_count N=12 |
| rec_watermill_var_paddles_n16 | afe3e6a1 | paddle_count N=16 |

## Parts / quick release clamp — 6 variants (slots: collar ×3, actuation ×3, nut ×3; no multiplicity)
Simple part — honest 6. 27 ≥ 10 via 3 real slots.
| record_id | parent | axis = value |
|---|---|---|
| rec_quick_release_clamp_var_pinch_collar | 1b80e476 | collar = closed pinch-collar (single slit) |
| rec_quick_release_clamp_var_hinged_collar | 1b80e476 | collar = hinged two-piece (extra revolute) |
| rec_quick_release_clamp_var_fold_lever | 1b80e476 | actuation = fold-flat lever bolt |
| rec_quick_release_clamp_var_hex_bolt | 1b80e476 | actuation = recessed hex bolt + flip key |
| rec_quick_release_clamp_var_wing_nut | 1b80e476 | nut = winged thumb nut |
| rec_quick_release_clamp_var_dome_nut | 1b80e476 | nut = domed acorn cap nut |

## Equipment / Power switch — 8 variants (slots: actuator ×6, mount ×4, gang_count N{1,2,3})
Two parents: wall plate (5b4ad2d8) + pendant box (621bac5e). Gang variants fork the flat plate.
| record_id | parent | axis = value |
|---|---|---|
| rec_power_switch_var_actuator_flip_toggle | 5b4ad2d8 | actuator = flip toggle/dolly |
| rec_power_switch_var_actuator_rocker | 5b4ad2d8 | actuator = rocker paddle |
| rec_power_switch_var_actuator_pushbutton | 5b4ad2d8 | actuator = push-button cap (prismatic) |
| rec_power_switch_var_actuator_rotary | 5b4ad2d8 | actuator = rotary cam selector |
| rec_power_switch_var_mount_enclosure | 621bac5e | mount = industrial enclosure box |
| rec_power_switch_var_mount_inline | 621bac5e | mount = inline cord barrel |
| rec_power_switch_var_gang_n2 | 5b4ad2d8 | gang_count N=2 (loop) |
| rec_power_switch_var_gang_n3 | 5b4ad2d8 | gang_count N=3 (loop) |

## Equipment / Pump2 — 9 variants (slots: handle ×4, base ×4, outlet ×4; no parent multiplicity)
Parent is all named singletons (no loop). tripod-foot variant intentionally introduces `leg_{i}` loop for copy-logic sample.
| record_id | parent | axis = value |
|---|---|---|
| rec_pump_var_handle_tbar | 621823e2 | handle = T-bar plunger |
| rec_pump_var_handle_dloop | 621823e2 | handle = D-loop plunger |
| rec_pump_var_handle_palmdisc | 621823e2 | handle = palm push-disc |
| rec_pump_var_base_flange | 621823e2 | base = flanged pedestal |
| rec_pump_var_base_tripod | 621823e2 | base = three-leg tripod (loop) |
| rec_pump_var_base_wallbracket | 621823e2 | base = wall bracket/saddle |
| rec_pump_var_outlet_gooseneck | 621823e2 | outlet = rigid gooseneck spout |
| rec_pump_var_outlet_barb | 621823e2 | outlet = straight barbed nipple |
| rec_pump_var_outlet_tapvalve | 621823e2 | outlet = valved outlet + tap lever (revolute) |
