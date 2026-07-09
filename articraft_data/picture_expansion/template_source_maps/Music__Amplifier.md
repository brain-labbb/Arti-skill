# Music / Amplifier — template source map

pattern: mixed (linear_chain body with a multiplicity of rotary knob children; one cabinet_form member also adds a 2-part FIXED stack)
parents: rec_marshall-style-mini-guitar-combo-amplifier-black_20260605_161808_582290_34661766 ← picture/Music/Amplifier/001.png

Baseline (parent) recap: single `body` part = rounded black-vinyl `cabinet` (cadquery solid, top recess + front grille pocket) carrying visuals `gold_panel` (top recess), `power_led`, `speaker_grille` (PerforatedPanelGeometry on +X), `piping_h_*`/`piping_v_*`, `marshall_logo`, `corner_cap_*`, `handle`. Articulation: 4× `knob_i` parts on CONTINUOUS joints `panel_to_knob_i` about +Z, off-axis raised pointer tab for spin observability.

## Slot 候选覆盖

### Slot A:cabinet_form
| 候选(future module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| full_combo (baseline) | rec_marshall-style-mini-guitar-combo-amplifier-black_20260605_161808_582290_34661766 | body / cabinet (solid) ; knob_i / panel_to_knob_i(continuous +Z) | one-piece combo box: grille + top gold panel + knobs on the same enclosure | converged(已同步) |
| head_unit | rec_guitar_amplifier_var_amp_head | body / cabinet ; front_vent_slots (SlotPatternPanelGeometry) replaces speaker_grille ; knob_i / panel_to_knob_i(+Z) | electronics-only head: low/wide/shallow box, NO speaker section; front face has vent slots not a grille | converged(已同步) |
| mini_half_stack | rec_guitar_amplifier_var_mini_stack | speaker_cabinet (root) + head_box (child) ; cabinet_to_head(FIXED) ; head_to_knob_i(continuous +Z) | TWO stacked enclosures: lower speaker_cabinet (grille/piping/logo/caps) + upper head_box (gold_panel/handle/knobs) joined by a FIXED joint on the cabinet top face | converged(已同步) |
| tilt_back_wedge | rec_guitar_amplifier_var_tilt_back_combo | body / cabinet (wedge _wedge_solid, trapezoidal XZ extrude) ; knob_i / panel_to_knob_i (axis = top-surface normal, ~18° off +Z) | wedge cabinet: rear taller than front, front baffle tilts back ~14°, gold panel + knobs ride the slanted top surface | converged(已同步) |

Notes: head_unit and tilt_back redefine the cabinet solid (`_cabinet_solid`/`_wedge_solid`) and the knob seat/axis derivation; mini_stack is the structurally largest change — it splits the single `body` into a 2-node FIXED chain (`speaker_cabinet` → `head_box`), and re-parents the knobs onto `head_box` via `head_to_knob_i`. Knob joint name/axis is the member-specific carrier (panel_to_knob vs head_to_knob, +Z vs facet-normal).

### Slot B:control_panel_placement
| 候选(future module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| top_recessed (baseline) | rec_marshall-style-mini-guitar-combo-amplifier-black_20260605_161808_582290_34661766 | gold_panel (top recess) ; panel_to_knob_i axis=(0,0,1) | gold panel recessed into the top (+Z) face; knobs point UP | converged(已同步) |
| front_faceplate | rec_guitar_amplifier_var_front_faceplate | gold_panel (front strip) ; panel_to_knob_i axis=(1,0,0) | gold panel is a horizontal strip on the FRONT (+X) face above the grille; knobs point FORWARD; top left plain w/ handle | converged(已同步) |
| angled_chamfer_facet | rec_guitar_amplifier_var_angled_chamfer_panel | gold_panel (on chamfer facet) ; panel_to_knob_i axis=(FACET_NX,0,FACET_NZ)≈(0.707,0,0.707) | top-front edge bevelled into a 45° facet; gold panel + knobs sit on the facet, knob axis = facet normal (up-and-forward) | converged(已同步) |

Notes: this slot is purely the gold_panel placement + the knob joint origin/axis frame; part name `gold_panel` is stable, the discriminator is the `panel_to_knob_i` axis vector and the panel's mount surface.

### Slot C:grille_style
| 候选(future module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| perforated_panel (baseline) | rec_marshall-style-mini-guitar-combo-amplifier-black_20260605_161808_582290_34661766 | speaker_grille (PerforatedPanelGeometry, rotated to +X) | single dark perforated sheet across the front baffle | converged(已同步) |
| woven_cloth | rec_guitar_amplifier_var_cloth_grille | speaker_grille (`_woven_grille_mesh`: backing + 2 diagonal rib families) ; material grille_cloth | fabric basket-weave cloth with real surface relief (over/under ribs) instead of perforations | converged(已同步) |
| dual_round_speakers | rec_guitar_amplifier_var_dual_round_speakers | baffle_board ; speaker_0 / speaker_1 (LatheGeometry drivers) ; grille_bar_i + grille_vert_0 | open coarse-bar grille revealing TWO lathed round speaker cones on a baffle_board behind the bars | converged(已同步) |
| quad_grid (2×2) | rec_guitar_amplifier_var_quad_grille | grille_cell_0..3 (`_add_grille_cell`) ; grille_rib_h / grille_rib_v | front grille split into a 2×2 grid of four perforated cells separated by cross ribs | converged(已同步) |

Notes: replaces only the `speaker_grille` visual cluster on the front baffle (and, for dual_round, adds `baffle_board` + lathed `speaker_i` driver children as visuals, not joints). Cabinet, panel, and knob articulation are untouched by this slot.

## Multiplicity / Copy Logic
- count_param: knob_count ; default = 4
- N 样本已覆盖: {2, 4, 6} → rec_guitar_amplifier_var_knobs_n2 / parent(=4) / rec_guitar_amplifier_var_knobs_n6
- 模板建议 N_range: [2, 6] (panel width `PANEL_W` must grow with N — n6 widens panel 0.150→0.165 and respaces; a clamp on KNOB_YS spread vs PANEL_W bounds the practical max)
- copied object: each knob is one `knob_i` part built from the shared KnobGeometry (knurled + raised indicator + off-axis pointer tab) — identical geometry per copy
- naming: `knob_{i}` part/visual + `panel_to_knob_{i}` (or `head_to_knob_{i}` in the stack member) joint, i in range(KNOB_COUNT)
- placement: evenly spaced row in Y across the gold panel; `KNOB_YS = tuple(centered spacing for i in range(KNOB_COUNT))`, centered on Y=0, at a fixed KNOB_X / panel surface Z
- joint policy: CONTINUOUS rotary about the panel-mount normal (+Z top / +X front / facet-normal chamfer), MotionLimits(effort=0.3, velocity=8.0); every knob carries an allow_overlap with `gold_panel` (press-fit seat) + expect_contact

## 排除项(future compatibility matrix 素材)
- mini_half_stack (Slot A) × front_faceplate / angled_chamfer_facet (Slot B): the head-on-cab stack puts the control panel on the upper `head_box` top; a front/chamfer faceplate placement conflicts with the head box being a separate top enclosure — the two cabinet/placement frames would need reconciliation (panel re-anchored to head_box surface). Flag as needs-resolution, not yet sampled.
- head_unit (Slot A) × any grille_style (Slot C): head_unit has NO speaker section (front_vent_slots, not a grille), so the grille_style slot is inapplicable/null for that cabinet_form — grille_style only applies to combo/stack/wedge members.
- All other axis values converged; remaining cabinet_form × control_panel × grille combinations are compatible (panel placement and grille style are independent of the combo/wedge cabinet solid).
