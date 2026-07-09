# Military / Granade — template source map

pattern: mixed

parents: rec_model-an-mk2-style-pineapple-fragmentation-hand-_20260610_080226_230556_a8628c01 ← picture/Military/Granade/001.png (ovoid_frag body + frag_knob_grid surface + mk2_pivot/cotter/single_ring fuze; fills Slot A ovoid_frag, Slot B frag_knob_grid, Slot C mk2_pivot+cotter+single_ring; multiplicity reference 5 frag rows)
parents: rec_model-an-m84-style-stun-flashbang-grenade-about-_20260610_080253_506027_b1beda97 ← picture/Military/Granade/002.png (cylindrical_tube body + perforated_vent_shell surface + m84_collar/twin_pins/twin_rings fuze; fills Slot A cylindrical_tube, Slot B perforated_vent_shell, Slot C m84_collar+twin_pins+twin_rings)

## Slot 候选覆盖

### Slot A:body_form
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| ovoid_frag | rec_model-an-mk2-style-pineapple-fragmentation-hand-_20260610_080226_230556_a8628c01 | grenade_body / visual frag_body (_lathe(BODY_PROFILE) revolve) | Olive cast-iron ovoid lathe, widest mid-body, flat rounded base grounded z=0, BODY_TOP_Z=0.085 | converged (parent) |
| cylindrical_tube | rec_model-an-m84-style-stun-flashbang-grenade-about-_20260610_080253_506027_b1beda97 | body / visuals hex_cap_bottom, perforated_shell, charge_tube, hex_cap_top (_hex_prism + annular extrude) | Straight ~0.048 dia tube, hex end caps ~0.056 across-corners, ~0.131 m tall | converged (parent) |
| tapered_ovoid | rec_grenade_var_taperegg | grenade_body / visual frag_body (steeper BODY_PROFILE, narrow base tip r=0.006) | Teardrop egg: narrow rounded base tip, widest ~73% up body, smooth taper; base_dia < 0.55*belly_dia | converged (workbench, rating pending sync) |

### Slot B:surface
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| frag_knob_grid | rec_model-an-mk2-style-pineapple-fragmentation-hand-_20260610_080226_230556_a8628c01 | grenade_body visual frag_body (knobs fused into body Compound via KNOB_ROWS_Z × KNOB_COLS=8 loop) | Beveled chamfered knobs over recessed CORE_PROFILE waffle grid, equal-z rows, 45° column spacing offset 22.5° | converged (parent) |
| perforated_vent_shell | rec_model-an-m84-style-stun-flashbang-grenade-about-_20260610_080253_506027_b1beda97 | body visuals perforated_shell + charge_tube (3 rows × HOLES_PER_ROW=5 boolean cut radial holes) | Annular olive shell with vent holes revealing tan charge_tube; brown mid_band wraps middle row; red flash_insert in one lower hole | converged (parent) |
| smooth_shell | rec_grenade_var_smoothovoid | grenade_body visual cast_shell (_lathe(BODY_PROFILE) only, no knobs) | Unbroken cast ovoid shell, no surface texture | converged (workbench, rating pending sync) |
| smooth_shell | rec_grenade_var_smoothcyl | body visual smooth_shell (solid annular extrude, no holes/knobs) | Plain cylindrical tube shell, no vents/knobs | converged (workbench, rating pending sync) |
| frag_knob_grid (组合抽检: frag-on-cyl) | rec_grenade_var_fragcyl | body visual solid_shell + frag_knob_{i_row}_{i_col} (N_KNOB_ROWS=6 × N_KNOB_COLS=8 individual FIXED knob visuals) | Cross: frag knobs grid on cylindrical tube body; knobs are per-knob named visuals on body, not a fused mesh | converged (workbench, rating pending sync) |
| perforated_vent_shell (组合抽检: vent-on-ovoid) | rec_grenade_var_ventovoid | grenade_body visuals perforated_shell + charge_tube (HOLE_ROWS_Z=5 × HOLE_COLS=10 radial cut + inner cavity + charge tube w/ support rings) | Cross: vent holes on ovoid body revealing inner charge_tube; hollow shell SHELL_WALL=0.003 | converged (workbench, rating pending sync) |

### Slot C:fuze
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| mk2_pivot+cotter+single_ring | rec_model-an-mk2-style-pineapple-fragmentation-hand-_20260610_080226_230556_a8628c01 | safety_lever(spoon) / safety_pin(cotter_pin) / pull_ring(ring_torus); joints lever_pivot(revolute), pin_slide(prismatic), ring_swivel(revolute) | Threaded collar + rect pivot housing/lug/axle; single cotter pull pin + single torus ring child of pin | converged (parent) |
| m84_collar+twin_pins+twin_rings | rec_model-an-m84-style-stun-flashbang-grenade-about-_20260610_080253_506027_b1beda97 | safety_lever(pivot_pin/top_plate/side_strap) / primary_pin + secondary_pin(shaft/eye_head) / primary_pull_ring + secondary_pull_ring(ring_loop); joints lever_pivot(rev), primary_pin_slide(pris), secondary_pin_slide(pris @165°), primary_ring_swing(rev), secondary_ring_swing(rev) | Silver collar + fuze_body; two pull pins on distinct horizontal axes (~75° apart), each carrying a splayed wire ring child | converged (parent) |
| triangular_bail_ring | rec_grenade_var_bailfuze | safety_lever(top_plate/side_strap) / safety_pin(shaft/eye_head) / bail_ring(bail_wire+pivot_pin); joints lever_pivot(rev), safety_pin_slide(pris), bail_swivel(revolute, parent=body, axis -X, 0..2.8 rad) | Single pull pin + triangular bent-wire bail on a revolute swivel above the collar (bail child of body, not pin) | converged (workbench, rating pending sync) |

## Multiplicity / Copy Logic
- count_param: fragmentation rows = `len(KNOB_ROWS_Z)` (mk2/taperegg/frag*) or `N_KNOB_ROWS` (fragcyl). Frag columns are NOT a separate axis — folded into row multiplicity (cols held KNOB_COLS=8 / N_KNOB_COLS=8). Smooth and perforated-vent shells have no knob copy (vent holes are HOLE_ROWS_Z × HOLES_PER_ROW boolean cuts, not parts).
- N 样本已覆盖: rows {3, 5, 7} → rec_grenade_var_frag3row / rec_model-an-mk2-...-a8628c01 (parent, 5 rows) / rec_grenade_var_frag7row ; cross frag-on-cyl 6 rows → rec_grenade_var_fragcyl
- 模板建议 N_range: rows [3, 7], cols [6, 12] (sampling domain; samples cluster at small N, large N safe by construction)
- copied object / naming / placement / joint policy:
  - copied object: one beveled fragmentation knob (chamfered box for mk2-spine; tapered _fragmentation_knob solid for fragcyl-spine)
  - naming: `frag_row_i` conceptually per row (mk2 fuses all knobs into one `frag_body` mesh Compound; fragcyl emits per-knob named visuals `frag_knob_{i_row}_{i_col}`)
  - placement: equal-z rows (KNOB_ROWS_Z) × equal-angle columns (45° spacing, offset 22.5° to keep lever meridian between columns), seated on the body lathe surface normal
  - joint policy: all knobs are FIXED into the body (no articulation); only the fuze parts articulate

## 排除项(未来 compatibility matrix 素材)
- color / scale: not axes (materials and overall dimensions are fixed identity, not swept).
- frag columns: folded into row multiplicity; not an independent count axis.
- combinations not built (except the two 组合抽检 fragcyl = frag-on-cyl and ventovoid = vent-on-ovoid): the full Slot A × Slot B × Slot C matrix is not enumerated.
- Joint invariant verified across ALL variants: lever joint stays REVOLUTE (lever_pivot, axis -Y), pull pin stays PRISMATIC (pin_slide / safety_pin_slide / primary_pin_slide+secondary_pin_slide), ring stays REVOLUTE (ring_swivel / primary_ring_swing+secondary_ring_swing / bail_swivel). bailfuze swaps the ring-on-pin chain for a bail-on-body revolute but keeps the rev/pris/rev type profile.
