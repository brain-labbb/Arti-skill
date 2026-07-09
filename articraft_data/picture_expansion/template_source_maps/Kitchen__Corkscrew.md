# Kitchen / Corkscrew — template source map

pattern: parallel_children
parents:
- rec_model-a-classic-winged-butterfly-corkscrew-about_20260610_080758_135836_65a27fb6 ← picture/Kitchen/Corkscrew/001.png (classic winged "butterfly" corkscrew: twin black plates + bell skirt, two chrome lever wings, rack spindle, T-bar + helical worm). Covers Slot A=spade, Slot B=straight-T-bar, Slot C=plain, Slot D=winged.

Winged butterfly corkscrew. Core kinematics shared by every winged candidate: a `body_frame`
(root: bell skirt + twin plates + legs + rivet bosses), two `wing_lever_*` levers on rivet
pivots (`wing_pivot_0`/`wing_pivot_1`, REVOLUTE y, ~100° range), a `rack_spindle` that descends
through the plate slot (`spindle_travel`, PRISMATIC −z, ~0.04 m), and a `t_handle_worm` part
(worm helix + shaft core + T-bar) spinning on the spindle (`worm_spin`, CONTINUOUS z). The four
independent structural slots are the wing geometry, the user handle/grip, the foil-cutter crown,
and the drive mechanism itself.

Geometry helpers carry the slot identity: `_build_wing` / `_build_paddle_wing` /
`_build_aero_blade` (Slot A), the handle visuals / `_build_bent_tbar` / `_build_ball_knob`
(Slot B), `_build_foil_cutter_collar` + `_build_cutter_blade` (Slot C), and the presence vs.
absence of the wing parts + `_build_body_frame` bridge cut (Slot D).

## Slot 候选覆盖

### Slot A:wing blade geometry (`wing_lever_*` / `wing_paddle_*` mesh via `_build_*` helper)
**Compatibility note: Slot A only applies when Slot D = winged.** The direct-pull mechanism has
no wings, so wing-geometry candidates are mutually exclusive with that branch (the template's
compatibility matrix must gate Slot A on mechanism=winged).

| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| spade_blade | rec_model-a-classic-winged-butterfly-corkscrew-about_...65a27fb6 (parent) | `wing_lever_0`/`wing_lever_1` parts, `wing_blade` visual, `_build_wing` (polyline forked spade arm + 4 round teeth), `wing_pivot_{i}` REVOLUTE | flat extruded spade-tip blade, forked rounded fork at tip; parent baseline (two hand-written wing parts) | converged |
| paddle_blade | rec_corkscrew_var_wing_paddle | `wing_paddle_0`/`wing_paddle_1` parts, `paddle_blade` visual, `_build_paddle_wing` (wide polyline blade + 6 radial box teeth) | broad flat paddle silhouette (half-width 0.015 m); wings refactored into `for i in range(2)` loop | converged |
| curved_aero_blade | rec_corkscrew_var_wing_curved | `wing_lever_{i}` parts, `wing_blade` visual, `_build_aero_blade` (3-section elliptical `loft`, 3 teeth) | sculpted scimitar/aero blade bowing outward (true 3D lofted form, not flat extrusion); loop-emitted wings | converged |
| lattice_blade | rec_corkscrew_var_wing_lattice | `wing_lever_{i}` parts, `wing_blade` visual, `_build_wing` + `PERF_ROWS` through-hole cut loop | spade blade pierced with staggered round perforations (lightened lattice); loop-emitted wings | converged |

### Slot B:handle / drive grip (top of `t_handle_worm`, on `worm_spin`)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| straight_t_bar | rec_model-a-classic-winged-butterfly-corkscrew-about_...65a27fb6 (parent) | `t_bar` (horizontal Cylinder) + `t_bar_tip_0`/`t_bar_tip_1` (Sphere caps, loop over ±sign) | classic horizontal T cross-bar with small ball ends; parent baseline | converged |
| angled_bar | rec_corkscrew_var_handle_angled_bar | `t_bar` visual = `_build_bent_tbar` mesh (boss + two arms tilted 30° up, ball tips) | V-shaped bent T-bar, arms angled upward from the hub | converged |
| ball_knob | rec_corkscrew_var_handle_ball_knob | part renamed `ball_knob_worm`; `ball_knob` visual = `_build_ball_knob` (lathe sphere + neck) | single spherical grip knob (~21 mm) on a short neck instead of a cross-bar | converged |

### Slot C:foil-cutter crown (optional `foil_cutter` part at bell bore)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| plain (no crown) | rec_model-a-classic-winged-butterfly-corkscrew-about_...65a27fb6 (parent) | (no `foil_cutter` part / no `cutter_spin` joint) | bare bell bore, no foil cutter; parent baseline | converged |
| rotating_cutter | rec_corkscrew_var_crown_rotating_cutter | `foil_cutter` part: `cutter_collar` (`_build_foil_cutter_collar` revolved annulus) + `blade_{i}` visuals (loop over `BLADE_COUNT`=3, `_build_cutter_blade`); `cutter_spin` CONTINUOUS z joint, press-fit in bell bore | annular foil-cutter collar with 3 inward blades that spins inside the bell mouth; adds a 4th non-fixed joint | converged |

### Slot D:drive mechanism (overall kinematic topology)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| winged | rec_model-a-classic-winged-butterfly-corkscrew-about_...65a27fb6 (parent) | `wing_lever_*` + `wing_pivot_0`/`wing_pivot_1` REVOLUTE, `rack_spindle`/`spindle_travel`, `t_handle_worm`/`worm_spin` | two rack-and-pinion lever wings drive the spindle down (the defining butterfly mechanism); parent baseline | converged |
| direct_pull | rec_corkscrew_var_mech_direct_pull | NO wing parts / NO `wing_pivot_*`; `body_frame` gains a top `bridge` (`_build_body_frame` adds bridge + shaft bore); only `spindle_travel` PRISMATIC + `worm_spin` CONTINUOUS | wingless: user spins the T-bar worm in then pulls straight up; small separate branch (no Slot A) | converged |

## Multiplicity / Copy Logic
- count_param: 无 multiplicity 轴。Wings are a **fixed pair of 2** (loop-emitted via `for i in range(2)`
  with `_build_<blade>(inner_sign=±1)` + mirrored `wing_pivot_{i}` axis, but N is fixed at 2 by the
  real object — a butterfly corkscrew always has exactly two opposed wings). The `range(2)` loop is a
  readability refactor, **NOT a multiplicity axis** — do not parameterize wing count.
- The only other looped sub-parts are also fixed-N decorative/structural sets, not N-axes:
  `t_bar_tip_{i}` (2 caps), `rivet_pin_{i}`/`rivet_cap_{i}_{...}` (2 pivots), spindle rack rings (3),
  and the foil cutter `blade_{i}` (`BLADE_COUNT`=3, a fixed crown blade count).
- N 样本: 无。模板建议 N_range: 无 multiplicity axis to sweep.
- copied object / naming / placement / joint policy: parent hand-writes the two wings as
  `wing_lever_0`/`wing_lever_1`; the paddle/curved/lattice variants refactor them into a clean
  `for i in range(2)` loop with `_build_*` geometry helper, mirrored-axis (`(0,±1,0)`) revolute joints,
  and identical motion limits — that loop is the canonical copy-logic sample the template should inherit.

## 组合数预审
Winged-mechanism branch: Slot A (4: spade / paddle / curved / lattice) × Slot B (3: straight-T-bar /
angled-bar / ball-knob) = **12 ≥ 10 ✓**. Slot C (plain / rotating-cutter ×2) multiplies this to 24.
Slot D direct-pull is a separate small branch (no Slot A, so Slot B ×3 × Slot C ×2 = 6 combos).
Every candidate keeps ≥1 non-fixed joint (winged: wing_pivot ×2 + spindle_travel + worm_spin [+ cutter_spin];
direct_pull: spindle_travel + worm_spin). pattern = parallel_children, no multiplicity.

## 排除项(未来 compatibility matrix 素材)
- **Slot A ⟂ Slot D=direct_pull**: wing-geometry candidates are incompatible with the wingless
  direct-pull mechanism (no wings to skin). Compatibility matrix must gate Slot A on mechanism=winged.
- Slot C rotating_cutter and Slot B angled_bar/ball_knob combinations not separately sampled —
  they are independent layers (crown at the bell base, handle at the top), generated by the template sampler.
- Pure dimensional knobs (wing length, blade width, bell taper, T-bar length, worm pitch/height,
  travel distance) are **not** candidates — they are controlled local parameters (continuous), not modules.
- Wing count is fixed at 2 (see Multiplicity) — not a candidate axis.
