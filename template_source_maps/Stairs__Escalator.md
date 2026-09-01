# Stairs / Escalator — template source map

pattern: mixed (multiplicity step-band + named structural slots: balustrade style / incline geometry / landing-pit)
parents:
- rec_build-a-realistic-articulated-3d-model-of-a-esca_20260609_215041_285956_e7415cae ← picture/Stairs/Escalator/002.png (single unit; glass balustrade; hinged maintenance pit-cover; N_STEPS=11)
- rec_build-a-realistic-articulated-3d-model-of-a-esca_20260609_215038_449368_8388963a ← picture/Stairs/Escalator/001.png (twin side-by-side units; dark-metal balustrade; guide-track/roller step chain; no pit-cover; n_steps≈16 computed)

## Slot 候选覆盖

### Slot A:balustrade_style (side balustrade panel material + transparency)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| tinted_glass_panel | rec_..._e7415cae (parent) | frame / glass_left, glass_right (tinted_glass) | see-through smoked glass side panels, welded rubber handrail on top edge | converged(parent) |
| solid_metal_panel | rec_..._8388963a (parent) | truss_frame / balustrade_left, balustrade_right (dark steel) | opaque metal side panels, welded rubber handrail band | converged(parent) |
| solid_metal_panel (on single-unit) | rec_escalator_var_metalpanel | frame / balustrade_* (opaque metal) | swap glass→solid metal on the glass parent shell | converged(forked) |
| tinted_glass_panel (on twin) | rec_escalator_var_glasspanel | truss_frame / balustrade_*_a, balustrade_*_b (tinted glass) | swap metal→tinted glass on the twin parent shell | converged(forked) |

### Slot B:incline_geometry (incline angle + horizontal run length)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| standard_30deg | rec_..._e7415cae (parent) | frame.truss_body / frame_to_steps (prismatic incl) | ~30° incline, medium run | converged(parent) |
| standard_28deg_long | rec_..._8388963a (parent) | truss_frame / step_travel_* (prismatic incl) | ~28° incline, long run | converged(parent) |
| steep_short_35deg | rec_escalator_var_steep | frame.truss_body / frame_to_steps | ~35° steep compact incline, axis re-derived | converged(forked) |
| shallow_long_22deg | rec_escalator_var_shallow | frame.truss_body / frame_to_steps | ~22° shallow long-run incline, axis re-derived | converged(forked) |

### Slot C:landing_pit (lower-landing maintenance access)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| hinged_pit_cover | rec_..._e7415cae (parent) | pit_cover / frame_to_pit_cover (revolute +Y) | rectangular pit + diamond-tread cover hinging up on landing edge | converged(parent) |
| open_flush_landing | rec_..._8388963a (parent) | truss_frame / landing_plates | flush comb/floor plates, no pit, no extra joint | converged(parent) |
| hinged_pit_cover (on twin) | rec_escalator_var_pitcover | pit_cover_* / frame_to_pit_cover_* (revolute) | add per-unit pit + hinged cover to the twin parent | converged(forked) |

## Multiplicity / Copy Logic
- count_param: step_count N (N_STEPS / n_steps) — number of treads+risers in the moving step band, emitted via for-i-in-range(n) with a shared step helper.
- N 样本已覆盖: {7, 11, 14, 16, 22}
  - 7  → rec_escalator_var_steps7 (single, short)
  - 11 → rec_..._e7415cae (parent, single)
  - 14 → rec_escalator_var_twinsteps14 (twin, per-unit)
  - 16 → rec_..._8388963a (parent, twin, computed)
  - 22 → rec_escalator_var_steps22 (single, tall)
- 模板建议 N_range: [4, 60](模板采样域,远大于样本覆盖值正常)
- copied object / naming / placement / joint policy:
  - copied object: one step = tread + riser (+ stringer/roller bracket on twin spine); naming: step/tread/riser_i (name_i style).
  - placement: regular along the incline axis, pitch = (STEP_DEPTH, STEP_RISE); shared incline-dir helper.
  - joint policy: single PRISMATIC step-band joint per unit (axis = incline unit vector), one step-pitch travel; handrails always FIXED (welded loop); pit cover REVOLUTE about +Y when present.

## 排除项(未来 compatibility matrix 素材)
- (none observed yet — both parents and all 8 planned cells are distinct grid positions; populate after fork results return.)

---

## Combo pre-audit (hard gate)
- Slot A (balustrade_style): 2 candidates {glass, metal}
- Slot B (incline_geometry): coarse {standard, steep, shallow} = 3 candidates
- Slot C (landing_pit): 2 candidates {hinged_pit_cover, open_flush}
- distinct N: 5 {7, 11, 14, 16, 22}
- product = 2 × 3 × 2 × 5 = 60 ≥ 10 ✅

## Grid placement (one variant per empty cell)
- Parent A (e7415cae): glass + standard-30° + hinged-pit + N=11
- Parent B (8388963a): metal + standard-28° + open-flush + N=16(twin)
- Variants:
  - steps7        → glass single, N=7   (multiplicity)        ← A
  - steps22       → glass single, N=22  (multiplicity)        ← A
  - metalpanel    → metal single        (balustrade_style)    ← A
  - steep         → glass single ~35°   (incline_geometry)    ← A
  - shallow       → glass single ~22°   (incline_geometry)    ← A
  - glasspanel    → glass twin          (balustrade_style)    ← B
  - pitcover      → twin + hinged pit   (landing_pit)         ← B
  - twinsteps14   → metal twin, N=14    (multiplicity)        ← B
