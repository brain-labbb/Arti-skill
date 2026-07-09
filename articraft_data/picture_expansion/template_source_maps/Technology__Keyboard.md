# Technology / Keyboard — template source map (DRAFT)

小类: keyboard (computer keyboard — NOT a music keyboard)
pattern: mixed (dominant = multiplicity of keys × N; accessories = parallel_children off the chassis root)

parents (origins; fork ONLY from these):
- rec_a-white-compact-wireless-computer-keyboard-tenke_20260624_123545_283861_38c84f4e ← picture/Technology/Keyboard/002.png — white compact wireless TKL. Code = 87 keys (function row + alpha block + right nav column INSERT/HOME/PGUP·DEL/END/PGDN·arrows, NO numpad). Keys loop-emitted via `_keyboard_layout()` → `_add_key()`, named `key_<label>`; shared `_keycap_mesh()`; each key one PRISMATIC `chassis_to_key_<label>`. Covers grid cells: Layout=TKL, Chassis=flat-low-profile(open tapered deck), Feet=none, Knob=none.
- rec_full-size-black-wireless-computer-keyboard-with-_20260605_173906_815714_c52b6770 ← picture/Technology/Keyboard/001.png — image is a FULL-SIZE black board WITH a numpad, but the code is a ~88-key compact layout (no numpad). Keys loop-emitted via nested `for row_index, row in enumerate(LAYOUT)`, named `key_r{row}_c{col:02d}`; shared `_keycap_mesh()`; each key one PRISMATIC `chassis_to_key_r{r}_c{c}`. Covers grid cells: Layout≈TKL(~88), Chassis=flat-low-profile(recessed well: `chassis_slab`+`rim_*`), Feet=none, Knob=none.

Origin reconciliation: BOTH origins are on-grid and forked-from (no exclusions). NOTE both sit in ~the same Layout cell (TKL-class, no numpad) and the same Chassis cell (flat low-profile) — so Layout and Chassis are under-covered by origins alone and are filled by variants. Origin2's image shows a numpad its code omits → the "full-size + numpad" candidate is fork-realized (keyboard-numpad), not read straight from source.

Readability (§4) audit: PASS for both. Keys are the dominant N-copy and are emitted by loops with shared cap helper + grid placement + PRISMATIC per-key joints in both origins (origin1 `key_<label>`, origin2 `key_r{r}_c{c}`). No hand-written per-key parts, no `left_panel`/`right_panel` collapse. Every variant prompt preserves the loop + shared `_keycap_mesh()` + per-key prismatic joint.

---

## Slot 候选覆盖

### Slot A: key_layout_block  (MULTIPLICITY / LAYOUT — dominant axis)
| 候选(未来 module) | source_type | record_id / evidence | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| tenkeyless (~87–88) | forked_anchor | rec_a-white…38c84f4e + rec_full-size…c52b6770 | `_keyboard_layout()`/`LAYOUT` → `key_<label>` / `key_r{r}_c{c}`, `chassis_to_key_*` prismatic | func row + alpha + right nav column, no numpad | converged (origins) |
| 60%_compact (~61) | forked_anchor | rec_keyboard_var_compact60 (← origin1) | same loop, fewer entries | drop function row + nav column; alpha + mod/space only | converged |
| full_size_numpad (~104) | forked_anchor | rec_keyboard_var_numpad (← origin2) | LAYOUT + right 17-key numpad block, same `key_r{r}_c{c}` loop | add numeric keypad block (realizes 001.png) | converged |
| tkl_plus_macro_column (~93) | forked_anchor | rec_keyboard_var_macrocolumn (← origin2) | LAYOUT + prepended left column, same loop | extra left G1..G6 macro column | converged |
| 65%_75%_compact_with_arrows | world_knowledge_extrapolation (① needs anchor — see 排除项) | anchors: compact60 + TKL | same part tree/interface, only block extents/N | intermediate N between 60% and TKL | template-side (see note) |

### Slot B: chassis_case_form  (③ Primary Form Family — case/layout form)
| 候选(未来 module) | source_type | record_id / evidence | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| flat_low_profile | forked_anchor | rec_a-white…38c84f4e (open tapered deck `tapered_shell`) + rec_full-size…c52b6770 (recessed well `chassis_slab`+`rim_*`) | `chassis` part; keys ride the deck | slim slab, keycaps proud/low | converged (origins; 2 sub-forms) |
| stepped_high_profile | forked_anchor | rec_keyboard_var_highprofile (← origin2) | `chassis_slab`+`rim_*` reworked into tall stepped tray | thick raised-wall case, keys sit deep | converged |
| tented_ergonomic_split | forked_anchor | rec_keyboard_var_split (← origin1) | `chassis` = two tented halves; keys regrouped, still `key_<label>` prismatic | split halves, center gap, outward tent | converged |
| Volumetric-Envelope / Macro-Surface variants | world_knowledge_extrapolation (③: Volumetric Envelope + Macro Surface Construction) | anchors: the 3 above + reviewer | same part tree/interface | thickness (low↔high), open-deck ↔ recessed-well ↔ tray | template-side |

### Slot C: tilt_feet  (② articulation — underside revolute)
| 候选(未来 module) | source_type | record_id / evidence | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| none_fixed | forked_anchor | both origins | (no feet; chassis flat on desk) | integrated flat base | converged (origins) |
| flip_out_tilt_legs | forked_anchor | rec_keyboard_var_tiltfeet (← origin2) | `foot_left`/`foot_right`, `chassis_to_foot_left/right` REVOLUTE (axis +X) | pair of rear flip-out legs, fold/deploy | converged |

### Slot D: media_knob  (② articulation — top-corner revolute)
| 候选(未来 module) | source_type | record_id / evidence | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| none | forked_anchor | both origins | (no knob) | no rotary control | converged (origins) |
| rotary_volume_knob | forked_anchor | rec_keyboard_var_knob (← origin1) | `media_knob`, `chassis_to_media_knob` continuous REVOLUTE (axis 0,0,1) | spinning volume/media dial top-right | converged |

---

## Multiplicity / Copy Logic
- count_param: `key_count` (driven by `_keyboard_layout()` entries / `LAYOUT` per-row width-unit lists).
- N 样本已覆盖: {61, 87, 88, 93, 104} → compact60(61) / origin1 TKL(87) / origin2(88) / macrocolumn(93) / numpad(104).
- 模板建议 N_range: [~40, ~110] (40% ortho minimal ~47 … full-size+macro ~110); sample values only demonstrate copy logic, template域 far exceeds them.
- copied object: one `keycap` visual = LoftGeometry rounded-rect chiclet (`_keycap_mesh`, cached by width-unit) on its own key part.
- naming: `key_<label>` (origin1, semantic) OR `key_r{row}_c{col:02d}` (origin2, grid) — both systematic loop names; template picks one scheme.
- placement: 2-D row/column grid; per-row list of width-units (1u, 1.25u, 1.5u, 1.75u, 2u, 2.25u, 6u space…), cumulative X cursor per row, row pitch in Y, back-rake Z.
- joint policy: EACH key independent PRISMATIC along local -Z, ~1.5mm travel (`MotionLimits(lower=0, upper≈0.0015)`); no chaining, no shared hub. Keys guarantee the ≥1 non-fixed joint on every variant.

---

## 视觉多样性 6 轴考察

| 轴 | 处理 | 本小类取值 / 范围 / 理由 |
|---|---|---|
| ① 骨架图(+N) | forked_anchor → Slot A / Multiplicity | chassis root + N key children; optional accessory children (feet, knob). N-blocks: 60% / TKL / full+numpad / +macro-column. Intermediate 65%/75% is ① (new N/block) → needs anchor or reviewer gate before template扇出; listed record-only/extrapolation for now. |
| ② 关节类型 | forked_anchor (随 module) | PRISMATIC key travel (axis 0,0,-1, ~1.5mm) on EVERY key (both origins); REVOLUTE flip-out tilt feet (axis +X, variant); continuous REVOLUTE media knob (axis 0,0,1, variant). |
| ③ 主体形态家族 | forked_anchor + world_knowledge_extrapolation | anchors: flat_low_profile (open tapered deck + recessed well), stepped_high_profile, tented_split. Extrapolate — Volumetric Envelope: profile thickness low↔high; Macro Surface Construction: open floating-key deck ↔ recessed key well ↔ deep tray case; Planar Boundary: straight rounded-rect footprint ↔ split/angled footprint. |
| ④ 表面装饰 | record_only + world_knowledge_extrapolation | real samples: per-keycap legends (`legend_*` 3×5 pixel glyphs, origin1; origin2 has none), `raised_back_edge`, rim seams (`rim_*`). Extrapolate host-conformal: keycap legends/sublegends, top-plate branding, status LEDs, rubber feet dots — non-structural, non-joint. |
| ⑤ 尺寸/行程 | record_only | chassis width ~0.36m, depth ~0.13m, slab thick ~0.02m, back-raise ~0.006m; key pitch ~0.0185–0.0188m (1u); keycap height ~0.0044–0.0075m; key travel ~0.0015m; key count 61–104. |
| ⑥ 涂装 | record_only | material大类: plastic (both origins); extrapolate anodized-aluminium case. Colorways ≥3–6: warm-white (origin1), matte-black/dark-gray (origin2), + gray, pastel keycap set, two-tone (dark case + light caps), RGB/backlit. Free via palette; used as (6) COMPANION on compact60 / highprofile / knob only. |

## Compatibility Probes
| probe_id | source_type | record_id | 组合轴值 | 验证目标 | 结论 |
|---|---|---|---|---|---|
| (none planned) | — | — | — | — | — |

Note: tented_split (Slot B) × full_numpad (Slot A) is geometrically awkward on real boards (split boards rarely carry a numpad); if the template ever samples it, add a `compatibility_probe` then. Not forked now (組合 = sampler's job).

## 排除项(未来 compatibility matrix 素材)
- (none yet — no fork attempted in planning stage.) Candidate risks to watch during fork:
  - tented_split (keyboard-split): re-tilting per-half key press axis may drift keys / open the center gap incorrectly; if it fails to converge ~2–3×, fall back to a milder wedge/stepped split or record blocked.
  - 65%/75% intermediate layouts: not forked (① needs a source anchor); left as world_knowledge_extrapolation pending reviewer gate, or promote compact60→65% as an extra anchor if the pool needs it.
  - wrist/palm rest and sculpted (non-chiclet) key profile: real but NOT forked this batch (kept to 4 structural slots). Recorded here as future ① / ⑤ candidates — a front palm-rest part or sculpted `_keycap_mesh` profile can be added as a 5th slot later.

## Notes
- Every variant keeps the per-key PRISMATIC joints → ≥1 non-fixed joint guaranteed; tiltfeet + knob add a second real mechanism (revolute).
- Sync: variants stay workbench-only; batch-write rating=5 on convergence per FORK_VARIANTS.md §7.
