# Technology / Audio_Device — template source map

pattern: mixed (named functional slots: body form × speaker grille × speaker layout × controls × carry handle, PLUS a multiplicity axis: control-button count)

parents (all 4 origins reconciled; each already occupies grid cells):
- rec_a-wooden-retro-tabletop-radio-a-horizontal-recta_20260624_122658_140987_c610b240 ← picture/Technology/Audio_Device/003.png — landscape wooden tabletop box; ribbed cream grille; dual REVOLUTE knobs; NO handle, NO antenna.
- rec_retro-vintage-portable-transistor-radio-bronze-a_20260605_173810_150349_43796658 ← picture/Technology/Audio_Device/001.png — landscape bronze box; SlotPattern ribbed grille; THREE CONTINUOUS knobs (knob_specs loop); folding REVOLUTE carry_handle; telescoping antenna.
- rec_a-tan-beige-minimalist-retro-portable-radio-a-re_20260624_122658_140174_25291653 ← picture/Technology/Audio_Device/005.png — compact landscape portable; perforated-mesh grille; FIVE top PRISMATIC push buttons (button_{idx} loop); FIXED arched carry handle; telescoping antenna; folding clip.
- rec_silver-portable-cd-radio-boombox-oval-body-with-_20260605_173820_379190_c648725c ← picture/Technology/Audio_Device/002.png — oval/rounded-slab boombox; single recessed perforated speaker; CD lid (REVOLUTE); 2 CONTINUOUS knobs + 5 transport keys (transport_button_{i} loop) + 2 function keys; telescoping antenna; 4 feet loop.

## Slot 候选覆盖

### Slot A：speaker_grille_construction
| 候选(未来 module) | source_type | record_id / evidence | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| horizontal_ribbed_slats | forked_anchor | rec_...wooden (003); also rec_...bronze (001) | grille_rib_{i} loop + grille_top_rail/grille_bottom_rail/grille_side_rail_{side}; (001) grille_panel SlotPatternPanelGeometry | stacked horizontal louver ribs across the opening | converged (origin) |
| perforated_hole_mesh | forked_anchor | rec_...tan (005); also rec_...boombox (002) | speaker_grille (PerforatedPanelGeometry) + speaker_bezel/speaker_backing | punched round-hole mesh panel | converged (origin) |
| vertical_bar_grille | forked_anchor | rec_audio_device_var_vertical_bar_grille (from 003) | grille_bar_{i} loop within same grille rails | vertical slats running top-to-bottom | converged |
| concentric_ring / sunburst | world_knowledge_extrapolation (Macro Surface Construction) | anchors: ribbed + perforated + reviewer | same grille opening/rails; loop/PerforatedPanel family | radial rings or sunburst bar fan | template-side |

### Slot B：speaker_layout
| 候选 | source_type | record_id / evidence | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| single_center_speaker | forked_anchor | rec_...wooden/bronze/tan/boombox (all 4) | speaker_grille / round_speaker_shadow | one front driver | converged (origin) |
| dual_stereo_flanking | forked_anchor | rec_audio_device_var_dual_stereo_speaker (from 002) | speaker_grille_{i} (i∈{0,1}) + per-side basket/surround/bezel/cap | two symmetric drivers flanking the controls | converged |

### Slot C：control_interface (mechanism)
| 候选 | source_type | record_id / evidence | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| rotary_knob_bank | forked_anchor | rec_...wooden (003, ×2); rec_...bronze (001, ×3) | tuning_knob/volume_knob + cabinet_to_tuning/cabinet_to_volume (REVOLUTE); (001) knob_specs → CONTINUOUS | rotary tuning/volume dials | converged (origin) |
| push_button_row | forked_anchor | rec_...tan (005) | button_{idx} + body_to_button_{idx} (PRISMATIC) | row of pressable preset keys | converged (origin) |
| transport_key_deck | forked_anchor | rec_...boombox (002) | transport_button_{i}_press + function_button_0/1_press (PRISMATIC) | CD/cassette transport key deck + function keys | converged (origin) |

### Slot D：carry_handle (mechanism)
| 候选 | source_type | record_id / evidence | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| no_handle | forked_anchor | rec_...wooden (003); rec_...boombox (002) | — | tabletop / boombox, no carry bail | converged (origin) |
| fixed_arched_bail | forked_anchor | rec_...tan (005) | handle_arch/handle_saddle_{} + body_to_handle (FIXED) | rigid arched carry bail | converged (origin) |
| folding_revolute_bail | forked_anchor | rec_...bronze (001) | carry_handle + cabinet_to_handle (REVOLUTE, X axis) | handle folds flat over top pivots | converged (origin) |
| side_strap / recessed_grip | world_knowledge_extrapolation | anchors + reviewer | same top-mount interface | alternative carry hardware | template-side (not forked) |

### ③ Primary Form Family (body macro form) — registered slot
| 候选 | source_type | record_id / evidence | 关键 part/joint 名 | 形态类别 / 结构特征 | 状态 |
|---|---|---|---|---|---|
| rectangular_box (landscape) | forked_anchor | rec_...wooden (003); rec_...bronze (001); rec_...tan (005) | cabinet/body shell via _rounded_cabinet()/_rounded_box() | Volumetric Envelope: horizontal filleted box (proportions/roundness vary as continuous params) | converged (origin) |
| oval_rounded_slab (boombox) | forked_anchor | rec_...boombox (002) | body_shell via _body_solid() (rounded-square footprint) | Volumetric Envelope: squashed oval slab that sits on the ground | converged (origin) |
| tombstone_vertical_arched | forked_anchor | rec_audio_device_var_tombstone_body (from 003) | cabinet (tall arched-top box) | Volumetric Envelope: upright taller-than-wide with rounded/arched top | converged |
| cathedral_pointed / lunchbox_dome | world_knowledge_extrapolation (Volumetric Envelope) | anchors above + reviewer | same part tree / same front-face interface | other real mantel/portable radio envelopes | template-side |

## Multiplicity / Copy Logic
- count_param: button_count (top preset push-button bank)
- N 样本已覆盖: {3, 5, 8} → rec_audio_device_var_button_count_low / rec_...tan 005 (baseline N=5) / rec_audio_device_var_button_count_high
- 模板建议 N_range: [2, 12]
- copied object / naming / placement / joint policy: pressable button cap (button_cap) ; named button_{idx} via `for idx, x in enumerate(button_xs)` ; placement = evenly spaced along the top control strip X ; joint policy = each button an independent PRISMATIC (body_to_button_{idx}, small down-travel).
- 其它已存在的复制 loop(登记为词汇表，未单独作 multiplicity 轴): grille ribs grille_rib_{i} (003) ; transport keys transport_button_{i} (002) ; base feet foot_{i} (002) ; knob bank knob_specs (001, 3 knobs) ; dial ticks fm_tick_{i}/am_tick_{i} (003) & merged dial_scale ticks (001).

## 视觉多样性 6 轴考察

| 轴 | 处理 | 本小类取值 / 范围 / 理由 |
|---|---|---|
| ① 骨架图(+N) | forked_anchor → 见 Slot 候选覆盖 / Multiplicity | Slots: body form × grille × speaker layout × controls × handle; multiplicity = control-button count. 无世界知识新增骨架 candidate。 |
| ② 关节类型 | forked_anchor(随 module) | REVOLUTE (knob dials, CD lid, folding handle, antenna swivel, folding clip; axis X/Y), CONTINUOUS (knob spins, +Z), PRISMATIC (push buttons, transport keys, antenna telescope). 无世界知识新增 candidate。 |
| ③ 主体形态家族 | forked_anchor + world_knowledge_extrapolation | anchors: rectangular_box (003/001/005), oval_slab (002), tombstone_vertical (fork). 可外推 Volumetric Envelope: cathedral/pointed-top, lunchbox dome-top, slanted-front clock-radio wedge — 同 part tree / 同 front-face interface。 |
| ④ 表面装饰 | record_only + world_knowledge_extrapolation | 真实样本: wood veneer grain strips (top_grain_{i}), front trim/bezel bands (front_trim/black_front_bezel), printed AM/FM dial scale + frequency letters/blocks, ROBERTS logo plate, knob knurling/fluting, LCD face. 可外推 host-conformal: brand plates, dial print variants, rivet/seam bands, knurl density (非结构、非关节)。 |
| ⑤ 尺寸/行程 | record_only | body W≈0.19–0.32 m, aspect landscape→tombstone; knob dia 0.016–0.054; button down-travel ≈0.0016–0.005; antenna telescope stroke ≈0.07–0.106; handle swing 0–90°; CD lid 0–72°; clip 0–~77°; antenna swivel 0–85°. |
| ⑥ 涂装 | record_only | 材质大类: painted/lacquered wood, leatherette-look plastic, silver/chrome ABS, bronze/copper metal, black plastic, cream/ivory buttons, brass hardware. 配色 ≥6: cherry-wood+cream, tan/beige+brass, silver+blue accent, bronze/copper+black, walnut, cream/ivory, matte-black. |

## Compatibility Probes
| probe_id | source_type | record_id | 组合轴值 | 验证目标 | 结论 |
|---|---|---|---|---|---|
| (none planned) | — | — | — | — | — |

## 排除项(未来 compatibility matrix 素材)
- (none observed yet — planning stage, no non-converging axis values recorded.)

## 备注 / 可读性契约
- Readability check on the multiplicity source (005): control buttons ARE loop-emitted (`for idx, x in enumerate(button_xs)` → `button_{idx}` + `body_to_button_{idx}`), so the button-count multiplicity axis reads cleanly with no rewrite required.
- Non-blocking readability note: origins 003 and 002 hand-write their TWO rotary knobs (two `add_knob(...)` / `_knob(...)` calls with semantic names `tuning_knob`/`volume_knob`) rather than a `knob_{i}` loop. Acceptable for N=2 functionally-distinct knobs (tuning vs volume). A KNOB-count multiplicity off those origins WOULD require rewriting to a loop-emitted `knob_{i}` chain — we avoid this by routing the multiplicity axis through 005's clean `button_{idx}` loop. Origin 001's knob bank (knob_specs list → 3 knobs) is already loop-shaped and could carry a knob-count axis if later needed.
- Sync/rating note: variants stay workbench-only; on sync into arti-template, batch-write rating=5. Do NOT promote / do NOT pass --category-slug.
