# Technology / Remote_Control — template source map

pattern: multiplicity  (homogeneous button caps x N is the dominant copy unit; body + cover + wheel are named slots on top)

parents (origins, forked only from these):
- rec_a-black-handheld-thermostat-style-remote-control_20260624_124939_052056_fdc967f6 <- picture/Technology/Remote_Control/002.png
  covers: SlotA=thermostat_slab, SlotB=dpad_ring_cluster(N~7), SlotC=all_prismatic_buttons. Model name `fireplace_thermostat_remote`.
- rec_black-sony-soundbar-remote-control-a-slim-rectan_20260605_173757_125686_b2c6bc8a <- picture/Technology/Remote_Control/001.png
  covers: SlotA=slim_wand, SlotB=two_column_grid(N~9), SlotC=prismatic_buttons + central_revolute_rocker. Model name `sony_soundbar_remote`.

status: converged  (7 variants planned; none forked yet)

## Readability audit (§4) of origins

- Origin B (Sony): buttons ARE looped — `for key,x,y,r in round_buttons:` emits `btn_{key}` with shared `_round_button_mesh` helper + per-button PRISMATIC joint `body_to_btn_{key}`; oval BASS pads a second `for ... in oval_buttons:` loop. Central `volume_rocker` a single named part with a REVOLUTE joint. CLEAN baseline for the multiplicity axis.
- Origin A (thermostat): dpad IS looped — `for part_name,joint_name,start,end,label in sectors:` emits dpad_up/down/left/right (annular-sector caps) with shared body + PRISMATIC joints. BUT the 2 round buttons are 2 hand-written `add_round_button(...)` calls and `mode_button` is an inline block. VIOLATION (mild): the homogeneous round caps are not loop-emitted. Count is tiny (2) and each button type differs, so acceptable as-is; however the `numeric_keypad` fork from Origin A MUST introduce the 12 keys as a nested `for r/for c` loop emitting `key_{r}_{c}` with a shared helper — its prompt says so explicitly. Do NOT let that fork hand-write keys.

## Slot 候选覆盖

### Slot A: body_form (③ Primary Form Family)
| 候选(未来 module) | source_type | record_id / evidence | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| thermostat_slab | forked_anchor | origin A | body / body_shell (_rounded_box) | wide portrait rounded rectangular slab, flat +Z face, LCD upper half | converged(origin) |
| slim_wand | forked_anchor | origin B | body / body_shell (_body_solid) | long uniform-width thin rounded-rectangle wand | converged(origin) |
| ergonomic_contour | forked_anchor | rec_remote_control_var_ergonomic_contour (parent A) | body / body_shell | waisted/pinched contoured TV-remote envelope, curved long sides, same flat front face | converged |
| tapered_wedge | forked_anchor | rec_remote_control_var_tapered_wedge (parent B) | body / body_shell | teardrop wand, wider at button end, narrowing to logo end | converged |
| compact_streaming_stick / candybar / symmetric_stick | world_knowledge_extrapolation (③ Volumetric Envelope / Planar Boundary) | anchors above + reviewer | same body part + same front-face control interface | short/compact or symmetric envelope, template-side reshape only | template-side |

### Slot B: button_layout (button count & arrangement — MULTIPLICITY)
| 候选(未来 module) | source_type | record_id / evidence | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| dpad_ring_cluster (N~7) | forked_anchor | origin A | dpad_{up,down,left,right} + mode_button + power_button + light_button, all PRISMATIC | central 4-sector directional ring around a MODE hub + 2 round keys | converged(origin) |
| two_column_grid (N~9) | forked_anchor | origin B | btn_{input,power,sound_field,voice,night,mute} + btn_bass_{up,down}, PRISMATIC | two side columns of round caps + oval BASS pair flanking central control | converged(origin) |
| numeric_keypad_3x4 (N=12 added) | forked_anchor | rec_remote_control_var_numeric_keypad (parent A) | key_{r}_{c} + body_to_key_{r}_{c} PRISMATIC (nested for-loop) | regular 3col x 4row numeric matrix below the dpad → universal TV remote | converged |
| minimal_media_cluster (N~4) | forked_anchor | rec_remote_control_var_minimal_cluster (parent B) | reduced btn_{key} loop, PRISMATIC | streaming-style sparse button set, same looped emission fewer copies | converged |

### Slot C: input_mechanism (② joint / mechanism type)
| 候选(未来 module) | source_type | record_id / evidence | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| prismatic_buttons_only | forked_anchor | origin A | body_to_* PRISMATIC (axis -Z) | every control presses straight into the face | converged(origin) |
| prismatic + central_revolute_rocker | forked_anchor | origin B | body_to_volume_rocker REVOLUTE (axis Y) | large central rocker tilts +/- about width axis | converged(origin) |
| jog_scroll_wheel | forked_anchor | rec_remote_control_var_jog_wheel (parent B) | volume_rocker / body_to_volume_rocker → CONTINUOUS (axis +Z) | central disc spins continuously about face normal | converged |
| flip_down_cover | forked_anchor | rec_remote_control_var_flip_cover (parent A) | cover / body_to_cover REVOLUTE (axis Y) at hinge boss | hinged lid swings up to reveal lower button cluster | converged |
| slide_cover | forked_anchor | rec_remote_control_var_slide_cover (parent B) | cover / body_to_cover PRISMATIC (axis X) on rails | panel slides along wand to expose/hide buttons | converged |

## Multiplicity / Copy Logic
- count_param: `button_count` (front-face pressable caps). Sub-loops: `key_count` for the numeric keypad grid (rows x cols), `round_button_count` / `oval_button_count` for the wand column layout, dpad ring fixed at 4 sectors.
- N 样本已覆盖: {~4 (minimal_cluster), ~7 (origin A dpad+keys), ~9 (origin B two-column), 12 (numeric_keypad 3x4)} → rec_..._minimal_cluster / origin A / origin B / rec_..._numeric_keypad
- 模板建议 N_range: total buttons [4, ~50]; numeric keypad grid rows in [0,4] x cols in [3,4] (0 = keypad absent); side columns [2, 8] per column. Sample coverage stays small on purpose — sweep fans the rest.
- copied object: one round (or annular-sector) button cap = mesh + material + a PRISMATIC press joint into the front face.
- naming: `btn_{key}` (wand) / `dpad_{dir}` (ring) / `key_{r}_{c}` (numeric grid) / `mode_button` hub.
- placement: dpad = equal-angle annular sectors around a center; wand = equal-pitch rows in two Y columns; keypad = equal-pitch X/Y grid.
- joint policy: every cap is an INDEPENDENT PRISMATIC press (axis into face, ~1.2–1.8 mm travel); no chaining, no shared hub motion. Cover/wheel are separate single-part mechanisms (revolute / prismatic / continuous).

## 视觉多样性 6 轴考察 (对齐 SPEC §8.5)

| 轴 | 处理 | 本小类取值 / 范围 / 理由 |
|---|---|---|
| ① 骨架图(+N) | forked_anchor → Slot A/B/C | body(root) + N pressable button parts + optional cover/wheel; no world-knowledge new skeleton candidates |
| ② 关节类型 | forked_anchor (随 module) | PRISMATIC (button press, axis into face) baseline; REVOLUTE (central rocker Y / flip cover Y); CONTINUOUS (jog wheel +Z); PRISMATIC (slide cover X). No WK-new joint types |
| ③ 主体形态家族 | forked_anchor + world_knowledge_extrapolation | anchors: thermostat_slab, slim_wand, ergonomic_contour, tapered_wedge. WK-extrapolate: Volumetric Envelope (compact streaming stick, chunky universal) + Planar Boundary (symmetric candybar, teardrop) keeping same body part + front-face interface |
| ④ 表面装饰 | record_only + world_knowledge_extrapolation | observed: SONY deboss relief, printed button labels (INPUT/VOICE/NIGHT/MODE), 7-seg + icon LCD glyphs, speaker/power icons, degree symbol. WK host-conformal extrapolation: brand wordmarks, silk-screen icon sets, recessed well rings, IR-emitter dome at top edge |
| ⑤ 尺寸/行程 | record_only | body length/width ratio wand ~3.8:1 → slab ~2.4:1; button press travel 1.2–1.8 mm; rocker tilt ±0.18 rad; cover swing 0→~1.7 rad; wheel continuous |
| ⑥ 涂装 | record_only | material: molded plastic (matte black / charcoal baseline). Colorways ≥6: matte black, charcoal grey, warm white, silver/graphite, two-tone body+cover, brushed-metal accent; LCD backlight orange/amber; accent green power ring |

## Compatibility Probes
| probe_id | source_type | record_id | 组合轴值 | 验证目标 | 结论 |
|---|---|---|---|---|---|
| (none planned) | — | — | — | — | — |

Note: numeric_keypad x slim_wand is a real interface risk (3-column grid too wide for a narrow wand) — flagged for the template compatibility matrix; not forked as a probe because the keypad anchor is built on the wider thermostat_slab where it fits. Record as a gated cell in the spec.

## 排除项 (未来 compatibility matrix 素材)
- numeric_keypad on slim_wand body: excluded as an anchor (3-wide grid overflows narrow wand width) — keypad anchored on thermostat_slab instead; template should gate keypad against narrow-wand form.
- flip_cover / slide_cover simultaneously with a full numeric keypad: untested combo; leave to template compatibility matrix, not a fork-side candidate.
