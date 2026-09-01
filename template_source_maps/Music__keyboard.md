# Music / keyboard — template source map

pattern: mixed
parents:
  - rec_compact-midi-keyboard-controller-with-twenty-fiv_20260611_163826_287630_99ed5d64 ← picture/Music/keyboard/001.png (covers: compact MIDI controller body `compact_midi_keyboard_controller`, faders/knobs/pads control surface, joystick bender, key_count 13/25/37)
  - rec_analog-synthesizer-keyboard-with-two-octaves-of-_20260611_163604_601709_c8a11966 ← picture/Music/keyboard/002.png (covers: analog synth body `analog_synth_keyboard`, pad-grid control surface, two-wheels bender, wood-cheeks chassis, key_count 49)

Both families share the core kinematic spine: a rear-hinged keybed of `white_key_{i}` + `black_key_{j}` parts (REVOLUTE about +X, lower=0→press) plus deck/panel-mounted control parts. They differ in part-naming convention and control mix:
- Parent A (`compact_midi_keyboard_controller`): chassis visuals `bottom_shell`/`control_deck`/`key_bed`/`{side}_end_cap`; key joints named `white_key_{i}_press` / `black_key_{j}_press`; native control = 8 drum pads (`drum_pad_{i}` 2×4, PRISMATIC z−) + 4 column knobs (`knob_{k}`, REVOLUTE z); static `display_strip`+`strip_rim_*` touch area. N_WHITE=15 → 25 keys default.
- Parent B (`analog_synth_keyboard`): chassis visuals `base_shell`/`panel_housing`/`keybed_cheek`/teal `{section}_frame_*`; key joints named `chassis_to_white_key_{i}` / `chassis_to_black_key_{j}`; native control = dense 20-knob field (`{section}_knob_{r}_{c}` 12 + `master_knob_{c}` 8, REVOLUTE z, each with `pointer`) + 4 `env_slider_{s}` (PRISMATIC +Y on `slider_slot_{s}`); static `bender_block`+`pitch_strip`/`mod_strip`/`bend_strip_{0,1}`. WHITE_KEY_COUNT=15 → 25 keys default.

## Slot 候选覆盖

### Slot A:control_surface
The deck/panel control cluster mounted above the keybed. Loop-emitted repeated parts; revolute knobs + the gridded interfaces carry kinematics, pads/faders are prismatic.

| 候选(future module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| pad_grid_16 | rec_music_keyboard_var_control_padgrid | `pad_{i}` ×16 (`pad_body`+`backlight`) / `chassis_to_pad_{i}` PRISMATIC axis(0,0,−1) lower=0..PAD_PRESS; chassis `pad_bezel_platform`+`pad_frame_{front/back/left/right}` | [B] 4×4 backlit drum-pad grid replaces B's 20-knob field; per-row backlight color; keeps keys + 4 env sliders | converged(已同步) |
| fader_bank_9 | rec_music_keyboard_var_control_faderbank | `fader_{i}` ×9 (`cap_body`+`cap_grip`) / `fader_{i}_slide` PRISMATIC axis(0,1,0) lower=0..FADER_TRAVEL; chassis `fader_slot_{i}`+`fader_rail_{i}_{left/right}` | [A] bank of 9 linear faders replaces A's drum pads; drops the 4 column knobs; keeps keys + display strip | converged(已同步) |
| knob_grid_8 | rec_music_keyboard_var_control_knobbank | `grid_knob_{i}` ×8 (`knob_cap`) / `grid_knob_{i}_turn` REVOLUTE axis(0,0,1) ±GRID_KNOB_RANGE; retains `knob_{k}` ×4 / `knob_{k}_turn`; chassis `grid_knob_bezel_{r}_{c}` | [A] 2×4 rotary-knob grid replaces A's drum pads; keeps the 4-knob left column; keeps keys | converged(已同步) |
| pad_block_8 (parent A baseline) | rec_compact-midi-keyboard-controller-with-twenty-fiv_20260611_163826_287630_99ed5d64 | `drum_pad_{i}` ×8 (`pad_cap`) / `drum_pad_{i}_press` PRISMATIC z−; `knob_{k}` ×4 / `knob_{k}_turn` REVOLUTE z; chassis `pad_bezel_{r}_{c}` | [A] baseline: 2×4 drum pads + 4 column knobs + static display strip | converged(parent) |
| knob_field_20 (parent B baseline) | rec_analog-synthesizer-keyboard-with-two-octaves-of-_20260611_163604_601709_c8a11966 | `{section}_knob_{r}_{c}` ×12 + `master_knob_{c}` ×8 (`knob_body`+`pointer`) / `chassis_to_{name}` REVOLUTE z ±KNOB_LIMIT_RAD; `env_slider_{s}` ×4 / `chassis_to_env_slider_{s}` PRISMATIC +Y | [B] baseline: dense 20-knob field + 4 env sliders | converged(parent) |

### Slot B:pitch_bender_interface
The expression controller at the front-left (left of the keybed). Parent baseline is a static touch/display strip; the two sampled modules add articulated benders.

| 候选(future module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| joystick_gimbal | rec_music_keyboard_var_bender_joystick | `joystick_gimbal`(`ring`) / `joystick_pitch` REVOLUTE axis(0,1,0) ±JOYSTICK_TILT (parent=chassis); `joystick_stick`(`shaft`) / `joystick_mod` REVOLUTE axis(1,0,0) (parent=gimbal, nested); chassis `joystick_socket` | [A] 2-DOF spring-return gimbal joystick; nested child chain (stick rides the gimbal); sits on A's drum-pad deck | converged(已同步) |
| pitch_mod_wheels | rec_music_keyboard_var_bender_twowheels | `wheel_{i}` ×2 (`wheel_body`, CadQuery disc w/ hub bore+grip groove) / `chassis_to_wheel_{i}` REVOLUTE axis(1,0,0) ±WHEEL_LIMIT_RAD spring-return; chassis `wheel_cheek` base + `wheel_bracket_{0,1,2}` axle walls | [B] upright pitch+mod wheel pair on a horizontal cross-axle; replaces B's bender_block/touch-strips | converged(已同步) |
| touch_strip (parent baseline) | rec_compact-midi-keyboard-controller-...99ed5d64 / rec_analog-synthesizer-keyboard-...c8a11966 | A: `display_strip`+`strip_rim_{front/rear/inner/outer}` (static chassis visuals); B: `bender_block`+`pitch_strip`/`mod_strip`/`bend_strip_{0,1}` (static) | [A/B] baseline: static touch/display strips, no articulation | converged(parent) |

### Slot C:chassis_form
Overall body shape carrying the keybed + control surface.

| 候选(future module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| flat_slab (parent baseline) | rec_compact-midi-keyboard-controller-...99ed5d64 / rec_analog-synthesizer-keyboard-...c8a11966 | A: `bottom_shell`+`control_deck`+`key_bed`+`{side}_end_cap`; B: `base_shell`+`panel_housing`(+`keybed_cheek`) | [A/B] baseline: flat desktop slab, horizontal control deck/panel | converged(parent) |
| upright_wood_cheeks | rec_music_keyboard_var_chassis_woodcheeks | chassis visuals `angled_panel` (wedge mesh, rises front→back) replacing flat panel + `base_shell`; `cheek_0`/`cheek_1` tall walnut end boards (mirrored, trapezoidal, taller at back); knobs/sliders mounted with `rpy=(PANEL_TILT_ANGLE,0,0)` so they sit flush on the tilt | [B] tilted wedge panel + tall wooden end cheeks; controls reseated on the inclined surface (`panel_surface_z(y)`) | converged(已同步) |

## Multiplicity / Copy Logic
- count_param: key_count — total = (# naturals) + (# sharps); naturals drive the keybed loop, sharps come from the `SHARP_AFTER` / `BLACK_AFTER_WHITE` boundary list; `N_WHITE` (A) / `WHITE_KEY_COUNT` (B) parametrically sets `CORE_W = N_WHITE * KEY_PITCH` so body width + end caps follow.
- N 样本已覆盖: {13, 25, 37, 49} → rec_music_keyboard_var_keycount_13 (8 white + 5 black) / parents(=25, 15 white + 10 black) / rec_music_keyboard_var_keycount_37 (22 white + 15 black) / rec_music_keyboard_var_keycount_49 (29 white + 20 black)
- 模板建议 N_range: [13, 61] (standard controller octaves; sampled 13–49, white-count→parametric core-width scaling proven, so larger N is safe by construction; ≥61 sparsely sampled)
- copied object / naming / placement / joint policy:
  - copied object: one part per natural `white_key_{i}` (visual `key_body`) + one per sharp `black_key_{j}` (visual `key_body`); each gets its own REVOLUTE joint.
  - naming: 0-based `white_key_{i}` / `black_key_{j}`; joint convention is family-specific — A uses `white_key_{i}_press` / `black_key_{j}_press`, B uses `chassis_to_white_key_{i}` / `chassis_to_black_key_{j}` (template must pick one canonical scheme).
  - placement: rear-hinged row along +X at fixed `KEY_PITCH` (A: cx=−CORE_HALF+KEY_PITCH·(i+0.5); B: FIRST_WHITE_X+i·KEY_PITCH); sharps centered on the natural boundaries in `SHARP_AFTER`; 49-key B also recenters the body (`BODY_CENTER_X`, larger `BODY_HALF_W`).
  - joint policy: every key REVOLUTE about +X axis(1,0,0), lower=0, upper=press_rad (rest at 0, front tip dips on press); all keys share identical limits.
  - sub-counts (control-surface-local copy loops, independent of key_count): pads `pad_{i}`/`drum_pad_{i}` = 8 (A baseline) or 16 (padgrid); faders `fader_{i}` = 9; grid knobs `grid_knob_{i}` = 8 + column `knob_{k}` = 4; B section knobs = 12 + `master_knob_{c}` = 8 (= 20); env sliders `env_slider_{s}` = 4; bender wheels `wheel_{i}` = 2 (fixed pair).

## 排除项(future compatibility matrix 素材)
- Single-axis sampling only: every variant changes exactly ONE axis off its own parent baseline, so no cross-axis combinations are sampled — e.g. wood-cheeks × faderbank, joystick × pad-grid, two-wheels × knobbank, padgrid × 49-key, joystick × flat-slab-on-B, etc.
- Control-surface↔parent coupling not yet decoupled: faderbank/knobbank/joystick were only forked from the compact-A body; padgrid/two-wheels/wood-cheeks only from synth-B. The future matrix should prove each control_surface and pitch_bender module on BOTH chassis families and both joint-naming conventions.
- key_count crossed only with each parent's default control surface (13/25/37 on A's pad+knob deck, 49 on B's knob field); key_count × {padgrid, faderbank, knobbank, joystick, two-wheels, wood-cheeks} unsampled.
- N beyond 49 (61/76/88) unsampled — parametric by construction but not validated for deck/control fit at extreme widths.
