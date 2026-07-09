# Technology / Laptop — template source map

pattern: mixed (parallel_children deck parts: base + lid + trackpad + pointing device + hinge; multiplicity: keyboard keys × N-by-layout and rubber feet × N; linear articulation chain base→lid)

parents (both are origins; each an anchor, each gets ≥1 fork):
- rec_a-black-budget-15-6-inch-clamshell-laptop-open-a_20260624_123651_161143_65d6f0bd ← picture/Technology/Laptop/002.png
  (black budget 15.6" clamshell, Box primitives; covers Slot A = full-size-with-numpad, Slot B = integrated clickpad, Slot C = single center hinge, Slot D = standard clamshell; feet N=2. Rich Box deck with looped feet/grilles/keys → base for the deck/hinge/form/feet forks.)
- rec_silver-thin-and-light-laptop-computer-with-a-hin_20260605_173856_302145_c183f0ed ← picture/Technology/Laptop/001.png
  (silver thin-and-light, CadQuery filleted aluminum mesh; covers Slot A = compact-no-numpad, Slot B = integrated clickpad, Slot C = single center hinge, Slot D = standard clamshell. Clean ultrabook → base for the business trackpoint fork.)

Readability audit (§4): PASS for both origins. Keyboard keys are loop-emitted (`key_{r}_{c}` via `_add_key`; `key_r{r}_c{c}` via `for r/for c`), each on a per-key PRISMATIC joint. The screen-to-base hinge is a real REVOLUTE joint in both (`base_to_screen` / `base_to_lid`). Origin A also loops `front_foot_{i}` and `speaker_grille_{i}`. No hand-written key parts. No violations.

## Slot 候选覆盖

### Slot A: keyboard_deck_layout (drives the key-count multiplicity, N by layout)
| 候选 (未来 module) | source_type | record_id / evidence | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| compact_no_numpad | forked_anchor | rec_silver-thin-and-light... (origin B) | `key_r{r}_c{c}` 6×14 grid + `base_to_key_r{r}_c{c}` PRISMATIC | ~84 keys, no numeric keypad, tighter deck | converged (origin) |
| full_with_numpad | forked_anchor | rec_a-black-budget... (origin A) | `key_{r}_{c}` 6×~16 grid + numpad block, `base_to_key_{r}_{c}` PRISMATIC | ~90 keys with distinct numeric keypad on the right | converged (origin) |

Note: 13" vs 15.6" vs 17" chassis SIZE is a continuous dimension (template ⑤ param), NOT a separate candidate — the only structural key-layout distinction is numpad-present vs absent. Both covered by origins; no Slot A fork needed.

### Slot B: pointing_device_deck
| 候选 (未来 module) | source_type | record_id / evidence | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| integrated_clickpad (baseline) | forked_anchor | both origins | `trackpad`(`pad_plate`/`click_seam` or `trackpad_glass`) + `base_to_trackpad` PRISMATIC | one glass pad, integrated click, no discrete buttons | converged (origin) |
| discrete_button_trackpad | forked_anchor | rec_laptop_var_trackbtn (parent A) | loop `trackpad_button_{i}` (2) + `base_to_trackpad_button_{i}` PRISMATIC; pad drops `click_seam` | separate physical L/R mouse-button bar below the pad | converged |
| trackpoint_three_button | forked_anchor | rec_laptop_var_trackpoint (parent B) | inline `trackpoint_nub` visual + loop `trackpoint_button_{i}` (3) + `base_to_trackpoint_button_{i}` PRISMATIC | business pointing nub in key gap + upper 3-button row | converged |

### Slot C: screen_hinge_style
| 候选 (未来 module) | source_type | record_id / evidence | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| single_center_barrel (baseline) | forked_anchor | both origins | `hinge_barrel` (one spanning cyl) + `bottom_hinge_cover`; one `base_to_screen`/`base_to_lid` REVOLUTE | single full-width hinge line | converged (origin) |
| dual_side_barrels | forked_anchor | rec_laptop_var_dualhinge (parent A) | loop `hinge_barrel_{i}` (2) + `bottom_hinge_cover_{i}`; SAME single REVOLUTE axis | two exposed side brackets on one hinge line (see 002.png) | converged |

Note: 2-in-1 360° convertible was considered and DROPPED — in a rigid model its only real difference from a clamshell is the lid-open ANGLE/limit, which is a continuous joint param, not a structural axis (would be a fake axis). Recorded as world-knowledge for the template ② limit range, not forked.

### Slot D: chassis_form_family (③ Primary Form Family — Volumetric Envelope)
| 候选 (未来 module) | source_type | record_id / evidence | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| standard_clamshell (baseline) | forked_anchor | both origins | `base_chassis`/`base` + `screen_lid`/`lid`, thin uniform slab | flat thin clamshell envelope | converged (origin) |
| rugged_thick_workstation | forked_anchor | rec_laptop_var_rugged (parent A) | thicker `lower_shell`/`lid_shell` + loop `corner_bumper_{i}` (4) inline visuals | deep chunky bumpered envelope; same part tree/primitive/interface | converged |
| gaming_angular / ultraslim_wedge | world_knowledge_extrapolation (Volumetric Envelope) | anchors: both origins + rugged fork + reviewer | same base/lid part tree + REVOLUTE hinge + key grid | template-side: only macro envelope shape changes (angular gaming deck / tapered wedge) | template-side |

## Multiplicity / Copy Logic
- count_param(s):
  - `key_count` — keyboard keys, N by layout (rows×cols; numpad adds a block).
  - `foot_count` — rubber feet under the base.
  - (world-knowledge extra copy object: `vent_fin_count` — rear cooling vent slats, template-side.)
- N 样本已覆盖:
  - keys: {84 (origin B, 6×14), ~90 (origin A, 6×~16 + numpad)} → 2 distinct N.
  - feet: {2 (origin A `front_foot_{i}`), 4 (rec_laptop_var_feet4 `foot_{i}`)} → 2 distinct N.
- 模板建议 N_range: keys — rows 5–6 × cols 12–19 (numpad toggle); feet — [2, 4]; rear vent fins — [0, ~14] (远大于样本覆盖为正常).
- copied object / naming / placement / joint policy:
  - keys: copied keycap; naming `key_{r}_{c}` / `key_r{r}_c{c}`; regular row/col grid placement; each on its own independent PRISMATIC press joint (axis -Z).
  - feet: copied rubber pad; naming `foot_{i}` / `front_foot_{i}`; regular corner/front-edge placement; inlined base visuals, NO joint (non-moving decoration per §4 Rule 3).

## 视觉多样性 6 轴考察 (对齐下游 SPEC §8.5)
| 轴 | 处理 | 本小类取值 / 范围 / 理由 |
|---|---|---|
| ① 骨架图 (+N) | forked_anchor → 见 Slot 候选覆盖 / Multiplicity | base + lid + trackpad + [pointing device] + hinge; key grid × N, feet × N. No world-knowledge new skeleton candidates. |
| ② 关节类型 | forked_anchor (随 module) | REVOLUTE lid hinge (axis ±X, ~0–130°); PRISMATIC per-key press (−Z); PRISMATIC trackpad/button click (−Z). No new joint types beyond source. |
| ③ 主体形态家族 / Primary Form Family | forked_anchor + world_knowledge_extrapolation | anchors: standard_clamshell (2 origins) + rugged_thick_workstation (fork). Extrapolate Volumetric Envelope: gaming_angular, ultraslim_wedge (same part tree/interface, envelope only). |
| ④ 表面装饰 | record_only + world_knowledge_extrapolation | observed: `back_logo`, `screen_wallpaper`, `speaker_grille_{i}`, key `legend`, `webcam_ring`. Extrapolate host-conformal: vendor logos, lid brushed/texture bands, rugged matte bands, backlit-key legends. |
| ⑤ 尺寸/行程 | record_only | chassis 13"–17" (BASE_W ~0.31–0.40, BASE_D ~0.22–0.26, thin<0.035 → rugged thicker); lid open 0–130°; key travel ~1.0–1.5 mm; trackpad travel ~0.5–0.9 mm. |
| ⑥ 涂装 | record_only | material: painted plastic (matte black) / brushed aluminum (silver). Colorways ≥6: matte black, silver aluminum, gunmetal grey, white, navy, olive-drab rugged, rose gold. |

## Compatibility Probes
| probe_id | source_type | record_id | 组合轴值 | 验证目标 | 结论 |
|---|---|---|---|---|---|
| (none) | — | — | — | — | — |

## 排除项 (未来 compatibility matrix 素材)
- 2-in-1 360° convertible hinge: excluded as a普通 candidate — its only rigid-model difference from clamshell is the lid-open angle limit (continuous ⑤/② param), not a structural axis. Belongs to the template hinge-limit range, not a fork.
- detachable-tablet form: out of scope — removing the keyboard deck as a separable body is a ① skeleton change (would leave category as tablet); not forked.

## 同步备注
- variants stay workbench-only; never promote, no `--category-slug` on finalize. Sync = record dir + materialization cache; rating=5 written by sync script.
