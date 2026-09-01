# Door / Sliding Door — template source map

slug: `sliding_door`  shard: `Sliding_Door`

pattern: multiplicity(N glazed leaves on parallel PRISMATIC tracks + 1 fixed pane, per-leaf for-i loop + shared leaf helper + uniform prismatic joint policy) × named structural slots (kinematics / track_style / infill / handle / framing)

## Parents (converged baselines on the grid)

- rec_door_slide_2panel ← picture/Door/Sliding Door/001.png （2-panel bypass: root `door_frame` + built-in `fixed_pane` + 1 `sliding_leaf` on single `leaf_slide` PRISMATIC; `head_track`/`sill_track` lip channels grown from the frame; slim black alu; full glass; flush bar `handle_lever`. baseline = N2 × kinematics:bypass × track:concealed-header × infill:full-glass × handle:flush-bar × framing:slim-black） — converged-parent
- rec_door_slide_3panel_tele ← 002.png （3-panel telescoping: root `frame_and_fixed_pane` (inlined fixed sub-frame) + `inner_sliding_pane` + `outer_sliding_pane`, two PRISMATIC `frame_to_inner_pane`/`frame_to_outer_pane` along -X; shared `_add_panel_frame` helper; separate Y track planes; full glass; `inner_handle`/`outer_handle` boxes. baseline = N3 × telescoping × concealed × full-glass × slim-black） — converged-parent
- rec_door_slide_4panel_tele ← 003.png （4-panel telescoping: root `frame` (inlined fixed pane) + `door_0`/`door_1`/`door_2`, three PRISMATIC `slide_0/1/2` along +X; shared `_add_leaf` helper + `track_rib` enumerate loop; panes hand-written door_0/1/2; `door_2` carries box `handle`. baseline = N4 × telescoping × concealed × full-glass × slim-black） — converged-parent
- rec_door_slide_5panel_tele ← 004.png （5-panel telescoping: root `frame` (inlined fixed pane) + `pane_0..pane_3`, four PRISMATIC `pane_i_slide`; **gold-standard loop emission** `for i in range(4)` panes+joints + `_add_leaf`/`_lane_y`/`_pane_center_x_closed` helpers; per-lane Y; full glass; in-leaf box `handle`. baseline = N5 × telescoping × concealed × full-glass × slim-black） — converged-parent
- rec_door_slide_4panel_center ← 005.png （4-panel center bi-parting: root `frame` w/ 2 fixed `outer_pane_0/1` + `mullion_i` enumerate loop + `head_track`/`sill_track` + `door_0`/`door_1` center leaves, two PRISMATIC `door_0_slide`/`door_1_slide` **mimic-coupled** (one scalar opens both); leaves hand-written; **wide white vinyl framing** + curved C-pull `_add_curved_handle` (Cylinder necks+grip). baseline = N4 × kinematics:center-biparting × concealed × full-glass × handle:curved-C-pull × framing:wide-white） — converged-parent

## Converged variants (FILLED cells — variant pool COMPLETE, do not re-fork)

- rec_sliding_door_var_6panel_tele ← off rec_door_slide_5panel_tele （**Slot A multiplicity N6, telescoping**: extends the gold-standard loop to `for i in range(5)` → 5 sliders + 1 fixed = 6 panes, six `pane_i_slide` PRISMATIC, deeper `LANE0_Y`/`FRAME_D` lane stack; `_add_leaf`/`_lane_y`/`_pane_center_x_closed` reused; run_tests asserts exactly 5 sliders + 5 joints. distinct-N extender N=6） — converged
- rec_sliding_door_var_d_loop_handle ← off rec_door_slide_2panel （**Slot E handle: proud round-tube D-loop**: replaces the flush bar with `handle_grip` Cylinder + two `handle_neck_i` standoff Cylinders (loop) on `sliding_leaf`; run_tests proves grip stands ≥15mm proud of the stile face. frame/track/joint unchanged from the 2-panel bypass parent） — converged
- rec_sliding_door_var_flush_finger_pull ← off rec_door_slide_4panel_tele （**Slot E handle: flush finger-pull pocket**: `_build_pocket_stile_cq` CadQuery `cutBlind` pocket → `mesh_from_cadquery` mesh reused as `stile_lead` on door_0/1/2; run_tests asserts mesh-backed pocket stile and that no proud bar `handle` exists. telescoping/glass/frame unchanged） — converged
- rec_sliding_door_var_muntin_grid ← off rec_door_slide_4panel_tele （**Slot D infill: muntin-grid divided-lite**: `_add_muntin_grid` helper emits a 2×3 grid of `lite_i` glass boxes + `vbar_i`/`hbar_i` aluminium glazing bars (nested for-loops) on the fixed pane and every leaf; run_tests asserts 6 lites + bars per pane, transparent lites, opaque bars. telescoping/frame/handle unchanged） — converged
- rec_sliding_door_var_raised_panel ← off rec_door_slide_2panel （**Slot D infill: solid opaque Shaker raised wood panel**: `add_shaker_panel` helper (stile/rail loops + recessed flat `*_center` field) replaces glass on both fixed and `sliding_leaf`; opaque `wood_border`/`wood_center` materials; run_tests proves border stands proud of recessed center and wood is opaque. bypass/frame/joint unchanged） — converged
- rec_sliding_door_var_slatted_louver ← off rec_door_slide_2panel （**Slot D infill: slatted louver slats**: `_add_louver_slats` helper emits ~N tilted `slat_fixed_i`/`slat_leaf_i` blade boxes via for-i loop with uniform roll tilt; opaque `slat_wood`; run_tests proves ≥20 slats/pane and non-zero slat tilt. bypass/frame/handle/joint unchanged） — converged
- rec_sliding_door_var_barn_rail ← off rec_door_slide_2panel （**Slot C track_style: exposed barn-track rollers**: replaces the parent's perimeter frame + concealed in-frame head/sill channels with a single exposed proud horizontal `rail_bar` (`barn_rail` root part) mounted above the opening, two top-hung roller hangers on the leaf (`for i in range(2)`: `hanger_i` bracket + curved `roller_i` wheel mesh via shared `_build_roller_mesh` CylinderGeometry rotated to the Y axle), two full-height wall mounting strips (`bracket_plate_i`/`bracket_arm_i` loop) + a bottom `floor_guide_pin`/`floor_guide_base` stub; keeps full glass + slim dark-alu leaf + lever handle + single horizontal PRISMATIC `leaf_slide` (axis -X, identical to bypass parent). run_tests asserts exposed `rail_bar`, 2 hangers + 2 rollers, floor guide, NO `jamb_*`/`head_track`/`sill_track`, rollers contact the rail bottom, hangers stay within rail width at rest + open. fills the second track_style candidate） — converged

## 组合数预审（HARD GATE — recomputed against converged state，pool COMPLETE）

结构槽与候选数（filled = parent ∪ converged variant；变体池已全部 converged 落盘）：
- track_style：**2**（concealed-header(5 parents + 6 other converged variants) / exposed-barn-track-rollers(`rec_sliding_door_var_barn_rail`, CONVERGED)）— 唯一曾不满足 ≥2 的槽，已由 barn_rail 填满。
- infill：**4**（full-glass(parents) / muntin-grid(muntin_grid) / solid-opaque-raised-panel(raised_panel) / slatted-louver(slatted_louver)）
- handle：**3**（flush-bar(parents) / proud-D-loop(d_loop_handle) / flush-finger-pull-pocket(flush_finger_pull)；center parent 的 curved-C-pull 为第 4）
- kinematics：**3**（telescoping / bypass / center-biparting）— 全部由 parents 采样
- framing：**2**（slim-black-alu(parents) / wide-white-vinyl(4panel_center)）
- distinct N：**5**（N=2,3,4,5 来自 parents + N=6 来自 6panel_tele converged）

每个 slot ≥2 ✓（track_style 现为 2: concealed + barn_rail）。Multiplicity distinct-N = 5（≥2-3 要求，已 ≥4 from parents）✓。
combo = 两个 headline 非-N 结构槽 × distinct-N = track(2) × infill(4) × N(5) = **40 ≥ 10 ✓**。
计入全部轴：track(2) × infill(4) × handle(3) × kinematics(3) × framing(2) × N(5) ≫ 10。**GATE P1 met.**

GAP FORKS：**0**（唯一历史 gap = exposed barn-track，已由 `rec_sliding_door_var_barn_rail` converged 填满）。变体池 COMPLETE。

## Slot 候选覆盖（converged 基线 = parent / converged variant；变体池 COMPLETE — 无 planned-forking）

### Slot A:multiplicity_N（主轴——玻璃扇片数；distinct N=2/3/4/5/6）
| 候选 N | record_id | 关键 part·joint·helper 名 | 状态 |
|---|---|---|---|
| N2 bypass | rec_door_slide_2panel | door_frame / sliding_leaf / leaf_slide | converged-parent |
| N3 tele | rec_door_slide_3panel_tele | frame_and_fixed_pane / inner+outer pane / _add_panel_frame | converged-parent |
| N4 tele | rec_door_slide_4panel_tele | frame / door_0/1/2 / slide_0/1/2 / _add_leaf | converged-parent |
| N5 tele | rec_door_slide_5panel_tele | frame / pane_0..3 / pane_i_slide / _add_leaf+_lane_y | converged-parent |
| N4 center | rec_door_slide_4panel_center | frame / door_0/1 (mimic) / _add_curved_handle | converged-parent |
| N6 tele | rec_sliding_door_var_6panel_tele | frame / pane_0..4 / pane_i_slide (range(5)) | converged |

### Slot B:kinematics（开合运动学）
| 候选 | record_id | 关键 joint / 结构 | 状态 |
|---|---|---|---|
| telescoping | 3p/4p/5p/6p variants | 逐扇 PRISMATIC，后扇行程更远，nest 堆叠 | converged-parent/converged |
| bypass | rec_door_slide_2panel | 单扇前 depth plane 越过固定扇 | converged-parent |
| center-biparting | rec_door_slide_4panel_center | 两中扇 mimic 对开 | converged-parent |

### Slot C:track_style（顶轨 + 吊挂样式）
| 候选 | record_id | 关键结构 | 状态 |
|---|---|---|---|
| concealed-header | 全 5 parents + 其余 6 converged variants | 框内暗藏 head/sill 槽道（`head_track`/`sill_track` 或 `track_rib_i`） | converged-parent/converged |
| exposed-barn-track-rollers | rec_sliding_door_var_barn_rail | 外露横钢轨 `rail_bar`(`barn_rail` root) + per-leaf 滚轮吊架(`for i in range(2)`: `hanger_i` + 曲面 `roller_i` 轮盘 via `_build_roller_mesh`) + 全高墙挂条 `bracket_plate_i`/`bracket_arm_i` + 底 `floor_guide_pin` 导向，off 2-panel bypass，单 PRISMATIC `leaf_slide` | converged |

### Slot D:infill（扇芯/嵌板样式）
| 候选 | record_id | 关键结构 | 状态 |
|---|---|---|---|
| full-glass | 全 5 parents | 整片透明玻璃 | converged-parent |
| muntin-grid-divided-lite | rec_sliding_door_var_muntin_grid | 透明玻璃 + 2×3 分格 muntin 栅（`_add_muntin_grid` 嵌套 for-i），off 4-panel | converged |
| solid-opaque-raised-panel | rec_sliding_door_var_raised_panel | 无玻璃，Shaker 实心木面板（`add_shaker_panel` 凸边框+凹中心场），off 2-panel | converged |
| slatted-louver | rec_sliding_door_var_slatted_louver | 倾斜百叶 slat（`_add_louver_slats` for-i + 统一 roll 倾角），off 2-panel | converged |

### Slot E:handle / pull（拉手样式）
| 候选 | record_id | 关键结构 | 状态 |
|---|---|---|---|
| flush-bar | 2p/3p/4p/5p parents | 薄竖条 box `handle`/`handle_lever` 拉手 | converged-parent |
| curved-C-pull | rec_door_slide_4panel_center | 弓形 Cylinder C-pull（`_add_curved_handle`） | converged-parent |
| proud-D-loop | rec_sliding_door_var_d_loop_handle | 凸出圆管 D 环（`handle_grip` + `handle_neck_i` Cylinder） | converged |
| flush-finger-pull-pocket | rec_sliding_door_var_flush_finger_pull | CadQuery `cutBlind` 凹槽 finger pull（mesh stile），无凸拉手 | converged |

### Slot F:framing（stile/rail 边框样式）
| 候选 | record_id | 关键结构 | 状态 |
|---|---|---|---|
| slim-black-alu | 2p/3p/4p/5p parents + 多数 converged | 细窄黑铝边框 | converged-parent |
| wide-white-vinyl | rec_door_slide_4panel_center | 宽白 vinyl 边框 | converged-parent |

## Multiplicity / Copy Logic

- count_param: **N = 玻璃扇总数**（telescoping: 1 fixed + (N-1) sliders；center-biparting: 2 fixed 侧扇 + 2 中分滑扇；bypass: N=2 单滑扇）。converged 采 N=2,3,4,5,6。
- 模板建议: N_range 主轴。telescoping 用逐扇 `for i in range(N-1)` + 行程 `(i+1)*pitch - i*nest` + 各扇独立 Y lane（`_lane_y(i)`）；center-biparting 用对称 per-side 循环 + 单标量 mimic 耦合。5/6-panel variants 已是 loop 标杆 helper（`_add_leaf`/`_lane_y`/`_pane_center_x_closed`），其余 parent 写 template 时统一改 loop-emission。
- copied object / naming / placement / joint policy: 滑扇 `pane_i` / `door_i`，关节 `pane_i_slide` / `slide_i`，统一 PRISMATIC（telescoping 同向、center-biparting 反向 mimic）。

## Loop-emission 问题（readability contract）

`grep -nE "for .* in (range|enumerate)"` 结果：
- **2panel**：单滑扇手写（N=1 moving，OK）；但其 fork 出的 infill/handle 变体（raised_panel/slatted_louver/d_loop_handle）已各自引入真实 for-i helper（shaker/louver/neck），合规。template 化做 N-multiplicity 须改 loop。
- **3panel_tele**：用 `_add_panel_frame` 共享 helper，但 inner/outer 两滑扇 + 固定扇 **逐个手写调用**。→ multiplicity 变体 off 3p 须请求 per-leaf `for i in range(N)` 重写。
- **4panel_tele**：仅 `track_rib` enumerate loop；三滑扇 door_0/1/2 手写。off-4p 的 muntin_grid / flush_finger_pull 继承手写三扇但各自 infill/pocket 用 helper，合规；template 化滑扇须改 loop。
- **5panel_tele / 6panel_tele**：**完整 loop 标杆** — `for i in range(N-1)` 造扇 + 关节，list-comp 取回，run_tests 全 loop。
- **barn_rail (off 2-panel bypass)**：hanger/roller/bracket 三组硬件均用 `for i in range(2)` loop emission（`hanger_i`/`roller_i`/`bracket_plate_i`/`bracket_arm_i`），共享 `_build_roller_mesh` helper；run_tests 对 hangers/rollers 用 for-i 断言。template 化做 N-multiplicity 时 roller hanger 须随扇数扩成 `for i in range(N)`。
- **4panel_center**：仅 mullion enumerate loop；两中扇 door_0/1 + 两固定扇手写。→ 任何 off-center 的 multiplicity 变体须请求 per-leaf loop + 对称 mimic 重写。

## 排除项（未来 compatibility matrix 素材 / dropped axes）

- **dropped: N=6+ center-biparting** — 早期规划的 biparting_n3 / biparting_n6 未 fork；center-biparting 已由 N4 parent 占 kinematics 槽，N 轴改由 telescoping 路线（N2..N6）覆盖，distinct-N=5 已足。biparting 的高 N 留作模板 compatibility 素材。
- **dropped: bottom-kick-rail storefront infill** — 早期规划的 bottom_kick_rail / storefront_n5 未 fork；infill 槽已由 full-glass / muntin / solid-raised / louver 四候选满足，kick-rail 留作模板第 5 infill 素材。
- **dropped: wide-commercial-stile framing 变体** — framing 槽已由 slim-black + wide-white 两 parent 候选满足；商用宽 stile 留作模板 framing 素材。
- **dropped: pocket-recess 墙袋 kinematics** — kinematics 槽已由 telescoping/bypass/center 三候选满足；墙袋 cassette 留作模板素材。
- **dropped: 纯 color/material/scale** — 按规则禁列为轴（raised_panel/louver 的木材/百叶是**结构 infill** 变化，非纯换色）。
- **dropped: 顶部固定亮窗(transom)** — 属附加固定装饰层而非主结构轴，留作模板 decoration param。
- 注意（converged barn_rail，已落盘的实现要点，template 化沿用）：roller hanger 锚在 leaf 顶、`leaf_slide` joint origin 落在 `rail_bar` 底面接触线（`RAIL_BOTTOM_Z`）而非 mm 级锚垫；外露 `rail_bar` + 全高墙挂条 + 底 `floor_guide_pin` 均 inline 为 root `barn_rail`.visual（非 FIXED-joint 装饰件）；曲面轮盘用共享 `_build_roller_mesh`（CylinderGeometry 绕 X 旋 90° 对齐 Y 轴）；run_tests 已含 `expect_contact`(roller↔rail) + `expect_within`(hanger 在轨宽内, rest+open) + `expect_gap`(floor guide 在扇底下方)。
