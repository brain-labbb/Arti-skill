# Window / Sliding Window — template source map

pattern: mixed (parallel_children 固定 named slots: outer_frame + sill/track 主机构 + fixed lite(s) + sliding sash(es) + glazing/muntins;叠加 multiplicity 两条 — divided-light 网格 N 与 sash/panel 数 N)。**类别身份 = sash 经 PRISMATIC 滑动**（horizontal ±X 或 vertical ±Z），区别于 casement/awning 的 REVOLUTE。

slug: sliding_window   shard: Window__Sliding_Window   prefix: rec_sliding_window_var_

## parents（3 个原始 + 90 forks，各占网格若干格子）

- rec_two-panel-horizontal-sliding-window-white-vinyl-_20260608_164438_890097_5d4512bc（2 扇横滑：1 FIXED 左 + 1 PRISMATIC 右,白 vinyl 厚框,无 muntin,cam latch）：覆盖 [orientation=horizontal] × [sash_count=2] × [sliding=1] × [muntin N=1] × [palette=white_vinyl]。helper `_slab` / `_build_frame_shape`(L127-136 实心 slab 切单一大开口) / `_build_sash_shape`(L139-152) / `_build_sash_glass_shape`(L155-160) / `_add_sash`(L167-179) / `_add_latch`(L182-208 Box keeper plate + Cylinder lever);part `frame_shell` / `fixed_sash_vinyl` / `fixed_sash_glass` / `sliding_sash_vinyl` / `sliding_sash_glass` / `sliding_sash_latch_plate` / `sliding_sash_latch_lever`;joint frame→fixed_sash FIXED(L235-241) + frame→sliding_sash **PRISMATIC axis (-1,0,0)**(L249-257,positive q 向 -X 开)。**=horizontal 2-panel 基线母体(最干净的滑窗起点)。**
- rec_three-panel-horizontal-sliding-window-with-white_20260608_163648_084076_860f2131（3 扇横滑：2 FIXED 侧 + 1 PRISMATIC 中,白 vinyl,colonial 4×5 muntin）：[horizontal] × [sash_count=3] × [sliding=1(center)] × [muntin N=20(4×5)] × [palette=white_vinyl]。helper `_slab`(L117-129) / `_build_frame_shape`(L132-142 slab 切**三个** lite 开口留两 mullion) / `_build_sash_grille_shape`(L145-193,内含**竖 muntin loop** `for c in range(1, GRILLE_COLS)` L172-180 + **横 muntin loop** `for r in range(1, GRILLE_ROWS)` L183-191) / `_build_sash_glass_shape`(L196-201) / `_add_lite`(L208-226);part `frame`(frame_shell) / `left_lite_*` / `right_lite_*` / `center_sash_*`;joint frame→left/right_lite FIXED(L268-281) + frame→center_sash **PRISMATIC axis (1,0,0)**(L289-297)。**=multiplicity 网格母体(GRILLE_COLS×GRILLE_ROWS loop-emitted PASS)。**
- rec_double-hung-sash-window-with-white-frame-two-sta_20260608_162713_405786_6c54f6e4（VERTICAL double-hung：上下两扇竖滑,白框,3×2 muntin,sash lock）：[orientation=vertical] × [sash_count=2(stacked)] × [sliding=2(上下都动)] × [muntin N=6(3×2)] × [palette=white_painted]。helper `_build_frame_shape`(L100-140,含 jamb side-track 槽 `for sign,edge_x` × `for track_y` L127-138) / `_build_sash_frame_shape`(L147-195,**嵌套 lite 切格 loop** `for ci in range(N_COLS): for ri in range(N_ROWS)` L182-193) / `_build_sash_glass_shape`(L198-229,同嵌套 pane loop L217-228) / `_add_sash`(L236-247);part `frame_shell` / `lower_sash_frame`/`lower_sash_glass` / `upper_sash_frame`/`upper_sash_glass` / `lower_sash_lock_body`/`lower_sash_lock_lever`;joint frame→lower_sash **PRISMATIC axis (0,0,1)**(L298-308,positive q 升) + frame→upper_sash **PRISMATIC axis (0,0,-1)**(L311-321,positive q 降)。**=vertical 母体 + sliding_sash_count=2 母体 + 嵌套网格 loop-emitted PASS。**

### forks（90 个,3 家族 ×30,填充 palette / 网格 N / 五金 / sliding 数 / orientation / 比例空格）

- 家族 001 (`rec_qwen37v_sliding_window_001_v01..v30`) fork 自 three-panel/two-panel 横滑母体;v01=square 2-panel,v10/v15/v20/v25/v30=landscape 3-panel,部分为 vertical 变体。
- 家族 002 (`rec_qwen37v_sliding_window_002_v01..v30`) fork 自 two-panel 横滑母体;含 both-sash-sliding(双滑)、roller blocks、insect screen、overlap fin。
- 家族 003 (`rec_qwen37v_sliding_window_003_v01..v30`) fork 自 vertical double-hung 母体;含 aluminum palette、horizontal 变体、2×3/3×2 muntin、pull-cup/pull-handle。

## loop-emission 合约审计

- **PASS(已 loop 发射重复子件)**:
  - three-panel(竖 muntin `for c in range(1,GRILLE_COLS)` L172-180 + 横 muntin `for r in range(1,GRILLE_ROWS)` L183-191) — muntin 网格全 loop。
  - double-hung(嵌套 lite 切格 `for ci in range(N_COLS): for ri in range(N_ROWS)` L182-193 + pane loop L217-228 + jamb side-track `for sign,edge_x` × `for track_y` L127-138) — 网格 + track 全 loop。
  - fork 003_v15(横滑 alu,2×3 muntin `for ci in range(N_COLS): for ri in range(N_ROWS)` L202-213 / pane L237-248) — 网格 loop。
  - fork 002_v10(双滑) roller blocks `for i,sign in ...` `{prefix}_roller_{i}` L274-289 — roller 对 loop。
  - fork 001_v25(alu,REVOLUTE latch) muntin `for c/for r` L179-200(GRILLE_COLS=4,GRILLE_ROWS=5) — 网格 loop。
- **FAIL/N/A(手写或无重复)**:
  - two-panel 母体 — sash 只有 2 扇(`_add_sash` 各调用一次,非 range loop);**单光无 muntin** 故无网格 loop。off 它做 *grid multiplicity* 变体必须先把 muntin 改 nested range loop;本批网格 multiplicity 变体 fork 自 **three-panel / double-hung 两个已 PASS loop-emission 母体**,不阻塞。
  - latch / lock / pull-cup 五金 — 单个手写(`_add_latch` 单实例,`lower_sash_lock_body`/`_lever` 各一);**非复制子件**,不触发 loop-emission 合约(五金是 named singleton,不属 multiplicity 轴)。
  - **结论**:两条 multiplicity 轴(muntin 网格 N、sash/panel 数 N)的派生变体均 fork 自 **已 PASS loop-emission 的母体**(three-panel GRILLE loop / double-hung 嵌套 N_COLS×N_ROWS loop),硬性满足 for-i-in-range 重写合约;palette / 五金 / 比例 / orientation 变体不新增重复子件。

## 组合数预审（HARD GATE）

- Slot A orientation_drive = 2 候选(horizontal_slide ±X / vertical_double_hung ±Z)
- Slot B panel_layout(sash_count 多重度 distinct N) = 4(2 / 3 / 4 / 2-stacked)
- Slot C sliding_sash_count = 2 候选(1 / 2)
- Slot D divided_light_grid 多重度 distinct N = 5(1 / 6 / 9 / 12 / 20)
- Slot E sash_hardware = 5 候选(cam_latch / revolute_latch / pull_cup / pull_handle / sash_lock)
- **组合数 = 2 × 4 × 2 × 5 × 5 = 400 ≥ 10 ✓**（即便仅 A×B×C = 16 ≥ 10 已单独过闸）。palette_style(6) / track_profile(4) 为并行视觉轴,另外乘上更大。

## Slot 候选覆盖

### Slot A:orientation_drive（主滑动机构槽 — sash 平移方向 / PRISMATIC 轴）
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| horizontal_slide（基线） | rec_two-panel-...5d4512bc / rec_three-panel-...860f2131 | frame_shell / sliding_sash + frame_to_sliding_sash(PRISMATIC axis ±1,0,0) | sash 沿头/底 track 左右平移;FIXED lite 在后,sliding sash proud +Y 从前滑过 | converged(parent) |
| vertical_double_hung | rec_double-hung-...6c54f6e4 | frame_shell(含 jamb side-track) / lower_sash(PRISMATIC z) / upper_sash(PRISMATIC -z) | sash 沿 jamb 竖轨上下升降;两扇 offset Y 平面错开;meeting rail 重叠 | converged(parent) |

### Slot B:panel_layout（panel/sash 数多重度 — N panels 沿滑动轴）
| 候选(N) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| two_panel_2（1 fixed + 1 slide,基线） | rec_two-panel-...5d4512bc | fixed_sash / sliding_sash + 单 mullion-less meeting | 横滑 2 格,中央 meeting stile 重叠(无固定 mullion) | converged(parent) |
| three_panel_3（2 fixed + 1 center slide） | rec_three-panel-...860f2131 | left_lite / right_lite / center_sash + 两 mullion | 横滑 3 格,两侧固定 + 中扇滑,两 mullion 分隔 | converged(parent) |
| stacked_2（vertical double-hung 上下两扇） | rec_double-hung-...6c54f6e4 | lower_sash / upper_sash 沿 Z 堆叠 | 竖向 2 扇堆叠,meeting rail 重叠 | converged(parent) |
| four_panel_4（XOX 或 OXXO bank） | rec_sliding_window_var_panel_4 | sash_{i} `for i in range(4)` + 3 mullion + per-unit fixed/slide policy | 横滑 4 格,部分滑部分固定 | planned(forked ← three-panel,空格) |

### Slot C:sliding_sash_count（可动滑扇数 S∈{1,2}）
| 候选(S) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| single_slider_1（基线） | rec_two-panel-...5d4512bc / rec_three-panel-...860f2131 | 1 个 PRISMATIC sash,其余 FIXED lite | 仅 1 扇可滑,余固定 | converged(parent) |
| dual_slider_2 | rec_double-hung-...6c54f6e4(上下都动) / rec_qwen37v_sliding_window_002_v10(横滑双滑) | 两 PRISMATIC sash(对向轴) | 两扇皆可滑;横滑 rear axis(1,0,0)+front axis(-1,0,0)(002_v10 L243-265),竖滑 lower(0,0,1)+upper(0,0,-1) | converged(parent + fork) |

### Slot D:divided_light_grid（muntin 分格多重度 — N panes per sash）
| 候选(N) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| no_muntin（单光,N=1,基线） | rec_two-panel-...5d4512bc | sliding_sash_glass(单 pane,无 muntin) | 单整玻璃光,patio-slider 风 | converged(parent) |
| colonial_3x2_6 | rec_double-hung-...6c54f6e4 | N_COLS=3,N_ROWS=2 + 嵌套 lite/pane loop(L182-193 / L217-228) | 每扇 3×2=6 格 | converged(parent) |
| colonial_2x3_6（landscape 取向） | rec_qwen37v_sliding_window_003_v15 | N_COLS=2,N_ROWS=3 + 嵌套 loop(L202-213) | 每扇 2×3=6 格(竖长格) | converged(fork) |
| colonial_3x3_9 | rec_qwen37v_sliding_window_001_v05 | GRILLE_COLS=3,GRILLE_ROWS=3 + 竖/横 muntin loop | 每扇 3×3=9 格 | converged(fork) |
| colonial_4x5_20 | rec_three-panel-...860f2131 / rec_qwen37v_sliding_window_001_v25 | GRILLE_COLS=4,GRILLE_ROWS=5(L65-66) + 竖 loop L172-180 / 横 loop L183-191 | 每扇 4×5=20 格 colonial 网格 | converged(parent + fork) |

### Slot E:sash_hardware（滑扇五金 — latch / lock / pull）
| 候选 | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| cam_latch（基线） | rec_two-panel-...5d4512bc / rec_qwen37v_sliding_window_002_v10 | `_add_latch`(L182-208) → sliding_sash_latch_plate(Box) + sliding_sash_latch_lever(Cylinder) | meeting stile 上 keeper plate + thumb-turn cam lever(静态 visual) | converged(parent) |
| revolute_latch（可动小翻拨） | rec_qwen37v_sliding_window_001_v25 | `_build_latch_shape`(L225) + latch part + `sash_to_latch`(REVOLUTE axis 0,0,1,L350-358) | center sash meeting rail 上可动 REVOLUTE 拨杆 | converged(fork) |
| sash_lock（meeting rail cam lock） | rec_double-hung-...6c54f6e4 | lower_sash_lock_body(Box L279-284) + lower_sash_lock_lever(Box L285-290) | double-hung meeting rail 中央 cam-action sash lock(静态 visual) | converged(parent) |
| pull_cup（嵌入式凹杯） | rec_qwen37v_sliding_window_003_v05 | `_build_pull_cup_shape` + 环形 rim + back plate | 下扇底 rail 圆形凹杯 grip | converged(fork) |
| pull_handle（meeting stile 立把手） | rec_qwen37v_sliding_window_003_v15 | HANDLE_BODY(L85)/HANDLE_GRIP(L86) + Box | sliding sash 立面把手 bar,mid-height | converged(fork) |

## Multiplicity / Copy Logic

- **两条多重度轴**:
  - count_param D `divided_light_grid (cols × rows)` — N 样本已覆盖 distinct {1, 6, 9, 20} → no_muntin(2-panel parent) / 3×2(double-hung parent) / 2×3·3×3(fork) / 4×5(three-panel parent)。模板建议 N_range cols×rows ∈ [1,5]×[1,6]。copied object = muntin 条(竖 loop `for c in range(1,cols)` + 横 loop `for r in range(1,rows)`)+ 对应玻璃格(嵌套 `for ci/for ri`);naming `vertical_muntin_{i}`/`horizontal_muntin_{i}` 或 `{sash}_pane_{ci}_{ri}`;placement = sash 内框均分规则网格;joint policy = muntin 与 pane 随所属 sash 的 PRISMATIC(无独立 joint)。母体均 loop-emitted PASS。
  - count_param B `panel_layout (sash_count N)` — N 样本已覆盖 distinct {2, 3, 4, 2-stacked} → two-panel(parent) / three-panel(parent) / stacked(parent) / panel_4(planned fork)。模板建议 N_range [2, 6](横滑);竖向固定 2 stacked。copied object = sash 扇 + 其框/玻璃/muntin/五金;naming `sash_{i}` / `{name}_lite` / mullion 分隔;placement = 沿滑动轴规则偏移,以 mullion 分隔(横滑)或 Z 堆叠(竖滑);joint policy = S 扇 PRISMATIC(对向/同向轴),余 FIXED lite。
- **Slot C sliding_sash_count S∈{1,2}** 不是连续复制轴而是 gating choice:决定 N 个 panel 中哪几个挂 PRISMATIC(其余 FIXED)。S=2 横滑 = rear/front 对向轴(002_v10),S=2 竖滑 = lower/upper 对向轴(double-hung)。
- 机构(orientation A)与五金(E)为固定 named slot,非复制轴(五金是 singleton,非 multiplicity)。

## 格子覆盖（parent 基线计入)

- Slot A:2 parent 格(horizontal / vertical)converged,无空格(滑窗仅此 2 滑动方向)。
- Slot B:3 parent 格(2 / 3 / stacked)converged;1 空格 → four_panel planned(fork ← three-panel)。
- Slot C:1+1 parent/fork 格(single / dual)converged。
- Slot D:2 parent N(1 / 20)+ 1 parent N(6=3×2)converged;2 空格 → 2×3 / 3×3 converged(fork)。
- Slot E:3 parent/family 格(cam_latch / sash_lock)+ 3 fork 格(revolute_latch / pull_cup / pull_handle)converged。
- 共 **1 NEW 计划变体(panel_4)** 填规划空格(≤10 cap);其余空格已被 90 forks 覆盖。每槽 ≥2 候选、两条多重度轴各 ≥3 distinct N。

## 排除项（未来 compatibility matrix 素材)

- **REVOLUTE-swing sash 属 `window` 小类**:本小类不把侧铰/顶铰/底铰 swing 作 identity,Slot A 仅 PRISMATIC 滑动(避免出类目混入 casement/awning/hopper)。
- **sliding_door 区分**:门落地(sill 至地面)、人通行整扇、无窗台之上立面 + 无多光分格 bank 语义;窗站窗台之上、以采光分格 + glazed-sash-in-frame 为身份。
- two-panel 母体**单光无 muntin loop**:任何从它派生 *grid multiplicity* 变体必须先把 muntin 改 nested range loop;本批 grid 变体仅 fork 自 three-panel/double-hung(已 PASS loop-emission),已避开。
- 颜色/材质/纯尺寸/glass tint/aspect 比例**不作拓扑轴**(SPEC §2,仅作 palette/scale 视觉轴)。
- 候选潜在不收敛风险待 fork 后回填:dual_slider 两扇 Y 平面必须 offset(否则 sliding 时 sash ring 互穿,见 double-hung SASH_Y_GAP L63-65 / 002_v10 rear/front Y 错位);revolute_latch 小拨杆 anchor 须在 sash meeting rail 实体面(否则 joint-origin 漂浮);panel_4 mullion 分隔不足会令相邻滑扇穿模 — 三者须在 run_tests 内分别断言。
