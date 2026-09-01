# Window / Window — template source map

pattern: mixed (parallel_children 固定 named slots: outer_frame + sill + sash 主机构 + glazing/muntins;叠加 multiplicity 两条 — divided-light 网格 N 与 sash/unit 数 N)

slug: window   shard: Window__Window   prefix: rec_window_var_

## parents（9 个免费样本,各占网格若干格子）

- rec_arched-top-wooden-casement-window-with-a-radial-_20260608_164259_895412_6cbe185b ← picture/Window/Window/002.png（拱顶木 casement + 上方 radial fanlight + 双扇侧铰）：覆盖 [机构=side-hung] × [轮廓=arched-top] × [muntin N = radial fan 7 + colonial 3×5=15] × [sash=2]。helper `_build_wood_frame_shape` / `_build_fanlight_shape` / `_build_stone_surround_shape` / `_build_keystone_shape` / `_build_sash_frame_shape(sign)` / `_add_sash`;part `stone_arch` / `keystone` / `sill_corbels` / `wood_frame` / `fanlight` / `{name}_frame` / `{name}_glass`;muntin loop `for i in range(FAN_RADIAL_COUNT)` + `for c in range(1,SASH_COLS)` / `for r in range(1,SASH_ROWS)`;joint side-hung REVOLUTE axis (0,0,±1)。
- rec_twin-sash-casement-window-with-dark-anthracite-a_20260608_163558_154599_16e9934d ← 001.png（双扇 casement + anthracite 框 + venetian 百叶）：[side-hung] × [rectangular] × [无 muntin] × [sash=2]。helper `_build_dark_frame_shape` / `_build_sash_frame_shape(sign)` / `_build_blind_slats_shape(sign)` / `_add_sash`;part `dark_frame` / `sill` / `{name}_frame` / `{name}_glass` / `{name}_blinds` / `{name}_handle_base` / `{name}_handle_lever`;slat loop `for i in range(SLAT_COUNT)`;joint REVOLUTE axis (0,0,±1)。
- rec_single-hung-window-with-sage-grey-frame-fixed-up_20260608_163323_437121_d91f35ac ← 005.png（single-hung,上扇固定下扇升降）：[single-hung 竖向 sash] × [rectangular] × [无 muntin] × [sash=2 堆叠,下扇动]。part `frame_shell` / `lower_sash_lock_base` / `lower_sash_cam` / `lower_sash_cam_lever` / `{name}_frame` / `{name}_glass`;helper `_build_frame_shape` / `_build_sash_frame_shape(sash_h)` / `_add_sash`;cam lever REVOLUTE axis (0,0,1)。**注:无任何 for-loop(见 §loop-emission)。**
- rec_round-porthole-window-with-a-glossy-black-ring-f_20260608_163258_577022_30a4c378 ← 008.png（圆 porthole + 黑亮环）：[顶 pivot REVOLUTE] × [round] × [无 muntin] × [sash=1]。helper `_ring_shape` / `_disc_shape` / `_build_outer_frame_shape` / `_build_sash_ring_shape`;part `frame_ring` / `sash_ring` / `sash_glass` / `pivot_pin_{tag}`;joint REVOLUTE axis (1,0,0)。**注:两个 pivot_pin 手写 tag,非 range loop。**
- rec_aluminium-awning-window-with-a-single-top-hung-s_20260608_163252_797981_0b523c52 ← 009.png（铝 awning 单扇顶铰）：[awning 顶铰 REVOLUTE] × [rectangular] × [无 muntin] × [sash=1]。helper `_frame_shape` / `_sash_ring_shape` / `_sash_glass_shape`;part `frame_shell` / `sash_ring` / `sash_glass` / `handle_base` / `handle_lever`;joint REVOLUTE axis (1,0,0),底缘外踢 +Y。**注:无 for-loop。**
- rec_double-casement-window-with-grey-multi-chamber-f_20260608_163228_634338_039f0c0a ← 004.png（双 casement + 多腔框）：[side-hung] × [rectangular] × [无 muntin] × [sash=2]。helper `_build_frame_shape` / `_build_sash_ring_shape` / `_maybe_mirror` / `_add_sash`;part `frame_profile` / `{name}_ring` / `{name}_glass` / `{name}_handle_base` / `{name}_handle_lever` / `{name}_hinge_{i}`;loop `for i in range(CHAMBER_RIBS)`(框腔)+ `for i in range(HINGE_COUNT)`(铰链);joint REVOLUTE axis (0,0,±1)。
- rec_bank-of-grey-aluminium-casement-windows-three-by_20260608_162732_561720_478f6f0c ← 003.png（3×2 bank,一扇可开 + 固定 lites）：[side-hung 1 扇可开] × [rectangular] × [grid 3×2=6] × [unit=6]。helper `_lite_bounds` / `_build_frame_shape` / `_build_fixed_glass_shape` / `_sash_size` / `_build_sash_ring_shape` / `_add_sash`;part `frame_web` / `fixed_glass_{col}_{row}` / `{name}_ring` / `{name}_glass` / `{name}_handle_*` / `{name}_hinge_{i}`;nested loop `for col in range(N_COLS)` / `for row in range(N_ROWS)`;joint REVOLUTE axis (0,0,1)。
- rec_side-hung-wooden-casement-window-swung-open-with_20260608_162725_845156_4f1abc4d ← 007.png（单扇侧铰木 casement,开启 + stay bar）：[side-hung] × [rectangular] × [无 muntin] × [sash=1]。helper `_build_outer_frame_shape` / `_build_sash_frame_shape` / `_build_sash_glass_shape`;part `frame_shell` / `sash_frame` / `sash_glass` / `hinge_{i}` / `stay_bar` / `stay_pivot`;loop `for i in range(2)`(铰链);joint REVOLUTE axis (0,0,1)。**= fork 基线母体(机构/muntin/outline 变体的最干净起点)。**
- rec_weathered-white-painted-steel-industrial-awning-_20260608_162720_563513_3dabb144 ← 006.png（风化钢工业 awning,多 light 网格 + 1 顶铰扇）：[awning 顶铰] × [rectangular] × [手写多 light] × [sash=1 + 固定 lites]。helper `_slab_with_openings` / `_fixed_lite_glass` / `_sash_ring_shape` / `_sash_glass_shape`(内含闭包 `opening` / `pane`);part `frame_shell` / `fixed_lite_glass` / `sash_ring` / `sash_glass`;joint REVOLUTE axis (1,0,0)。**注:固定 lites 用闭包逐个手写,非 range loop。**

## loop-emission 合约审计

- **PASS(已 loop 发射重复子件)**: arched(fan + col/row muntin)、twin(slat)、doublecase(chamber rib + hinge)、bank(col×row 网格 + hinge)、sidehung(hinge)。
- **FAIL/N/A(手写或无重复)**:
  - single-hung — **无任何 for-loop**;上下两扇手写。off 它做 *multiplicity* 变体必须先重写为 `for i in range(n)`。本批未从它派生 multiplicity 变体(机构变体不触发重复子件),故不阻塞。
  - porthole — pivot_pin 用 `_{tag}` 手写两个(非 range);单扇无网格。
  - awningalu / awningsteel — 无 for-loop;awningsteel 的固定 lites 用闭包逐个手写。off awningsteel 做 light/unit multiplicity 必须先改 nested range loop;本批 hopper 变体 fork 自 **awningalu**(机构变体,无重复子件),不触发重写。
  - **结论**:所有派生 multiplicity 变体(muntin 2×2/3×3、triple_mullion、bank_2×2)均 fork 自 **已 PASS loop-emission 的母体**(sidehung / doublecase / bank),硬性满足 for-i-in-range 重写合约;machanism/outline 变体 fork 自手写母体亦无问题(它们不新增重复子件)。

## 组合数预审（HARD GATE）

- Slot A 机构 = 5 候选(side-hung / awning / single-hung / fixed-picture / hopper)
- Slot B 轮廓 = 4 候选(rectangular / arched-top / round-porthole / segmental-arch)
- Slot C muntin-grid 多重度 distinct N = 4（4 / 6 / 9 / 15）
- Slot D sash/unit 多重度 distinct N = 5（1 / 2 / 3 / 4 / 6）
- **组合数 = 5 × 4 × 4 × 5 = 400 ≥ 10 ✓**（即便仅 A×B = 20 ≥ 10 已单独过闸）。

## Slot 候选覆盖

### Slot A:opening_mechanism（主机构槽 — 开启方式 / sash 类型）
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| side_hung_casement（基线） | rec_side-hung-...4f1abc4d (+twin/double/arched/bank) | sash_frame / hinge_{i} / stay_bar(REVOLUTE z) | 侧铰立轴外开 | converged(parent) |
| awning_top_hung | rec_aluminium-awning-...0b523c52 (+steel...3dabb144) | sash_ring / handle_lever(REVOLUTE x,底缘外踢 +Y) | 顶铰横轴,下缘外开 | converged(parent) |
| single_hung_vertical | rec_single-hung-...d91f35ac | frame_shell / lower_sash_cam_lever(REVOLUTE) | 上扇固定下扇竖向升降 | converged(parent) |
| fixed_picture_light | rec_window_var_fixed_picture | 主扇冻结为固定 picture light + 单个 top-rail trickle vent flap(REVOLUTE x) | 大固定光 + 1 小通风翻板 | converged(forked ← sidehung) |
| hopper_bottom_hung | rec_window_var_hopper | sash_ring 底铰(REVOLUTE x,顶缘内倾) | 底铰横轴,顶缘内开 | converged(forked ← awningalu) |

### Slot B:frame_outline（外框/扇轮廓形状）
| 候选 | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| rectangular（基线） | rec_side-hung-...4f1abc4d (+多数 parent) | frame_shell / sash_frame | 直角矩形开口 | converged(parent) |
| arched_top（拱顶 + fanlight） | rec_arched-top-...6cbe185b | wood_frame / fanlight / stone_arch | 圆拱顶 + 上方 radial fanlight | converged(parent) |
| round_porthole | rec_round-porthole-...30a4c378 | frame_ring / sash_ring(lathe `_ring_shape`) | 整圆环框 | converged(parent) |
| segmental_arch | rec_window_var_outline_segmental_arch | 弧头 frame_shell + 弧顶 sash(lathe/CadQuery arc) | 平底圆顶 segmental 开口 | converged(forked ← sidehung) |

### Slot C:divided_light_grid（muntin 分格多重度 — N panes）
| 候选(N) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| radial_fan_7 + colonial_3x5=15（基线） | rec_arched-top-...6cbe185b | muntin loops `for i in range(FAN_RADIAL_COUNT)` + col/row | 扇形 7 + 殖民 3×5 网格 | converged(parent) |
| bank_grid_6 (3×2) | rec_bank-...478f6f0c | `fixed_glass_{col}_{row}` nested loop | 3×2 = 6 格 | converged(parent) |
| colonial_2x2_4 | rec_window_var_muntin_grid_2x2 | muntin_bar_{i} 竖/横 loop + 共享 helper(4 玻璃格) | 2×2 = 4 格 | converged(forked ← sidehung) |
| colonial_3x3_9 | rec_window_var_muntin_grid_3x3 | muntin_bar_{i} 竖/横 loop + 共享 helper(9 玻璃格) | 3×3 = 9 格 | converged(forked ← sidehung) |

### Slot D:sash_unit_count（sash / 单元数多重度）
| 候选(N) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| single_1 | rec_side-hung-...4f1abc4d (+awning/porthole/awningsteel) | sash_frame | 1 扇 | converged(parent) |
| twin/double_2 | rec_twin-...16e9934d / rec_double-...039f0c0a | `_add_sash(sign/mirror_x)` ×2 | 2 扇 | converged(parent) |
| triple_mullion_3 | rec_window_var_triple_mullion | sash_{i} `for i in range(3)` + mullion + hinge loop | 3 扇 + 2 中梃 | converged(forked ← doublecase) |
| bank_2x2_4 | rec_window_var_bank_2x2 | nested col/row loop(N_COLS=2) | 2×2 = 4 单元 | converged(forked ← bank) |
| bank_3x2_6 | rec_bank-...478f6f0c | nested col/row loop(N_COLS=3) | 3×2 = 6 单元 | converged(parent) |

## Multiplicity / Copy Logic

- **两条多重度轴**:
  - count_param C `divided_light_grid (cols × rows)` — N 样本已覆盖 distinct {4, 6, 9, 15} → muntin_grid_2x2 / bank(parent) / muntin_grid_3x3 / arched(parent)。模板建议 N_range cols×rows ∈ [1,6]×[1,6](采样域远大于样本正常)。copied object = muntin 条(竖 loop + 横 loop)+ 对应玻璃格;naming `muntin_bar_{i}` / `lite_{i}`;placement = 内框均分规则网格;joint policy = muntin 与玻璃随所属 sash(单扇随 sash 的 REVOLUTE,无独立 joint)。
  - count_param D `sash_unit_count (N_COLS × N_ROWS 或线性 N)` — N 样本已覆盖 distinct {1, 2, 3, 4, 6} → single(parent) / twin·double(parent) / triple_mullion / bank_2x2 / bank(parent)。模板建议 N_range [1, 12]。copied object = sash 扇 + 其框/玻璃/铰/把手;naming `sash_{i}` / `{name}_hinge_{i}` / `fixed_glass_{col}_{row}`;placement = 沿框宽/网格规则偏移,以 mullion 分隔;joint policy = 每扇统一 side-hung REVOLUTE(bank 中仅 1 扇可开其余固定 lite,统一策略 = 一致铰接 + 余者 FIXED-as-visual 固定玻璃)。
- 机构(A)与轮廓(B)为固定 named slot,非复制轴。

## 格子覆盖（parent 基线计入)

- Slot A:3 parent 格(side-hung / awning / single-hung)converged;2 空格 → fixed_picture / hopper converged。
- Slot B:3 parent 格(rectangular / arched / round)converged;1 空格 → segmental_arch converged。
- Slot C:2 parent N(15+7 / 6)converged;2 空格 → 4 / 9 converged。
- Slot D:3 parent N(1 / 2 / 6)converged;2 空格 → 3 / 4 converged。
- 共 **7 个 NEW 变体**填满规划空格(≤10 cap,按空格数推导无 padding)。每槽 ≥2 候选、两条多重度轴各 ≥3 distinct N。

## 排除项（未来 compatibility matrix 素材)

- **sliding sash 已属另一小类**:本小类不把滑动 sash 作 identity,Slot A 不含 sliding(避免出类目)。
- single-hung / awningalu / awningsteel / porthole 母体**无 for-loop 或手写重复子件**:任何从它们派生的 *multiplicity* 变体必须先重写为 nested range loop;本批已避开(multiplicity 变体仅 fork 自 sidehung/doublecase/bank 这三个已 loop-emission 母体)。
- 颜色/材质/纯尺寸**不作轴**(SPEC §2)。
- 候选潜在不收敛风险待 fork 后回填:fixed_picture 的 trickle-vent 翻板若 anchor 在不可见框顶薄面上会触发 joint-origin 漂浮;segmental_arch 弧头若用 boxy 近似将违反 lathe/arc 几何要求 — 两者需在 run_tests 内分别断言"主光固定 + 小 vent REVOLUTE"与"弧头非矩形顶 rail"。
