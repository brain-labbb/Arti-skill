# Door / folding door — template source map

slug: folding_door   shard: folding_door
pattern: multiplicity_chain（leaf_count N 个铰接叶片串成 concertina 折叠链；static root frame + per-leaf infill 槽 + 折叠 kinematics 槽 + top-track/suspension 槽）

parents:
- rec_a-4-panel-bi-fold-interior-glass-partition-door-_20260608_160138_925763_9234fbaa ← picture/Door/folding door/003.png（4-panel bi-fold 室内玻璃隔断门 / accordion concertina glass door；helper `_build_leaf_frame_shape` / `_build_glass_shape` / `_build_root_frame_shape` / `_add_leaf` / `_add_handle`；4 REVOLUTE 折叠铰（左对 stack 左 jamb + 右对 stack 右 jamb = center-biparting）；基线 = N:4 × infill:framed_glass_midrail × kinematics:center_biparting_bifold × track:flush_header_pivot；**全批 fork 基线**）

参考图：001.png = 4 叶关门正视（mid-rail framed glass，中央 pull handle）；002.png = 左对折开、右对关；003.png = 左右两对都 concertina 折开（hero 折叠姿态）。

## Loop-emission 现状（给模板作者 — 关键可读性信号）

- parent 已 loop 的：`HINGE_X` 列表推导、每叶 hinge knuckle 列（`_add_leaf` 内 `for i in range(KNUCKLE_COUNT)`）。
- parent **未 loop 的（gotcha）**：四个叶片本体是 **手写** 的四次 `_add_leaf(model, "leaf_0"..)` 调用，四个 articulation 也是 **手写** 的显式 `model.articulation(...)`。**叶片 + 铰链链不是 `for i in range(LEAF_COUNT)` 发射的。**
- 三个 multiplicity 变体（n2 / n6 / n8）**已完成 loop 重写**：叶片装配 + 铰链链改成 `for i in range(LEAF_COUNT)` 单循环 + 共享 `_add_leaf` helper + 统一线性链铰链策略（首叶铰左 jamb，后续叶铰前一叶共享竖边，交替折向 zig-zag）。这三者同时是 **single_direction_stack kinematics** 的样本（单条连续链铰单 jamb，整体 stack 一侧），与 parent 的 center-biparting 双链形成 kinematics 轴的两个候选。模板侧 count_param = leaf_count。

## 组合数预审（HARD GATE — 已由现存变体满足）

distinct N(4: 2/4/6/8) × infill(5) × kinematics(2) × track(3) = **120 ≥ 10** ✓。

**GATE P1 STATUS：已满足 — 零新 fork。** 每个槽位 ≥2 候选；kinematics 的 single_direction_stack 槽并非空缺，已由 n2/n6/n8 三个单链变体覆盖（它们本身就是单向 stack 拓扑）。无需新增 single-direction 4-leaf 专用变体。

## Slot 候选覆盖

### Slot A：leaf_count（MULTIPLICITY N — 折叠叶片数；触发 loop 重写）
| 候选（count_param=leaf_count） | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| N=4（基线） | rec_a-4-panel-bi-fold-interior-glass-partition-door-_20260608_160138_925763_9234fbaa | leaf_0..leaf_3 / left_jamb_to_leaf_0 / leaf_0_to_leaf_1 / right_jamb_to_leaf_3 / leaf_3_to_leaf_2 | 4 叶，4 REVOLUTE，center-biparting（左对+右对，手写未 loop） | parent |
| N=2 | rec_folding_door_var_n2 | leaf_0..leaf_1 / frame_to_leaf_0 / leaf_i_to_leaf_{i+1} | 单 concertina 对，`for i in range(LEAF_COUNT)` 链式铰，single chain | converged |
| N=6 | rec_folding_door_var_n6 | leaf_0..leaf_5 / frame_to_leaf_0 / 链式 | 6 叶紧凑链，loop 发射，single chain | converged |
| N=8 | rec_folding_door_var_n8 | leaf_0..leaf_7 / frame_to_leaf_0 / 链式 | 8 叶连续 accordion 链，loop 发射，single chain | converged |

### Slot B：leaf_infill（叶片填充；每候选一个 infill helper）
| 候选 | record_id | helper / part 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| framed_glass_midrail（基线） | rec_a-4-panel-bi-fold-interior-glass-partition-door-_20260608_160138_925763_9234fbaa | `_build_leaf_frame_shape` + `_build_glass_shape`（`{leaf}_frame` / `{leaf}_glass`） | 双开口 + 横 mid-rail framed glass | parent |
| frameless_full_glass | rec_folding_door_var_frameless_glass | `_build_leaf_frame_shape`（仅周边 edge rail，无 mid-rail）+ 单整片玻璃 | 去 mid-rail，单整片 edge-to-edge 高玻璃，极简周框 | converged |
| solid_panel | rec_folding_door_var_solid_panel | `{leaf}_frame` + 实心 opaque wood panel insert | 实心 flush 木板填入 stile-and-rail 框 | converged |
| louvered_slats | rec_folding_door_var_louvered | 共享 slat geometry helper，`for i in range(slat_count)` 发射水平百叶（`{leaf}_slat_i`，inline 视件） | 叶内水平木百叶填充 | converged |
| muntin_grid_glass | rec_folding_door_var_muntin_grid | `_add_muntin_grid_to_leaf`，`for i in range(1, n_rows)` / `for j in range(1, n_cols)` 发射横竖 muntin 条（`{leaf}_muntin_h_*` / `_v_*`，Box，inline 视件） | 玻璃 + 细玻璃格条分小 lite | converged |

### Slot C：folding_kinematics（折叠链拓扑 — 铰链策略）
| 候选 | record_id | 链结构 | 状态 |
|---|---|---|---|
| center_biparting_bifold（基线） | rec_a-4-panel-bi-fold-interior-glass-partition-door-_20260608_160138_925763_9234fbaa | 左对 stack 左 jamb（leaf_0/leaf_1）+ 右对 stack 右 jamb（leaf_3/leaf_2），双对称链，中央 biparting | parent |
| single_direction_stack | rec_folding_door_var_n2 / rec_folding_door_var_n6 / rec_folding_door_var_n8 | 全叶一条连续链铰单（左）jamb，交替折向，整体 stack 一侧（与 multiplicity 同一组样本） | converged |

### Slot D：top_track / suspension（顶轨/吊挂/导向样式）
| 候选 | record_id | 结构特征 | 状态 |
|---|---|---|---|
| flush_header_pivot（基线） | rec_a-4-panel-bi-fold-interior-glass-partition-door-_20260608_160138_925763_9234fbaa（louvered/solid_panel/muntin_grid/frameless_glass/n2/n6/n8 共用此 track） | flush 连续 header track + 朴素 jamb pivot；header 顶轨 + 地脚轨 | parent |
| perimeter_cased_U_channel | rec_folding_door_var_perimeter_frame | 全周 cased 框：header 底面 U-channel 槽 + threshold 轨顶面 U-channel 槽，叶上下边捕入双槽 + 每叶 hinge 边 pivot guide pins（`{leaf}_pivot_pin_*`） | converged |
| bottom_track_floor_guided | rec_folding_door_var_bottom_track | 厚地轨双导轨承重 + 每叶底边可见 guide roller/pin（`{leaf}_guide_roller`）在地槽内滑；顶仅细导向槽（不承重）；铰原点锚在地轨接触面 | converged |

## Multiplicity / Copy Logic

- count_param: **leaf_count**（Slot A）。
- N 样本已覆盖：{2, 4, 6, 8} → rec_folding_door_var_n2 / parent / rec_folding_door_var_n6 / rec_folding_door_var_n8（distinct N = 4）。
- 模板建议 N_range: [2, 100]（模板采样域远大于样本覆盖是正常的）。
- copied object / naming / placement / joint policy：复制对象 = leaf（`_add_leaf` helper）；命名 = `leaf_{i}`；placement = 沿 X 等宽排布（`HINGE_X[i]`）；joint policy = 线性链 REVOLUTE，首叶铰 jamb，后续叶铰前一叶共享竖边，axis 交替 ±Z 做 zig-zag concertina（single_direction）；center-biparting 时拆成两条对称链各铰一个 jamb。
- **模板必须 loop 化**：parent 的叶片+铰链链是手写的（见上 Loop-emission 现状）；n2/n6/n8 已示范 `for i in range(leaf_count)` 重写，模板照此做。

## 连续尺寸参数（非候选；模板侧缩放，勿当 slot 候选）

OPENING_WIDTH / OPENING_HEIGHT / LEAF_W / FRAME_T（stile 料宽）/ MID_RAIL_FRACTION（横档高度比）/ HANDLE_LEN / glass tint —— 写 spec 时作连续尺寸/外观参数（开口宽高 / 边框料宽 / 横档高度），不要当 Slot 候选。

## 格子覆盖（全 converged；parent 基线计入）

N 3 个非基线格（n2/n6/n8） + infill 4（frameless_glass/solid_panel/louvered/muntin_grid） + kinematics single（由 n2/n6/n8 复用覆盖，无独立 4-leaf 专样） + track 2（perimeter_frame/bottom_track）= **9 个变体已全部收敛**。parent 填基线格。组合空间（120）已铺开，**folding door 小类样本池就绪**。

## 排除项 / 丢弃轴

- **frame_stile_width / 边框料宽 / 横档高度比**：连续尺寸，丢弃为模板缩放参（不当 slot）。
- **color / material / pure-scale / glass tint**：仅外观，规则禁止当 change（可作 infill 候选的免费叠加）。
- single_direction_stack 未单独 fork 4-leaf 专样：判定为冗余 —— 该 kinematics 拓扑已由 n2/n6/n8 三个单链变体充分示范，GATE P1 无需补格。
- （暂无不收敛排除项；全 9 变体 compile/run_tests 收敛。）
