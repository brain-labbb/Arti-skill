# Door / Door — template source map

pattern: mixed（固定 named slots:leaf_style(门叶面结构范式槽) + door_hardware(开合/操作五金槽) **外加** louver_panel_count 一根多重性轴。每扇门核心拓扑都是 frame/casing(root) → leaf(REVOLUTE 竖铰) 的 linear_chain;round_knob / lever / fluted/oak 门会再挂一根 leaf→hardware REVOLUTE,bar pull 与 ornate pull 走 inline 视觉或 FIXED）

parents（6 个母资产,覆盖 leaf_style × hardware 网格的多个格子）:
- P1 graphite rec_single-hinged-door-with-a-matte-dark-graphite-fi_20260606_115708_888790_d9a8adeb ← picture/Door/Door/002.png（flush graphite slab + 竖向窄玻璃 vision strip;lever_handle on rose + door_lock(FIXED) keyhole;覆盖 A=glazed_vision_strip, B=lever_on_rose）
- P2 louver rec_single-louvered-utility-door-gray-painted-steel-_20260606_115203_244494_72379bfd ← 006.png（钢制工具门:raised perimeter frame + upper_louver_panel + lower_louver_panel(VentGrille);door_knob + knob_rose + latch_plate;覆盖 A=louvered_vented(N=2), B=round_knob）
- P3 brass rec_reflective-glass-door-with-ornate-curved-brass-p_20260606_115152_968871_61324bd6 ← 005.png（双扇镜面玻璃 French door + center_mullion;ornate 波浪 S 曲线黄铜拉手 *_leaf_handle_bar + 两 standoff;覆盖 A=full_glass(double), B=ornate_s_pull）
- P4 frameless rec_frameless-look-glass-door-a-large-clear-tempered_20260606_115151_982675_03743a3b ← 004.png（无框清玻 glass_leaf + 固定 side_panel;细暗 edge_frame;直筒不锈钢 leaf_handle/_bar_handle + standoff;覆盖 A=full_glass(single+side), B=straight_bar_pull）
- P5 fluted rec_modern-dark-entry-door-with-a-tall-vertical-flut_20260606_115137_754364_286d9579 ← 003.png（现代暗色门 + 固定 fluted sidelight_glass(flute 循环) + leaf_glass_strip;长竖向 bar_handle;**叶面 leaf_panel 目前是 plain slab**;覆盖 A=fluted_entry_sidelight, B=straight_bar_pull）
- P6 oak rec_single-hinged-interior-door-light-oak-wood-one-l_20260606_115124_332093_00107e3e ← 001.png（浅橡木内门;leaf_body = routed groove + 1 raised flat 中央板;lever_handle on rose;覆盖 A=solid_raised_panel, B=lever_on_rose）

## 组合数预审（HARD GATE P1）

Slot A leaf_style 候选值 = 5（solid_raised_panel / glazed_vision_strip / louvered_vented / full_glass / fluted_entry_sidelight）
Slot B door_hardware 候选值 = 4（lever_on_rose / round_knob / straight_bar_pull / ornate_s_pull）
Multiplicity louver_panel_count N 样本 = {1, 2, 3}（3 个 distinct N）

Π = leaf_style(5) × door_hardware(4) × N(3) = **60 ≥ 10 → PASS**。即使只取 leaf_style(5) × door_hardware(4) = 20 也已 ≥ 10。
是否已被 existing+parents 单独满足 GATE P1:**是**——每个轴均 ≥2 候选值,multiplicity 覆盖 3 个 distinct N,组合数 60。新增 2 个 fork 仅为把两个 single-candidate 候选值(fluted_entry_sidelight、ornate_s_pull)补成 ≥2 收敛记录,使其成为可成模的 module 而非孤例(规则 4:优先填补单候选槽)。

## Slot 候选覆盖

### Slot A:leaf_style（**主外观槽**——门叶面结构范式）
| 候选(未来 module) | record_id | 关键 part/joint/helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| solid_raised_panel | P6 (parent) | door_leaf/leaf_body;helper _build_leaf_cq（routed groove + 1 raised panel）| 实木 slab + 槽线边框 + 1 块大 raised 中央板 | converged-parent |
| solid_raised_panel | rec_door_var_solid_knob | door_leaf/leaf_body;door_knob/leaf_to_knob;helper _build_leaf_cq + _build_knob_cq | 同 raised-panel 叶 + 圆 domed knob | converged |
| solid_raised_panel | rec_door_var_solid_barpull | door_leaf/leaf_body + bar_tube + standoff_i(循环);helper _build_leaf_cq/_build_bar_tube_cq/_build_standoff_cq | 同 raised-panel 叶 + 长直筒 bar pull(inline) | converged |
| glazed_vision_strip | P1 (parent) | door_leaf/leaf_body + vision_pane;helper _build_leaf_cq（vision opening 通切）| graphite flush slab + 竖窄玻璃通切 | converged-parent |
| glazed_vision_strip | rec_door_var_glazed_knob | leaf_body + vision_pane;door_knob/leaf_to_knob(REVOLUTE +Y);helper _build_knob_cq | 同 glazed 叶 + 圆 domed knob 转动 | converged |
| glazed_vision_strip | rec_door_var_glazed_barpull | leaf_body + vision_pane + bar_tube + standoff_i(循环);leaf_to_pull(FIXED)| 同 glazed 叶 + 长直筒 bar pull | converged |
| glazed_vision_strip | rec_door_var_glazed_louvered_split | leaf_body + glass_pane + lower_louver_panel + glazing_bead_i(循环);helper _louver_panel | 上半 glazed vision + 下半单块 louver(split 叶,N_louver=1) | converged |
| louvered_vented | P2 (parent) | leaf/upper_louver_panel + lower_louver_panel;helper _louver_panel(VentGrilleGeometry) | 钢叶 + 上下两块横百叶(N=2) | converged-parent |
| louvered_vented | rec_door_var_solid_louvered | door_leaf/leaf_body + louver_panel;helper _build_leaf_cq + _build_louver_panel_mesh | 橡木叶满铺 1 块 louver field(N=1) | converged |
| louvered_vented | rec_door_var_louvered_n1 | leaf/louver_panel_i(循环, LOUVER_PANEL_COUNT=1);helper _louver_panel | 钢叶单块满高 louver(N=1) | converged |
| louvered_vented | rec_door_var_louvered_n3 | leaf/louver_panel_i + inter_rail_j(循环, N_PANELS=3);helper _louver_panel | 钢叶三块等高 louver + 两 inter rail(N=3) | converged |
| louvered_vented | rec_door_var_louvered_lever | leaf/upper_louver_panel + lower_louver_panel;lever_handle/leaf_to_lever(REVOLUTE)| 钢百叶叶(N=2) + lever 把手(替 knob) | converged |
| full_glass | P3 (parent) | right_leaf/left_leaf + *_leaf_glass + center_mullion;helper _build_leaf/_add_glass_trim | 双扇镜面玻璃 French door | converged-parent |
| full_glass | P4 (parent) | glass_leaf/leaf_glass + side_panel + edge_frame;helper _glass_pane/_edge_frame | 单扇无框清玻 + 固定侧板 | converged-parent |
| fluted_entry_sidelight | P5 (parent) | door_leaf/leaf_panel + leaf_glass_strip;frame/sidelight_glass(flute 循环);helper _frame_with_sidelight/_sidelight_glass | 暗色门 + fluted 玻璃侧灯（**叶面是 plain slab**)| converged-parent |
| fluted_entry_sidelight | rec_door_var_fluted_sidelight_bar | door_leaf/leaf_panel + flute_i(**新循环**) + leaf_glass_strip + bar_handle;shared rib helper | **修正 P5 plain slab → 真竖向 fluted 叶面**,满铺竖棱(其余层不变)| converged |

### Slot B:door_hardware（操作/开合五金机构）
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| lever_on_rose | P1 / P6 (parents) | lever_handle/leaf_to_lever(REVOLUTE +Y) + door_lock(FIXED, 仅 P1) | 压杆把手绕 spindle 转 | converged-parent |
| lever_on_rose | rec_door_var_solid_louvered | lever_body/leaf_to_lever(REVOLUTE);helper _build_handle_cq | lever-on-rose（在 louver 叶上）| converged |
| lever_on_rose | rec_door_var_louvered_lever | lever_handle/leaf_to_lever(REVOLUTE);Cylinder rose | lever-on-rose（在钢百叶叶上,替 P2 knob）| converged |
| round_knob | P2 (parent) | door_knob + knob_rose + latch_plate;KnobGeometry(leaf inline 视觉)| 圆球 knob + latch plate | converged-parent |
| round_knob | rec_door_var_solid_knob | door_knob/leaf_to_knob(REVOLUTE +Y);helper _build_knob_cq(sphere dome) | 圆 domed knob 绕 spindle 转 | converged |
| round_knob | rec_door_var_glazed_knob | door_knob/leaf_to_knob(REVOLUTE +Y);helper _build_knob_cq | 圆 domed knob（graphite glazed 叶上)| converged |
| round_knob | rec_door_var_louvered_n1 / _n3 | door_knob + knob_rose;KnobGeometry inline | 圆 knob + latch plate（钢百叶叶上)| converged |
| round_knob | rec_door_var_glazed_louvered_split | door_knob + knob_rose;KnobGeometry inline | 圆 knob（split 叶上)| converged |
| straight_bar_pull | P4 / P5 (parents) | leaf_handle/_bar_handle;helper _bar_handle/_build_handle（standoff）| 长直筒拉手 + standoff | converged-parent |
| straight_bar_pull | rec_door_var_solid_barpull | bar_tube + standoff_i(循环);helper _build_bar_tube_cq/_build_standoff_cq | 长直筒 bar pull(inline, leaf_to_pull 隐式)| converged |
| straight_bar_pull | rec_door_var_glazed_barpull | bar_tube + standoff_i(循环);leaf_to_pull(FIXED)| 长直筒 bar pull（graphite glazed 叶上)| converged |
| ornate_s_pull | P3 (parent) | *_leaf_handle_bar + *_standoff_top/_bottom;helper _s_handle_points + tube_from_spline_points | 波浪 S 曲线黄铜拉手(2 standoff)| converged-parent |
| ornate_s_pull | rec_door_var_ornate_s_pull | *_leaf_handle_bar + standoff_i(**新循环, n=3**);helper _s_handle_points + shared standoff helper | **同 S 曲线黄铜拉手,2→3 standoff 多重 + 更明显多瓣 S 波**(双玻璃叶其余层不变)| converged |

## Multiplicity / Copy Logic
- **count_param: `louver_panel_count`**（门叶内横百叶 vented 板的堆叠块数）
  - N 样本已覆盖: {1, 2, 3} → rec_door_var_louvered_n1 / rec_door_var_solid_louvered（N=1)、P2 / rec_door_var_louvered_lever（N=2)、rec_door_var_louvered_n3（N=3）
  - 模板建议 N_range: [1, 6]（采样域可大于样本覆盖值;过大 N 板会太薄,实门极少 >5）
  - copied object: 一块横百叶 VentGrille 板（由 shared helper _louver_panel / _build_louver_panel_mesh 生成）
  - naming: `louver_panel_i`；placement: 沿 leaf 竖轴(Z)等距,块间以 inter_rail_j(horizontal) 分隔;joint policy: 百叶板为 leaf 的 inline 视觉(非 jointed 装饰),门叶整体走 frame/casing_to_leaf REVOLUTE 唯一摆动
  - **loop-emission 契约**:louvered_n1 / louvered_n3 已用 `for i in range(N_PANELS)` + `louver_panel_i` + inter_rail_j 循环 + shared _louver_panel helper;P2 与 solid_louvered 的 1~2 块为手写,做正式模板时按 louvered_n3 范式统一为循环。
- 次级重复轴(非 count_param):bar pull 的 standoff(`standoff_i`)、glazed_louvered_split 的 glazing_bead_i 已是循环;新 ornate_s_pull fork 的 standoff 亦强制 `for i in range(n)` 循环。

## 格子覆盖
| 槽 | 候选值数 | 每候选值收敛记录数(existing+parent) | 新填(已 fork 收敛) |
|---|---|---|---|
| A leaf_style | 5 | solid_raised_panel 3 / glazed_vision_strip 4 / louvered_vented 5 / full_glass 2 / fluted_entry_sidelight 1→2 | rec_door_var_fluted_sidelight_bar |
| B door_hardware | 4 | lever_on_rose 4 / round_knob 6 / straight_bar_pull 4 / ornate_s_pull 1→2 | rec_door_var_ornate_s_pull |
| multiplicity louver_panel_count | N∈{1,2,3} | N=1:2 / N=2:2 / N=3:1 | 无（3 distinct N 已满足）|

GATE P1:每轴 ≥2 候选值且每候选值补到 ≥2 收敛记录(fork 后);multiplicity 覆盖 3 个 distinct N;组合数 60 ≥ 10。**两个 fork 均为单候选值补强,一格一变体,不重复任何已填格,各槽家族总数 ≤ ~12。Door 小类样本池就绪。**

## 排除项（未来 compatibility matrix 素材）
- 暂无连续不收敛记录(本批为 gap-fill,仅补 2 个单候选值,无失败)。
- full_glass 与 fluted_entry_sidelight 含 fixed side_panel / sidelight(双扇或带侧灯),与单扇 solid/glazed/louvered 叶的 frame→leaf linear_chain 拓扑不完全正交:做 compatibility matrix 时 full_glass × round_knob / lever 等格子按"玻璃叶用 pull/standoff,不混 knob/lever"处理,避免不真实组合。
- push_plate / 商用平推板暂未覆盖(无任何收敛记录),如未来需扩 hardware 轴可单独 fork;本批不强行造,以免引入新单候选值。
- color/material 不作为轴(纯换色/换材不计变更)。
- N>5 的 louver 多重性下板厚趋薄,采样应下调权重(参照 template_test_vs_product_N_domain 经验)。
