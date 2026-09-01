# Source Map — Electrical_Wiring / Circuit breaker

一 slug（`circuit_breaker`）：两 origin 共享抽象脊柱 `housing(fixed) → [housing_to_toggle revolute] →
toggle_body`，per-pole rotor_drum 挂在共享 pivot_shaft 上；pole 特征沿 x 位置元组复制。DIN vs 表面安装是
mount 轴，MCB flag toggle vs MCCB 宽手柄是 handle 轴，均非拆分点。

## Origins（全量对账，2/2 上格）

| id | pic | 建成形态 | 网格角色 | 承载 fork |
|---|---|---|---|---|
| A `rec_use-the-attached-reference-image-as-the-primary-_20260626_083623_497039_eeb289c4` | 001 | 3P 白 MCB（CHINT NB1-63H），蓝 flag 拨杆共 tie-bar，螺纹端子上下，触点指示窗，DIN 卡 | ③white_MCB / N=3 / handle=flag / term=screw / front=indicator | 4pole · mccb_rotary_handle · rcbo_test_button |
| B `rec_use-the-attached-reference-image-as-the-primary-_20260625_164631_228695_7bb9b32f` | 002 | 2P 黑（Eaton/CH），灰 thumb 拨片，圆角 extrude 壳，带红黑引线螺纹端子，DIN 卡 | ③black_MCB / N=2 / handle=rocker / term=screw+leads / front=plain | 1pole · plugin_stab_terminals · surface_mount_base |

## Slots

- **A ① pole_count N（multiplicity）**：1P(fork@B) / 2P(B) / 3P(A) / 4P(fork@A)——模板可外推更高 N
- **B ② handle_form**：flag_toggle(A) / thumb_rocker(B) / mccb_wide_handle(fork@A) —— RCBO test-button 归前面板轴
- **C ③ terminal_type**：screw_cavity(A) / screw+wire_leads(B) / plugin_stab_tab(fork@B) —— 模板可外推 comb-busbar
- **D front_feature**：indicator_window(A) / plain_label(B) / rcbo_test_button(fork@A) —— 三态覆盖 MCB/RCBO
- **E mount**：din_clip(A,B 两种卡型) / surface_screw_base(fork@B)

## Multiplicity / Copy Logic

- pole N ∈ {1,2(B),3(A),4}——沿 pole_x 元组 loop 发射（A 侧 blue_toggle_paddle 原为复制粘贴，fork 时须改 loop）
- 端子行 top/bottom = 2 行 loop；per-pole 端子/螺丝/print 均 loop
- tie_bar / pivot_shaft 长度随 N 求解

## Forks（6，全部 EXIT=0 + compile success + ≥1 非fixed joint + workbench-only + run_tests 断言主轴 + 轴↔图绑定已核对）

1pole@B(N=1) · 4pole@A(N=4) · mccb_rotary_handle@A · plugin_stab_terminals@B ·
rcbo_test_button@A · surface_mount_base@B

## 排除项

- 无 origin 排除（2/2 上格）
- 纯颜色（白/黑/其它 DIN 色）= palette_style 采样，不 fork
