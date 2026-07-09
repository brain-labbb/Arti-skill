# Source Map — Electrical_Wiring / Cable reel

一 slug（`cable_reel`）：核心脊柱 = frame(fixed) → [frame_to_reel revolute/continuous, X 轴] → reel（圆
drum_core + 两 flange 盘 + 缠绕电缆），可选 crank + 自旋 grip。frame 形态是 ③ 主轴。

## Origins（全量对账，2/2 上格）

| id | pic | 建成形态 | 网格角色 | 承载 fork |
|---|---|---|---|---|
| A `rec_use-the-attached-reference-image-as-the-primary-_20260625_164631_234180_ea686860` | 002 | 米色实心侧板落地卷盘，skid 底+脚，实心 cheek 盘，橡胶缆单螺旋缠绕，偏置手摇+自旋握把（双关节） | ③solid_stand / flange=solid / drive=crank+free_grip / base=skid | closed_housing · spoked_flange · wheeled_cart |
| B `rec_use-the-attached-reference-image-as-the-primary-_20260625_164631_232518_83201f3e` | 001 | 橙管开笼钢架卷盘，三角减重开孔侧 cheek+顶提手，tie-rod 笼，固定轴，缠绕 31 圈，盒摇柄+旋钮（无握把关节） | ③open_cage / flange=solid / drive=crank+fixed_knob / base=skid | wall_bracket · motorized_drive |

## Slots

- **A ①③ frame_form（主轴）**：open_cage(B) / solid_stand(A) / closed_housing(fork@A，封闭 drum 罩) —— 模板可外推
- **B ② flange_form**：solid_disc(A,B 双源) / spoked_disc(fork@A，N 辐条) —— 模板可外推 open_rod_cage
- **C ④ drive_type**：hand_crank_free_grip(A) / hand_crank_fixed_knob(B) / motorized(fork@B，马达+齿箱) —— 可外推 t_handle/free_spool
- **D ⑤ mount_base**：skid_u_base(A,B 双源) / wall_bracket(fork@B) / wheeled_cart(fork@A)
- **N**：cage tie-rod / flange 辐条 / hub rib 数——整数 multiplicity，叠在 A/B 上

## 交叉矩阵（frame × drive；接口=X 轴自旋关节，多数外推）

| frame × drive | crank_grip | crank_knob | motor |
|---|---|---|---|
| solid_stand | 源A | 外推 | 外推 |
| open_cage | 外推 | 源B | fork motorized_drive@B |
| closed_housing | fork closed_housing@A | 外推 | 外推(常配马达) |

## Multiplicity / Copy Logic

- drum_core/flange cheek/hub 全用圆 annular mesh + torus（不许 Box 盘）
- 缠绕电缆：A=单参数螺旋 spline tube（优）；B=31 个 torus 逐圈（相邻重叠读连续）——模板取单螺旋更佳
- rail/foot/upright/side plate/tie-rod/rivet/flange bolt/vent 均 loop 发射
- 注：B `_top_handle_shell` 死代码；A `axle_shaft`/`rear_axle_stub` 近重复 stub

## Forks（5，全部 EXIT=0 + compile success + ≥1 非fixed joint + workbench-only + 绑定已核对）
> 注：spoked_flange 几何已实现（Spoked/spoke/cutout/rim/cheek 件齐全、compile+2 关节 ok），但其 run_tests 断言的是继承的曲柄/轮毂机构而非新辐条盘——几何已独立核实，留目检把关。

closed_housing@A(③) · wall_bracket@B(⑤) · spoked_flange@A(②) · motorized_drive@B(④) · wheeled_cart@A(⑤)

## 排除项

- 无 origin 排除（2/2 上格）
- 颜色（米/橙/其它）= ⑥ palette，不 fork
