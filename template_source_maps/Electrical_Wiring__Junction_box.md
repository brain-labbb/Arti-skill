# Source Map — Electrical_Wiring / Junction box

一 slug（`junction_box`）：三 origin 共享抽象脊柱 `enclosure(fixed) → [cover_hinge revolute] → cover`，
cable gland 沿侧壁 loop 复制，内部 terminal_strip。房体透明/黑/灰 = ⑥ 材质（palette 采样），非拆分点；
盖机构（铰链 vs 螺丝掀盖）是 ② joint 轴。

## Origins（全量对账，3/3 上格）

| id | pic | 建成形态 | 网格角色 | 承载 fork |
|---|---|---|---|---|
| S1 `rec_use-the-attached-reference-image-as-the-primary-_20260626_091402_790970_518a36b7` | 002 | 黑 IP68，6 gland 四面，铰链盖(默认开)，terminal_strip 独立 part，安装耳 | 材质=black / N=6 / lid=hinge / interior=terminals | screw_lid |
| S2 `rec_use-the-attached-reference-image-as-the-primary-_20260626_091402_791782_058eea1f` | 003 | 灰 opaque，4 侧 gland+长导管，铰链服务盖，terminals 内联，齐平无耳 | 材质=gray / N=4 / lid=hinge / interior=terminals | empty_passthrough |
| S3 `rec_use-the-attached-reference-image-as-the-primary-_20260626_091402_793801_d3744a0e` | 001 | 透明 PC，3 gland(2前1后)，KnobGeometry 滚花 gland 螺母(最保真)，铰链盖，安装耳 | 材质=clear / N=3 / lid=hinge / footprint=rect | 2way · round_box |

## Slots

- **A gland_count N（multiplicity）**：2-way(fork@S3) / 3-way(S3) / 4-way(S2) / 6-way(S1)——模板可外推更高
- **B ② lid_mechanism**：hinged_flip(S1,S2,S3 三源) / screw_liftoff_cover(fork@S1，prismatic 掀盖)
- **C interior_fitout**：terminal_strip(S1,S2,S3) / empty_passthrough(fork@S2)
- **D ③ footprint_envelope**：rectangular(全源) / round_conduit_box(fork@S3)
- **⑥ housing 透明度**：clear(S3) / black(S1) / gray(S2)——三态全源，palette 采样

## 交叉矩阵（gland_N × lid；接口=已验证 gland loop + hinge，多数外推）

| N × lid | hinged | screw_liftoff |
|---|---|---|
| 2-way | fork 2way@S3 | 外推 |
| 3-way | 源S3 | 外推 |
| 4-way | 源S2 | 外推 |
| 6-way | 源S1 | fork screw_lid@S1 |

## Multiplicity / Copy Logic

- gland N ∈ {2,3(S3),4(S2),6(S1)}——**须改为 gland_specs list 的 loop**（三源均把 _add_gland 逐个复制粘贴，
  fork 时收敛为一根 N 变量）
- corner screw_boss(4) / mount_ear(4) / terminal socket(3~8) / thread ring(3~4) 均 loop
- footprint L/W 由常量驱动；round_box 改 envelope 为 Cylinder/Lathe

## Forks（4，全部 EXIT=0 + compile success + ≥1 非fixed joint（screw_lid=prismatic 掀盖）+ workbench-only + run_tests 断言主轴 + 绑定已核对）

2way@S3(N=2) · screw_lid@S1(掀盖 prismatic) · empty_passthrough@S2 · round_box@S3(③圆形)

## 排除项

- 无 origin 排除（3/3 上格）
- 纯 footprint 长宽比（square 等）= ⑤ 比例，模板参数化，不 fork
- 房体颜色 = ⑥ palette，不 fork
