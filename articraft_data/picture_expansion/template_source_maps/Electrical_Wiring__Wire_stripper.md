# Source Map — Electrical_Wiring / Wire stripper

一 slug（`wire_stripper`）：核心脊柱 = 双臂在 pivot 绕 revolute 挤压 + jaw 头。两 origin 属**不同 ③
机构/头家族**（手动多档 notch 钳 vs 自动自调），joint 拓扑不同（后者多一根 prismatic 长度挡）——③ 是主轴。

## Origins（全量对账，2/2 上格）

| id | pic | 建成形态 | 网格角色 | 承载 fork |
|---|---|---|---|---|
| A `rec_a-yellow-and-black-multifunction-electrician-wir_20260705_045817_224605_a2129614` | 001 | 黄黑多功能电工钳，CadQuery jaw+notch/crimp 布尔切，鼻部锯齿刃，弹簧，thumb 锁；1 revolute pivot(+latch) | ③manual_notch_plier / handle=straight / jaw=notch+cutter / N=4 | fixed_hole_gauge · crimp_die_station · gauge_n6 |
| B `rec_an-automatic-self-adjusting-wire-stripper-with-r_20260705_045103_622531_e60caa10` | 002 | 自动自调剥线钳，红黑交叉板，夹/剥 jaw 块，长度挡滑块+回弹，铜张力螺丝；revolute pivot + prismatic 挡 | ③auto_selfadjust / handle=angled / jaw=clamp+screw | pistol_grip · auto_clamp_jaw |

## Slots

- **A ③ mechanism_family（主轴）**：manual_notch_plier(A) / auto_selfadjust(B) / fixed_hole_gauge(fork@A，钻孔量规板)——三大形态
- **B handle_form**：straight_plier(A) / angled_offset(B) / pistol_grip(fork@B)
- **C jaw_feature**：notch+cutter(A) / clamp+screw(B) / crimp_die(fork@A，加压接模站) —— 模板可外推 gauge_hole_plate
- **D ② auto_clamp_dof**：abstracted(B 现状，无独立夹持 DOF) / real_clamp_joint(fork@B，加真实第二夹持关节)
- **E gauge_station N**：3 / 4(A) / 6(fork@A)——模板可外推 5/7/8（保持升序半径不变量）

## Multiplicity / Copy Logic

- NOTCH_SPECS N=4 → loop `_cut_edge_circles` 双刃配对布尔；CRIMP_SPECS N=2；鼻齿 ×8；latch rib ×3 均 loop
- B: shank 齿 ×5 loop；head_rivet_0/1 复制粘贴（fork 时改 loop）
- 手柄 grip 用 spline+fillet（不许 Box）；notch 为真实体积去除布尔（test 已断言）

## Forks（5，全部 EXIT=0 + compile success + ≥1 非fixed joint（auto_clamp_jaw=+1 clamp DOF）+ workbench-only + run_tests 断言主轴 + 绑定已核对）

fixed_hole_gauge@A(③) · crimp_die_station@A(jaw) · pistol_grip@B(handle) ·
gauge_n6@A(N=6) · auto_clamp_jaw@B(②加夹持DOF)

## 排除项

- 无 origin 排除（2/2 上格）
- 颜色（黄黑/红黑）= ⑥ palette，不 fork；纯尺寸 = ⑤，模板参数化
- 热剥/旋转剥线等 exotic 形态 = gate（不入类目主流）
