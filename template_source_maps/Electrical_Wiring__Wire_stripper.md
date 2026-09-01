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
- **B handle_form**：straight_plier(A) / angled_offset(B) / pistol_grip(fork@B) · **外推** offset_kick_plier(A) / inline_slim(B)
- **C jaw_feature**：notch+cutter(A) / clamp+screw(B) / crimp_die(fork@A，加压接模站) / gauge_hole_plate(fork@A) ·
  **外推** notch_bolt_shear(A) / gauge_slot_plate(gauge) / clamp_vgroove_die(B)
- **D ② auto_clamp_dof**：abstracted(B 现状，无独立夹持 DOF) / real_clamp_joint(fork@B，加真实第二夹持关节)
- **E gauge_station N**：3 / 4(A) / 6(fork@A)——模板外推 5/7/8（保持升序半径不变量）；
  2026-08-01 起 auto 族也有真实 N（3–6 档剥线模口），源池 B 只有光板夹口，属外推

## 外推候选（无独立源 record，2026-08-01）

正式流程要求新结构候选先成为可审阅源资产。以下 5 个候选**没有**对应 record，是按类目实物形态外推的，
在此登记以便后续补源或回退。它们全部通过 random-16/36 + corner，并各自带作者测试：

| candidate | 槽位 | 外推依据 | 与既有候选的结构差 |
|---|---|---|---|
| `offset_kick_plier` | handle(A) | 电工钳常见的外折手柄 | 手柄根部起加外折 ramp，钢柄+包胶+配色分界同步折 |
| `inline_slim` | handle(B) | 紧凑型自动剥线钳 | 扇开后近乎平行的细长柄，鼻端更长 |
| `notch_bolt_shear` | jaw(manual) | Klein/Ideal 类螺栓剪断station | pivot 侧 3 个升序螺纹剪孔 + 凸起 shear rail，notch 站外移到 x0=28 |
| `gauge_slot_plate` | jaw(gauge) | 开口槽式量规板 | 量规孔从工作边开喉口（keyhole），线可侧向放入而非穿孔 |
| `clamp_vgroove_die` | jaw(auto) | 可换模盒式自调剥线钳 | 模口由 V 形改方肩阶梯，模盒凸出并由 2 颗盖头螺钉固定 |

## 2026-08-01 修复记录（源保真回归）

- B 族包胶手柄原先用独立 rake 常量摆放，相对钢柄中线外偏 8.3mm（pistol 12.6mm）、角度差 5.4°，
  每条手柄读成"裸钢板 + 悬空胶柄"两根。origin B 本身只偏 4.1mm，属移植放大。现改为钢柄与包胶
  共用 `_B_HANDLE_SPINES` 脊线；原来的外偏同时承担了两柄净空，改回中线后由头后"扇开"段提供净空。
- B 族头部两片半板中间留 2mm 全长通缝（可透视）。现在窗口后两半跨中线互搭，只保留 jaw 窗口开口。
- pivot chrome 钮 Φ27 > 钢板宽 Φ21，改为 Φ22（hub 仍 Φ33）。
- gauge 孔半径 0.34–1.28mm（裸铜导体尺寸）在整机渲染上是不可见针孔，改为 0.85–2.55mm（过绝缘线径）。
- B 族 jaw 原为两个光板 Box + 平垫，无任何剥线口。现在 jaw pad 带 N 档升序模口，N 进入 slot_choices。

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
