# Source Map — Technology / Security_Camera

一 slug（`security_camera`）：4 origin 共享同一抽象云台脊柱 `rigid_mount → [pan revolute] → carriage
→ [tilt revolute] → camera_head`（pan 轴恒为安装面法向），bullet vs dome 是 ③ housing 轴而非拆分点。

## Origins（全量对账，4/4 上格）

| id | pic | 建成形态 | 网格角色 | 承载 fork |
|---|---|---|---|---|
| S1 `rec_a-silver-bullet-cctv-security-camera-a-short-cyl_20260624_124946_096010_1da3777f` | 002 | 银 bullet，桌面座+立柱轭，yaw+pitch，IR 环 N=20，短檐 | ①desk 锚 / ③bullet / D short_lip / IR N=20 | dual_lens |
| S2 `rec_a-white-bullet-security-camera-a-cylindrical-bod_20260624_125234_670793_03a3be03` | 004 | 白 bullet，壁板+臂，tilt-only 单铰，LED N=4 | ①wall_arm 锚 / ②tilt_only 锚 | wall_arm_pan |
| S3 `rec_gray-cctv-bullet-security-camera-with-a-cylindri_20260624_134448_028125_4e6dbba3` | 001 | 灰 bullet，壁板直装，pan+tilt，长开口遮阳罩（cadquery） | ①wall_direct 锚 / D long_hood 锚 | box_housing, ceiling_bullet |
| S4 `rec_a-white-ptz-speed-dome-security-camera-a-truncat_20260624_125044_036850_1892c604` | 003 | 白 PTZ 速度球，吸顶+转台轭，内部 pan+tilt，IR N=8 | ①ceiling 锚 / ③speed_dome 锚 | turret_eyeball, wall_arm_dome |

## Slots

- **A ① mount_form（4 锚，全源）**：desk_base(S1) / wall_plate_arm(S2) / wall_plate_direct(S3) / ceiling(S4)
- **B ② gimbal_dof**：tilt_only(S2) / full_pan_tilt(S1,S3,S4 + fork wall_arm_pan 补 @wall_arm)
- **C ③ housing_form（4 锚，其中 2 个行发现铸造）**：bullet(S1-3) / speed_dome(S4) / turret_eyeball(fork) / box(fork)
- **D sunshield**：none(S2) / short_lip(S1) / long_open_hood(S3)——三态全源

## 交叉矩阵（①×③ 关键格；接口=已验证云台关节，故多数交叉走外推）

| mount×housing | bullet | dome | turret | box |
|---|---|---|---|---|
| desk | 源S1 | gate(桌面球机不真实) | gate | 外推 |
| wall_arm | 源S2 | fork wall_arm_dome(probe) | 外推 | 外推 |
| wall_direct | 源S3 | gate | 外推 | 外推 |
| ceiling | fork ceiling_bullet(probe) | 源S4 | 外推(turret 原生吸顶) | gate |

其他 gates：dome/turret 不佩遮阳罩；tilt_only 不配 dome（无 pan 即废）。

## Multiplicity / Copy Logic

- 镜头模组 N ∈ {1(源), 2(fork dual_lens)}——for-loop 发射，模板可外推 3
- IR 灯环 N ∈ {4(S2), 8(S4), 20(S1)}——三档全源，免 fork
- 均 loop 发射 ✓

## Forks（6，全部 EXIT=0 + compile success + 轴断言在 run_tests）

turret_eyeball@S4 · box_housing@S3(⑥浅灰伴随) · dual_lens@S1 · wall_arm_pan@S2 ·
ceiling_bullet@S3(⑥白色伴随) · wall_arm_dome@S4

## 排除项

- 无 origin 排除（4/4 上格）；gate 见矩阵

> 2026-07-04 目检回炉：wall_arm_pan / tilted_console_tri / perimeter_ring / round_puck 四变体按图诊断重铸（落座/贴合/分区打靶约束），二轮均 compile success。
