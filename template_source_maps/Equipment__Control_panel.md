# Equipment / Control panel — template source map

pattern: mixed (mount + control + readout 三个固定结构槽 + push-button multiplicity)

slug: control_panel

parents（三个 parent = 三种 device archetype，各覆盖不同的 mount/control/readout 取值）:
- rec_build-a-realistic-articulated-3d-model-of-a-cont_20260609_180035_243303_c28c270c (P1 "rod") ← picture/Equipment/Control panel/001.png — `pendant_control_station`：铸铝小盒挂在竖直钢吊杆上。覆盖 mount=pendant_rod，control=round_push_buttons（手写 top/bottom 两按钮 + 侧 toggle），readout=none（裸 bezel）。
- rec_build-a-realistic-articulated-3d-model-of-a-cont_20260609_180037_817594_647d2061 (P2 "rail") ← picture/Equipment/Control panel/002.png — `rail_mounted_control_panel`：浅灰圆角塑料壳卡在双横轨上。覆盖 mount=rail_clamp，control=round_push_buttons（手写 left/right），readout=rect_lcd_with_leds（LCD+3 LED+vent slots）。
- rec_build-a-realistic-articulated-3d-model-of-a-cont_20260609_180040_391603_ab3b9f65 (P3 "conduit") ← picture/Equipment/Control panel/003.png — `industrial_disconnect_panel`：灰钣金柜挂在竖直线管前。覆盖 mount=conduit_wall，control=rotary_disconnect_handle（hero）+ 门面 `button_{i}` 行（**干净 `for i in range(4)` 循环**），readout=digital_display_window。**multiplicity 的 copy-logic 源就是 P3 的门按钮循环**（P1/P2 的按钮是手写 tuple，未循环化 — 已读码确认）。

## 组合数预审

4(mount)× 5(control)× 4(readout)× N{2,3,4,6} ≫ 10 ✓（实际只造收敛代表 + multiplicity 锚点，非全笛卡尔）

## Slot 候选覆盖

### Slot A:mount（device 后部承载机构 — 决定 root 与 FIXED 联接）
| 候选(未来 module) | record_id | 关键 part/joint/helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| pendant_rod | rec_...c28c270c (P1 parent) | `suspension_rod`/`rod_shaft`，`_build_cable_gland`→`top_gland`/`bottom_gland`，`rod_to_housing`(FIXED) | 竖直钢吊杆为 root，盒体经上下电缆 gland captive 挂在杆上 | parent(现成) |
| rail_clamp | rec_...647d2061 (P2 parent) | `rear_clamp`(`_build_clamp_shape`)，`rail_top`/`rail_bottom`(`_build_rail_shape`)，`clamp_to_rail_top`/`clamp_to_rail_bottom`(FIXED) | 壳后夹块开半管槽，双横轨各为独立 part 卡入槽 | parent(现成) |
| conduit_wall | rec_...ab3b9f65 (P3 parent) | `base`/`conduit_runs`(`_build_conduit_shape`：4 竖管+strap+drop+jbox)，`base_to_enclosure`(FIXED) | 竖直线管束为 root，横向 strap 贴柜后承载柜体 | parent(现成) |
| wall_backplate | rec_control_panel_var_mountA_backplate (fork 647d2061) | `back_plate`/`wall_back_plate`(`_build_wall_back_plate_shape`)，4×`mounting_tab_{i}`(`_build_mount_tab_shape`)，`housing_to_back_plate`(FIXED@BACK_Y) | 平矩形墙板 + 四角 keyhole 螺柱凸台，替换双轨夹块 | built ✓ |
| rail_clamp@enclosure | rec_control_panel_var_mountA_railN (fork ab3b9f65) | `rail_0`/`rail_1`(`_build_horizontal_rail_shape`)，`rear_clamp`(`_build_rear_clamp_shape`：saddle+web+stand-off pads)，`base_to_enclosure`(FIXED) | 把 rail_clamp 移植到大 enclosure 机体（跨 body 实现 rail mount 格子） | built ✓ |

### Slot B:control（主输入簇 — hero 关节所在）
| 候选(未来 module) | record_id | 关键 part/joint/helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| round_push_buttons(基线) | rec_...c28c270c / rec_...647d2061 (parents) | `top_button`/`bottom_button` + `housing_to_{label}_button`(PRISMATIC -X) ; `button_left`/`button_right` + `press_button_{left,right}`(PRISMATIC -Y)，`_build_button_cap`/`_build_button_shape` | 圆形动量按钮，沿面法向 PRISMATIC 短行程下压 | parent(现成) |
| rotary_disconnect_handle | rec_...ab3b9f65 (P3 parent) | `handle`/`disconnect_handle`(`_build_handle_shape`)，`operator_base`，`operator_handle`(REVOLUTE +X，160° throw) | 侧壁旋转隔离手柄 hub+lever，OFF↓/ON↑ | parent(现成) |
| rotary_selector_knob | rec_control_panel_var_ctrlB_rotaryknob (fork 647d2061) | `selector_knob`/`selector_knob_shell`(`KnobGeometry` knurled)，`selector_stem`，`pointer_mark`，`turn_selector`(REVOLUTE +Y，±135°) | 单个滚花选择旋钮绕面法向转，带指针刻度 | built ✓ |
| toggle_switch_bank | rec_control_panel_var_ctrlB_togglebank (fork 647d2061) | `switch_{idx}`/`{sname}_bat`(`_build_toggle_shape`)，`switch_socket_{idx}`，`housing_to_switch_{idx}`(REVOLUTE +X，±0.45)，`SWITCH_COUNT=3` | 一排小拨杆，各在前面 bushing 内 pitch 摆动 | built ✓ |
| mushroom_estop | rec_control_panel_var_ctrlB_mushroom (fork c28c270c) | `emergency_stop`：`safety_guard`/`retaining_gasket`/`mushroom_cap`(`_build_estop_mushroom_cap` lathe)，`housing_to_emergency_stop`(PRISMATIC -X，深 0.012 latch) | 单个大蘑菇急停，深行程锁定柱塞，替换两圆按钮 | built ✓ |
| push_buttons_drop_rotary | rec_control_panel_var_ctrlB_pushbtn_railrotary (fork ab3b9f65) | `side_door_panel`，`side_button_{i}`/`side_button_cap_{i}`(`_build_side_button_shape`)，`side_button_slide_{i}`(PRISMATIC +X)，`SIDE_BUTTON_COUNT=4` | 删去旋转手柄，改侧门一排弹簧 push-button（round_push_buttons 在 P3 机体上的实现） | built ✓ |

注:Slot B 收敛为 5 个 distinct module（push_buttons / rotary_handle / rotary_knob / toggle_bank / mushroom_estop）；`pushbtn_railrotary` 是 round_push_buttons 模块在 P3 family 的跨体实现 + 演示 rotary→pushbtn 替换。

### Slot C:readout（前面板显示/指示簇 — 全为 housing 固定 visual，无关节）
| 候选(未来 module) | record_id | 关键 part/joint/helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| rect_lcd_with_leds | rec_...647d2061 (P2 parent) | `display_glass`/`display_bezel`(`_build_display_glass`/`_build_display_bezel`)，`led_0..2`(`_build_led`)，vent slots(`SLOT_COUNT=5` cut loop) | 矩形凹陷 LCD + 顶部 3 LED + 左列通风槽 | parent(现成) |
| digital_display_window | rec_...ab3b9f65 (P3 parent) | `display_glass`(`_build_display_glass_shape`)，门 `pocket` cut | 门上单块凹陷数显窗 | parent(现成) |
| analog_round_gauge | rec_control_panel_var_readC_gauge (fork 647d2061) | `gauge_rim`/`gauge_dial`/`gauge_hub`/`gauge_needle`(`_build_gauge_*`)，`gauge_tick_{idx}`(`for idx in range(11)`)，`gauge_pocket` | 沉孔圆形模拟表：金属圈+刻度盘+11 tick+指针+中心 hub | built ✓ |
| lcd_led_vent_cluster | rec_control_panel_var_readC_lcdrow (fork c28c270c) | `lcd_frame`/`lcd_glass`(`_build_lcd_frame`)，`indicator_led_{i}`(range 3)，`vent_slot_{i}`(`VENT_SLOT_COUNT=4` loop) | 给原裸 bezel(P1) 加凹陷 LCD + 3 色 LED 行 + 竖排通风槽 | built ✓ |

注:Slot C 四格已由两 parent + 两变体全覆盖（含给 P1 补 readout 的 lcdrow）。

## Multiplicity / Copy Logic
- count_param: `BUTTON_COUNT`（前排圆 push-button 数；门面控制行）
- copied object: 单个圆 push-button cap（`_build_button_shape`/`_build_button_cap`）+ 门体对应 counterbore/stem bore，每个一条独立 PRISMATIC 内压 joint
- naming: 部件 `button_{i}`（visual `button_{i}_cap`/`button_{i}_plunger`），joint `button_slide_{i}`（N6）/ `door_to_button_{i}`（N3），`for i in range(BUTTON_COUNT)` 发射
- placement: 居中等距 `bx = (i - (BUTTON_COUNT-1)/2) * BUTTON_PITCH`（N3 pitch=0.060，N6 pitch=0.044），display 在上、按钮行在下
- joint policy: 每个按钮独立 PRISMATIC，axis=(0,-1,0)（内压门面），统一 lower=0 upper=`BUTTON_TRAVEL=0.006`，effort/velocity 一致，互不联动
- N 样本已覆盖: {2(P1/P2 parent，**手写 tuple 未循环**)，3 → rec_control_panel_var_N3_buttons，4(P3 parent，门面 `range(4)` 循环但为 fixed visual)，6 → rec_control_panel_var_N6_buttons}
- 模板建议 N_range: **[2, 12]**（采样域远大于样本；真实控制面按钮行常 2–8，留余量到 12）
- **注意:P1/P2 的 N=2 是手写 `top/bottom`、`left/right` tuple，非循环;P3 parent 的 `range(4)` 把按钮当门体 fixed visual 发射。N3/N6 变体已把该循环重写为「articulated `button_{i}` part + `button_slide_{i}` PRISMATIC」的干净 `for i in range(BUTTON_COUNT)`，模板应以 N3/N6 变体（fork 自 P3）作为 multiplicity 的 copy-logic 源码，而非任何 parent。**

## 跨层接口(未来 InterfaceSpec 预填)
- control 簇 ↔ housing 前面:所有 push-button/knob/toggle/mushroom 的 joint origin 贴前面板面（P2/P3 = +Y `FACE_Y`/门面;P1 = +X `FRONT_X` bezel 面），mating face = 前控制面，anchor = 各控制件 (x,z) 中心;rotary_handle 例外 — operator_base 法兰/shaft 在 -X 侧壁，REVOLUTE 绕 +X operator 轴。
- mount ↔ housing:rail_clamp = 夹块半管槽 seat 双轨（`clamp_to_rail_*` FIXED，axis 沿 X 轨向）;wall_backplate = 平板 y=0 贴壳后缘 flush（`housing_to_back_plate` FIXED@`BACK_Y`，expect_gap 验贴合）;conduit_wall = strap 贴柜后承载（`base_to_enclosure` FIXED，allow_overlap 表 bolted seat）;pendant_rod = rod 经 gland captive 穿过（`rod_to_housing` FIXED，expect_within 验居中）。mating face = 后缘/侧轨面。
- readout ↔ housing 前面:display/gauge 沉入前面 pocket cut（`disp_pocket`/`gauge_pocket`/门 pocket），LED 坐 well，全为 housing 固定 visual（无 joint）。anchor = pocket 中心 (x,z)。

## 排除项(未来 compatibility matrix 素材)
- 暂无不收敛取值（本批 fork 全部 last compile = success）。
- 跨格组合留给模板 compatibility matrix 裁决、本批未造:rotary_disconnect_handle 装到小 rail 壳（P2 body 太浅，operator shaft 无侧壁余量）;mushroom_estop 装到 conduit 门;analog_gauge 与 toggle_bank 同面争位。这些 mount×control×readout 跨家族组合不在 fork 批内枚举。
- 已主动排除(出类目风险):把整排按钮换成单旋钮再叠大屏（读作仪表/收音机）;给 pendant_rod 盒加双轨（mount 语义冲突）。

---
## Post-fork verification (SEGMENT 1 complete)
All 10 planned variants forked via `articraft fork`，逐一 on-disk 校验：last compile = success，≥1 非 FIXED 关节存在（push-button PRISMATIC / knob·toggle·handle REVOLUTE / mushroom PRISMATIC），collections=['workbench']（workbench-only，未 promote），picture.json 已绑入 `Equipment__Control_panel` subcat shard（reconcile 已重建）。上表状态格已按结果翻 planned→built ✓。Ready for SEGMENT 2 (spec authoring)。
