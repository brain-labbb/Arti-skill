# Equipment / Power switch — template source map

pattern: mixed(actuator 槽 + mount 结构槽 + gang_count 链式 multiplicity)

slug: power_switch

parents:
- rec_build-a-realistic-articulated-3d-model-of-a-wall_20260609_154028_849119_5b4ad2d8 ← picture/Equipment/Power switch/001.png(扁平 WALL PLATE:竖向圆角 faceplate + 凹陷倒角 inner field + 顶部 louver 排气垫 + 中央 draw-latch 滚柱拉闸;helper `_build_faceplate`/`_build_louver_pad`/`_build_keeper`/`_build_side_bolts`/`_build_screws`,moving `roller_bail`=`_build_bail_arms`+`_build_roller`,joint `plate_to_bail` REVOLUTE X。**actuator=roller_bail 基线 + mount=flat_wall_plate 基线 + gang N=1 基线;gang 变体均 fork 自此**)
- rec_build-a-realistic-articulated-3d-model-of-a-powe_20260609_180112_553528_621bac5e ← picture/Equipment/Power switch/002.png(PENDANT BOX:八角 ABS 吊挂控制箱,固定 conduit_tubes/gland_collar,8 条 cage_bars(`_cage_bar_specs`),中央 `selector_faceplate` + 竖向抓手 `slider`(`housing_to_slider` PRISMATIC Y)+ 底排 3 个 `button_{i}`(`housing_to_button_{i}` PRISMATIC −Z);**mount=pendant_box 基线 + actuator=linear_slider 基线**)

两个 parent 各覆盖自己的 actuator+mount 基线格;actuator 4 个 fork 变体全部 fork 自扁平 plate(5b4ad2d8),mount 2 个 fork 变体全部 fork 自 pendant box(621bac5e)。

## 组合数预审

6(actuator)× 4(mount)× 3(gang N 样本,保守下界)= 72 ≥ 10 ✓(实际 actuator×mount 跨格组合需 compatibility matrix 裁剪,见排除项)

## Slot 候选覆盖

### Slot A:actuator(被铰接的执行机构——开关动作主体)
| 候选(未来 module) | record_id | 关键 part/joint/helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| roller_bail_drawlatch(基线) | rec_..._5b4ad2d8(parent) | part `roller_bail` / `_build_bail_arms`+`_build_roller`(hub+arm+roller stub);fixed `_build_keeper`+`_build_side_bolts`;`plate_to_bail` REVOLUTE axis X | 双臂滚柱拉闸越中扳过 keeper,绕侧螺栓 X 轴翻摆 | converged(parent) |
| grab_handle_slider(基线) | rec_..._621bac5e(parent) | part `slider` / `_build_slider_mesh`(block+neck+grab loop);`housing_to_slider` PRISMATIC axis +Y;slot 在 `selector_faceplate` | 抓手块在 faceplate 竖槽内上拉/下推的线性隔离手柄 | converged(parent) |
| flip_toggle_dolly | rec_power_switch_var_actuator_flip_toggle(fork 5b4ad2d8) | part `toggle` / `_build_toggle_lever`(hub+stem+pad+web);fixed `_build_toggle_boss`(`raised_boss` 双 ear+hub relief);`plate_to_toggle` REVOLUTE X,throw −10°..9° | 单根中央 dolly 拨杆在 raised boss 两 ear 间捕获翻转 | built ✓ |
| rocker_paddle | rec_power_switch_var_actuator_rocker(fork 5b4ad2d8) | part `rocker_paddle` / `_build_rocker_paddle`(`paddle_shell`+top_mark/bottom_mark);fixed `_build_rocker_well`(`rocker_well`);`plate_to_paddle` REVOLUTE X,±0.24 | 宽矩形跷板在 molded well 内绕中央横轴 see-saw | built ✓ |
| pushbutton_cap | rec_power_switch_var_actuator_pushbutton(fork 5b4ad2d8) | part `button_cap` / `_build_button_cap`;fixed `_build_button_bezel`(`button_bezel`)+`_build_bore_shadow`;joint PRISMATIC axis (0,0,−1),lower/upper=PRESS_LOWER/UPPER | 圆顶瞬动按钮压入 bezel 镗孔(唯一 PRISMATIC 法向行程) | built ✓ |
| rotary_cam_selector | rec_power_switch_var_actuator_rotary(fork 5b4ad2d8) | part `selector_knob` / `_build_knob_body`+`_build_pointer_skirt`;fixed `_build_rotary_mount`(`rotary_mount`);joint REVOLUTE axis (0,0,1)=Z 面法向,lower/upper=SELECT_LOWER/UPPER | 带指针 skirt 的旋转凸轮选择钮,绕面法向 Z 转 | built ✓ |

### Slot B:mount(承载 actuator+面板的结构本体——结构槽,非复制体)
| 候选(未来 module) | record_id | 关键 part/joint/helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| flat_wall_plate(基线) | rec_..._5b4ad2d8(parent) | root `faceplate` / `_build_faceplate`(`faceplate_shell` 圆角竖板+凹 field+nameplate)+`_build_louver_pad`+`_build_screws` | 薄壁竖向壁装面板,顶 louver 垫+角螺钉,gang 变体复用此体 | converged(parent) |
| pendant_box(基线) | rec_..._621bac5e(parent) | root `housing` / `_build_shell_mesh`(八角 ABS 箱)+`_build_cage_mesh`+`gland_collar`+FIXED `conduit_tubes` | 吊挂八角控制箱,顶 conduit 管+白 gland,带 cage 框 | converged(parent) |
| industrial_enclosure_box | rec_power_switch_var_mount_enclosure(fork 621bac5e) | root `housing` / `_build_shell_mesh`(方箱)+`_build_lid_lip_mesh`+`_build_lug_boss_mesh`(`_lug_center(i)` 角耳);名 `square_industrial_isolator` | 方形工业隔离箱,盖唇+四角安装 lug boss;保留 slider+3 button | built ✓ |
| inline_cord_barrel | rec_power_switch_var_mount_inline(fork 621bac5e) | root `housing` / `_build_shell_mesh`(圆角桶)+`_end_boss`/`_cord_stub`(`_build_cord_bosses_mesh`/`_build_cord_stubs_mesh`)+`_build_top_cover_mesh`+`_build_slider_track_mesh`;名 `inline_cord_power_switch` | 线缆中段在线开关桶,两端 cord 出线 boss+顶盖滑轨;保留 `slider`/`housing_to_slider` | built ✓ |

注:actuator 与 mount 在两个 parent 间正交分布——actuator 变体均带 flat_wall_plate(继承 5b4ad2d8),mount 变体均带 grab_handle_slider(继承 621bac5e)。某 actuator×某 mount 的跨格混搭(如 rotary 装进 inline barrel)属模板 compatibility matrix 裁决,不在本 fork 批造,见排除项。

## Multiplicity / Copy Logic
- count_param: `gang_count`(变体源码命名不统一:n2 用 `UNIT_COUNT`,n3 用 `BAIL_COUNT`;模板应统一为 `gang_count`)
- copied object: 一个开关 gang 单元 = 一条 moving `roller_bail` + 其专属固定板特征(`_build_keeper`/`_build_side_bolts`,n3 还含 `_build_louver_pad(x_off)`),共享 `faceplate` 壁板为根
- naming: part `bail_{i}`、joint `plate_to_bail_{i}`,`for i in range(gang_count)` 循环发射;n2 用 `_add_bail_unit_visuals(bail, i)` 封装单元视觉
- placement: 沿板宽 X 等距,`_unit_x(i) = (i - (N-1)/2) * UNIT_PITCH` 居中对称(n2 UNIT_PITCH=0.066,n3=0.060)
- joint policy: 每个 gang 自带独立 `plate_to_bail_{i}` REVOLUTE,axis X,limits(ROCK_LOWER/UPPER)全单元一致,互不联动
- N 样本已覆盖: {1(parent 扁平板基线,单 `plate_to_bail` 手写未循环), 2 → rec_power_switch_var_gang_n2, 3 → rec_power_switch_var_gang_n3}
- 模板建议 N_range: **[1, 6]**(真实联排开关通常 1–4 gang,留余量到 6;N=1 退化为单 `plate_to_bail` 命名)
- **注意:parent(N=1)以 `roller_bail`/`plate_to_bail` 手写单元,未循环化;n2/n3 已重写为 `bail_{i}`/`plate_to_bail_{i}` + `_unit_x(i)` 干净循环链,模板应以变体(而非 parent)作为 multiplicity 的 copy-logic 源码。**

## 跨层接口(未来 InterfaceSpec 预填)
- actuator ↔ mount:所有 actuator 都挂在 mount 前控制面(plate `faceplate_shell` 前面 z≈PLATE_T 或 pendant `selector_faceplate` 面),joint origin 贴前面/凹 field;mating face = 前控制面,anchor = actuator (x,y) 中心。每种 actuator 自带其固定承托件(roller=keeper+side_bolts;toggle=raised_boss 双 ear;rocker=rocker_well;pushbutton=button_bezel 镗孔;rotary=rotary_mount;slider=faceplate slot/track),换 actuator 时连承托件一起替换。
- actuator joint 轴向随候选而变(模板需声明 per-candidate joint axis):roller/toggle/rocker = REVOLUTE X(横轴翻摆);rotary = REVOLUTE Z(面法向转);pushbutton = PRISMATIC −Z(法向压入);slider = PRISMATIC +Y(竖向行程)。这是 actuator 模块各自的 InterfaceSpec consumer joint,不可统一硬编码为 X 轴。
- gang 单元 ↔ plate:每个 `bail_{i}` 的 `plate_to_bail_{i}` 原点 = `(_unit_x(i), PIVOT_Y, PIVOT_Z)`,mating face = 共享 `faceplate_shell` 前面,anchor = 各单元 keeper/side_bolt 联接处。

## 排除项(未来 compatibility matrix 素材)
- gang multiplicity 仅在 flat_wall_plate(roller_bail)上验证收敛;gang_count > 1 套在 pendant_box / inline_barrel / rotary 等候选上未造样本,留给模板 compatibility matrix 裁决(不假定任意 mount × 任意 N 都收敛)。
- 已主动排除(出类目风险,未列为候选):把 actuator 整体换成连续旋转的旋钮+大刻度盘读作调光器/定时器、或换成纯指示灯无可动件(读作信号灯非开关);inline_barrel × gang>1(线缆在线开关现实中不联排)。
- 暂无连续 N 次不收敛的轴值(P0 规划阶段记录;fork 已执行,8 变体均 built ✓)。

---
## Post-fork verification (SEGMENT 1 complete)
All 8 planned variants forked via `articraft fork`, then verified on-disk: last compile = success, ≥1 non-fixed joint present(actuator REVOLUTE/PRISMATIC 或 gang 多 REVOLUTE),collections=['workbench'](workbench-only, not promoted),picture.json bound into the `Equipment__Power_switch` subcat shard(reconcile rebuilt). Status cells above flipped planned→built ✓ accordingly. Slot A=actuator 覆盖 6(2 parent 基线 + 4 fork),Slot B=mount 覆盖 4(2 parent 基线 + 2 fork),gang N∈{1,2,3} 样本就位。Ready for SEGMENT 2 (spec authoring).
