# Equipment / Game console — template source map

pattern: mixed(body 形态槽 + control 机构槽 + 玩家站位 multiplicity)

slug: arcade_cabinet_game_console

注:本小类是**站立/台面式街机柜(arcade cabinet)**整柜家族,与 `Sports/game console`(掌机 PSP/手柄)是不同小类,二者只共享文件格式不共享内容。

parents:
- rec_build-a-realistic-articulated-3d-model-of-a-game_20260609_180045_745274_a5689b50 ← picture/Equipment/Game console/001.png(蓝色风化钣金 **wedge 楔形柜**:竖直后背 + 斜置上前脸 GAME OVER 屏 + 竖直下前脸带螺栓检修板;斜置控制带上左右两块金色键盘格夹中央红控制板;红板上立一根蓝球头摇杆;底座踏板块。可动件 = 摇杆 REVOLUTE 前后摆)→ 覆盖 Slot A=wedge_cabinet(基线)、Slot B=ball_top_joystick(基线)、N=1(单站位,手写命名)

母资产是**单站位、手写命名**的:键盘 `keypad_left`/`keypad_right`、螺栓 `access_screw_{l/r}_{t/b}`、单摇杆 part `joystick` + 关节 `panel_to_joystick`,**均为语义角名/单体,非 `range(n)` 循环**。站位 multiplicity 的 copy-logic 源在 N 变体(stations_x2/x4),不在 parent —— 已在读码中确认。

## 组合数预审

4(body)× 4(control)× 3(N 样本,保守)= 48 ≥ 10 ✓

## Slot 候选覆盖

### Slot A:body / cabinet form(柜体形态——承载屏与控制面的根 part)
| 候选(未来 module) | record_id | 关键 part/joint/helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| wedge_cabinet(基线) | rec_...745274_a5689b50(parent) | `_build_cabinet_shell`(YZ `polyline` 楔形)/ `_sloped_face_geometry` / `cabinet_shell` / `base_pedestal` / `screen_bezel`+`screen_glass`+`game_over_text`(斜面 pitch) | 楔形:竖后背 + 斜上前脸(屏)+ 竖下前脸(检修板)+ 斜置控制带(pitch≈atan2) | converged(parent) |
| upright_box | rec_game_console_var_body_upright_box | `_build_cabinet_shell`(矩形 side)/ `_front_screen_geometry`(roll=π/2)/ `control_shelf` / `shelf_front_lip` | 直立矩形箱体 + 竖直平前脸屏 + 从前脸水平挑出的控制搁板 | built ✓(fork from wedge) |
| cocktail_flattop | rec_game_console_var_body_cocktail | `_build_cabinet_shell`(低宽 box)/ `top_rail_0..3`(四边黑边轨)/ `screen_glass`(+Z 平嵌)/ `_rect_frame` / `_keypad_geometry` | 低矮宽台面式,屏平嵌 +Z 顶台,控制簇置于顶台屏前(CAB_W≈0.78,CAB_H≈0.32) | built ✓(fork from wedge) |
| bartop_crown | rec_game_console_var_body_bartop_crown | `_build_cabinet_shell`(`threePointArc` 圆顶)/ `_build_curved_marquee_crown`(YZ 弧)/ `curved_marquee_crown` / `_screen_face_geometry`(roll=-π/2) | bartop 紧凑柜:竖直屏脸 + 上方圆弧 marquee 顶冠(取代楔形平顶) | built ✓(fork from wedge) |

### Slot B:primary control mechanism(主控制机构——挂在控制面上的可动件)
| 候选(未来 module) | record_id | 关键 part/joint/helper 名 | 结构特征 + joint 策略 | 状态 |
|---|---|---|---|---|
| ball_top_joystick(基线) | rec_...745274_a5689b50(parent) | part `joystick`:`joystick_shaft`/`joystick_boot`/`joystick_ball`;固定座 `joystick_collar`;joint `panel_to_joystick` | 球头摇杆经红板 collar 立起;**REVOLUTE** axis=(-1,0,0) 前后摆,limits ±0.45 | converged(parent) |
| trackball | rec_game_console_var_ctrl_trackball | part `trackball`:`trackball_sphere`;固定座 `trackball_cup_liner`+`trackball_bezel_cup`(红板 `cutThruAll` 圆孔);joint `plate_to_trackball` | 轨迹球嵌入红板凹杯;**REVOLUTE** axis=(0,0,1) 原地自旋(无柄无倾),limits ±π | built ✓(fork from wedge) |
| spinner_knob | rec_game_console_var_ctrl_spinner | part `spinner`:`spinner_stem`/`spinner_body`(`_build_knurled_spinner_body`);固定座 `spinner_bearing`;joint `panel_to_spinner` | 滚花 spinner/桨钮立于轴承;**CONTINUOUS** axis=(0,0,1) 无止挡连续旋转(注:实为 continuous 非 revolute) | built ✓(fork from wedge) |
| linear_slider | rec_game_console_var_ctrl_slider | part `slider`:`slider_runner`/`slider_thumb`;固定座 `slider_slot_floor`(红板 `cut` 通槽 + 暗底);joint `panel_to_slider` | 拇指滑块沿红板通槽左右滑;**PRISMATIC** axis=(1,0,0),limits ±0.018 | built ✓(fork from wedge) |

注:Slot A 四个 body 变体均保留基线 `ball_top_joystick` 控制;Slot B 四个 control 变体均保留基线 `wedge_cabinet` 柜体 —— 两槽各自独立单变量收敛,A×B 跨格组合留给模板 compatibility matrix。

## Multiplicity / Copy Logic
- count_param: `joystick_count` / `station_count`(x4 变体用常量 `STATION_COUNT`;x2 变体硬编 `range(2)`)
- copied object: **一个完整玩家站位** = 红控制板 + 金按钮簇 + gimbal collar + 该站位自己的球头摇杆 + 四角螺栓,由共享 helper 发射
- naming: `station_{i}`(板)/ `station_{i}_buttons`(x2)或 `station_buttons_{i}`(x4)/ `joystick_collar_{i}` / 子 part `joystick_{i}` / 关节 `panel_to_joystick_{i}`(x2)或 `deck_to_joystick_{i}`(x4),`for i in range(n)` 循环发射
- placement: 沿 X 左右等距一排;x2 用 `station_x = (i-0.5)*station_spacing`(spacing≈0.190);x4 用 `_station_x_positions()` 对称 pitch 排布,且 `CAB_W` 加宽到 0.900 容纳四站位;站位面偏移经 `band_point()`/`_panel_point()` 投到倾斜控制带法向(`band_face_pitch≈-0.18`)
- joint policy: 每站位独立 **REVOLUTE**,axis=(-1,0,0) 前后摆,limits lower=-0.45 upper=0.45,effort/velocity=4.0,互不联动
- N 样本已覆盖: **N=1 → parent(单摇杆,手写 `joystick`/`panel_to_joystick`,未循环化)**;N=2 → rec_game_console_var_stations_x2;N=4 → rec_game_console_var_stations_x4
- 模板建议 N_range: **[1, 6]**(采样域大于样本;真实街机柜 1-4 玩家常见,留余量到 6;sweep 小 N 高频)
- **注意:parent 的 N=1 是手写单体 `joystick`/`panel_to_joystick`,未循环化;stations_x2/x4 已重写为 `station_{i}`/`joystick_{i}` 循环链,模板应以 N 变体(而非 parent)作为 multiplicity 的 copy-logic 源码。两 N 变体中,x4 更"模块化"(`STATION_COUNT` 常量 + `_add_joystick_station()` helper + `_station_x_positions()`),x2 用 `_build_control_station_geometry()` dict + 内联 `range(2)`;建议以 x4 为首选源码,因其 count 已参数化为单一常量。**

## 跨层接口(未来 InterfaceSpec 预填)
- 控制机构 ↔ body 控制面:所有 joystick / trackball / spinner / slider 都挂在 body 的控制面上 —— wedge/bartop 为倾斜控制带(`band_y`/`band_z`,法向含 `band_face_pitch≈-0.18`),upright_box 为水平 `control_shelf` 顶面,cocktail 为 `deck_z` 水平顶台;joint origin 贴该面法向,mating face = 控制面,anchor = 红板(x,y)中心。
- 固定座 = 承托面:`joystick_collar` / `trackball_cup_liner`+bezel / `spinner_bearing` / `slider_slot_floor` 是 body 上的固定 visual,子可动件原点贴座顶面;captured fit(boot/shaft 入 collar、ball 入 cup、stem 入 bearing)有 `allow_overlap` 声明。
- multiplicity:每站位红板/collar 在控制面上 `band_point()`/`_panel_point()` 等距投点,consumer joint = REVOLUTE,axis = -X(共享前后摆轴),limits ±0.45。

## 排除项(未来 compatibility matrix 素材)
- 暂无不收敛取值(P0 fork 阶段,8 变体全部 built ✓)。
- 机构差异备注:spinner 的 joint 实为 **CONTINUOUS**(无止挡),与 joystick/trackball 的 REVOLUTE 不同;模板写 Slot B compatibility 时勿强行统一为 revolute。
- 跨格组合(如 trackball/spinner/slider × N 站位、cocktail/upright body × 倾斜带专用控制)未在本批 fork,留给模板 compatibility matrix 裁决。

---
## Post-fork verification (SEGMENT 1 complete)
All planned variants forked from `a5689b50` and verified on-disk: last compile = success, ≥1 non-fixed joint present (REVOLUTE/CONTINUOUS/PRISMATIC per Slot B; per-station REVOLUTE for N variants), collections=['workbench'] (workbench-only, not promoted), and picture.json bound into the correct `Equipment__Game_console` subcat shard. Status cells above flipped planned→built ✓ accordingly. Ready for SEGMENT 2 (spec authoring).
