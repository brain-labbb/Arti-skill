# Sports / game console — template source map

pattern: mixed (parallel_children control cluster on a single body + multiplicity face-button loop)

slug: handheld_game_console

parents:
- rec_blue-psp-style-handheld-game-console-with-a-cent_20260605_170024_819411_f576a977 ← picture/Sports/game console/003.png (PSP: horizontal slab, central landscape LCD, dpad + 4 round face buttons + analog nub + L/R shoulders) → covers Slot A=psp_slab, Slot B=dpad_plus_round_buttons, Slot C=landscape_lcd, N=4
- rec_purple-wireless-game-controller-with-two-thumbst_20260605_170016_335600_a59b32a9 ← picture/Sports/game console/002.png (controller: ergonomic two-grip gamepad, dual thumbsticks + dpad + ABXY, no screen) → covers Slot A=controller_ergo, Slot B=dual_thumbsticks, Slot C=no_screen, N=4 (ABXY)
- rec_red-handheld-brick-game-console-tetris-style-wit_20260605_170005_146230_2a2ee0bb ← picture/Sports/game console/001.png (red Tetris brick: vertical slab, square LCD, dpad rocker + 6 round buttons + power slide) → covers Slot A=brick_slab, Slot B=dpad_plus_round_buttons, Slot C=square_lcd, N=6

## Slot 候选覆盖

### Slot A:device archetype / body form
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| psp_slab | rec_blue...f576a977 (parent) | body/shell (_slab_solid), bezel_*, body_to_dpad/body_to_face_btn_i | single rigid horizontal slab, screen recessed center, grips at both ends | converged (parent) |
| controller_ergo | rec_purple...a59b32a9 (parent) | body/body_shell (_body_shell loft + grip horns), left/right_stick_dish | ergonomic two-grip lofted gamepad, no screen | converged (parent) |
| brick_slab | rec_red...2a2ee0bb (parent) | body/shell (_slab_solid wedge), bezel/screen_face, dpad_rocker | vertical wedge brick slab, square LCD at top | converged (parent) |
| clamshell | rec_handheld_game_console_var_clamshell | lower_panel / upper_panel / panel_hinge (REVOLUTE about X, barrel knuckle) | two flat lathe/cadquery panels folding open ~0-130deg on a back-edge hinge; lower=controls, upper=screen | built ✓ (fork from PSP) |

### Slot B:primary input cluster
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| dpad_plus_round_buttons | rec_blue...f576a977 / rec_red...2a2ee0bb (parents) | dpad + face_btn_i / btn_i, body_to_dpad, body_to_face_btn_i | cross dpad left + N round face buttons right (diamond/2x2) | converged (parent) |
| dual_thumbsticks | rec_purple...a59b32a9 (parent) | left_thumbstick / right_thumbstick, body_to_left/right_thumbstick (REVOLUTE tilt) | two analog tilt sticks in dished rings, plus dpad+ABXY | converged (parent) |
| numeric_keypad | rec_handheld_game_console_var_numeric_keypad | key_i (3x3 / 3x4 grid via loop), key_i_press (PRISMATIC), dpad | phone-style rectangular grid of small numeric keys beside a directional pad | built ✓ (fork from BRICK) |
| single_8way_joypad_lever | rec_handheld_game_console_var_joypad_lever | joypad_lever (shaft+ball, lathe), lever_collar, body_to_joypad_lever (REVOLUTE tilt 8-way) | one tall directional stick with ball top on a raised collar, replaces the cross dpad | built ✓ (fork from PSP) |

### Slot C:screen form
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| landscape_lcd | rec_blue...f576a977 (parent) | screen (Box), bezel_top/bot/left/right (silver bars), _slab_solid well cut | large landscape recessed LCD centered in the front face | converged (parent) |
| square_lcd | rec_red...2a2ee0bb (parent) | bezel, screen_face, pixel_grid (_grid_mesh), _recess_box | small near-square LCD in a gray bezel with pixel grid, at the top end | converged (parent) |
| no_screen | rec_purple...a59b32a9 (parent) | (no screen part; logo disc only) | gamepad with no display at all | converged (parent) |

注:Slot C 三个候选已由三个 parent 全部覆盖,本批无空格子 → 不另造 Slot C 变体。

## Multiplicity / Copy Logic
- count_param: face_button_count(右侧/动作圆按钮数;parent 命名 face_btn_i / btn_i)
- copied object: 单个圆形动作按钮 = 共享 dome+stem(PSP)或 cylinder cap(brick)几何 helper,每个一条独立 PRISMATIC press-down(axis 0,0,-1)joint
- naming: face_btn_{i}(PSP 风格)/ btn_{i}(brick 风格),for i in range(n) 循环发射
- placement: diamond(N=4,上右下左)/ 2x2+下排(N=6)/ in-line 或半 diamond(N=2,3);角度或行列等距
- joint policy: 每个按钮独立 PRISMATIC,统一 lower=0 upper≈0.0015-0.002,effort/velocity 一致;互不联动
- N 样本已覆盖: N=2 → rec_handheld_game_console_var_n2 ; N=3 → rec_handheld_game_console_var_n3 ; N=4 → PSP/controller parents ; N=6 → brick parent
- 模板建议 N_range: [2, 8](采样域远大于样本;真实手柄圆按钮通常 2-6,留余量到 8)
- 备注:parent PSP 已是干净的 for-i-in-range(face_offsets) 循环发射(face_btn_{i}),N 变体只改循环上界 + placement 表,copy logic 一眼可读;brick parent 用了 big_specs/small_specs 手写表(非纯循环),N 变体优先 fork 自 PSP 以继承干净循环。

## 跨层接口(未来 InterfaceSpec 预填)
- 控制簇 ↔ body:所有 dpad / 按钮 / 摇杆 / 拨杆都挂在 body 前控制面(PSP/brick z≈FRONT_Z/TOP_Z;controller z≈TOP_Z),joint origin 贴前面;mating face = 前控制面,anchor = 各部件 (x,y) 中心。
- clamshell hinge:lower_panel 后上边缘 ↔ upper_panel 后下边缘,mating face = 共享后边缘线,anchor = barrel knuckle 轴心,consumer joint = REVOLUTE,axis = X(共享边),limits ~ [0, 2.27rad]。
- joypad_lever / thumbstick:raised collar/dish ring(body 上的固定 visual)= 承托面,lever/stick 的 REVOLUTE 原点贴 collar 顶面。

## 排除项(未来 compatibility matrix 素材)
- 暂无不收敛取值(P0 规划阶段,fork 未执行)。
- 已主动排除(出类目风险,未列为候选):把圆形动作按钮整体换成单个旋钮/拨盘(读作收音机/计算器,非游戏机);给 controller 加大屏(会读成掌机而非手柄,与 Slot A 其它候选冲突)。Slot A×Slot C 的 controller+screen 跨格组合留给模板 compatibility matrix 裁决,不在 fork 批造。

---
## Post-fork verification (SEGMENT 1 complete)
All planned variants forked via `articraft fork` (dashscope qwen3.7-max, thinking medium), then verified on-disk: last compile = success, ≥1 non-fixed joint present, collections=['workbench'] (workbench-only, not promoted), and picture.json bound into the correct `Sports__<小类>` subcat shard (reconcile rebuilt). Status cells above flipped planned→built ✓ accordingly. Ready for SEGMENT 2 (spec authoring).
