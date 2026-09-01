# Equipment / LED work light — template source map

pattern: mixed(mount 槽 + head 槽 + panel 槽 三个结构格子 + led_count 网格 multiplicity)

slug: led_work_light

parents:
- rec_build-a-realistic-articulated-3d-model-of-a-led-_20260609_180048_908359_f7e038e0 ← picture/Equipment/LED work light/001.png(黄色管式 H-frame 工作灯:`stand_frame`(`side_rail_*`/`base_cross_member`/`foot_*`/`upright_*`)托一个黑色矩形泛光头 `light_head`,`stand_to_head` REVOLUTE 侧轴俯仰;头面是矩形玻璃 `led_glass_panel` + 黑边框 `bezel_*` + LED 点阵 `led_{r}_{c}`(5×8 循环)+ 黄色电池盒 `battery_pack` + U 形提手 `carry_handle`/`handle_grip`。**fork 基线;LED 阵列已是干净的双层 for 循环发射,copy-logic 现成,无需重写**)

本批 9 个 fork 全部 forked from f7e038e0,填 3 个结构槽 + 1 个 multiplicity 轴。

## 组合数预审

4(mount)× 3(head)× 3(panel)× 3(led_count 样本,保守下界)= 108 ≥ 10 ✓

## Slot 候选覆盖

### Slot A:mount(底座/承托机构槽——决定 head 如何离地与被托起)
| 候选(未来 module) | record_id | 关键 part/joint/helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| h_frame_stand(基线) | rec_..._f7e038e0(parent) | part `stand_frame`;`side_rail_pos_y/neg_y`、`base_cross_member`、`foot_fr/br/fl/bl`、`upright_pos_y/neg_y`(`_tube` helper 弯管)、`pivot_knob_*`;joint `stand_to_head` REVOLUTE Y | 黄色管式 H 底 + 两根外张立柱夹住头侧 boss + 四个黑橡胶端帽脚 | converged(parent) |
| folding_aframe | rec_led_work_light_var_mount_aframe | `stand_frame` 上 `apex_crossbar`、`leg_pin_0/1`、`pivot_yoke_pos_y/neg_y`、`yoke_gusset_*`;child parts `folding_leg_0/1`(`hinge_barrel`、`side_strut_0/1`、`foot_bar`、`rubber_foot_0/1`);joints `stand_to_leg_0/1` REVOLUTE Y(折叠)+ `stand_to_head` | 三角化折叠 A-frame:apex 桥架 + 两条绕可见 apex 销折叠的腿(各加一条 REVOLUTE) | built ✓ |
| tripod_mast | rec_led_work_light_var_mount_tripod | `stand_frame`:`hub_collar`、`leg_0/1/2`(`for i in range(3)` 径向循环)、`foot_0/1/2`、`mast`、`mast_top_collar`、`pivot_yoke_0/1`、`pivot_axle`、`pivot_knob_0/1`;joint `stand_to_head` | 桌面三脚架 hub + 三条等角放射腿(循环)+ 中央立柱,mast 顶 yoke + 贯穿 axle 托头 | built ✓ |
| handheld_hook | rec_led_work_light_var_mount_hook | part `handheld_base`(`_rounded_box` helper:`base_shell`、`top_saddle`、`control_panel`、`power_button`、`rear_stand_foot`、`front_rubber_pad`、`hook_lug_0/1`、`hook_pin_cap_0/1`、`yoke_arm_0/1`、`yoke_socket_0/1`、`tilt_knob_0/1`);child `hanging_hook`(`hook_hinge_barrel`、`folding_hook`);joints `base_to_hook` REVOLUTE Y + `base_to_head` REVOLUTE Y | 紧凑手持模塑底 pod + 后台脚 + 绕可见铰耳折出的挂钩(额外一条 REVOLUTE) | built ✓ |

### Slot B:head(头部承托链/铰接形态槽——决定头有几个自由度)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| side_tilt(基线) | rec_..._f7e038e0(parent) | head `pivot_boss_pos_y/neg_y`;joint `stand_to_head` REVOLUTE Y(单铰) | 头直接挂在 mount 立柱顶,仅侧轴俯仰 1 DOF | converged(parent) |
| tilt_pan_yoke | rec_led_work_light_var_head_tiltpan | 插入 part `u_yoke`;stand `vertical_post`/`pan_bearing_top`;yoke `yoke_turntable`、`support_socket`、`center_post`、`lower_fork_crossbar`、`fork_arm_pos_y/neg_y`、`pivot_cheek_*`、`pivot_axle`;joints `stand_to_yoke` REVOLUTE **Z**(pan ±π)+ `yoke_to_head` REVOLUTE Y(tilt) | 立柱上加转盘 U-yoke,先 pan 后 tilt,2 个 revolute 串联(头 `battery_pack` 被移除以让出 yoke 中空) | built ✓ |
| telescope_tilt | rec_led_work_light_var_head_telescope | 插入 part `inner_mast`;stand `outer_post_*`、`outer_sleeve_*`(cadquery `_open_tube` 环形套筒)、`mast_clamp_*`、`clamp_knob_*`;mast `upright_pos_y/neg_y`、`yoke_plate_*`、`pivot_axle`;joints `stand_to_mast` PRISMATIC **Z**(travel 0→`MAST_TRAVEL`=0.055)+ `mast_to_head` REVOLUTE Y | 固定外套筒 + 内伸缩 mast(prismatic 升降)+ 头侧轴俯仰(revolute),prismatic+revolute 串联 | built ✓ |

### Slot C:panel(发光面/头壳形态槽——即 LED 被排布与封装的载体)
| 候选(未来 module) | record_id | 关键 part/joint/helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| rect_flood(基线) | rec_..._f7e038e0(parent) | head `housing_back`、`housing_wall_top/bottom/left/right`、`bezel_top/bottom/left/right`、`led_glass_panel`(Box)、`led_{r}_{c}` 矩形网格 | 矩形开口 tub 壳 + 矩形玻璃漫射板 + 黑边框 + 矩形 LED 点阵 | converged(parent) |
| cob_round_disc | rec_led_work_light_var_panel_cob_round | `_lathe` helper:`round_housing_shell`、`ring_bezel`、`round_glass_panel`(Cylinder)、`cob_carrier_disc`;LED 用同心环 `led_{i}`(`LED_RING_COUNTS=(1,8,14,20)`=43,`LED_RING_RADII`) | 车削成型圆形 COB 浅杯 + 环形压圈 + 圆玻璃,LED 按同心环放射排布 | built ✓ |
| dual_flood_bar | rec_led_work_light_var_panel_dual | `_emit_flood_housing(head, …, center_y)` helper 被 `for i in range(PANEL_COUNT=2)` 循环;`shared_crossbar`、每壳 `led_{index}_{r}_{c}` 子网格、`bezel_screw_{index}_{corner}`;`FLOOD_W=(HEAD_W-HOUSING_GAP)/PANEL_COUNT` | 一根共享 `shared_crossbar`/tilt 上并排两个泛光壳,读作单一俯仰单元 | built ✓ |

注:Slot C 的 cob_round 与 dual 各自带**不同的 LED 放置策略**(同心环 / 每壳子网格),与下方矩形网格 `led_count` 轴正交——它们换的是 panel 载体形态,不是矩形阵列的计数轴。

## Multiplicity / Copy Logic
- count_param: `led_count`(parent 以 `led_rows, led_cols = 5, 8` 表达 → 40;模板可暴露为 rows×cols 或总数)
- copied object: 单颗 LED 发射子 = `Box((led_size, led_size, 0.0025))` 小方点(`led_size≈0.009`),贴在玻璃前 `led_z = glass_z + 0.004`
- naming: `led_{r}_{c}`,嵌套 `for r in range(led_rows): for c in range(led_cols)`(parent 现成,sparse/dense 沿用同结构,可直接作 module 源码)
- placement: 沿玻璃面 `span_x × span_y` 等距矩形网格(`px = -span_x/2 + span_x*r/(rows-1)`,`py` 同理),居中
- joint policy: **LED 全部 FIXED on `light_head`(inline visual,无 joint)**;multiplicity 纯视觉计数,不增自由度 —— 与 Fence panel 链式不同,这里复制体不铰接
- N 样本已覆盖: {15(3×5)→ rec_led_work_light_var_leds_sparse;40(5×8)→ parent 基线;88(8×11)→ rec_led_work_light_var_leds_dense}
- 模板建议 N_range: **[9, 120]**(约 3×3 至 10×12;采样域远大于样本,sweep 建议低 N 高频、高 N 长尾并设上限控编译时长)
- **注:parent 的 LED 阵列已是干净的 `led_{r}_{c}` 双层循环(本批已确认),parent 自身即 copy-logic 源码,sparse/dense 仅改 `led_rows, led_cols` 两上界与隐含间距,无需重写。** cob_round(`led_{i}` 同心环)与 dual(`led_{index}_{r}_{c}` 每壳子网格)是 panel 槽下的替代放置策略,不在此矩形计数轴内。

## 跨层接口(未来 InterfaceSpec 预填)
- mount ↔ head(俯仰轴):head 的 `pivot_boss_pos_y/neg_y`(Y 向钢 boss,`boss_y ≈ HEAD_W/2 + 0.004`)被 mount 顶部承托几何捕获——基线 `upright_*` 顶、aframe `pivot_yoke_*`、tripod `pivot_yoke_0/1`+`pivot_axle`、hook `yoke_arm_*`、telescope `inner_mast.upright_*`+`pivot_axle`。consumer joint = REVOLUTE about Y,origin `(0,0,PIVOT_Z)`。mating face = 立柱/yoke 顶内侧面,anchor = boss 轴心。`PIVOT_Z` 随 mount 变(H-frame `UPRIGHT_H`=0.150;aframe 0.200;tripod `MAST_H`=0.175;hook `BASE_TOP_Z+YOKE_H`=0.200;telescope 经 mast 局部 `PIVOT_LOCAL_Z`)。
- 多关节 mount/head 在 root(`stand_frame`/`handheld_base`)与 `light_head` 之间插入中间 part(`u_yoke`/`inner_mast`/`folding_leg_*`/`hanging_hook`),并把头俯仰铰**改挂**到中间件(`yoke_to_head`/`mast_to_head`),而非直接 `stand_to_head`——模板需把"头铰的 parent"作为可被 head-slot 改写的接口端。
- panel ↔ head 壳:`led_glass_panel`/`round_glass_panel` 与 `bezel_*`/`ring_bezel` 座入头壳前 recess,玻璃塞进 bezel 唇下(不漂浮);LED 阵列贴玻璃前微凸。mating face = 头壳前开口,anchor = 面中心。

## 排除项(未来 compatibility matrix 素材)
- P0 规划阶段无不收敛取值(9 个 fork 均编译通过、含 ≥1 非 fixed joint、collections=['workbench'])。
- 已知需 matrix 裁决、未在本批跨格组合的项:dual_flood_bar × telescope_tilt(双头质量 + 升降臂力臂,潜在 CoM/穿插)、cob_round_disc 与 dual_flood_bar 互斥(同为 panel 载体形态,不可叠);head 槽改写(tilt_pan / telescope)会**移除头部 `battery_pack`/`battery_port_panel`**,故 panel/led_count 与某些 head 候选并存时需重新安置后部电池furniture——matrix 应记录哪些 panel 形态与哪些 mount/head 兼容。

---
## Post-fork verification (SEGMENT 1 complete)
All 9 planned variants forked from f7e038e0 via `articraft fork`, then verified on-disk: last compile = success, ≥1 non-fixed joint present (mount/head 变体新增 REVOLUTE/PRISMATIC;panel/led_count 变体保留 `stand_to_head` 俯仰铰), collections=['workbench'](workbench-only, not promoted), and picture.json bound into the `Equipment__LED_work_light` subcat shard (reconcile rebuilt). Parent confirmed as the copy-logic source for `led_count` (LED array already loop-emitted `led_{r}_{c}`, no rewrite). Status cells above flipped planned→built ✓ accordingly. Ready for SEGMENT 2 (spec authoring).
