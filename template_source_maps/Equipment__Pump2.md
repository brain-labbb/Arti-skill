# Equipment / Pump2 — template source map

pattern: parallel_children(handle + base + outlet 三个结构槽挂在同一刚体上;无 parent 级 ×N multiplicity)

slug: manual_barrel_hand_pump

parents:
- rec_build-a-realistic-articulated-3d-model-of-a-pump_20260609_180115_462868_621823e2 ← picture/Equipment/Pump2/001.png(手动桶式活塞泵:铸件 base_fitting + yoke 夹座承托竖直 barrel;piston 杆经 cap gland 上伸、顶端 ball_knob,向下 PRISMATIC 打气;black foot lever 绕 yoke 销 REVOLUTE 摇摆;base 侧 outlet stub 接一根 loose rubber hose 落地)→ 同时填 Slot A=ball_knob_plunger、Slot B=cast_hex_foot、Slot C=loose_rubber_hose 三个 baseline。

注:parent 为**全命名 singleton**(除 `cap` 的 `n_flutes` 装饰滚花循环外无结构复制),三个结构槽都是固定 named 部件。`base_tripod` 变体**主动引入** `leg_{i}` 循环作为 copy-logic 样本(见 Multiplicity 节)。全部 9 变体 forked from 621823e2。

## Slot 候选覆盖

### Slot A:handle / 顶部柱塞抓握(挂在 `piston` 部件,经 `barrel_to_piston` PRISMATIC 主关节驱动)
| 候选(未来 module) | record_id | 关键 part/joint/helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| ball_knob_plunger(基线) | rec_..._621823e2(parent) | `piston` 部件 `ball_knob`(Sphere)+ `rod`(Cylinder)+ `piston_head`(`_piston_head_shape`);`barrel_to_piston` PRISMATIC(axis 0,0,-1,upper=STROKE) | 大号黑色实心球柄压在杆顶 | converged(parent) |
| handle_tbar | rec_pump_var_handle_tbar | `t_bar_grip`(helper `_t_bar_grip_shape`):横向 crossbar + 两端 end_collar + 向下 socket 套住 rod | 水平 T 型横握把,代替球柄 | built ✓ |
| handle_dloop | rec_pump_var_handle_dloop | `pull_loop`(helper `_pull_loop_geometry`,closed Catmull-Rom 弯管)+ `loop_socket`(Cylinder) | 封闭 D 形提拉环,中间留手孔 | built ✓ |
| handle_palmdisc | rec_pump_var_handle_palmdisc | `push_disc`(helper `_push_disc_geometry` → LatheGeometry) | 车削掌推圆盘,掌压式握面 | built ✓ |

### Slot B:base / 底座支承(`pump_body` 根刚体上的承托脚)
| 候选(未来 module) | record_id | 关键 part/joint/helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| cast_hex_foot(基线) | rec_..._621823e2(parent) | `base_fitting`(`_base_fitting_shape`,polygon(6) foot flange + neck)+ `yoke`(`_yoke_shape`) | 铸件六角脚盘 + 夹座,直接坐地 | converged(parent) |
| base_flange | rec_pump_var_base_flange | `_flange_bolt_shape`;`flange_bolt_{i}` 视觉循环 `for i in range(4)`(角度 2πi/4,FLANGE_BOLT_CIRCLE_R) | 法兰圆盘底座 + 4 颗螺栓圈,落地固定 | built ✓ |
| base_tripod | rec_pump_var_base_tripod | `_tripod_leg_shape(angle)`;`leg_{i}` 视觉循环 `for i in range(3)`(角度 120°)| 三脚撑架,三条铸腿等分外撑落地 | built ✓(引入 leg_{i} 循环) |
| base_wallbracket | rec_pump_var_base_wallbracket | `back_plate`(`_back_plate_shape`)+ `saddle_clamp_band`(`_saddle_clamp_band_shape`)+ `_fastener_head_shape` | 墙挂背板 + 抱箍鞍座,环抱 barrel 贴墙 | built ✓ |

### Slot C:outlet / 出水口(挂在 base 侧 `_base_fitting_shape` 的 outlet stub,位于 (0,-0.038,0.030))
| 候选(未来 module) | record_id | 关键 part/joint/helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| loose_rubber_hose(基线) | rec_..._621823e2(parent) | `hose`(`tube_from_spline_points`,9 控制点,free end 落地) | 黑橡胶软管绕泵一圈、末端松落地面 | converged(parent) |
| outlet_gooseneck | rec_pump_var_outlet_gooseneck | `spout_socket`(`_spout_socket_shape`)+ `gooseneck_spout` + `downturned_nozzle`(`_downturned_nozzle_shape`) | 刚性鹅颈弯管喷口,末端下弯 | built ✓ |
| outlet_barb | rec_pump_var_outlet_barb | `barbed_nipple`(`_barbed_nipple_shape`,`for i in range(3)` 倒刺环) | 直插带倒刺接管嘴 | built ✓ |
| outlet_tapvalve | rec_pump_var_outlet_tapvalve | `tap_body`(`_tap_body_shape`,挂 body)+ `tap_handle` 部件(`_tap_handle_shape`);新增关节 `tap_to_handle` REVOLUTE(axis 0,0,1,lower=0 upper=π/2) | 带阀体的出口 + 可转 tap 手柄(第三个非固定关节) | built ✓ |

注:三个 Slot 的 baseline 都由唯一 parent 覆盖,本批每个变体各补一格;Slot C 的 tapvalve 是唯一在三槽之外额外引入新铰接关节的候选。

## Multiplicity / Copy Logic
- count_param:**本小类核心结构为固定 named slots,无 parent 级 ×N 复制逻辑。** 主关节是 `piston` 的 `barrel_to_piston` PRISMATIC 打气冲程(唯一 hero 真关节),次关节 `yoke_to_lever` REVOLUTE 摇杆,均为单实例。
- 局部 copy-logic 样本(由 `base_tripod` 变体演示):
  - count_param: `leg_count`
  - copied object: 单条三脚撑腿(helper `_tripod_leg_shape(angle)` 发射的一条铸腿)
  - naming: `leg_{i}`,循环 `for i in range(3)`
  - placement: 角向等分,`angle = 2.0 * pi * i / 3.0`(120° 间隔),脚底落在 workbench(z≈0)
  - joint policy: **FIXED** —— 三条腿都是 `pump_body` 根刚体上的 visual,不带独立关节(撑脚是静态承托,非活动件)
  - 模板建议 N_range: **[3, 6]**(三脚架物理下界=3;采样到 4/5/6 脚泛化为多脚底盘,留小余量)
- 次级 copy-logic 样本(`base_flange` 变体):`flange_bolt_{i}` 螺栓圈 `for i in range(4)`(角度 2πi/4),同为 FIXED 装饰/紧固件复制,可作 bolt-circle 采样参数 `bolt_count`(建议 [3, 8])。
- 备注:两处循环都纯 `for i in range(N)` + 角向公式发射,copy logic 一眼可读,可直接作 module 源码;其余槽位与 baseline 仍是手写命名 singleton。

## 跨层接口(未来 InterfaceSpec 预填)
- handle ↔ piston rod:所有 handle 候选(球柄/T 把/D 环/掌盘)都用一个向下 socket/collar 套住 `rod` 顶端,统一挂在 `piston` 部件;consumer joint = `barrel_to_piston` PRISMATIC(origin (0,0,0),axis 0,0,-1,upper=STROKE),mating face = 杆顶轴线,anchor = (0,0,rod_top)。
- base ↔ 地面/墙:base 候选(hex foot / flange / tripod / wallbracket)都是 `pump_body` 根刚体的承托结构,落地式脚底 z≈0;wallbracket 例外为贴墙抱箍(`saddle_clamp_band` 环抱 barrel footprint)。base 与 barrel 经 `_base_fitting_shape` neck 正向就位。
- outlet ↔ base_fitting outlet stub:spout/nipple/tap 的进口套接在 `_base_fitting_shape` 侧向 outlet stub(沿 -Y,中心 (0,-0.038,0.030));baseline 为软管,gooseneck/barb 为刚性固定附件;tapvalve 额外引出 `tap_handle` 部件,consumer joint = `tap_to_handle` REVOLUTE(origin (0,TAP_Y,TAP_BOSS_TOP_Z),axis 0,0,1,limits [0, π/2])。
- lever ↔ yoke pin(全候选共享,未改):`yoke_to_lever` REVOLUTE,origin (CLEVIS_X,0,YOKE_PIN_Z),axis 0,1,0,limits [-0.5,0.5];lever hub bore 抱住 yoke pin(allow_overlap capture)。

## 排除项(未来 compatibility matrix 素材)
- 暂无不收敛取值(全部 9 变体均 compile success)。
- 已主动排除(出类目/跨格冲突风险,未列为候选):把 handle 换成电动马达驱动(读作电泵,失去手动柱塞 hero 关节);wallbracket base × loose_rubber_hose outlet 的落地软管几何会与贴墙姿态冲突(软管末端"落地"约束在墙挂下不成立),该 Slot B×Slot C 跨格组合留给模板 compatibility matrix 裁决,不在 fork 批造。

---
## Post-fork verification (SEGMENT 1 complete)
All 9 planned variants forked from 621823e2 via `articraft fork`, then verified on-disk: last compile = success, ≥1 non-fixed joint present(全部保留 `barrel_to_piston` PRISMATIC + `yoke_to_lever` REVOLUTE;tapvalve 另加 `tap_to_handle` REVOLUTE),collections=['workbench'](workbench-only,未 promote),picture.json 绑入 `Equipment__Pump2` 子类 shard(reconcile 重建)。上表状态格 planned→built ✓ 已翻转。Ready for SEGMENT 2 (spec authoring).
