# Machinery / Watermill — template source map

pattern: mixed(wheel_type 槽 + mount 槽 + spokes 槽 + paddle_count 链式 multiplicity)

slug: watermill_waterwheel

parents:
- rec_model-a-stylized-wooden-watermill-waterwheel-mou_20260610_081149_220030_afe3e6a1 ← picture/Machinery/Watermill/001.png(CadQuery 实体;`waterwheel` 上 `paddle_{pi}` 已循环发射;`trestle_frame`↔`waterwheel` 单条 CONTINUOUS hub spin;**fork 基线 + paddle_count copy-logic 源码**)

parent 基线即:`flat_paddle` 桨板 × `radial_spoke_bars`(3 根直径杆/侧 → 6 辐)× `trestle` A 字台架 × N=9(`PADDLE_COUNT=9`,`for pi in range(PADDLE_COUNT)` 角度等分发射,全部 FIXED 到轮上)。

## 组合数预审

3(wheel_type)× 3(mount)× 3(spokes)× 3(N 样本,保守)= 81 ≥ 10 ✓

## Slot 候选覆盖

### Slot A:wheel_type(轮缘被复制单元的几何样式——决定 `paddle_{pi}` 循环里发射什么)
| 候选(未来 module) | record_id | 关键 part/joint/helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| flat_paddle(基线) | rec_..._afe3e6a1 | `Box(PADDLE_DIMS)` → `paddle_{pi}`(`waterwheel` 上循环) | 平直桨板,切向贴轮缘,`rpy=(0,ang,0)` 随角对齐 | parent(现成) |
| enclosed_bucket(overshot) | rec_watermill_var_wheeltype_overshot | helper `_bucket_solid()` → `bucket_{pi}` 循环 | L 形封闭水斗槽,绕周长发射(顶进水式) | converged |
| angled_scoop(breastshot) | rec_watermill_var_wheeltype_breastshot | helper `_scoop_vane_geometry()` / `_scoop_vane_origin(angle)` → `paddle_{pi}` | 斜置约 30° 离径的斗形扇叶(腰进水式) | converged |

### Slot B:mount(静态地参——`...→waterwheel` joint 的 parent part)
| 候选(未来 module) | record_id | 关键 part/joint/helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| trestle_aframe(基线) | rec_..._afe3e6a1 | part `trestle_frame`;`leg_{fi}_{li}`/`foot_{fi}_{li}`/`cross_brace_{fi}`/`bearing_block_{fi}`/`diagonal_brace`;joint `trestle_to_waterwheel` | 自立 A 字木台架,双侧斜腿 + 顶端轴承块 | parent(现成) |
| millhouse_wall | rec_watermill_var_mount_millhouse | part `mill_wall`;`plank_wall`/`plank_seam_{si}`/`wall_batten_{bi}`/`bearing_bracket_{fi}`/`angle_gusset_{fi}`/`bearing_block_{fi}`/`bearing_bolt_{fi}_{zi}`;joint `wall_to_waterwheel` | 竖板墙立面 + 两根外伸轴承托架 | converged |
| masonry_sluice | rec_watermill_var_mount_sluice | part `sluice_mount`;`channel_floor`/`channel_wall_{wi}`/`bearing_pier_{wi}`/`bearing_block_{bi}`;joint `sluice_to_waterwheel` | 双侧渠墙 + 底板水槽 + 墙顶轴承墩,轮落槽内 | converged |

### Slot C:spokes(轮缘内辐条结构——`waterwheel` 内的固定填充)
| 候选(未来 module) | record_id | 关键 part/joint/helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| radial_spoke_bars(基线) | rec_..._afe3e6a1 | `spoke_bar_{ri}_{si}`(`SPOKE_BARS=3`,`rpy=(0,si*pi/3,0)`)+ `hub_{ri}` 圆柱毂 | 3 根直径杆/侧 → 6 直辐,汇于圆柱轮毂 | parent(现成) |
| clasp_compass_arm | rec_watermill_var_spokes_clasparm | helper `_segment_box_origin()`;`hub_box_{ri}` 方毂 + 成对 `clasp_arm_{ri}_{si}_{ai}` / `clasp_chord_{ri}_{si}` | 弦切落缘的抱箍/罗盘臂成对辐,框住方毂盒 | converged |
| solid_web_disc | rec_watermill_var_spokes_solidweb | helper `_web_disc_solid()` → `web_disc_{ri}` 网盘 + `hub_{ri}` | 实心圆腹板盘带中心毂孔,取代开放辐条 | converged |

注:三槽基线均由同一 parent 覆盖,变体各补两个候选 → 每槽 3 候选,无空格子。

## Multiplicity / Copy Logic
- count_param: `paddle_count`(parent 命名 `PADDLE_COUNT`)
- copied object: 单块桨板/水斗/扇叶 = 共享几何(`Box(PADDLE_DIMS)` 或 helper `_bucket_solid`/`_scoop_vane_geometry`),每个一条 `paddle_{pi}`(overshot 为 `bucket_{pi}`)visual,全部 **FIXED 到 `waterwheel`**(非独立 joint)
- naming: `paddle_{i}`(overshot 风格为 `bucket_{i}`),`for pi in range(PADDLE_COUNT)` 循环发射
- placement: 沿圆周角度等分,`ang = 2*pi*pi_i/PADDLE_COUNT`,`xyz=(R*sin(ang),0,R*cos(ang))`,`rpy=(0,ang,0)` 随角对齐;桨中心半径 `PADDLE_R`
- joint policy: **桨/斗本身无 joint(随轮刚体)**;整轮仅一条 hub 关节 `trestle_to_waterwheel`(mount 变体改首段名)= `ArticulationType.CONTINUOUS`,`parent=mount`,`child=waterwheel`,`origin z=AXLE_Z`,`axis=(0,1,0)`(水平 +Y 轴),`MotionLimits(effort=80, velocity=6)`;改 N 不增 joint
- N 样本已覆盖: {9(parent 基线), 12, 16} → rec_..._afe3e6a1 / rec_watermill_var_paddles_n12 / rec_watermill_var_paddles_n16
- 模板建议 N_range: **[6, 24]**(模板采样域;真实木水轮桨板常 8–20,留余量;采样建议中段加权,N 上限控编译时长)
- 备注:parent 已是干净的 `for pi in range(PADDLE_COUNT)` 循环发射,n12/n16 变体仅改 `PADDLE_COUNT` 常量上界 + 同一角度等分公式,copy logic 一眼可读,模板可直接以 parent 或任一 N 变体作 multiplicity 源码。

## 跨层接口(未来 InterfaceSpec 预填)
- wheel ↔ mount:`bearing_block_*` 的镗孔(`_bearing_block_solid` 沿局部 Y 的 `BORE_R` 通孔)= 轴颈承托面;consumer joint `<mount>_to_waterwheel` 原点贴轴心 `(0,0,AXLE_Z)`,axis = Y(轴线),`axle`/`axle_collar_{ci}` 故意 captured 在轴颈孔内(parent `allow_overlap` + `expect_overlap` 已规约,变体需沿用)。
- paddle/bucket/scoop ↔ wheel:挂在 `waterwheel` 缘上,中心半径 `PADDLE_R`,mating face = 轮缘外圈,anchor = 各桨 `(sin,cos)` 角向中心;随轮刚体,无独立 joint。
- spokes ↔ rim/hub:`spoke_bar`/`clasp_arm`/`web_disc` 落在 `rim_{ri}` 平面(`RIM_Y` 偏置)与 `hub_{ri}`/`hub_box_{ri}` 之间,承托面 = 轮毂外缘 + 轮缘内圈。

## 排除项(未来 compatibility matrix 素材)
- 暂无不收敛取值(8 个规划变体全部 converged)。
- 已主动排除(出类目风险,未列为候选):给每块桨板单独加 REVOLUTE/可折桨(读作可调距桨轮机构,偏离静态展示水轮,且与"整轮单 hub 自旋"的核心铰接冲突);把 mount 换成完整磨坊建筑外壳(喧宾夺主,读作 building 而非 watermill)。Slot A×Slot B×Slot C 的跨格组合(如 overshot+sluice+web)留给模板 compatibility matrix 裁决,不在 fork 批造。

---
## Post-fork verification (SEGMENT 1 complete)
All 8 planned variants forked from `afe3e6a1` and verified on-disk: last compile = success, `run_tests` 通过 baseline 门控,恰含 1 条非 fixed joint(`<mount>_to_waterwheel` CONTINUOUS hub spin)、桨/斗循环计数 = `PADDLE_COUNT`,collections=['workbench'](workbench-only,未 promote),picture.json 绑入 `Machinery__Watermill` subcat shard(reconcile 已重建)。Status cells above flipped planned→converged accordingly. Ready for SEGMENT 2 (spec authoring).
