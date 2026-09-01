# Container / Bottle serum — template source map

pattern: parallel_children

parents:
- rec_small-amber-glass-serum-bottle-with-a-white-rubb_20260606_074521_982612_98768519 ← picture/Container/Bottle serum/001.png ；占格子 (Slot A = round_cylinder) × (Slot B = dropper_prismatic)

母资产形态(读码确认):
- 功能层 = body(root，琥珀玻璃中空壳：直筒身 + loft 肩 + 颈 + 内 bore，附白色 label 纸带 visual) + dropper(单刚体：collar 套颈 + 球泡 bulb + bulb_stem + 透明 pipette)。
- 唯一活动关节 `body_to_dropper`(PRISMATIC，沿 +Z 直拉，pipette 拔出瓶口)。
- 无 `for i in range(n)` 复制层——血清瓶本身不含 N 个同构子件,不存在 multiplicity 轴。
- §4 可读性契约:命名按功能层(body_glass / label_band / collar / bulb / bulb_stem / pipette),曲面用 loft/Sphere,label 内联为 parent.visual,无 FIXED-joint 装饰件,joint 锚在真实颈面 → 达标,无需在变体 prompt 里附加"重写为循环"要求。

## Slot 候选覆盖

### Slot A:body_shape(瓶身轮廓 / 形状家族)
| 候选(未来 module) | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| round_cylinder | rec_small-amber-glass-serum-bottle-with-a-white-rubb_20260606_074521_982612_98768519 | body / _body_glass_mesh / label_band | 直筒圆柱身 + loft 圆肩 + 圆颈,内 bore 中空开口 | converged(parent) |
| tapered_cone | rec_container_bottle_serum_var_tapered_body | body / _body_glass_mesh(锥形 loft) | 底宽顶窄的圆锥收腰身,平滑过渡到肩颈 | converged |
| squared_faceted | rec_container_bottle_serum_var_squared_body | body / _body_glass_mesh(方截面) | 方形/矩形截面平面身 + 圆角竖边,升至肩 | converged |
| boston_round | rec_container_bottle_serum_var_boston_round_body | body / _body_glass_mesh(球肩 loft) | 高身 + 鼓圆下肩 + 明显收腰窄颈 | converged |
| tall_slim_vial | rec_container_bottle_serum_var_tall_slim_vial | body / _body_glass_mesh(高瘦直管) | 高纵横比细长直管身 + 极简肩直入颈(安瓿/试管式血清瓶) | converged |
| faceted_prism | rec_container_bottle_serum_var_faceted_prism | body / _body_glass_mesh(多棱柱截面) | 正多边形(六/八棱)平面棱柱身 + 棱面升至肩颈(区别于 4 面 squared) | converged |

### Slot B:closure(封口 / 分配机构)
| 候选(未来 module) | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| dropper_prismatic | rec_small-amber-glass-serum-bottle-with-a-white-rubb_20260606_074521_982612_98768519 | dropper / body_to_dropper(prismatic z) / _collar_mesh / pipette | 滴管:collar+球泡+pipette 单刚体,直拉拔出 | converged(parent) |
| screw_cap_revolute | rec_container_bottle_serum_var_screw_cap | cap / body_to_cap(revolute z) | 旋盖,绕瓶轴 revolute 拧开 | converged |
| pump_dispenser_prismatic | rec_container_bottle_serum_var_pump_dispenser | pump_head / body_to_pump(prismatic z) / dip_tube | 乳液泵头,直压 prismatic 下按 + 伸入瓶的 dip tube | converged |
| mist_sprayer_prismatic | rec_container_bottle_serum_var_mist_sprayer | spray_actuator / body_to_sprayer(prismatic z) / nozzle / dip_tube | 喷雾扁按钮直压 prismatic + 前置 nozzle + dip tube | converged |
| roller_ball_revolute | rec_container_bottle_serum_var_roller_ball | roller_ball / housing_to_ball(revolute/ball) / overcap | 滚珠涂抹封口:瓶口圆窝座捕获滚珠(原位滚动)+ 可摘外盖 | converged |
| brush_wand_prismatic | rec_container_bottle_serum_var_brush_wand | wand_cap / body_to_wand(prismatic z) / wand_stem / brush_tip | 唇彩式刷头/棒头盖:盖+长杆+刷头单刚体,直拉拔出(杆深入瓶) | converged |

注:每个变体单轴控制——A 列变体封口恒为 parent dropper,B 列变体瓶身恒为 parent round_cylinder。parent 自身免费占据 (round_cylinder × dropper_prismatic) 这一格,故只填 A 行其余 5 格 + B 列其余 5 格 = 10 个空格。本轮 deepen:A 列新增 tall_slim_vial / faceted_prism(各为真实、结构独立的血清瓶身轮廓家族);B 列(主机构槽,优先 deepen)新增 roller_ball / brush_wand(各为真实、结构独立的血清瓶封口/涂抹机构)。

## Multiplicity / Copy Logic
- count_param: 无,核心结构为固定 named slots(body 单件 + closure 单件,无 N 个同构子件)。
- N 样本已覆盖: 无(此小类无复制逻辑轴)。
- 模板建议 N_range: 无。
- copied object / naming / placement / joint policy: 无。

## 组合数预审
组合数预审: Π(body_shape=6, closure=6) × N(无,=1) = 36 ≥ 10 ✓

## 排除项(未来 compatibility matrix 素材)
- 暂无连续不收敛项(规划阶段未 fork)。潜在风险候选待 fork 时观察:
  - squared_faceted × pump_dispenser 等跨轴组合不在本批覆盖(只造单轴变体,组合留给模板采样器)。
  - mist_sprayer / pump 的 dip_tube 若与 squared/tapered 瓶身内 bore 干涉,可能在对应跨轴模板组合处触发(本批不造该组合,记此为未来 compatibility matrix 关注点)。
- deepen 评估时被否的候选(记录理由,避免重复评估):
  - airless treatment pump(气压式护理泵)— 否:可见机构与已有 pump_dispenser_prismatic 几乎相同(都是直压下按泵头),"airless" 差异在瓶内不可见的活塞/真空腔,非可见结构差异 → 近重复,不增轴。
  - 颈/领口形态(crimp-on collar vs threaded neck)作为第 3 轴 — 否:不是独立结构轴,颈口接口由封口选择(dropper/pump/cap/roller)耦合决定,无法与 closure 槽正交独立。
