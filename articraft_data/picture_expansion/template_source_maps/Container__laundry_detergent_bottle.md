# Container / laundry detergent bottle — template source map

pattern: parallel_children

parents:
- rec_orange-tide-liquid-laundry-detergent-bottle-with_20260606_075035_179705_8f73f587 ← picture/Container/laundry detergent bottle/001.png ；占格子 (Slot A = flat_oval_jug) × (Slot B = side_loop_handle) × (Slot C = measuring_cup_cap)

母资产形态(读码确认):
- 功能层 = body(root，光面橙色 HDPE 中空壳：rounded-rect 截面竖向 loft 身 + 内缩肩 + 螺纹颈 + 沿瓶轴的 mouth bore；融入 +X 侧整体环形提手 `_handle_solid`,带椭圆穿透抠手孔;前 -Y 面 Tide bullseye/label_band/he_badge 内联 visual;FIXED pour_spout) + cap_carrier(无质量解耦载体,无 visual) + cap(蓝色半透明量杯盖,tapered hollow cup + 偏轴 marker tab)。
- 活动关节两根,经 carrier 解耦共用颈顶 +Z 轴:`cap_rotate`(CONTINUOUS,body→carrier 绕 +Z 旋)+ `cap_slide`(PRISMATIC,carrier→cap 沿 +Z 直拔)。
- 无 `for i in range(n)` 复制层——洗衣液瓶本身不含 N 个同构子件,不存在 multiplicity 轴。
- §4 可读性契约:命名按功能层(jug_shell / pour_spout / bullseye_label / label_band / he_badge / cap_cup / cap_marker),曲面用 loft + fillet,label/badge 内联为 parent.visual,无 FIXED-joint 装饰件,joint 锚在真实颈顶 seat 面(CAP_SEAT_Z)→ 达标,无需在变体 prompt 里附加"重写为循环"要求。

## Slot 候选覆盖

### Slot A:body_shape(瓶身轮廓 / 截面形状家族)
| 候选(未来 module) | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| flat_oval_jug | rec_orange-tide-liquid-laundry-detergent-bottle-with_20260606_075035_179705_8f73f587 | body / _body_shell / jug_shell | rounded-rect 扁椭圆截面竖向 loft 身 + 内缩肩,广面朝 ±Y | converged(parent) |
| round_cylinder | rec_container_laundry_detergent_bottle_var_cyl_body | body / _body_shell(圆截面 loft/lathe) / jug_shell | 高身圆柱截面瓶,同等容积,圆肩收颈 | converged |
| square_shoulder | rec_container_laundry_detergent_bottle_var_square_body | body / _body_shell(矩形截面) / jug_shell | 方截面矩形棱柱身,平广面 + 近竖侧壁 + 方肩台阶收颈 | converged |
| waisted_contour | rec_container_laundry_detergent_bottle_var_waist_body | body / _body_shell(中段收腰 loft) / jug_shell | 中部内凹收腰握位,底/肩宽中段窄,人体工学轮廓 | converged |

### Slot B:handle_grip(提手 / 握持机构)
| 候选(未来 module) | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| side_loop_handle | rec_orange-tide-liquid-laundry-detergent-bottle-with_20260606_075035_179705_8f73f587 | body / _handle_solid / jug_shell | +X 侧整体环形提手,板坯 + 椭圆穿透抠手孔,外握杆封闭成环 | converged(parent) |
| recessed_grip | rec_container_laundry_detergent_bottle_var_grip_indent | body / _handle_solid(凹槽 cut) / jug_shell | 广面压出凹形手指 scoop,壁连续无穿孔,模制抠手凹位 | converged |
| top_carry_handle | rec_container_laundry_detergent_bottle_var_top_handle | body / _handle_solid(倒 U 拱) / jug_shell | 跨肩倒 U 拱形顶提梁,从顶部提携 | converged |

### Slot C:closure(封口 / 分配机构)
| 候选(未来 module) | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| measuring_cup_cap | rec_orange-tide-liquid-laundry-detergent-bottle-with_20260606_075035_179705_8f73f587 | cap / cap_carrier / cap_rotate(continuous z) / cap_slide(prismatic z) / _cap_mesh | 半透明量杯盖,经 carrier 解耦,绕 +Z 旋拧 + 沿 +Z 直拔两根关节 | converged(parent) |
| flip_top_cap | rec_container_laundry_detergent_bottle_var_flip_cap | flip_lid / neck_to_lid(revolute 水平轴) / _lid_mesh | 颈领上活动铰 snap 翻盖,绕水平轴 revolute 开合 | converged |
| pump_dispenser | rec_container_laundry_detergent_bottle_var_pump_cap | pump_head / body_to_pump(prismatic z) / dip_tube / _pump_mesh | 螺纹领泵头 + dip tube,actuator 沿 +Z 直压 prismatic 分配 | converged |

## Multiplicity / Copy Logic
- count_param: 无,核心结构为固定 named slots(body / handle / closure)。洗衣液瓶不含 N 个同构子件(无抽屉/叶片/链节),不存在复制逻辑。
- N 样本已覆盖: 无
- 模板建议 N_range: 无(N/A)
- copied object / naming / placement / joint policy: 无

## 组合数预审
组合数预审: Π(Slot A 4 × Slot B 3 × Slot C 3) × N(无,记 1) = 36 ≥ 10 ✓

## 排除项(未来 compatibility matrix 素材)
- 无 multiplicity 轴(本小类无同构复制子件),不计入排除——属结构本质,非阻塞。
- top_carry_handle × pump_dispenser 实物上罕见且顶提梁会与泵头 actuator 抢占颈顶空间(潜在干涉),留作未来 compatibility matrix 裁决项;变体批不专门造该跨轴组合(组合由模板采样器产出)。
