# Container / Can — template source map

pattern: parallel_children

parents:
- rec_round-metal-canister-tin-with-a-lift-off-lid_20260606_074656_096323_fb4296aa ← picture/Container/Can/006.png ；占格子 (Slot A = round_cylinder) × (Slot B = lift_off_lid)
- rec_clear-round-plastic-deli-tub-food-container-with_20260606_074616_834085_6b6cb24d ← picture/Container/Can/002.png ；同占格子 (round_cylinder × lift_off_lid)，与 006 为同格收敛冗余(锥壁/卷边变体口味，结构格相同)
- rec_square-metal-tin-box-with-a-press-fit-lift-off-l_20260606_074633_504919_c6ba1d09 ← picture/Container/Can/004.png ；占格子 (Slot A = rounded_square) × (Slot B = lift_off_lid)
- rec_clear-plastic-square-food-storage-container-with_20260606_074608_787060_926ea5c0 ← picture/Container/Can/001.png ；同占格子 (rounded_square × lift_off_lid)，与 004 为同格收敛冗余(透明扁身食盒口味)
- rec_hexagonal-metal-tea-tin-with-a-lift-off-lid_20260606_074647_744909_f4f34d34 ← picture/Container/Can/005.png ；占格子 (Slot A = hex_prism) × (Slot B = lift_off_lid)
- rec_metal-rectangular-fuel-flask-with-a-round-screw-_20260606_074624_981526_6e35123f ← picture/Container/Can/003.png ；占格子 (Slot A = flat_rect_flask) × (Slot B = screw_cap)

母资产形态(读码确认):
- 共同功能层 = body(root，中空壳，floor + 四壁/筒壁，开口在 +Z) + closure(lid/cap，盖住口部)。这是 body + closure 的 parallel_children 结构;closure 是唯一(或主要)活动关节。
- closure 关节类型现状:5 个 lift-off 母资产用 PRISMATIC(沿 +Z 直拔，盖裙裹住口沿:`body_to_lid` / `tub_to_lid` / `tin_to_lid`);唯一的螺纹封口在 fuel flask,用 `cap_rotate`(CONTINUOUS) + `cap_slide`(PRISMATIC)经 massless `cap_carrier` 解耦旋拧+提起,另带 `bail_flip`(REVOLUTE)折叠提环。
- 无 `for i in range(n)` 复制层——罐/盒本身不含 N 个同构子件,本小类不存在 multiplicity 轴(参见 §Multiplicity)。
- §4 可读性契约:6 个母资产均按功能层命名(body/tub/tin_body + lid/cap + helper `_body_solid`/`_tub_solid`/`_hex_body`/`_lid_solid`/`_hex_lid`/`_cap_mesh`),曲面用 CadQuery loft / extrude / polygon、cap 用 CylinderGeometry+Torus mesh,装饰(label/marker)内联为 parent.visual,joint 锚在真实口沿/颈面 → 全部达标,无需在变体 prompt 里附加"重写为循环"要求。

## Slot 候选覆盖

### Slot A:body_shape(罐身截面 / 形状家族)
| 候选(未来 module) | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| round_cylinder | rec_round-metal-canister-tin-with-a-lift-off-lid_20260606_074656_096323_fb4296aa | body / _body_solid / body_to_lid | 中空圆柱筒身,直壁,开口圆口沿(deli tub 002 为同格锥壁/卷边口味) | converged(parent) |
| rounded_square | rec_square-metal-tin-box-with-a-press-fit-lift-off-l_20260606_074633_504919_c6ba1d09 | tin_body / _rounded_square_prism / _body_solid | 圆角方截面中空壳 + 内缩 rabbet 口沿(食盒 001 为同格透明扁身口味) | converged(parent) |
| hex_prism | rec_hexagonal-metal-tea-tin-with-a-lift-off-lid_20260606_074647_744909_f4f34d34 | body / _hex_body(polygon 6) | 正六棱柱中空身,across-flats 截面,floor 封底开口顶 | converged(parent) |
| flat_rect_flask | rec_metal-rectangular-fuel-flask-with-a-round-screw-_20260606_074624_981526_6e35123f | body / _flask_body(filleted box) | 扁矩形 hip-flask 身,宽>深,圆角竖边+软化顶底边 | converged(parent) |

### Slot B:closure(封口 / 盖机构 joint 类型)
| 候选(未来 module) | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| lift_off_lid | rec_round-metal-canister-tin-with-a-lift-off-lid_20260606_074656_096323_fb4296aa | lid / body_to_lid(prismatic z) / _lid_solid | 平顶盖+下垂裙圈裹口沿,沿 +Z 直拔(press-fit / 套盖) | converged(parent) |
| screw_cap | rec_metal-rectangular-fuel-flask-with-a-round-screw-_20260606_074624_981526_6e35123f | cap / cap_carrier / cap_rotate(continuous z) / cap_slide(prismatic z) / _cap_mesh | 螺纹颈 + 旋拧螺盖,绕轴 continuous 拧开再 prismatic 提起,massless carrier 解耦 | converged(parent) |
| hinge_lid | rec_container_can_var_round_hingelid | lid / body_to_lid(revolute, 水平铰轴) | 盖铰接在后口沿,绕水平边轴翻开上掀(REVOLUTE),非直拔 | converged |

#### Slot B 跨形状空格(变体填,单轴控制:只换 closure,身形恒为各自最近 parent)
| 候选格 | record_id | parent(最近) | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| round_cylinder × screw_cap | rec_container_can_var_round_screwcap | rec_round-metal-canister-tin-with-a-lift-off-lid_20260606_074656_096323_fb4296aa | cap / cap_carrier / cap_rotate(continuous) / cap_slide(prismatic) | 圆筒身换螺纹颈+旋盖,旋拧+提起 | converged |
| round_cylinder × hinge_lid | rec_container_can_var_round_hingelid | rec_round-metal-canister-tin-with-a-lift-off-lid_20260606_074656_096323_fb4296aa | lid / body_to_lid(revolute) | 圆筒身换后铰翻盖,绕水平轴掀开 | converged |
| rounded_square × screw_cap | rec_container_can_var_square_screwcap | rec_square-metal-tin-box-with-a-press-fit-lift-off-l_20260606_074633_504919_c6ba1d09 | cap / cap_carrier / cap_rotate(continuous) / cap_slide(prismatic) | 方身顶面中心立螺纹颈+旋盖 | converged |
| rounded_square × hinge_lid | rec_container_can_var_square_hingelid | rec_square-metal-tin-box-with-a-press-fit-lift-off-l_20260606_074633_504919_c6ba1d09 | lid / tin_to_lid(revolute) | 方身换后铰翻盖,绕后顶边轴掀开 | converged |
| hex_prism × screw_cap | rec_container_can_var_hex_screwcap | rec_hexagonal-metal-tea-tin-with-a-lift-off-lid_20260606_074647_744909_f4f34d34 | cap / cap_carrier / cap_rotate(continuous) / cap_slide(prismatic) | 六棱身顶中心立螺纹颈+旋盖 | converged |
| hex_prism × hinge_lid | rec_container_can_var_hex_hingelid | rec_hexagonal-metal-tea-tin-with-a-lift-off-lid_20260606_074647_744909_f4f34d34 | lid / body_to_lid(revolute) | 六棱身换后铰翻盖,绕一条后顶边轴掀开 | converged |
| flat_rect_flask × lift_off_lid | rec_container_can_var_flatrect_liftofflid | rec_metal-rectangular-fuel-flask-with-a-round-screw-_20260606_074624_981526_6e35123f | lid / body_to_lid(prismatic z) / _lid_solid | 扁矩身去掉颈/螺盖/提环,改全宽直拔套盖 | converged |
| flat_rect_flask × hinge_lid | rec_container_can_var_flatrect_hingelid | rec_metal-rectangular-fuel-flask-with-a-round-screw-_20260606_074624_981526_6e35123f | lid / body_to_lid(revolute) | 扁矩身去掉颈/螺盖/提环,改后铰翻盖 | converged |

注:每个变体单轴控制——身形恒为各自最近 parent,只替换 closure 机构(或反之)。Slot A 四种身形全部由 parent 免费占据(无需 A 行变体);Slot B 三种封口里 lift_off 与 screw_cap 已有 parent,hinge_lid 为全新候选。4×3 网格 12 格中,4 格被 parent 占(round/square/hex × lift_off + flat_rect × screw),其余 8 个空格各造 1 变体。

## Multiplicity / Copy Logic
- count_param: 无,核心结构为固定 named slots(body 单件 + closure 单件;螺盖的 knurl ridge 已是 parent 内部 `for i in range(n)` 装饰循环,非小类级结构 multiplicity 轴)。
- N 样本已覆盖: 无(此小类无结构复制逻辑轴)。
- 模板建议 N_range: 无。
- copied object / naming / placement / joint policy: 无。

## 组合数预审
组合数预审: Π(body_shape=4, closure=3) × N(无,=1) = 12 ≥ 10 ✓

## 排除项(未来 compatibility matrix 素材)
- 暂无连续不收敛项(规划阶段未 fork)。潜在风险候选待 fork 时观察:
  - 跨轴组合(如 hex_prism × screw_cap 之外的所有 A×B 笛卡尔积全集)不在本批覆盖——本批只造单轴控制变体,组合留给模板采样器免费产出。
  - hinge_lid 在 hex_prism / flat_rect_flask 上的铰轴需锚在真实后顶边(六棱选一条后 flat 边、扁矩选后长边),fork 时注意 joint origin 落在实际口沿面而非 mm 级锚垫;翻开 90° 时盖与身可能在铰侧轻微 overlap,需复制 parent 的 allow_overlap 习惯。
  - screw_cap 移植到方/六棱身时,颈为圆形而身为多边/方截面,颈基座坐落在平顶面中心是合法的;若未来要求颈与身同形(方颈/六棱颈)则属另一候选,不在本批。
  - flat_rect_flask 的 bail_flip 提环层在 lift_off / hinge 变体里被显式移除(prompt 已指明),不保留为悬挂装饰件。
