# Urban Environment / Trashcan2 — template source map

slug: `trashcan2` · shard: `Trashcan2` · picdir: `picture/Urban Environment/Trashcan2`

pattern: parallel_children(固定 named slots:body_shape + lid_mechanism(主机构,SWING REVOLUTE)+ mount + inner_liner;无 multiplicity except hex 6-panel / liner)

identity: a PUBLIC / STREET trash can whose DEFINING joint is a SWING lid/flap (REVOLUTE). 所有变体必须保留这个 swing REVOLUTE 主机构,并继续读作街道/公共垃圾桶。

## CRITICAL GOTCHA — swing flap must NOT seal the trash mouth

来自既往 swing-lid 工作(swing-lid-opening-seal bug):swing flap/lid 背后的投放口必须是**真实开放的空腔**。三个 parent 已正确处理,fork 时必须复刻:
- 圆顶/dome 顶:`CylinderGeometry(..., closed=False)` 的 skirt,**不要**用 closed=True 的端盖 disc 把口封死 (parent1 `_lid_mesh` skirt 注释)。
- 平面/坡面 flap 开口:背面只画**四条 border strip**(围着洞),**绝不**画一整块 back-face quad(parent2 `_roof_mesh` +X 面注释)。
- frustum/hood 面:在 flap 那一面**跳过/切掉** outer+inner 面,留实洞(parent3 `_hood_shell` 跳过 front -Y 面 `if i == 0: continue`)。

每个 swing-lid 变体 prompt 都已 VERBATIM 写入该要求。

## 组合数预审 (HARD GATE)

slots:body_shape(3:round / rectangular / square)× lid_mechanism(4:top_swing_flap_rocker / front_swing_door / dome_push_flap / open_hooded_top)× mount(3:free_standing / post_mounted / wall_hoop)× inner_liner(2:none / removable_liner)。
- 全组合 = 3 × 4 × 3 × 2 = **72 distinct cells ≫ 10** ✓
- product(candidates) × distinct-N >= 10:**SATISFIED**

## parents (pre-fill cells,现成)

- rec_small-cylindrical-black-plastic-swing-lid-trash-_20260608_164713_118294_d5a40ab1 ← Trashcan2/003.png(round 圆柱体 × top swing-flap rocker(teardrop,REVOLUTE Y)× free_standing × no_liner;`lid_to_flap` REVOLUTE;LatheGeometry shell + dome mesh)
- rec_blue-rectangular-swing-flap-trash-bin-with-a-gra_20260608_164656_369383_6e382be6 ← Trashcan2/002.png(rectangular 矩形体 × gable 坡顶 top swing-flap(square,REVOLUTE Y)× free_standing × no_liner;`roof_to_flap` REVOLUTE)
- rec_square-green-painted-steel-street-trash-can-with_20260608_164550_287081_e7cea0e6 ← Trashcan2/001.png(square 方体 × pyramidal hood "PUSH" 顶 swing-flap(顶边铰,REVOLUTE X)× free_standing(corner posts + 4 feet)× no_liner;`body_to_push_flap` REVOLUTE;CadQuery body + mesh hood)

3 parent 覆盖了 body_shape 全部 3 候选 + lid_mechanism 的 top_swing_flap_rocker 候选 + mount 的 free_standing 候选 + inner_liner 的 none 候选。新变体补齐其余候选。

## Slot 候选覆盖

### Slot A:lid_mechanism（主机构槽 —— SWING 开合动作，保留 REVOLUTE）
| 候选 | variant / parent | 关键 joint / 结构 | 状态 |
|---|---|---|---|
| top_swing_flap_rocker（基线） | parent1/2/3 | 顶/坡面 rocker swing flap REVOLUTE | parent(现成) |
| front_swing_door | rec_trashcan2_var_lidfrontdoor | 上前壁矩形 hatch,门顶边铰 REVOLUTE 外掀;顶固定;口为实洞 | converged |
| dome_push_flap | rec_trashcan2_var_liddomepush | 圆顶 crown 中嵌小圆 push flap rocker REVOLUTE;dome 口切实洞 | converged |
| open_hooded_top | rec_trashcan2_var_lidopenhood | 固定 hood/canopy 留侧开口 + 盖该开口的 swing flap REVOLUTE | converged |

### Slot B:body_shape（体形;连续尺寸由模板侧缩放,这里只列结构形态）
| 候选 | variant / parent | 结构特征 | 状态 |
|---|---|---|---|
| round（基线） | parent1 | 锥度圆柱 LatheGeometry shell | parent |
| rectangular（基线） | parent2 | 锥度矩形四壁 shell | parent |
| square（基线） | parent3 | 方体 CadQuery + corner posts + feet | parent |
| round_drum + front_door | rec_trashcan2_var_bodydrumdoor | 直壁圆桶 drum + 曲面前 swing-door hatch REVOLUTE | converged |
| hexagonal | rec_trashcan2_var_bodyhex | 六面棱柱(for-range(6) 面)+ hex hood + 顶 swing flap REVOLUTE | converged |

### Slot C:mount（落地 / 安装方式）
| 候选 | variant / parent | 结构特征 | 状态 |
|---|---|---|---|
| free_standing（基线） | parent1/2/3 | 直接落地(parent3 带 4 feet) | parent |
| post_mounted | rec_trashcan2_var_postmount | 立柱 pole + 地面 base plate + cradle bracket 托起,桶悬空;保留 swing flap | converged |
| wall_hoop | rec_trashcan2_var_wallhoop | 墙面平板 + 环箍 hoop ring 抱桶,无腿;保留 swing flap | converged |

### Slot D:inner_liner（内胆机构）
| 候选 | variant / parent | 关键 joint / 结构 | 状态 |
|---|---|---|---|
| none（基线） | parent1/2/3 | 无内胆 | parent |
| removable_liner | rec_trashcan2_var_innerliner | 嵌套内桶,竖直 PRISMATIC 提拉取出;外桶保留 swing flap REVOLUTE(双 joint) | converged |

## loop emission notes

- parent1(round):mesh dome/flap 全用 `for i in range(rings)` / `for j in range(seg)` 环网格 + helper(`_body_shell` / `_lid_mesh` / `_flap_mesh`),无手写重复零件 — 干净。
- parent2(rect):四壁 `quad()` helper 手写四次(+X/-X/+Y/-Y),eave skirt 用 `for (xs,ys)` / `for ys` 小循环;四壁可改 for-loop 但量小,可接受。
- parent3(square):corner posts + feet 用 `for sx,sy,nm in (...)` 循环 + `_corner_post`/`_foot` helper(含内层 `for k in range(n_riv)` rivet 循环),hood 面 `for i in range(4)` — 良好的 loop emission 范例,fork 应沿用。
- 新变体要求:hex(`for i in range(6)` 面板)、postmount/wallhoop 的 bolts、front-door 的 hinge knuckles 均须 for-i-in-range + name_i + shared helper。

## dropped axes

- color / material / 纯 scale:按规则排除(suffix 明说 material 不算 change)。
- "tipping / wheeled / pedal-foot" 等机构:超出 Trashcan2 街道桶 identity(更像 Garbage_bin / Large_Trashcan 邻类),不纳入以保持 swing-lid 身份纯净。
