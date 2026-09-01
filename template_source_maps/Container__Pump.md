# Container / Pump — template source map

pattern: parallel_children

(The object is a bottle root carrying two independent functional layers: a body
profile layer and a top dispenser-head/closure layer. There is no "N identical
sub-parts" copy logic — a single pump bottle has one body and one head.)

parents:
- rec_clear-soap-dispenser-bottle-with-a-white-press-d_20260606_074720_740450_74587dd4 ← picture/Container/Pump/001.png
  - occupies grid cell (Slot A = round_body, Slot B = press_pump).
  - NOTE: this is the only reference image and the only parent for this 小类.
    The image shows an opaque-white body with a clear over-cap and white press
    pump; the parent model.py renders the body as a clear lathe shell — a
    color/material difference only, not a structural one, so it does not change
    the axis grid.

## Slot 候选覆盖

### Slot A:body_profile (瓶身轮廓 / footprint shape family)
Functional layer = the lathe/mesh body shell (parent: `bottle` part, `_bottle_mesh`
helper, `bottle_shell` visual; label inlined as `label_band` visual). Varying the
cross-section / silhouette family, NOT the scale.

| 候选(未来 module) | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| round_body | rec_clear-soap-dispenser-bottle-with-a-white-press-d_20260606_074720_740450_74587dd4 | bottle / _bottle_mesh (LatheGeometry.from_shell_profiles) / bottle_shell | 圆截面直筒身 + 肩 + 颈,旋转体壳 | converged(parent) |
| boxy_oval | rec_container_pump_var_boxy_oval | bottle / boxy/oval body helper / bottle_shell | 圆角矩形(扁椭圆)截面瓶身,非旋转体,需 CadQuery/mesh 拉伸壳 | converged |
| tapered_waisted | rec_container_pump_var_tapered_waisted | bottle / waisted lathe profile / bottle_shell | 收腰/锥形侧轮廓(底宽—中段收—颈),lathe 侧轮廓改写 | converged |
| tall_rectangular | rec_container_pump_var_tall_rectangular | bottle / rectangular slab extrude helper / bottle_shell | 高直立矩形(直角棱柱/扁板)截面瓶身,平正面+侧面,区别于 boxy_oval 的圆润椭圆 | converged |

### Slot B:dispenser_head (顶部分配/闭合机构 + 其活动关节)
Functional layer = the collar interface + the top mechanism riding on it (parent:
`collar` part FIXED to bottle, `head_carrier` REVOLUTE swivel + `head` PRISMATIC
press, with stem + curved spout + dip tube). The collar/neck mating interface is
shared and held constant across all candidates; only the mechanism above the collar
changes.

| 候选(未来 module) | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| press_pump | rec_clear-soap-dispenser-bottle-with-a-white-press-d_20260606_074720_740450_74587dd4 | head_carrier / pump_swivel(REVOLUTE z) / head / pump_press(PRISMATIC z) / _head_mesh / _dip_tube_mesh | 竖直按压泵头:绕 z 回转 + 沿 z 下压回弹,弯嘴 + 内管 | converged(parent) |
| flip_top_cap | rec_container_pump_var_flip_top_cap | flip_lid / lid_hinge(REVOLUTE) / collar | 翻盖碟形盖,单铰链开合,露出出料孔;去掉泵/内管 | converged |
| trigger_sprayer | rec_container_pump_var_trigger_sprayer | sprayer_head / trigger_lever / trigger_pivot(REVOLUTE) / dip tube | 扳机喷雾头:前伸喷嘴 + 手指扳机绕铰链摆动回弹,内管保留 | converged |
| twist_lock_pump | rec_container_pump_var_twist_lock_pump | head_carrier / lock_swivel(REVOLUTE z) / head / pump_press(PRISMATIC z) | 旋转锁定下压泵:回转在解锁/锁死位之间切换,锁死位封住下压行程 | converged |
| foaming_pump | rec_container_pump_var_foaming_pump | head_carrier / pump_swivel(REVOLUTE z) / head / pump_press(PRISMATIC z) / 宽混合腔 _head_mesh | 高大胖泡沫泵头:宽圆柱混合腔高出领圈,平顶下压片,短粗泡嘴(无长鹅颈),短内管 | converged |
| disc_top_cap | rec_container_pump_var_disc_top_cap | disc_cap / disc_press(PRISMATIC z) / collar | 碟形顶按压盖:中央圆碟片下压(一侧翘开出料缝)回弹开合,非铰链翻盖、非泵;去掉泵/内管 | converged |
| gooseneck_pump | rec_container_pump_var_gooseneck_pump | head_carrier / pump_swivel(REVOLUTE z) / head / pump_press(PRISMATIC z) / 长鹅颈 spout(tube_from_spline_points) | 长曲鹅颈乳液泵头:保留下压泵+回转,短嘴换成高拱长鹅颈出嘴,内管保留 | converged |

## Multiplicity / Copy Logic
- count_param: 无,核心结构为固定 named slots(单瓶身 + 单顶部机构,无同构 ×N 子件)。
- N 样本已覆盖: 无。
- 模板建议 N_range: 无。
- copied object / naming / placement / joint policy: 无复制逻辑。Slot B 内部若某候选(如 flip_top_cap 的铰链片)需要小批量重复装饰,应按 §4 用 for-i 循环发射,但单瓶整体不存在 multiplicity 轴。

组合数预审: Π(Slot A 4 × Slot B 7) × N(无) = 28 ≥ 10 ✓

## 排除项(未来 compatibility matrix 素材)
- 纯尺寸/比例(更高/更瘦/更大容量)与配色/材质(透明 vs 不透明、瓶身/泵头颜色):非结构轴,留给模板连续参数与 palette,不造变体。
- squat_round(矮胖圆瓶):仍是 round_body 同一旋转体截面族,只是更矮更宽——属比例/尺寸变化(留给模板连续参数),非结构上不同的截面族,排除;真正不同的瓶身截面族(boxy_oval / tapered_waisted / tall_rectangular)已分别覆盖。
- clear_dome_overcap(透明防尘罩):参考图泵头外确有一只可掀离的透明罩,但它本身不携带任何非 fixed 关节(只是被取下),单独作一个候选不构成 ≥2 候选的真实轴,且与 dispenser_head 槽强耦合;暂不另立第 3 轴,留作模板可选装饰/罩件参数。
- screw_cap_only(无泵纯旋盖):会丢掉唯一的真实活动机构(0 非 fixed joint),违反 §3 第 2 条,排除;闭合机构候选都保留至少一个 revolute/prismatic 活动关节。
- 跨轴组合(如 boxy_oval × trigger_sprayer):组合由模板采样器免费产出,不造组合变体;collar/neck 接口在所有 Slot A 候选上保持轴对称且尺寸一致,Slot B 任一候选都能复用同一 mating face,暂无特殊干涉风险需要组合抽检。
