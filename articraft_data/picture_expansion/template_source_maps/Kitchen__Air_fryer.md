# Kitchen / Air fryer — template source map

pattern: parallel_children
parents:
- rec_model-a-compact-countertop-air-fryer-approximate_20260610_080653_823182_92e8cb25 ← picture/Kitchen/Air fryer (rounded body, digital touch control, windowed pull-out basket drawer, single basket). Covers Slot A=rounded, Slot B=digital, Slot C=windowed-drawer, basket_count=single.

Countertop pull-out-basket air fryer. Core kinematics shared by all drawer-style
candidates: a `body` (root) and a `basket_drawer` child that pulls out of the lower
front pocket via a PRISMATIC joint (`drawer_slide`, +X axis, travel 0 → 0.16 m). The
body carries `shell`, `rim_trim_band`, the top control surface, `brand_logo` and a
pair of `basket_slide_rail_{idx}` rails; the drawer carries `drawer_face`, `handle`,
`basket` and `fries_heap`. The body silhouette, the control interface, and the
basket-opening mechanism are the three independent structural slots below; the number
of baskets is the multiplicity axis. The clamshell candidate replaces the defining
PRISMATIC drawer with a REVOLUTE lid hinge (the basket then becomes a fixed body
element) — the one candidate that swaps the defining motion.

## Slot 候选覆盖

### Slot A:body silhouette (`shell` via `_build_*_shell`)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| rounded_taper | rec_model-a-compact-countertop-air-fryer-approximate_..._92e8cb25 (parent) | `body.shell` from `_build_body_shell` (lofted rounded-rect bottom→top sketches, `CORNER_R` fillet); `_build_trim_band` rounded ring | classic compact rounded-corner box with slight inward taper; lofted between two filleted rect sketches | converged |
| square_tower | rec_air_fryer_var_body_square | `body.shell` from `_build_body_shell` (`cq.box` + `edges("|Z").fillet(EDGE_R)`); `_build_trim_band` sharp rect ring | upright square tower, uniform footprint (no taper), near-90° vertical corners, flat faces | converged |
| cylindrical_drum | rec_air_fryer_var_body_cylindrical | `body.shell` from `_build_drum_shell` (revolve rectangular profile around Z, flat-front facet cut, D-shaped `_build_trim_band` + `_build_top_glass`) | circular drum footprint truncated by a flat front facet where the drawer pulls out; lathe/revolve primitive | converged |

### Slot B:control interface (top deck of `body`)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| digital_touch | rec_model-a-compact-countertop-air-fryer-approximate_..._92e8cb25 (parent) | `body.top_glass_panel` (single smoked-glass Box visual, no joint) | flush glass touch panel inset in the top recess; zero added joints | converged |
| rotary_dials | rec_air_fryer_var_control_dial | `timer_knob` + `temp_knob` parts (KnobGeometry caps + `_build_dial_shaft`), `body_to_timer_knob` / `body_to_temp_knob` (REVOLUTE z); `escutcheon_plate` + `dial_bezel_{i}` body visuals | two mechanical rotary dials on a copper escutcheon; each adds a vertical-axis REVOLUTE joint; shafts seat through deck holes | converged |
| push_buttons | rec_air_fryer_var_control_buttons | `body.control_panel` (matte Box) + `push_button_{i}` (loop over `range(BUTTON_COUNT)`, 2×3 grid via `_build_push_button`) | recessed tactile push-button grid on a matte panel; loop-emitted, no added joints (decorative buttons) | converged |

### Slot C:basket-opening mechanism (`basket_drawer` / `drawer_slide`)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| windowed_drawer | rec_model-a-compact-countertop-air-fryer-approximate_..._92e8cb25 (parent) | `basket_drawer` part, `drawer_slide` (PRISMATIC +X); `_build_drawer_face` with `window` cut + `window_glass` tinted Box | pull-out drawer with a viewing window cut through the face and a recessed tinted pane | converged |
| solid_drawer | rec_air_fryer_var_door_solid | `basket_drawer` part, `drawer_slide` (PRISMATIC +X); `_build_drawer_face` returns solid panel, `window_glass` visual removed | same prismatic pull-out drawer but an opaque solid front (no window cut, no pane) | converged |
| clamshell_lid | rec_air_fryer_var_door_clamshell | `lid` part, `lid_hinge` (REVOLUTE −y at rear rim), `hinge_barrel_{i}`, `lid_handle`; `basket` becomes a fixed body element | top-opening clamshell lid hinged at the rear upper rim swinging up/back; basket fixed in the lower bowl. Swaps the defining motion PRISMATIC→REVOLUTE | converged |

## Multiplicity / Copy Logic
- count_param: `basket_count`(变体里命名 `NUM_BASKETS`)。
- N 样本已覆盖: {1, 2} → parent (single, named `basket_drawer`) / rec_air_fryer_var_basket_dual (dual, loop `drawer_{i}`)。
- 模板建议 N_range: [1, 3](真实台式 air fryer 基本是 single 或 dual side-by-side;3 篮极少见但结构上可平铺,采样域 ≥ 样本覆盖是正常的)。
- copied object / naming / placement / joint policy:
  - copied object = 一个完整 drawer 子树(`drawer_face` + `window_glass` + `handle` + `basket` + `fries_heap`)外加其 body 侧 `slide_rail_{i}_{j}` 与 PRISMATIC `drawer_slide_{i}` 关节。
  - naming = `drawer_{i}` part / `drawer_slide_{i}` joint / `slide_rail_{i}_{j}` rail(嵌套 i=篮,j=轨)。
  - placement = 沿 Y 对称分布:`POCKET_CY[i]`(中央分隔墙 `DIVIDER_HALF` 两侧 ±0.090),body 宽度随 N 加宽(single 0.28 → dual 0.40)。
  - joint policy = 每篮一个独立 PRISMATIC 滑轨,互不联动(测试断言 drawer_0 开不带动 drawer_1)。
  - copy 实现 = `for i in range(NUM_BASKETS)` 循环,共享 `_populate_drawer` helper + 预建共享 mesh(`_build_drawer_face`/`_build_handle`/`_build_basket`/`_build_fries_heap` 各物化一次复用),符合可读性契约。

## 组合数预审
Slot A(3) × Slot B(3) × Slot C(3) = 27 ≥ 10 ✓(× multiplicity N {1,2} → 54)。每个 slot 3 候选。
pattern = parallel_children:body 之下 drawer / dials / lid 均为并列子件,multiplicity 篮亦为并列复制。

## 排除项(未来 compatibility matrix 素材)
- clamshell_lid (Slot C) 与 multiplicity (basket_count>1) 互斥的真实倾向:单个翻盖罩通常对应单一上掀腔,dual 翻盖罕见;模板若组合 clamshell × dual 需新增双铰链证据样本(本批未抽检)。
- clamshell_lid 是唯一把 Slot C 的定义运动从 PRISMATIC 改为 REVOLUTE 的候选,且令 `basket` 从 drawer 子件变为 body 固定件 —— 模板侧需把 Slot C 视为"既决定面板机构又决定 basket 归属(drawer vs body)"的耦合槽,而非单纯 face 换皮。
- 跨槽组合(如 cylindrical_drum × rotary_dials、square_tower × clamshell_lid)未专门抽检;组合由模板采样器生成(每槽样本均与 parent 单轴 diff 干净)。
- 纯尺寸(envelope 高/宽/深、travel 行程)不作为候选 —— 属模板连续参数(controlled local parameterization),不入 slot。
