# Kitchen / Hood — template source map

pattern: parallel_children

parents:
- rec_model-a-wall-mounted-t-style-box-linear-kitchen-_20260610_080847_092440_e562777f ← picture/Kitchen/Hood (T-style box/linear canopy; covers Slot A `t_box`, Slot B `single_telescope`, Slot C `fixed_mesh`, Slot D `push_button`)

Wall-mounted chimney range hood. Core kinematics shared by every candidate: a
`canopy` (root part) carrying a `blower_fan` rotor that spins on a CONTINUOUS
vertical joint `canopy_to_blower_fan` (axis (0,0,1)) behind/below the filter —
this is the always-present non-fixed joint. Four independent structural slots
hang off the canopy: the canopy form/silhouette, the telescoping chimney duct,
the grease-filter mechanism, and the fascia control. The motor housing
(`motor_housing` cylinder visual), recessed `lamp_lens_{i}` LED lenses, the
`brand_logo` and `indicator_lamp` decorations are inline canopy visuals shared
across all candidates and are not slots.

## Slot 候选覆盖

### Slot A:canopy form / silhouette (`canopy` shell + fascia)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| t_box | rec_model-a-wall-mounted-...e562777f (parent) | `_canopy_shell` helper → `canopy_shell` visual; flat-fronted fascia, buttons on vertical front (y=+0.25) | tapered skirt (z 0..0.06) lofting out to a 0.90×0.50×0.12 m box fascia + integrated bottom plate; classic linear box hood | converged |
| pyramid | rec_hood_var_form_pyramid | `_pyramid_shell` helper → `canopy_shell` visual; `FACE_TILT_FROM_Z` / `FRONT_NY,FRONT_NZ` angled-fascia math; fascia visuals tilted by `rpy=(FACE_TILT_*,…)`; power-button axis = inward face normal `(0,-FRONT_NY,-FRONT_NZ)` | truncated-pyramid loft (0.90×0.50 bottom → 0.34×0.30 top at z=0.28); dramatically sloped front; controls mount on the angled face | converged |
| curved_glass | rec_hood_var_form_curved_glass | `_metal_body_shell` (slim box body) + `_glass_visor_geometry` (`section_loft` over `LoftSection`/`SectionLoftSpec` following `GLASS_CURVE`) → `body_shell` + `glass_visor` visuals | slim stainless body (0.90×0.20×0.14 at z 0.30..0.44) with a concave section-lofted tempered-glass visor sweeping down to z≈0.04; visor form, not a box | converged |

### Slot B:chimney duct (`lower_duct` fixed on canopy + telescoping sleeve(s))
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| single_telescope | rec_model-a-wall-mounted-...e562777f (parent) | `chimney_sleeve` part (`sleeve_shell` + `guide_pad_{i}`); `canopy_to_chimney_sleeve` PRISMATIC axis (0,0,1) 0..0.35 m | one upper sleeve (`_rect_tube` 0.336×0.296) sliding up over the fixed `lower_duct`; 4 friction guide pads ride the duct wall | converged |
| dual_telescope | rec_hood_var_chimney_telescope_dual | `_build_sleeve_stage` helper, loop `for i in range(2)` → `sleeve_0`/`sleeve_1` parts (`sleeve_shell_{i}` + `guide_pad_{i}_{j}`); joints `canopy_to_sleeve_0` then `sleeve_0_to_sleeve_1`, both PRISMATIC +Z 0..0.35 m | two nested telescoping stages (inner 0.336×0.296, outer 0.352×0.312); stage 1's parent is `sleeve_0` (a serial chain), full extension = 2×travel | converged |

> Note: dual_telescope's two stages form a short PRISMATIC linear_chain internal
> to this slot (stage1 parented on stage0). It is emitted via a stage loop, not
> a multiplicity count axis — sample only covers N=2 stages.

### Slot C:grease filter (under-canopy filter mechanism)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| fixed_mesh | rec_model-a-wall-mounted-...e562777f (parent) | `filter_mesh_panel` (`SlotPatternPanelGeometry`) inline visual on `canopy`, no joint | single dark slotted aluminum-mesh panel recessed above the bottom plate; rigid, no part/joint | converged |
| hinged | rec_hood_var_filter_hinged | `grease_filter` part (`filter_mesh_panel` + `hinge_knuckle_{i}` ×3); `canopy_to_grease_filter` REVOLUTE axis (1,0,0) 0..π/2 | single mesh panel that swings down on an X-axis hinge at its front edge for cleaning access; adds a 2nd non-fixed joint | converged |
| dual_baffle | rec_hood_var_filter_dual_baffle | `_baffle_panel_cq` helper, loop `for i in range(2)` → `filter_{i}` parts (`baffle_panel_{i}`); FIXED joints `canopy_to_filter_{i}`; canopy-side `filter_divider` + `filter_rail_{fi}_{ri}` rails | two side-by-side removable baffle filter panels (framed parallel-strip baffles) on support rails; MULTIPLICITY axis (see below) | converged |

### Slot D:fascia control (user input on the canopy front)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| push_button | rec_model-a-wall-mounted-...e562777f (parent) | `power_button` part (`button_cap`); `canopy_to_power_button` PRISMATIC axis (0,-1,0) 0..0.004 m; static `button_{i}` ×4 inline + `brand_logo`/`indicator_lamp` | rightmost button is the live press-in power button (4 mm travel); four static dummy buttons beside it | converged |
| rotary_knob | rec_hood_var_control_rotary_knob | `speed_knob` part (`knob_body` via `KnobGeometry`+`KnobSkirt`/`KnobGrip`/`KnobIndicator`); `canopy_to_speed_knob` REVOLUTE axis (0,1,0) 0..270° ; canopy-side `knob_escutcheon` bezel ring | single knurled rotary speed knob turning on a Y-axis revolute, protruding through a bezel escutcheon; replaces the button row | converged |
| slider | rec_hood_var_control_slider | `slider_tab` part (`slider_cap` + `slider_stem`); `canopy_to_slider_tab` PRISMATIC axis (1,0,0) 0..0.060 m; canopy-side `slider_track` slot + `slider_tick_{i}` ×3 | horizontal low/med/high slider tab riding a track slot; tab+stem slide 60 mm along +X | converged |

## Multiplicity / Copy Logic
- count_param: `filter_count`(Slot C 的复制轴) ;parent/hinged 用 1 块 named filter,dual_baffle 用 N=2 块。其余 slot 为固定 named slots(canopy form / chimney / control 不是复制轴;dual_telescope 的 2 段是固定 named 链,非 count 轴)。
- N 样本已覆盖: filter_count ∈ {1, 2} → 1 = parent `filter_mesh_panel` (fixed) / hinged `grease_filter`(N=1,REVOLUTE);2 = rec_hood_var_filter_dual_baffle(`filter_0`/`filter_1`,loop-emitted)。
- 模板建议 N_range: filter baffle 板数 [1, 4](真实抽油烟机常见 1–4 块并列百叶滤网;采样域可大于样本覆盖的 {1,2})。
- copied object / naming / placement / joint policy: copied = 单块 baffle filter 面板;naming = `filter_{i}` part + `baffle_panel_{i}` visual,canopy-side `filter_rail_{fi}_{ri}` 支撑轨;placement = 沿 X 等距并列(中心 `sign*BAFFLE_X_OFFSET`,中间 `filter_divider` 分隔);joint policy = 每块 FIXED 到 canopy(`canopy_to_filter_{i}`)—— 复制件本身不活动,小类的活动关节由共享的 blower fan + 其余 slot 提供。dual_baffle 与 dual_telescope 两处复制都已用 `for i in range(n)` + 共享 geometry helper(`_baffle_panel_cq` / `_build_sleeve_stage`)循环发射,可机械读出。

## 组合数预审
Slot A(3:t_box / pyramid / curved_glass) × Slot B(2:single / dual telescope) × Slot C(3:fixed_mesh / hinged / dual_baffle) × Slot D(3:push_button / rotary_knob / slider) = 3×2×3×3 = **54 ≥ 10** ✓。每个 slot ≥2 候选;blower fan CONTINUOUS spin 在所有候选中共享,保证每个组合 ≥1 非 fixed joint。pattern = parallel_children(四个 slot 都直接挂在 canopy 根上;dual_telescope 的两段内部是 PRISMATIC linear_chain,dual_baffle 是 filter_count multiplicity)。

## 排除项(未来 compatibility matrix 素材)
- 跨 slot 组合未抽检(由模板采样器生成),但有两处真实接口风险供 compatibility matrix 注意:
  - curved_glass(Slot A)用 slim body(深度 0.20)且 fan 半径缩到 0.050、duct top 较低;与 dual_telescope(Slot B)叠加时 sleeve 行程/插入余量(代码里 min_overlap 已从 0.30 降到 0.25/0.02)需复核。
  - pyramid(Slot A)把 power_button 轴改成倾斜面法向 `(0,-FRONT_NY,-FRONT_NZ)`;Slot D 换成 rotary_knob/slider 时,knob/slider 的安装轴与 placement 需重算到倾斜 fascia 面(parent/curved_glass 用的是垂直前面)——这是 A×D 的真实接口耦合。
- dual_baffle 把 Slot C 的单一活动可能性(hinged 的 REVOLUTE)换成全 FIXED 复制件:单独看该候选的小类活动关节完全依赖共享 blower fan spin + 其余 slot,符合 §3(≥1 非 fixed joint 由 fan 保证)。
- filter_count N>2 未用变体覆盖(只展示 copy logic,N 域由模板侧放大);纯尺寸(canopy 宽/高、duct 行程长度、knob 直径)属模板连续参数,不入 slot。
