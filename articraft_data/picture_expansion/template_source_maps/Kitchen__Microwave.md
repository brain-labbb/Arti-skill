# Kitchen / Microwave — template source map

pattern: parallel_children
parents:
- rec_model-a-stylized-countertop-microwave-oven-appro_20260610_080946_066456_b2de7afe ← picture/Kitchen/Microwave/001.png (stylized low-poly countertop microwave: side-hinged door, spinning turntable, membrane/strip control panel)

Stylized countertop microwave oven. Core kinematics shared by all candidates: a root
`body` (chamfered pale-blue-gray shell with a hollow dark cooking cavity cut into the
front-left ~3/4, an inset control-panel strip on the right quarter, four block feet),
a `door` part hinged to the body (the door-mechanism slot), and a `turntable` part
(`drive_hub` + 3 `coupler_rib_{k}` + `glass_plate`) spinning in the cavity (the
turntable slot). Geometry helpers `_shell_solid()` (cavity cut + panel recess) and
`_door_solid()` (frame + window pocket) are reused across every source. The four
independent structural slots below are: door mechanism, turntable, control, and the
optional interior rack.

Every candidate keeps ≥1 non-fixed joint (verified on materialized URDFs):
parent / drop-down / drawer / top-hinged / touch-glass / rack = 2 non-fixed; rotary
dials = 4 non-fixed; flatbed = 1 non-fixed (door revolute survives — turntable removed).

## Slot 候选覆盖

### Slot A:door mechanism (`door` part + `door_hinge` / `drawer_slide`)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| side_hinge | rec_model-...-microwave-...-b2de7afe (parent) | `door` (`door_frame`/`window_glass`/`handle_bar`/`handle_standoff_{top,bottom}`), `door_hinge` REVOLUTE axis (0,0,-1) at left front edge, 0..100° | vertical-axis swing-out door; full-height vertical handle bar; `_door_solid` origin on left hinge edge | converged |
| drop_down | rec_microwave_var_door_drop_down | same `door` parts, `door_hinge` REVOLUTE axis (1,0,0) at bottom front edge, 0..90° | oven-style drop-down door; horizontal handle bar at top; `_door_solid` origin at bottom center | converged |
| top_hinge | rec_microwave_var_door_top_hinged | same `door` parts, `door_hinge` REVOLUTE axis (-1,0,0) at top front edge, 0..100° | hood-style upward swing; door hangs from top hinge; `_door_solid` origin at top center | converged |
| drawer_prismatic | rec_microwave_var_door_drawer | `drawer` (`drawer_frame`/`window_glass`/`handle_bar`/`drawer_tray`/`tray_lip_{i}`), `drawer_slide` PRISMATIC axis (0,-1,0) 0..0.22 m; body adds `guide_rail_{i}` | pull-out drawer microwave; tray/flatbed extends into cavity; turntable re-parented onto drawer (`turntable_spin` parent=drawer) | converged |

### Slot B:turntable (`turntable` part + `turntable_spin`)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| rotating | rec_model-...-b2de7afe (parent) | `turntable` part (`drive_hub`/`coupler_rib_{0..2}`/`glass_plate`), `turntable_spin` CONTINUOUS axis (0,0,1) on cavity floor | spinning glass plate on driven hub; off-axis ribs prove rotation; parent=body | converged |
| flatbed | rec_microwave_var_turntable_flatbed | `flatbed_glass` inline body visual (no `turntable` part, no `turntable_spin` joint) | fixed flat glass cooking floor; no rotating member; door revolute remains the live joint | converged |

### Slot C:control (panel on the right-quarter front face)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| membrane | rec_model-...-b2de7afe (parent) | `control_panel` inline body visual (flat charcoal strip in shell recess) | flat membrane/keypad strip; no moving control parts | converged |
| rotary_dials | rec_microwave_var_control_rotary_dials | `power_knob`/`timer_knob` parts (`{name}_cap` KnobGeometry skirted + `{name}_pointer`), `{name}_dial` REVOLUTE axis (0,-1,0) 0..270° | two skirted appliance rotary dials on panel face; each a revolute knob about the front-face normal; off-center pointer proves spin | converged |
| touch_glass | rec_microwave_var_control_touch_glass | `touch_glass_panel` + `touch_panel_backing` + `touch_mark_{0..2}` inline body visuals | single flat recessed dark glass touch pane with inline touch-zone marks; no moving control parts | converged |

### Slot D:interior rack (cavity contents above the turntable)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| none | rec_model-...-b2de7afe (parent) | (no rack part) | empty cavity above the turntable | converged |
| shelf | rec_microwave_var_rack_shelf | `shelf_rack` part (`frame_{front,back,left,right}` + `cross_wire_{0..9}` + `long_wire_{0..3}`), body adds `rack_support_{left,right}` ledges, `body_to_rack` FIXED | mid-level removable wire-grid shelf resting on cavity-wall support ledges; rack fixed to body (door + turntable remain the live joints) | converged |

## Multiplicity / Copy Logic
- count_param: 无显著 multiplicity 轴。唯一带 N 的复制结构是 Slot D shelf 内部的网格丝
  (`cross_wire_{i}` ×N_CROSS_WIRES=10、`long_wire_{i}` ×N_LONG_WIRES=4),用循环 +
  `_rack_wire(length, along)` helper + 等距 placement 发射;但它是 rack 模块的内部填充密度,
  不是小类级的功能复制轴(整机只有 0/1 个 rack)。Slot D 本身的 multiplicity = rack 个数 ∈ {0,1},
  无意义(none vs 1 shelf),按任务要求记此一行。
- N 样本: 无小类级 multiplicity 轴需要覆盖。
- 模板建议 N_range: rack wire grid 内部 `cross_wires` 可参数化 ~[6,14]、`long_wires` ~[3,6]
  (controlled local parameterization 范畴,非 slot 候选轴)。
- copied object / naming / placement / joint policy: rack 网格丝用 `for i in range(N)` +
  `f"cross_wire_{i}"`/`f"long_wire_{i}"` + 共享 `_rack_wire` helper + 等距 Y/X placement,
  全部 inline 为 `shelf_rack` 的 visuals(无独立 joint)。turntable `coupler_rib_{k}` 用
  `for k in range(3)` 等角发射(全 inline 在 `turntable` part 上)。feet 用
  `for i,(fx,fy)` 循环。所有复制均循环发射、命名规整,模板可直接读出。

## 组合数预审
Slot A(4) × Slot B(2) × Slot C(3) = 24 ≥ 10 ✓(Slot D ×2 → 48)。每个 slot ≥2 候选,
主机构 slot A 满配 4 个候选。pattern = parallel_children;无显著 multiplicity 轴(rack 0/1)。

## 排除项(未来 compatibility matrix 素材)
- flatbed (Slot B) × drawer (Slot A) 未抽检:drawer 变体把 turntable 重新 parent 到抽屉托盘上
  (`turntable_spin` parent=drawer),若 drawer × flatbed 组合则需把 fixed 玻璃地板也挂到抽屉而非
  body — 这是模板侧 compatibility matrix 要裁决的真实接口风险(drawer 的活动地板归属)。
- rack (Slot D shelf) × drawer (Slot A):rack 的 `rack_support_{l,r}` 与 `body_to_rack` FIXED 都锚在
  body 的固定 cavity 壁上;若与 drawer 组合,rack 与拉出的抽屉托盘/turntable 是否干涉未抽检。
- rotary_dials (Slot C) 与各 door 机构组合未抽检;knob 都锚在 body 右侧 `control_panel` 面,
  与 door 在不同面,组合风险低(由模板采样器生成)。
- 纯尺寸(更宽/更高/更扁的机身、cavity 尺寸、handle 长度)不作为候选——属模板连续参数
  (controlled local parameterization),不入 slot。
