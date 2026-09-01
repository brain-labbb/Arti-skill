# Bathroom / washmachine — template source map

pattern: parallel_children
parents:
- rec_white-front-loading-washing-machine-with-a-pull-_20260605_154143_807145_3205b533 ← picture/Bathroom/washmachine/001.png (white front-loading washing machine with hinged round door, rotating drum, rotary dial, pull-out detergent drawer, lower service panel). Covers Slot A=round_porthole_door, Slot B=single_dial_controls, Slot C=pull_out_drawer, Slot D=flat_lower_panel.

Front-loading washing machine. The body is the root and carries parallel child mechanisms:
`drum` (CONTINUOUS), `door` (REVOLUTE), `dial` or control parts (CONTINUOUS/visual),
and dispenser/service modules (PRISMATIC or REVOLUTE where applicable). The batch isolates
door/window shape, control interface, detergent dispenser mechanism, and lower plinth/service
panel as independent slots.

## Slot 候选覆盖

### Slot A:front door / window module
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| round_porthole_door | rec_white-front-loading-washing-machine-with-a-pull-_20260605_154143_807145_3205b533 (parent) | `door`, `body_to_door`, `door_bezel`, `door_glass` | round hinged glass door with circular gasket/bezel | converged |
| rounded_square_window | rec_washmachine_var_square_window_door | `door`, `body_to_door`, square/rounded glass module | rounded-square hinged glass window with thick gasket frame | converged |
| convex_porthole | rec_washmachine_var_convex_porthole_door | `door`, `body_to_door`, convex glass/bezel | deep convex porthole door with thick chrome outer ring and latch block | converged |

### Slot B:control interface
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| single_dial_display | rec_white-front-loading-washing-machine-with-a-pull-_20260605_154143_807145_3205b533 (parent) | `dial`, `body_to_dial`, display visuals | one rotary dial plus rectangular display/control area | converged |
| touch_panel_buttons | rec_washmachine_var_touch_panel_controls | touch display panel and flush buttons | wide black touch display with multiple small flush buttons; parent dial removed/replaced | converged |
| twin_dial_controls | rec_washmachine_var_twin_dial_controls | two dial parts/joints plus display | two smaller rotary dials plus compact display | converged |

### Slot C:detergent dispenser
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| pull_out_drawer | rec_white-front-loading-washing-machine-with-a-pull-_20260605_154143_807145_3205b533 (parent) | `drawer`, `body_to_drawer` | top-front pull-out detergent drawer with prismatic travel | converged |
| flip_lid_tray | rec_washmachine_var_flip_lid_dispenser | dispenser lid/tray hinge | top-front flip-lid tray with a visible revolute hinge | converged |

### Slot D:lower base / service panel
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| flat_service_panel | rec_white-front-loading-washing-machine-with-a-pull-_20260605_154143_807145_3205b533 (parent) | lower body service-panel visual | small flat rectangular lower service panel on the front face | converged |
| raised_plinth_flip_panel | rec_washmachine_var_raised_plinth_panel | raised plinth and hinged service panel | raised base/plinth with larger rectangular flip-down service panel | converged |

## Multiplicity / Copy Logic
- count_param: 无 topology-level multiplicity；buttons may be local repeated visuals within `touch_panel_buttons`.
- N 样本已覆盖: 无。
- 模板建议 N_range: 无。
- copied object / naming / placement / joint policy: local button grids should use loop-emitted `button_{i}` visuals with regular placement; they are not independent template multiplicity slots unless future samples add articulated repeated controls.

## 组合数预审
Slot A(3) × Slot B(3) × Slot C(2) × Slot D(2) = 36 ≥ 10 ✓.

## 排除项(未来 compatibility matrix 素材)
- No blocked cells in this batch; all planned cells converged.
- touch_panel_buttons replaces the parent rotary dial and may reduce one control articulation; this is intentional for that candidate while the object still has door/drum/drawer mechanisms.
- flip_lid_tray and raised_plinth_flip_panel add front-face revolute mechanisms; template combinations should preserve clearance with the large front door swing.
