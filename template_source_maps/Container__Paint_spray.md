# Container / Paint spray — template source map

pattern: parallel_children
parents:
- rec_aerosol-spray-paint-can-with-a-lift-off-dust-cap_20260606_074905_121496_5eaf6974 ← picture/Container/Paint spray/001.png (tall splatter-label aerosol can, crimped dome shoulder, central valve stem; press-down nozzle button; lift-off dark-grey dust cap shown beside the can). Occupies cells: Slot A `lift_off_cap` × Slot B `press_button` × Slot C `straight_cylinder`.

Aerosol spray paint can. Shared kinematics: a `can_body` (root) is a tall cylinder with a
crimped top dome and a central valve stem. Two independent articulated functional layers hang
off the body — a top closure (`dust_cap`, the removable lid) and an actuator (`spray_nozzle`,
the spray button) — plus the body footprint itself as a third structural slot. The single parent
fills exactly one cell of each slot; variants fill the remaining empty cells, each changing
exactly one slot off the parent. No N-replicated sub-parts exist (a spray can has no array of
identical units), so there is no multiplicity axis.

## Slot 候选覆盖

### Slot A:closure / cap mechanism
| 候选(未来 module) | record_id | 关键 part/joint/helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| lift_off_cap | rec_aerosol-spray-paint-can-with-a-lift-off-dust-cap_20260606_074905_121496_5eaf6974 (parent) | `dust_cap` part, `cap_lift` joint (PRISMATIC +Z, large lift), `_dust_cap_solid` helper | hollow domed shell sleeves over top shoulder, lifts straight off | converged(parent) |
| flip_top_cap | rec_container_paint_spray_var_flip_top | `flip_cap` part, `cap_hinge` joint (REVOLUTE, horizontal axis at rear rim) | one-piece lid hinged at rear shoulder, swings up | converged |
| screw_cap | rec_container_paint_spray_var_screw_cap | `screw_cap` part, `cap_unscrew` joint (REVOLUTE about +Z), visible `thread_collar` mating ring | threaded cap unscrews about can axis off a collar | converged |
| no_cap_collar | rec_container_paint_spray_var_no_cap_collar | `lock_collar` part + `lock_tab` part, `tab_slide` joint (PRISMATIC, lock/unlock) | no overcap; nozzle exposed, spray-lock tab slides | converged |

### Slot B:actuator / nozzle mechanism
| 候选(未来 module) | record_id | 关键 part/joint/helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| press_button | rec_aerosol-spray-paint-can-with-a-lift-off-dust-cap_20260606_074905_121496_5eaf6974 (parent) | `spray_nozzle` part, `nozzle_press` joint (PRISMATIC -Z, ~0.004 m), `_nozzle_solid` helper | small rounded fingertip button presses straight down on valve stem | converged(parent) |
| trigger_lever | rec_container_paint_spray_var_trigger_lever | `trigger_cap` / `trigger_lever` part, `trigger_pivot` joint (REVOLUTE, horizontal axis) | Montana-style finger trigger swings to press valve | converged |
| pistol_grip | rec_container_paint_spray_var_pistol_grip | `grip_body` clamp part + `grip_trigger` part, `grip_trigger_pivot` joint (REVOLUTE) | clip-on gun grip with finger trigger linkage to valve | converged |
| fan_spin_cap | rec_container_paint_spray_var_fan_spin_cap | `fan_cap` part, `fan_press` joint (PRISMATIC -Z) + `fan_twist` joint (REVOLUTE +Z) | wide fat-tip fan cap presses to spray and twists to change pattern | converged |

### Slot C:body footprint / cross-section
| 候选(未来 module) | record_id | 关键 part/joint/helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| straight_cylinder | rec_aerosol-spray-paint-can-with-a-lift-off-dust-cap_20260606_074905_121496_5eaf6974 (parent) | `can_body` (`_can_body_solid`), constant `CAN_R` straight wall + crimped dome loft | perfectly straight constant-diameter cylinder | converged(parent) |
| waisted_body | rec_container_paint_spray_var_waisted_body | `can_body` profile via lathe/revolve with necked-in upper third | hourglass / waisted contour before the dome | converged |
| oval_section | rec_container_paint_spray_var_oval_section | `can_body` swept on elliptical section, oval rim + dome | flat-oval (elliptical) horizontal cross-section can | converged |

## Multiplicity / Copy Logic
- count_param: 无,核心结构为固定 named slots(closure / actuator / body)。A spray can has no array of N identical sub-parts (no drawers/slats/spokes/links), so there is no per-N replication and no `for i in range(n)` copy layer at the small-类 level.
- N 样本已覆盖: 无。
- 模板建议 N_range: 无(此小类无 multiplicity 轴)。
- copied object / naming / placement / joint policy: 无。

## 组合数预审
组合数预审: Slot A(4) × Slot B(4) × Slot C(3) × N(none) = 48 ≥ 10 ✓。每个 slot ≥2 候选(各 3–4 个);pattern = parallel_children;无 multiplicity 轴,候选已堆厚(两槽各 4 个)以越过 ≥10 门槛。

## 排除项(未来 compatibility matrix 素材)
- spray_can 无 N-复制子件 → 主动放弃 multiplicity 轴(非阻塞,真实物体如此);组合数靠候选数堆厚补足。
- 跨槽组合(如 pistol_grip × oval_section)留给模板采样器,不做组合抽检变体(无特殊接口/干涉风险)。
- 纯尺寸(can 高度 / 直径 / 标签图案 / 颜色)是模板连续参数与材质,不入 slot。
- 单参考图(001.png)即足以支撑全部三轴候选(均为真实存在的喷漆罐形态);未出现需要更多图才能确认的候选,故无 blocked 轴。
