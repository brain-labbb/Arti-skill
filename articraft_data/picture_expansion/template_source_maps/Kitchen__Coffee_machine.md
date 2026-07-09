# Kitchen / Coffee machine — template source map

pattern: parallel_children
parents:
- rec_model-a-delonghi-magnifica-style-super-automatic_20260610_080724_417430_ed06768c ← picture/Kitchen/Coffee machine/001.png (DeLonghi Magnifica-style super-automatic bean-to-cup; baseline for Slot A=super-automatic, Slot B=rear-hinge lid, Slot C=internal tank)

Upright espresso/coffee machine. Core shared layout across all candidates: a `body`
root (base block / `core_shell` / tilted `fascia_panel` / `top_deck`) plus a rotary
`selection_dial` (CONTINUOUS, axis normal to the tilted fascia) and a `drip_tray`
(PRISMATIC, slides forward +X). On the super-automatic spine the `steam_wand`
(REVOLUTE) is also part of the shared core. The brew front end, the bean-hopper lid,
and the water tank are the three independent structural slots below. Every candidate
keeps ≥1 real non-fixed joint (the dial alone guarantees this even when a slot's own
mechanism is FIXED).

## Slot 候选覆盖

### Slot A:brew type / dispensing front end (front recess between the cheeks)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| super_automatic | parent (…_ed06768c) | `spout_block` part with `left_nozzle`/`right_nozzle`; `body_to_spout` (PRISMATIC -Z, 0.06 m on `spout_rail`); plus `selection_dial` / `fascia_to_dial` (CONTINUOUS) | dual-nozzle dispenser block rides a vertical rail and lowers to cup height; bean-to-cup machine | converged (parent baseline) |
| portafilter | rec_coffee_machine_var_brew_portafilter | `group_head` part (`gh_lock_ring`/`gh_body`/`gh_flange`, brass) on `body_to_group_head` (FIXED) + `portafilter` part (`pf_basket`/`pf_rim`/`pf_ear_0`/`pf_ear_1`/`pf_handle_lug`/`pf_handle_shaft`/`pf_handle_grip`) on `body_to_portafilter` (FIXED); body gains `group_mount_boss`; spout_block removed | cylindrical brew group + removable portafilter with side handle; spout/hopper deleted, dial+tray+wand retained | converged |
| pod_capsule | rec_coffee_machine_var_brew_pod | `single_spout`/`spout_tip` + `capsule_slot` on body; `capsule_flap` part (`flap_plate`/`flap_grip`/`flap_hinge_pin`) on `body_to_flap` (REVOLUTE -Y, rear top hinge); `control_dial`/`body_to_dial` (CONTINUOUS) | compact Nespresso-style pod machine: rounded `body_shell` (ExtrudeGeometry), single chrome spout, top capsule flap; whole front end re-bodied | converged |

> Note: the pod candidate is a larger re-body (compact shell, single spout, top capsule
> flap, integral water tank visual) — its diff vs the super-automatic parent is broader
> than one layer, but it represents the genuine third brew-type structural form. The
> template author should treat its `capsule_flap` (Slot B) and integral `water_tank`
> visual (Slot C) as the pod-spine defaults rather than mixing them onto the
> super-automatic spine.

### Slot B:bean-hopper lid / fill access (top deck, rear half)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| rear_hinge | parent (…_ed06768c) | `hopper_lid` (`lid_plate`/`lid_grip_bar`) on `deck_to_hopper_lid` (REVOLUTE, axis -Y, rear edge hinge at X_REAR, ~100°) | classic rear-edge flip-up lid; plate extends +X from the hinge | converged (parent baseline) |
| side_hinge | rec_coffee_machine_var_hopper_side_hinge | `hopper_lid` (`lid_plate`/`lid_grip_bar`) on `deck_to_hopper_lid` (REVOLUTE, **axis +Z**, vertical hinge at left-side edge `LID_HINGE_X/Y/Z`, ~100°) | same lid plate but pivots sideways about a vertical hinge at the left flank instead of flipping up | converged |
| removable_canister | rec_coffee_machine_var_hopper_removable | `hopper_canister` part (`canister_shell` CadQuery hollow shell + `canister_lid_plate`/`canister_grip`/`canister_align_rib`) on `body_to_hopper` (**PRISMATIC +Z**, 0.10 m); body gains a 4-wall guide bay (`hopper_guide_front/rear/left/right`, `hopper_bay_floor`) | lift-out bean canister replaces the hinged lid; vertical extraction out of a guide bay | converged |

### Slot C:water tank (reservoir mount)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| internal | parent (…_ed06768c) | no dedicated tank part (reservoir concealed inside `core_shell`) | no separate, articulated tank — concealed internal reservoir; fold-into-body default | converged (parent baseline) |
| side_removable | rec_coffee_machine_var_tank_side_removable | `water_tank` part (`tank_body` translucent + `tank_cap`/`tank_handle`) on `body_to_tank` (**PRISMATIC -Y**, 0.10 m); body gains `tank_bay_plate` + `tank_rail_0`/`tank_rail_1` on the right flank | translucent reservoir slides out sideways from a right-flank docking bay | converged |
| top_reservoir | rec_coffee_machine_var_tank_top_reservoir | `water_tank` part (`tank_vessel` Extrude rounded-rect + `tank_water_fill`/`tank_lid`/`tank_grip`) on `body_to_tank` (**PRISMATIC +Z**, 0.15 m); body gains rear cradle `tank_bay_platform`/`tank_bay_back_wall`/`tank_bay_left_wall`/`tank_bay_right_wall` | clear rear-mounted reservoir with visible water level, lifts straight up out of a rear cradle | converged |

## Multiplicity / Copy Logic
- count_param: 无,核心结构为固定 named slots(brew front end / hopper lid / water tank)。fascia 上的按钮(`{tag}_button_{k}`)是装饰,不带关节,不构成 multiplicity 轴。
- N 样本: 无 multiplicity 轴(buttons are decoration; not copied as articulated units)。
- 模板建议 N_range: 不适用。
- copied object / naming / placement / joint policy: symmetric pairs already emitted by
  small loops — `{tag}_nozzle`, `{tag}_front_cheek`, `tray_{tag}_wall`, `{tag}_button_{k}`
  (3 buttons × 2 sides), `pf_ear_{i}` (×2), `tank_rail_{i}` (×2), `hopper_guide_{tag}`
  (4 bay walls) — all decoration/structure, none are user-facing articulated copies.

## 组合数预审
Slot A(3) × Slot B(3) × Slot C(3) = **27 ≥ 10 ✓**. 每个 slot ≥3 candidate, 全部 converged.
pattern = parallel_children(slots are independent functional layers off the shared
`body` root, no chaining), 无 multiplicity.

## 排除项(未来 compatibility matrix 素材)
- 无不收敛取值;9 个候选(parent baseline ×3 + 6 fork 变体)全部 converged。
- 真实兼容性约束(下游 compatibility matrix 素材,非 fork 失败):
  - Slot A=pod_capsule 自带独立的 Slot B(`capsule_flap` 顶部翻盖)与 Slot C(机身集成 `water_tank` 视觉)默认值——pod 机身上一般不会再挂 super-automatic 的 bean-hopper lid 或可拆水箱;把 pod 视作自带 B/C 默认的整机 spine。
  - Slot A=portafilter 删除了 `hopper_lid`(豆斗)——portafilter 机用预磨咖啡粉,通常无豆斗;portafilter × Slot B(任意 hopper 候选)在真实物体上是弱组合,模板侧可标为 down-weighted。
  - 纯尺寸(更高/更宽机身、更大/更小水箱容量、travel 行程长度)不作为候选——属模板连续参数(controlled local parameterization),不入 slot。
