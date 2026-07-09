# Sports / Exercise bike — template source map

pattern: mixed (parallel_children core: body root with flywheel/crank/saddle-post/handlebar-post children + linear crank→pedal chain; multiplicity on stabilizer feet)
parents: rec_white-upright-stationary-exercise-bike-with-red-_20260605_165843_884664_7f5ac918 ← picture/Sports/Exercise bike/001.png (covers: frame=upright, resistance=front_disc_red_ring, handlebar=ramhorn_console, feet N=2)

Single parent. It fills exactly one cell of each slot below; every variant forks from this parent and changes exactly one slot.

## Slot 候选覆盖

### Slot A:frame_type (overall body / rider-station posture)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| upright_shroud | rec_white-upright-stationary-exercise-bike-with-red-_20260605_165843_884664_7f5ac918 | body (root, body_shroud) / _body_solid teardrop loft; saddle_post & handlebar_post rise vertically | parent: tall vertical molded plastic shroud, saddle above crank, bars on forward mast | converged (parent) |
| recumbent | rec_exercise_bike_var_recumbent | body (long horizontal beam) / seat_carriage with backrest, body_to_seat_carriage prismatic; flywheel/crank low front | long low horizontal beam, reclined bucket seat + backrest at rear, side grips | built ✓ |
| spin_tube_frame | rec_exercise_bike_var_spin | body (open tube frame: backbone/down/seat/head tubes via tube/spline mesh) / flywheel+crank+post joints on tube junctions | exposed welded round-tube triangulated frame instead of molded cowl | built ✓ |

### Slot B:resistance_form (flywheel / resistance presentation)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| front_disc_red_ring | rec_white-upright-stationary-exercise-bike-with-red-_20260605_165843_884664_7f5ac918 | flywheel (flywheel_disc + flywheel_red_ring + flywheel_marker) / body_to_flywheel continuous(Y) | parent: exposed solid gray dished disc, red TorusGeometry accent ring, off-axis bolt marker | converged (parent) |
| perforated_spoked | rec_exercise_bike_var_perforated_flywheel | flywheel (rim ring + hub + spoke_{i} loop) / body_to_flywheel continuous(Y) | open cast wheel: outer rim + central hub joined by for-i loop of equal-angle spokes, holes between | built ✓ |
| magnetic_shroud_knob | rec_exercise_bike_var_magnetic_shroud | flywheel (covered resistance pod) + resistance_knob / body_to_flywheel continuous(Y) + body_to_knob revolute(Z) | no bare wheel: closed molded cowl over an internal rotating mass + top tension knob (2nd real joint) | built ✓ |

### Slot C:handlebar_form (handlebar + console cockpit)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| ramhorn_console | rec_white-upright-stationary-exercise-bike-with-red-_20260605_165843_884664_7f5ac918 | handlebar_post (handlebar_tube swept ram-horn + console_pad + handlebar_clamp) / body_to_handlebar_post prismatic(Z) | parent: curved red ram-horn tube (tube_from_spline_points) + tilted white display console tablet | converged (parent) |
| straight_bar_no_console | rec_exercise_bike_var_straight_bar | handlebar_post (straight crossbar + 2 end grips, no console) / body_to_handlebar_post prismatic(Z) | plain horizontal grip bar, foam end grips, console pad removed | built ✓ |
| aero_multigrip | rec_exercise_bike_var_aero_grip | handlebar_post (base crossbar + side grips + aero_extension_{i} loop ×2 + armrest pad + console_pad) / body_to_handlebar_post prismatic(Z) | multi-position triathlon cockpit: forward aero extensions (for-i ×2 mirrored) + forearm pad, console kept | built ✓ |

## Multiplicity / Copy Logic
- count_param: stabilizer_foot_count (front-to-back chrome cross-tube feet under the body)
- N 样本已覆盖: {2, 3, 4} → parent (N=2, currently a hand-written front_foot/rear_foot tuple — NOT yet a loop) / rec_exercise_bike_var_feet3 (N=3) / rec_exercise_bike_var_feet4 (N=4)
- 模板建议 N_range: [2, 6] (real exercise bikes use 2 long cross-feet; 3–4 short feet on heavier bases; cap ~6)
- copied object: one chrome stabilizer cross-tube (shared `_foot_mesh` helper) with its down-leg + 2 black end caps + 2 ground pads
- naming: `stabilizer_foot_{i}` (parent must be rewritten from the front_foot/rear_foot tuple into a `for i in range(n)` loop; feet3/feet4 prompts require this rewrite explicitly)
- placement: regular, evenly spaced front-to-back under the body footprint; each cross-tube is the widest span on the ground
- joint policy: uniform — every foot FIXED up to the body root (`body_to_stabilizer_foot_{i}`); the bike's articulation budget comes from flywheel/crank/pedals/posts, so feet stay fixed

## 排除项(未来 compatibility matrix 素材)
- (none recorded yet — P0 planning only; record any non-converging cell after fork)
- watch: spin_tube_frame × magnetic_shroud_knob — an open tube frame has no cowl to hide a shrouded resistance pod against, so that cross-combo may need a separate small housing bracket (compatibility-matrix candidate, not sampled in this batch).
- watch: recumbent × ramhorn_console handlebar mast geometry differs (recumbent routes grips to seat-side, not a forward mast); the handlebar slot may need a recumbent-specific mount in the template (interface note for spec author).

---
## Post-fork verification (SEGMENT 1 complete)
All planned variants forked via `articraft fork` (dashscope qwen3.7-max, thinking medium), then verified on-disk: last compile = success, ≥1 non-fixed joint present, collections=['workbench'] (workbench-only, not promoted), and picture.json bound into the correct `Sports__<小类>` subcat shard (reconcile rebuilt). Status cells above flipped planned→built ✓ accordingly. Ready for SEGMENT 2 (spec authoring).
