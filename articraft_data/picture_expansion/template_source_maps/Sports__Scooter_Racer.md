# Sports / Scooter Racer — template source map

pattern: mixed (parallel_children steering+wheel branches off a shared deck body; multiplicity over wheel count)
parents: rec_vintage-green-kick-scooter-with-a-tall-curved-ha_20260605_165922_546417_d68bd852 ← picture/Sports/Scooter Racer/001.png (vintage green curved running-board kick scooter; tall curved swept stem + T-bar; 2 large whitewall wheels with curved fenders; rigid stem; covers cells Slot A=curved_swept, Slot B=running_board, Slot C=rigid, N=2)

## Slot 候选覆盖

### Slot A: handlebar / steering form (steering_column geometry; revolute steering joint about tilted head axis)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| curved_swept (parent) | rec_vintage-green-kick-scooter-...d68bd852 | steering_column / curved_stem / handlebar_crossbar / grip_0,grip_1 / steering(revolute, STEER_AXIS) | tall gracefully curved vintage stem sweeping up then forward to a narrow T crossbar | converged(parent) |
| tbar_straight | rec_kick_scooter_var_tbar_straight | steering_column / straight_stem / handlebar_crossbar / grip_i / steering(revolute) | single straight vertical stem tube + level horizontal crossbar = clean upright modern T | built ✓ |
| bmx_riser | rec_kick_scooter_var_bmx_riser | steering_column / riser_bar (U-bend) / cross_brace / grip_i / steering(revolute) | short stem topped by tall U-shaped riser bend braced by a horizontal cross-brace tube | built ✓ |
| swept_cruiser | rec_kick_scooter_var_swept_cruiser | steering_column / cruiser_bar / grip_i / steering(revolute) | wide swept-back beach-cruiser bar, grips set back and wider apart | built ✓ |

### Slot B: foot-deck form (body; foot_deck visual + branding inlay)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| running_board (parent) | rec_vintage-green-kick-scooter-...d68bd852 | body / foot_deck / branding / front_neck | low vintage running-board deck with raised branding plate, cast neck riser | converged(parent) |
| flat_plank | rec_kick_scooter_var_flat_plank | body / foot_deck / grip_surface / front_neck | long flat low rectangular plank deck with non-slip grip inlay + chamfered/rounded long edges | built ✓ |

### Slot C: stem articulation / folding mechanism (steering_column; optional fold revolute)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| rigid (parent) | rec_vintage-green-kick-scooter-...d68bd852 | steering_column / curved_stem (one rigid piece) | one-piece rigid stem, no fold; only the steering revolute moves the column | converged(parent) |
| folding_hinge | rec_kick_scooter_var_folding_hinge | steering_column(lower stem) / upper_stem (child part) / hinge_knuckle / hinge_pin / stem_fold(revolute, axis X) | stem split into lower+upper sections joined by a visible hinge knuckle+pin; upper section (with T-bar+grips) folds forward/down on a limited revolute | built ✓ |

## Multiplicity / Copy Logic
- count_param: wheel_count (rear-wheel multiplicity is the variable axis; the steered front wheel stays singular)
- N 样本已覆盖: {2, 3} → parent (front+single rear, N_rear=1 → total 2) / rec_kick_scooter_var_three_wheel (front + 2 rear on a left-right axle, N_rear=2 → total 3)
- 模板建议 N_range: total wheels [2, 3]; rear_wheel_count [1, 2] (kids tri-wheel tops out at 3; do not push higher — a 4-wheel object exits the kick-scooter category)
- copied object: a rear road wheel = shared _wheel_meshes helper (rim + tire + whitewall + off-axis valve marker)
- naming: rear_wheel_{i} parts with rear_rim_{i}/rear_tire_{i}/rear_whitewall_{i}/rear_valve_{i} visuals
- placement: symmetric left/right offsets along X on a shared rear axle at REAR_WHEEL_Y, AXLE_Z (single rear wheel = centered i=0)
- joint policy: one CONTINUOUS roll joint per rear wheel, axis (1,0,0), child of body, origin on the rear axle at the actual hub face; uniform across the loop. Front wheel keeps its own CONTINUOUS roll as child of steering_column.

## 接口点位(跨 slot mating)
- Slot A↔body: steering head tube on body (head_tube visual, tilted to STEER_AXIS at HEAD_BASE) is the mating bearing; steering revolute origin = HEAD_BASE, axis = STEER_AXIS. All Slot-A candidates seat their stem base in this tube.
- Slot C fold (when present) splits inside Slot A: lower stem stays child of body via steering; upper stem is child of lower stem via stem_fold revolute, pin axis X, origin at the knuckle contact face partway up the stem.
- Slot B↔wheels: deck/body carries the rear axle (REAR_WHEEL_Y, AXLE_Z) and the front_neck→head riser; deck form changes must keep rear axle Y and deck-top z (<0.15) so wheels still reach ground.
- fork legs (steering_column) straddle the front wheel hub at fw_axle_local — front wheel roll origin sits at that axle face.

## 排除项(未来 compatibility matrix 素材)
- 暂无(规划阶段,尚未 fork)。
- 预期风险点(留给模板侧验证,非本批排除):folding_hinge × 任一非 parent handlebar 候选时,upper-stem 折叠包络须避开 deck 与前 fender;swept_cruiser/bmx_riser 的后掠/外扩 bar 在大转向角下须避免 grip 扫到 front_neck。本批按单轴控制变量隔离,不预造这些组合(组合由模板采样器产出)。

---
## Post-fork verification (SEGMENT 1 complete)
All planned variants forked via `articraft fork` (dashscope qwen3.7-max, thinking medium), then verified on-disk: last compile = success, ≥1 non-fixed joint present, collections=['workbench'] (workbench-only, not promoted), and picture.json bound into the correct `Sports__<小类>` subcat shard (reconcile rebuilt). Status cells above flipped planned→built ✓ accordingly. Ready for SEGMENT 2 (spec authoring).
