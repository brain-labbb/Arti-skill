# Military / Turret — template source map

pattern: mixed
parents: rec_model-a-stylized-automated-sentry-gun-turret-abo_20260610_080449_802592_d996dd05 ← picture/Military/Turret/001.png (fills Slot A=splayed_quad_legs, Slot B=quad{4}, Slot C=none; primary base-yaw + pitch + recoil chain)

## Slot 候选覆盖

### Slot A:base (root + yaw stage carrier)
| 候选(module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| splayed_quad_legs | rec_model-a-stylized-automated-sentry-gun-turret-abo_20260610_080449_802592_d996dd05 | leg_base / base_to_collar_yaw(continuous z) / helpers `_segment_tube`,`_rect_tube`,`_perforated_shell`; elems `leg_strut_{i}`,`knee_gusset_{i}`,`foot_pad_{i}`,`column_core`,`perforated_wrap`,`ring_platform`,`platform_lip` | 4 splayed box-section struts (LEG_ANGLES 45/135/225/315) on round foot pads, central perforated column + ring platform; ~1.2 m footprint, foot pads grounded at z≈0 | converged (parent) |
| tripod_3legs | rec_sentry_turret_var_tripod3 | leg_base / base_to_collar_yaw(continuous z); same leg/column elems | same strut leg system but LEG_ANGLES=(0,120,240), 3-leg tripod at 120° spacing | converged (workbench, rating pending sync) |
| solid_pedestal | rec_sentry_turret_var_pedestal | pedestal / pedestal_to_collar_yaw(continuous z); helper `_pedestal_drum`; elems `pedestal_drum`,`shaft_band`,`base_rim`,`flange_bolt_{i}` | lathe-revolved single drum column (base flange → shaft → top flange) with bolt ring on floor flange; no legs | converged (workbench, rating pending sync) |
| ceiling_mount | rec_sentry_turret_var_ceiling | ceiling_plate / plate_to_collar_yaw(continuous z); helpers `_ceiling_plate_mesh`,`_boss_shroud_mesh`,`_shroud_ring_mesh`; elems `plate_body`,`mounting_boss`,`boss_shroud`,`shroud_ring_top`,`shroud_ring_bot`,`bolt_head_{i}` | flat circular ceiling plate with cable bore + perimeter bolt holes, downward mounting boss inside perforated shroud; collar hangs below plate | converged (workbench, rating pending sync) |

### Slot B:barrel-count (recoiling muzzle multiplicity)
| 候选(module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| quad{4} | rec_model-a-stylized-automated-sentry-gun-turret-abo_20260610_080449_802592_d996dd05 | barrel_cluster / receiver_to_barrels_recoil(prismatic -x); elems `carriage`,`barrel_{idx}`,`muzzle_shroud_{idx}`,`muzzle_cap_{idx}` | 2×2 grid of barrels (nested `for sz in (±1): for sy in (±1)` loop, ±BARREL_DY/±BARREL_DZ), 4 bores in receiver front wall | converged (parent) |
| single{1} | rec_sentry_turret_var_single1 | barrel_cluster / receiver_to_barrels_recoil(prismatic -x); same elems, `BARREL_OFFSETS=[(0.0,0.0)]` | one centered barrel; nested 2×2 loop rewritten to single `for i,(dy,dz) in enumerate(BARREL_OFFSETS)` offset-list loop, bores cut per offset | converged (workbench, rating pending sync) |
| twin{2} | rec_sentry_turret_var_twin2 | barrel_cluster / receiver_to_barrels_recoil(prismatic -x); same elems, `BARREL_OFFSETS=[(-BARREL_DY,0.0),(BARREL_DY,0.0)]` | side-by-side pair on horizontal axis; same single offset-list loop (demanded the 2×2→list rewrite) | converged (workbench, rating pending sync) |

### Slot C:optics (sensor/sight mounted on receiver)
| 候选(module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| none | rec_model-a-stylized-automated-sentry-gun-turret-abo_20260610_080449_802592_d996dd05 | receiver / `top_panel_{i}` only (no optics elems) | bare receiver top, twin recessed top panels, no sight | converged (parent) |
| camera | rec_sentry_turret_var_cameraoptic | receiver elems `sight_body`,`sight_lens`,`sight_led`; helper `_camera_sight_body` | recessed-lens camera sight body bolted to receiver top with red status LED; rides with pitch | converged (workbench, rating pending sync) |
| laser_pod | rec_sentry_turret_var_laserpod | receiver elems `sensor_pod_body`,`sensor_lens`,`sensor_rear_cap`,`sensor_mount_bracket` | cylindrical sensor/laser pod on a mount bracket atop receiver, front glass lens + rear cap; rides with pitch | converged (workbench, rating pending sync) |

## Multiplicity / Copy Logic

### (1) barrel count — Slot B multiplicity
- count_param: `BARREL_OFFSETS` (list of `(dy, dz)` offsets relative to the pitch axis; `len()` = barrel count)
- N 样本已覆盖: {1, 2, 4} → 1:rec_sentry_turret_var_single1 / 2:rec_sentry_turret_var_twin2 / 4:rec_model-a-stylized-automated-sentry-gun-turret-abo_20260610_080449_802592_d996dd05
- 模板建议 N_range: [1, 6]
- copied object / naming / placement / joint policy: copied object = barrel triplet (`barrel_{i}` tube + `muzzle_shroud_{i}` + `muzzle_cap_{i}`); naming = single index `{i}` over `enumerate(BARREL_OFFSETS)`; placement = each at `(BARREL_X0/SHROUD_X0/CAP_X0, dy, dz)` within barrel_cluster, with one matching guide bore cut into `receiver_shell` front wall per offset; joint policy = all barrels share the single `receiver_to_barrels_recoil` prismatic (-x, 0..RECOIL_TRAVEL) — barrel count never adds joints. Parent's nested `for sz/for sy` 2×2 loop was rewritten to one offset-list loop in single1/twin2 so any N follows the same code path.

### (2) leg count — Slot A secondary multiplicity (leg-type bases only)
- count_param: `LEG_ANGLES` (tuple of yaw angles about Z; `len()` = leg count)
- N 样本已覆盖: {3, 4, 6} → 3:rec_sentry_turret_var_tripod3 / 4:rec_model-a-stylized-automated-sentry-gun-turret-abo_20260610_080449_802592_d996dd05 / 6:rec_sentry_turret_var_legs6
- 模板建议 N_range: [3, 6]
- copied object / naming / placement / joint policy: copied object = one leg assembly (`leg_strut_{i}` from shared `leg_mesh`/`_segment_tube` + `knee_gusset_{i}` + `foot_pad_{i}`); naming = index `{i}` over `enumerate(LEG_ANGLES)`; placement = each rotated by `yaw=radians(ang)` about Z, knee/foot at `(KNEE_PT.r·cos/sin, …)` so feet land on a common ground circle; joint policy = legs are rigid within `leg_base`, no per-leg joints. APPLIES ONLY TO leg-type bases (splayed_quad_legs, tripod_3legs); solid_pedestal and ceiling_mount have no `LEG_ANGLES` and ignore this parameter.

## 排除项(未来 compatibility matrix 素材)
- barrel N=3 omitted to keep the muzzle-count ladder tight (1/2/4 already span single→pair→2×2 grid); N=3 has no clean symmetric grid placement around the pitch axis.
- pan + tilt (yaw continuous + pitch revolute) preserved in ALL Slot A and Slot B candidates — no variant drops the 2-DOF aiming chain; recoil prismatic likewise retained in every barrel-count module.
- leg count is intentionally NOT crossed with pedestal/ceiling bases (LEG_ANGLES absent there) — those base modules are single-piece and out of the leg-multiplicity domain.
