# Science / First aid cabinet — template source map

pattern: mixed
parents:
- rec_build-a-realistic-articulated-3d-model-of-a-firs_20260609_183625_780787_99727092 ← picture/Science/First aid cabinet/ (`cabinet_body` root + `cabinet_door`+`body_to_door`(revolute); shelves looped along Z, no drawers). Covers Slot A=glass_front_hinged, Slot B=open_shelves, Multiplicity N=2 shelves.

Wall-mount first-aid cabinet. `cabinet_body` is the root; a `cabinet_door` swings on a single
REVOLUTE `body_to_door`. The batch isolates door type, interior fitment (shelves vs. drawers), and
shelf count. Drawers, when present, are looped `drawer_{i}` each on a +Y PRISMATIC; shelves are FIXED
loop visuals.

## Slot 候选覆盖

### Slot A:door
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| glass_front_hinged | rec_build-...-firs_20260609_183625_780787_99727092 (parent) | `cabinet_door`, `body_to_door`(revolute) | single glass-front hinged door, one revolute | converged(parent) |
| solid_panel_hinged | rec_first_aid_cabinet_var_solid_door | `cabinet_door`, `body_to_door`(revolute) | single solid (opaque) panel hinged door, one revolute | converged(workbench, rating pending sync) |
| double_doors | rec_first_aid_cabinet_var_double_doors | `door_{i}`, two `body_to_door` revolutes | twin doors meeting at center, looped `door_{i}`, 2 revolutes | converged(workbench, rating pending sync) |

### Slot B:interior fitment
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| open_shelves | rec_build-...-firs_20260609_183625_780787_99727092 (parent) | `cabinet_body`, shelf loop, `body_to_door`(revolute) | open fixed shelves only, no drawers | converged(parent) |
| shelves_plus_drawer | rec_first_aid_cabinet_var_drawer_base | `drawer`, drawer prismatic + door revolute | shelves plus one pull-out `drawer` (1 revolute + 1 prismatic) | converged(workbench, rating pending sync) |
| shelves_plus_drawer_stack | rec_first_aid_cabinet_var_drawer_stack | `drawer_{i}`, 3 drawer prismatics + door revolute | shelves plus a stack of looped `drawer_{i}`, each +Y PRISMATIC (1 revolute + 3 prismatic) | converged(workbench, rating pending sync) |

## Multiplicity / Copy Logic
- count_param: `shelf_count`(fixed interior shelves)
- N 样本已覆盖: {1, 2, 3} → rec_first_aid_cabinet_var_one_shelf / parent / rec_first_aid_cabinet_var_three_shelf
- 模板建议 N_range: [1, 6]
- copied object / naming / placement / joint policy: shelves looped as `shelf_{i}`, evenly stacked along the interior Z axis, FIXED visuals (no joints). Secondary multiplicity: drawer stack looped `drawer_{i}`, each an independent +Y PRISMATIC.

## 组合数预审
Slot A(3) × Slot B(3) × Multiplicity(3) = 27 ≥ 10 ✓.

## 排除项(未来 compatibility matrix 素材)
- Free-standing / legged cabinet excluded — this is the wall-mount category.
- Roll-up / tambour door excluded (no clean single-axis articulation in samples).
- Door handle / latch is decoration, not a slot.
- Drawer N>3 is the sampler's job; samples only demonstrate the drawer loop copy logic.
