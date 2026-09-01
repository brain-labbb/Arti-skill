# Sports / Bike — template source map

pattern: mixed (parallel_children frame->fork/wheels/crank tree + multiplicity spoke ring per wheel)
parents: rec_orange-hardtail-mountain-bike-with-a-front-suspe_20260605_165808_319413_f497242f <- picture/Sports/Bike/001.png (orange Trek-style hardtail MTB; covers Slot A=diamond, Slot B=suspension_fork, Slot C=flat_bar, spoke N=28)

Core kinematics kept by every variant (do not vary): steering REVOLUTE (frame->fork about the tilted head-tube axis), front_wheel_roll CONTINUOUS (fork->front_wheel), rear_wheel_roll CONTINUOUS (frame->rear_wheel), crank_spin CONTINUOUS (frame->crank about the bottom-bracket axis). Part tree: frame (root) -> fork (steered) -> front_wheel; frame -> rear_wheel; frame -> crank.

## Slot 候选覆盖

### Slot A: frame_geometry
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| diamond (parent) | rec_orange-hardtail-mountain-bike-with-a-front-suspe_20260605_165808_319413_f497242f | frame.top_tube / down_tube / seat_tube / chainstay_left,right / seatstay_left,right / bb_shell; tube() helper | classic straight-tube front triangle + rear triangle, horizontal top tube | converged (parent baseline) |
| step_through | rec_bicycle_var_stepthrough | frame.top_tube (single low swept curve) / down_tube / seat_tube / bb_shell | open low-entry frame: deeply curved single top tube dropping to BB, no high crossbar | built ✓ |
| bmx_compact | rec_bicycle_var_bmx | frame.top_tube / seat_tube (short) / chainstay_* (short) / bb_shell | compact steep front triangle, near-horizontal top tube close to seat tube, very short chainstays, low saddle | built ✓ |
| cruiser_cantilever | rec_bicycle_var_cruiser | frame.top_tube (arced) / seat_tube (swept) / bb_shell | relaxed double-curved cantilever beach-cruiser silhouette, laid-back long profile | built ✓ |

### Slot B: fork_front_end
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| suspension_fork (parent) | rec_orange-hardtail-mountain-bike-with-a-front-suspe_20260605_165808_319413_f497242f | fork.steerer / fork_crown / fork_stanchion_{left,right} / fork_lower_{left,right}; steering(revolute) | single-crown telescoping fork: silver upper stanchion + black lower leg per side | converged (parent baseline) |
| rigid_fork | rec_bicycle_var_rigidfork | fork.steerer / fork_crown / fork_blade_{left,right}; steering(revolute) | one-piece curved rigid blade per side crown->dropout, no stanchion/seal, gentle rake | built ✓ |
| dual_crown | rec_bicycle_var_dualcrown | fork.steerer / fork_crown_lower / fork_crown_upper / fork_stanchion_{left,right}; steering(revolute) | triple-clamp downhill fork: two crowns clamping fat long-travel legs at two heights | built ✓ |

### Slot C: handlebar_form
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| flat_bar (parent) | rec_orange-hardtail-mountain-bike-with-a-front-suspe_20260605_165808_319413_f497242f | fork.stem / handlebar / grip_{left,right} | near-straight flat bar with slight back sweep, grip at each end | converged (parent baseline) |
| riser_bar | rec_bicycle_var_riserbar | fork.stem / handlebar (riser) / grip_{left,right} | bar climbs up from stem then sweeps back to raised grips, upright posture | built ✓ |
| drop_bar | rec_bicycle_var_dropbar | fork.stem / handlebar (drop) / grip_{left,right} | road drop bar: short top, forward over hoods, deep loop down to lower drops | built ✓ |

## Multiplicity / Copy Logic
- count_param: n_spokes (spoke count per wheel; same N applied to both wheels via the shared _wheel_geometry helper)
- N 样本已覆盖: {28 (parent), 18, 36} -> rec_orange-hardtail...f497242f / rec_bicycle_var_spokes18 / rec_bicycle_var_spokes36
- 模板建议 N_range: [12, 48] (real bicycle wheels run roughly 16-48 spokes; sampler may sweep the whole band, samples only demonstrate copy logic)
- copied object: one slender straight spoke wire (CylinderGeometry) from hub flange (r~0.030) out to just inside the rim (r = WHEEL_R - 2*TIRE_T)
- naming: f"{prefix}_spoke_{i}" inside _wheel_geometry, prefix in {front, rear}
- placement: equal angular spacing a = 2*pi*i/n_spokes + 0.18 (constant offset so no spoke lies on a coordinate axis), each spoke rotated radially and translated to the radial midpoint
- joint policy: spokes are non-jointed visuals of their wheel part; they ride the wheel's single CONTINUOUS roll joint (no per-spoke joint). The two real wheels are NOT a multiplicity loop (they are distinct front/rear parts with different parents fork vs frame).

## 排除项(未来 compatibility matrix 素材)
- (none — all 9 cells forked, built & workbench-bound; no axis dropped to a single candidate)
- Cross-axis interaction to watch (not a fork variant, note for template compatibility matrix): drop_bar (Slot C) with dual_crown (Slot B) is geometrically odd in the real world (road bar on a downhill fork) — flag as a compatibility-matrix exclusion candidate, but each axis is sampled independently here so no combo variant is built.

---
## Post-fork verification (SEGMENT 1 complete)
All planned variants forked via `articraft fork` (dashscope qwen3.7-max, thinking medium), then verified on-disk: last compile = success, ≥1 non-fixed joint present, collections=['workbench'] (workbench-only, not promoted), and picture.json bound into the correct `Sports__<小类>` subcat shard (reconcile rebuilt). Status cells above flipped planned→built ✓ accordingly. Ready for SEGMENT 2 (spec authoring).
