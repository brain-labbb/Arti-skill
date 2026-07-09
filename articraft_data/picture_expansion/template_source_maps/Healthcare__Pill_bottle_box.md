# Healthcare / Pill bottle_box — template source map

> TWO FAMILIES in one 小类: (A) pill BOTTLE / vial (round or square body + cap closure) and
> (B) pill ORGANIZER / box (compartment wells + lids). Candidates are NOT freely composable
> across families (a screw cap does not belong on a 7-day organizer); the template must model
> them as two ③ primary-form families and gate cross-family cells. See compatibility notes.

pattern: mixed — Family A: linear_chain (body → cap); Family B: parallel_children/multiplicity (tray hub → N lids)

parents (2 originals, one per family):
- rec_a-cylindrical-supplement-fish-oil-omega-3-pill-b_20260623_174436_815042_9552f090  ← Family A: tall round supplement bottle, softgel fill, PRISMATIC lift cap (bottle, softgels, cap)
- rec_a-portable-7-day-pill-organizer-box-a-flat-recta_20260623_174436_818406_81682422  ← Family B: flat 7-day organizer, 7 flip lids + outer lid + latch (base_tray, compartment_lid_{1..7} REVOLUTE, outer_lid, front_latch)

## Slot 候选覆盖

### Family A — Slot A1: bottle body form (③ planar boundary / volumetric envelope)
| 候选 | source_type | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| round_cylinder | forked_anchor (parent) | pill_bottle | bottle (revolved shell) | round supplement bottle | converged |
| square_prism | forked_anchor | rec_pillbox_var_square_bottle | bottle (square section) + square cap | rounded-square vitamin bottle | converged |

### Family A — Slot A2: closure mechanism
| 候选 | source_type | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| screw_lift_cap | forked_anchor (parent) | pill_bottle | cap ← PRISMATIC z | unscrew/lift cap | converged |
| fliptop_hinged | forked_anchor | rec_pillbox_var_fliptop | flip_lid ← REVOLUTE (collar_to_flip_lid) | snap flip-top | converged |

### Family B — Slot B1: compartment lid mechanism
| 候选 | source_type | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| individual_flip_lids | forked_anchor (parent) | pill_organizer | compartment_lid_{n} REVOLUTE | per-cell hinged lids | converged |
| single_sliding_cover | forked_anchor | rec_pillbox_var_sliding_lid | sliding_cover ← PRISMATIC x | one shared sliding lid | converged |
| rotating_dial_lid | forked_anchor | rec_pillbox_var_round_weekly | dial_lid ← REVOLUTE z (round base) | round rotating dispenser | converged |

### Family B — Slot B2: compartment count N (multiplicity)
| 候选 | source_type | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| weekly_7 | forked_anchor (parent) | pill_organizer | compartment_lid_{1..7} | 7-day row | converged |
| twice_daily_14 | forked_anchor | rec_pillbox_var_14cell | compartment_lid_{n} n in range(14) | 7×2 AM/PM grid | converged |

## Multiplicity / Copy Logic
- count_param: compartment_count (Family B). Family A has no copy loop for the closure (softgels loop is interior fill, not a structural slot).
- N 样本已覆盖: {7 → parent, 14 → var_14cell}; round dispenser = 7 pie wells.
- 模板建议 N_range: organizer compartments [4, 28] (weekly/AM-PM/4x-daily); softgel fill count is decorative interior, not a slot.
- copied object / naming / placement / joint policy: compartment_lid_{n} in a grid, each REVOLUTE (axis x) hinged at its rear edge on base_tray; uniform hinge policy.

## 视觉多样性 6 轴考察
| 轴 | 处理 | 本小类取值 / 范围 / 理由 |
|---|---|---|
| ① 骨架图(+N) | forked_anchor | Family A body→cap chain; Family B tray→N lids parallel. |
| ② 关节类型 | forked_anchor | PRISMATIC (screw/lift cap, sliding cover), REVOLUTE (flip lids, flip-top, rotating dial). |
| ③ 主体形态家族 | forked_anchor + world_knowledge_extrapolation | bottle: round/square (Planar Boundary); organizer: flat-row/round-dial (Volumetric Envelope). Template may extrapolate oval/hex bottle. |
| ④ 表面装饰 | record_only + world_knowledge_extrapolation | printed label sleeve, ribbed cap knurling, embossed day markings (1st–7th / Mon–Sun), braille dots. |
| ⑤ 尺寸/行程 | record_only | bottle H 0.06–0.14 m; organizer footprint 0.08–0.18 m; cap lift/ slide travel small. |
| ⑥ 涂装 | record_only | amber/clear/white translucent plastic, black/white cap; organizer pastel translucent (blue/green/pink/amber) + cream frame; ≥5 colorways. |

## Compatibility Probes
| probe_id | source_type | record_id | 组合轴值 | 验证目标 | 结论 |
|---|---|---|---|---|---|
| gate | — | — | Family A closure × Family B tray | cross-family cells invalid | GATE: closures only on bottles, lids only on organizers |

## 排除项
- Family cross-cells (e.g. screw cap on a 7-day tray) are structurally invalid — gated, not forked.
- none — all forks converged.
