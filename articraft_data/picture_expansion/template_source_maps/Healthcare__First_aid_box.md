# Healthcare / First aid box — template source map

> IDENTITY: anchored on the PORTABLE first-aid hard case (rigid box, hinged lid, latches,
> lift-out tray, carry handle). The wall-mounted cabinet origin is kept as a secondary form
> anchor only and is NOT forked here — it overlaps the already-built `Science/First aid cabinet`
> template (see 排除项). All forks derive from the portable case.

pattern: mixed (box base → hinged lid + latches; interior tray(s) prismatic; handle layer)

parents (2 originals):
- rec_a-portable-first-aid-hard-case-a-rigid-rectangul_20260623_175416_875087_f2b56491  ← PRIMARY: rigid rectangular case (base, tray PRISMATIC, lid REVOLUTE, handle, latch_0/latch_1 REVOLUTE)
- rec_a-wall-mounted-first-aid-cabinet-a-shallow-recta_20260623_175408_640340_93ce5d6a  ← secondary form anchor only (cabinet_body + door REVOLUTE); NOT forked (overlaps Science/First aid cabinet)

## Slot 候选覆盖

### Slot A: lid / closure mechanism
| 候选 | source_type | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| single_top_lid | forked_anchor (parent) | aid_portable | lid ← REVOLUTE x | one top-hinged lid | converged |
| clamshell_dual | forked_anchor | rec_firstaid_var_clamshell | lid + front_flap REVOLUTE | both halves open | converged |

### Slot B: interior organization
| 候选 | source_type | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| single_lift_tray | forked_anchor (parent) | aid_portable | tray ← PRISMATIC z | one lift-out tray | converged |
| cantilever_tiers | forked_anchor | rec_firstaid_var_cantilever_trays | tray_{0,1} + tray_arm_{i} REVOLUTE | fold-out tackle-box tiers | converged |
| stacked_trays_N | forked_anchor | rec_firstaid_var_stacked_trays | tray_{0..2} PRISMATIC | N nested lift trays | converged |

### Slot C: handle
| 候选 | source_type | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| folding_top_handle | forked_anchor (parent) | aid_portable | handle (folding loop) | top carry handle | converged |
| fixed_side_grips | forked_anchor | rec_firstaid_var_side_handles | end_handle_{left,right} | recessed end grips | converged |

### Slot D: body form (③)
| 候选 | source_type | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| rigid_rect_box | forked_anchor (parent) | aid_portable | base + lid (sharp box) | rectangular hard case | converged |
| rounded_tin | forked_anchor | rec_firstaid_var_rounded_tin | base+lid rounded/domed | vintage rounded metal tin | converged |
| wall_cabinet | (origin, NOT synced) | aid_wall | cabinet_body + door | shallow wall cabinet | excluded — see 排除项 |

## Multiplicity / Copy Logic
- count_param: latch_count (=2 draw latches) and tray_count (1 → N stacked).
- N 样本已覆盖: latches {2}; trays {1 (parent), 2 (cantilever), 3 (stacked)}.
- 模板建议 N_range: latches {1,2}; interior trays [1, 3].
- copied object / naming / placement / joint policy: latch_{i} mirrored on the front face, each REVOLUTE draw-latch; tray_{i} stacked along z, each PRISMATIC lift, uniform policy.

## 视觉多样性 6 轴考察
| 轴 | 处理 | 本小类取值 / 范围 / 理由 |
|---|---|---|
| ① 骨架图(+N) | forked_anchor | box base hub + lid/flap + N tray children + latch pair. |
| ② 关节类型 | forked_anchor | REVOLUTE (lid, flap, latches, cantilever arms), PRISMATIC (lift trays, sliding). |
| ③ 主体形态家族 | forked_anchor + world_knowledge_extrapolation | rigid rect / rounded tin / (wall cabinet anchor). Template may extrapolate soft-corner intermediate. |
| ④ 表面装饰 | record_only + world_knowledge_extrapolation | red-cross decal, FIRST AID lettering, corner bumpers, ribbed lid, latch escutcheons. |
| ⑤ 尺寸/行程 | record_only | case ~0.25–0.35 m wide; lid swing 0–105°; tray lift small. |
| ⑥ 涂装 | record_only | red/white/green/orange painted metal or plastic, chrome latches; weathered olive-drab tin; ≥5 colorways. |

## Compatibility Probes
(none critical — lid × tray × handle combinations are all real.)

## 排除项
- wall-mounted cabinet origin (aid_wall): **NOT synced** to arti-template (excluded from the 5★ source set). Reason: user directive to de-emphasize it + it overlaps the already-built `Science/First_aid_cabinet` template. The First aid box template identity is the PORTABLE hard case only; a layperson picture of a wall cabinet is served by the Science template. Origin accounted-for here per the origin-reconciliation rule.
- none of the 5 forks were excluded — all converged.
