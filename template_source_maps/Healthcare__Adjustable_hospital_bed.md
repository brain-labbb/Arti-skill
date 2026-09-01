# Healthcare / Adjustable hospital bed — template source map

pattern: linear_chain (base/deck frame → hinged articulating section(s); "adjustable" = the section-tilt joints)

parents (2 originals):
- rec_a-single-section-adjustable-hospital-bed-a-recta_20260623_174436_818326_5bfade45  ← caster-base bed, tubular head/foot boards, single head backrest tilt (base_frame + backrest REVOLUTE y)
- rec_an-adjustable-examination-treatment-couch-medica_20260623_174436_819096_4031599a  ← 4-leg treatment couch, backrest tilt + side tray, casters (frame, seat_pad, leg_pad, backrest REVOLUTE, support_arm, side_tray, caster_{i})

## Slot 候选覆盖

### Slot A: deck articulation (the "adjustable" mechanism — primary axis)
| 候选 | source_type | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| single_backrest | forked_anchor (parent) | bed_section / bed_couch | backrest ← REVOLUTE (base_to_backrest, y) | head raises only | converged |
| two_section_gatch | forked_anchor | rec_hospbed_var_knee_gatch | + knee_section ← REVOLUTE | head + knee bend | converged |
| three_section_profiling | forked_anchor | rec_hospbed_var_three_section | + thigh_section + calf_section REVOLUTE | full profiling deck | converged |

### Slot B: base / mobility
| 候选 | source_type | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| caster_base | forked_anchor (parent) | bed_section | base_frame + fixed casters | 4 swivel casters | converged |
| four_leg_couch | forked_anchor (parent) | bed_couch | frame + leg_{i} + caster_{i} | rigid 4-leg exam couch | converged |
| hi_lo_column | forked_anchor | rec_hospbed_var_hilo_column | lift_column ← PRISMATIC z | central elevating column | converged |

### Slot C: end boards (③ macro surface of head/foot)
| 候选 | source_type | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| tubular_rail_boards | forked_anchor (parent) | bed_section | base_frame board visuals | bent-tube head/foot | converged |
| open_no_board | forked_anchor (parent) | bed_couch | (couch, no end boards) | open exam couch | converged |
| solid_panel_boards | forked_anchor | rec_hospbed_var_footboard_panel | panel head/foot board visuals | molded flat panels | converged |

### Slot D: side safety rails
| 候选 | source_type | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| none | forked_anchor (parent) | both parents | — | no side rails | converged |
| dropdown_side_rails | forked_anchor | rec_hospbed_var_side_rails | side_rail_{left,right} REVOLUTE | full-length drop-down guards | converged |

## Multiplicity / Copy Logic
- count_param: caster_count (=4, corners) and deck_section_count (1→2→3, = Slot A).
- N 样本已覆盖: casters {4}; sections {1 (parents), 2 (gatch), 3 (profiling)}.
- 模板建议 N_range: casters {4}; profiling sections {1..3} discrete (Slot A). Not a free continuous N.
- copied object / naming / placement / joint policy: caster_{i} at 4 corners FIXED to frame; deck sections chained head→foot each REVOLUTE (y) about the boundary rail with the neighboring section.

## 视觉多样性 6 轴考察
| 轴 | 处理 | 本小类取值 / 范围 / 理由 |
|---|---|---|
| ① 骨架图(+N) | forked_anchor | frame → 1..3 hinged deck sections; side rails as optional child pair. |
| ② 关节类型 | forked_anchor | REVOLUTE (section tilts, side-rail drop), PRISMATIC (hi-lo column lift). |
| ③ 主体形态家族 | forked_anchor + world_knowledge_extrapolation | end-board form (tubular / open / panel); base form (legs / casters / column). |
| ④ 表面装饰 | record_only + world_knowledge_extrapolation | mattress seams, deck ratchet slots, bumper corners, control label decals. |
| ⑤ 尺寸/行程 | record_only | deck ~1.9×0.8 m; backrest tilt 0–70°; hi-lo lift travel 0.1–0.4 m. |
| ⑥ 涂装 | record_only | white/grey powder-coat frame, chrome, blue/green/beige mattress fabric; ≥5 colorways. |

## Compatibility Probes
| probe_id | source_type | record_id | 组合轴值 | 验证目标 | 结论 |
|---|---|---|---|---|---|
| (deferred) | — | — | three_section_profiling × dropdown_side_rails | rail clearance over profiling deck | template-side gate |

## 排除项
- none — all 5 forks converged; the hi_lo_column base-replacement rewrite succeeded (renders as a central elevating wheeled column base).
