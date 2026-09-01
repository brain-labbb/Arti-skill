# Healthcare / Wheelchair — template source map

pattern: parallel_children (rigid frame hub with paired wheel/caster/footplate/armrest children; L/R mirrored multiplicity)

parents (1 original):
- rec_a-manual-wheelchair-two-large-rear-drive-wheels-_20260623_174436_818325_dcd9a412  ← manual self-propel wheelchair: tubular frame + sling seat/back, 2 large spoked rear drive wheels (push-rim), 2 front swivel casters, flip-up twin footplates, fixed tubular armrests

parent part tree: frame · {left,right}_rear_wheel (CONTINUOUS y) · {l,r}_caster_fork (CONTINUOUS z swivel) · {l,r}_caster_wheel (CONTINUOUS y) · {l,r}_footplate (REVOLUTE flip-up)

## Slot 候选覆盖

### Slot A: rear drive wheel / propulsion
| 候选 | source_type | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| large_pushrim_manual | forked_anchor (parent) | wheelchair | {l,r}_rear_wheel CONTINUOUS + push_rim | self-propel spoked wheel | converged |
| small_transit | forked_anchor | rec_wheelchair_var_transit | small rear wheels + attendant handles | attendant-push, no push-rim | converged |
| powered_drive | forked_anchor | rec_wheelchair_var_powered | motor housings + battery box + joystick | electric power chair | converged (least-distinct; joystick+motor housing present — verify at gate) |

### Slot B: footrest
| 候选 | source_type | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| swingaway_flipup | forked_anchor (parent) | wheelchair | {l,r}_footplate REVOLUTE | twin flip-up plates | converged |
| elevating_legrest | forked_anchor | rec_wheelchair_var_elevating_legrest | {l,r}_legrest REVOLUTE + calf pad | raising calf rest | converged |

### Slot C: backrest
| 候选 | source_type | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| fixed_sling_back | forked_anchor (parent) | wheelchair | frame sling back (fixed) | standard low back | converged |
| reclining_back | forked_anchor | rec_wheelchair_var_reclining_back | backrest REVOLUTE + headrest | high reclining back | converged |

### Slot D: armrest
| 候选 | source_type | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| fixed_tubular_arm | forked_anchor (parent) | wheelchair | frame armrest visuals (fixed) | fixed full-length arm | converged |
| desk_flipback_arm | forked_anchor | rec_wheelchair_var_desk_armrest | {l,r}_armrest REVOLUTE | flip-back desk arm | converged |

### Slot E: frame (rigid vs folding)
| 候选 | source_type | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| rigid_frame | forked_anchor (parent) | wheelchair | tubular_frame (rigid) | non-folding | converged |
| cross_brace_folding | forked_anchor | rec_wheelchair_var_folding_xframe | fold_brace_{l,r} REVOLUTE scissor | X-brace collapse | converged |

## Multiplicity / Copy Logic
- count_param: side (L/R mirror pair) for wheels/casters/footplates/armrests — always 2, mirrored.
- N 样本已覆盖: 2 (L/R) throughout; rear wheels 2, front casters 2.
- 模板建议 N_range: fixed L/R pairs (2); front casters {2}; not a free multiplicity axis.
- copied object / naming / placement / joint policy: `{label}_<part>` for label in (left,right), mirrored across x=0, each wheel its own CONTINUOUS roll, casters CONTINUOUS swivel+roll.

## 视觉多样性 6 轴考察
| 轴 | 处理 | 本小类取值 / 范围 / 理由 |
|---|---|---|
| ① 骨架图(+N) | forked_anchor | frame hub + mirrored L/R children; folding X-brace as alt skeleton. |
| ② 关节类型 | forked_anchor | CONTINUOUS (wheels/casters), REVOLUTE (footplate/legrest/backrest/armrest/fold). |
| ③ 主体形态家族 | forked_anchor + world_knowledge_extrapolation | manual/transit/powered propulsion; rigid vs folding frame. Template may extrapolate wheel-size intermediates. |
| ④ 表面装饰 | record_only + world_knowledge_extrapolation | spoke count/pattern, upholstery ribs, brake levers, anti-tip bars, side skirt guards. |
| ⑤ 尺寸/行程 | record_only | seat width ~0.40–0.56 m; rear wheel dia 0.30–0.61 m; footplate flip 0–90°. |
| ⑥ 涂装 | record_only | chrome/powder-coat frame (blue, black, red, grey), upholstery (navy, black, teal); ≥5 colorways. |

## Compatibility Probes
| probe_id | source_type | record_id | 组合轴值 | 验证目标 | 结论 |
|---|---|---|---|---|---|
| (deferred) | — | — | powered_drive × folding frame | motor pods vs fold clearance | template-side gate if both selected |

## 排除项
- none — all 6 forks converged and stay in category. powered_drive is the least-distinct (still spoked wheels) but adds a joystick pod + motor/gearbox housings; flag at the visual gate.
