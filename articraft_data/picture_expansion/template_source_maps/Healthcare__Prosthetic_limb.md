# Healthcare / Prosthetic limb — template source map

> IDENTITY: below-knee (trans-tibial) prosthetic LEG. Both seeded originals are legs; the
> template models a prosthetic leg (socket → pylon → foot chain, optional knee for above-knee).
> Prosthetic ARMS/hands are a structurally distinct object and are excluded (see 排除项).

pattern: linear_chain (socket → [knee] → pylon → ankle → foot)

parents (2 originals):
- rec_a-below-knee-trans-tibial-prosthetic-leg-a-conto_20260623_174436_819830_1fb23a2c  ← socket + adapter + pylon + carbon running blade (ankle REVOLUTE); mesh socket shell
- rec_a-modern-below-knee-prosthetic-leg-with-a-blue-s_20260623_174436_819367_a594c50c  ← blue socket + articulated foot with toe + grab handle (2 REVOLUTE: ankle + toe/foot)

## Slot 候选覆盖

### Slot A: foot / terminal device (③ primary form family)
| 候选 | source_type | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| running_blade | forked_anchor (parent) | prosth_tibial | blade ← ankle REVOLUTE | carbon J-blade | converged |
| articulated_foot | forked_anchor (parent) | prosth_blue | foot ← REVOLUTE ankle + toe | jointed foot w/ toe | converged |
| sach_foot | forked_anchor | rec_prosthetic_var_sach_foot | sach_foot ← ankle REVOLUTE | solid foam SACH foot | converged |

### Slot B: knee (below-knee vs above-knee)
| 候选 | source_type | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| below_knee_none | forked_anchor (parent) | both | socket→pylon fixed | trans-tibial, no knee | converged |
| above_knee_polycentric | forked_anchor | rec_prosthetic_var_above_knee | knee_joint ← REVOLUTE (socket_to_knee_joint) | trans-femoral knee | converged |

### Slot C: pylon / shank
| 候选 | source_type | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| exposed_tube_pylon | forked_anchor (parent) | both | pylon / leg tube | bare metal shank | converged |
| shock_pylon | forked_anchor | rec_prosthetic_var_shock_pylon | shock_pylon ← PRISMATIC z | telescoping damper | converged |
| foam_cosmesis_cover | forked_anchor | rec_prosthetic_var_foam_cover | cosmesis_cover (shell) | lifelike leg cover | converged |

## Multiplicity / Copy Logic
- count_param: none — this is a single named-slot chain (socket/adapter/pylon/foot), no N-copy of identical sub-parts.
- N 样本已覆盖: n/a
- 模板建议 N_range: n/a (linear chain)
- copied object / naming / placement / joint policy: socket→adapter→pylon→foot along z, ankle REVOLUTE (y) at the foot; optional knee REVOLUTE (y) between socket and pylon.

## 视觉多样性 6 轴考察
| 轴 | 处理 | 本小类取值 / 范围 / 理由 |
|---|---|---|
| ① 骨架图(+N) | forked_anchor | socket → [knee] → pylon → foot chain. No world-only skeleton added. |
| ② 关节类型 | forked_anchor | REVOLUTE (ankle, knee, toe), PRISMATIC (shock pylon). |
| ③ 主体形态家族 | forked_anchor + world_knowledge_extrapolation | foot: blade/articulated/SACH (Volumetric Envelope); pylon exposed vs foam-covered (Macro Surface). Template may extrapolate multi-axial foot. |
| ④ 表面装饰 | record_only + world_knowledge_extrapolation | carbon-weave panels, socket trim line, cuff strap, alignment screws, brand bands. |
| ⑤ 尺寸/行程 | record_only | shank length 0.20–0.45 m; ankle flex ±20°; knee flex 0–90°; shock travel small. |
| ⑥ 涂装 | record_only | carbon black + blue/red accent, titanium/brushed alloy, tan foam liner, skin-tone cosmesis; ≥5 colorways. |

## Compatibility Probes
| probe_id | source_type | record_id | 组合轴值 | 验证目标 | 结论 |
|---|---|---|---|---|---|
| (deferred) | — | — | above_knee_polycentric × running_blade | sprint-knee + blade alignment | template-side, both real |

## 排除项
- prosthetic arm / hand / hook terminal device: structurally distinct object (would leave the "prosthetic leg" reading of every seed); excluded — needs its own seeds if 小类 later broadened.
- none — all forks converged.
