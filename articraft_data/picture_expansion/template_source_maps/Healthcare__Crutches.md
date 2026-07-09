# Healthcare / Crutches — template source map

> NOTE ON IDENTITY: every reference image + every seeded original in this 小类 is a
> **walking cane / walking stick** (single-point, tripod, and quad-base canes), NOT an
> underarm/forearm crutch. The template identity is therefore **"walking cane"**. Underarm
> and forearm crutches are a structurally distinct object (double upright + axilla pad /
> forearm cuff + mid-shaft grip) and would leave the cane category if forked from a cane
> parent, so they are excluded here (see 排除项). Downstream spec should title this a
> walking-cane template while living under the picture 小类 "Crutches".

pattern: mixed (single named-slot chain: base/foot → shaft(telescoping) → handle; multiplicity only on the multi-foot bases)

parents (5 originals, all on-grid — each is a free Slot A/B anchor):
- rec_a-single-point-walking-cane-with-a-narrow-base-a_20260623_180041_173109_118f96a0  ← single-point ferrule tip, T/derby grip, telescoping (LatheGeometry tubes)
- rec_a-walking-cane-with-a-stabilizing-tripod-base-a-_20260623_175723_915348_0e009043  ← tripod 3-foot base, telescoping
- rec_a-quad-cane-with-a-small-rectangular-base-plate-_20260623_175732_600348_41768fed  ← quad 4-foot small rectangular base plate, telescoping
- rec_a-quad-cane-with-a-wide-four-leg-base-a-black-er_20260623_180237_292973_f6e00764  ← quad 4-foot WIDE splayed base, telescoping
- rec_a-walking-cane-with-a-bronze-copper-straight-sha_20260623_175632_065837_74ba54ab  ← single-point, bronze straight shaft, T-handle, telescoping

## Slot 候选覆盖

### Slot A: base / foot (③ primary form family — ALREADY richly covered by parents)
| 候选 (module) | source_type | record_id / evidence | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| single_point_ferrule | forked_anchor (parent) | cane_single / cane_bronze | lower_shaft + rubber ferrule tip | one rubber foot | converged |
| tripod_base | forked_anchor (parent) | cane_tripod | lower_sleeve + leg_{0..2} (3) | 3 splayed feet | converged |
| quad_small_base | forked_anchor (parent) | cane_quad_small | base_plate + foot_{0..3} | small rect plate, 4 feet | converged |
| quad_wide_base | forked_anchor (parent) | cane_quad_wide | base_hub + leg_{0..3} | wide splayed 4-leg base | converged |

### Slot B: handle / grip (primary fork axis this batch)
| 候选 (module) | source_type | record_id / evidence | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| t_derby_handle | forked_anchor (parent) | most parents | handle (ergonomic_t_handle/grip) | T / derby top | converged |
| crook_handle | forked_anchor | rec_cane_var_crook_handle | handle (curved swept hook) | round shepherd's-crook | converged |
| offset_handle | forked_anchor | rec_cane_var_offset_handle | handle (S-neck offset grip) | swan-neck offset grip | converged |
| fritz_handle | forked_anchor | rec_cane_var_fritz_handle | handle (palm/fritz grip) | anatomical palm grip | converged |

### Slot C: shaft / height mechanism
| 候选 (module) | source_type | record_id / evidence | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| telescoping_2piece | forked_anchor (parent) | all parents | upper_shaft ← PRISMATIC height | push-button telescoping | converged |
| folding_4section | forked_anchor | rec_cane_var_folding | shaft_seg_{0..3} + fold_joint_{0..2} REVOLUTE | 4-segment folding | converged |

## Multiplicity / Copy Logic
- count_param: foot_count (only on multi-foot bases: tripod=3, quad=4). Single-point base has no copy loop.
- N 样本已覆盖: {3 → cane_tripod, 4 → cane_quad_small / cane_quad_wide}
- 模板建议 N_range: base-dependent discrete set {1 (single-point), 3 (tripod), 4 (quad)} — not a free continuous range.
- copied object / naming / placement / joint policy: foot_{i} / leg_{i}, radial or rectangular placement around the base hub, each foot FIXED to the base (rigid), rubber ferrule on each.

## 视觉多样性 6 轴考察
| 轴 | 处理 | 本小类取值 / 范围 / 理由 |
|---|---|---|
| ① 骨架图(+N) | forked_anchor | base→shaft→handle chain; foot multiplicity {1,3,4}. No world-knowledge-only skeleton added. |
| ② 关节类型 | forked_anchor | PRISMATIC (telescoping height) on all; REVOLUTE fold-joints on the folding variant. |
| ③ 主体形态家族 | forked_anchor + world_knowledge_extrapolation | base-form anchors: single/tripod/quad-small/quad-wide (Volumetric Envelope of the base). Handle anchors: T/crook/offset/fritz (Planar Boundary of the grip). Template may extrapolate close handle sub-forms (e.g. derby vs fritz nuance). |
| ④ 表面装饰 | record_only + world_knowledge_extrapolation | height-hole rows on the telescoping tube, wrist-strap eyelet, knurled collar bands, brand ring — host-conformal only. |
| ⑤ 尺寸/行程 | record_only | shaft length ~0.72–0.95 m; telescoping travel ~0.08–0.15 m; base footprint ~0.05–0.20 m. |
| ⑥ 涂装 | record_only | anodized aluminium (black / bronze / silver / champagne), chrome, lacquered wood; grip in black foam / rubber / tan. ≥5 colorways. |

## Compatibility Probes
(none — handle × base combinations are all real and freely composable; template sampler covers them.)

## 排除项
- underarm (axillary) crutch and forearm (Lofstrand) crutch: structurally distinct object (double upright / axilla pad / forearm cuff + mid-shaft grip); forking from a cane parent would leave category. Excluded from this cane-identity pool; would need their own seeds if the 小类 is later re-scoped to true crutches.
- fixed one-piece wooden cane (no telescoping): would have zero non-fixed joints → violates ≥1-joint rule; represented instead by the folding variant for the non-telescoping structural story.
