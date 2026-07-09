# Facade Element / Lamp1 - template source map

pattern: mixed wall lantern
parents:
- rec_build-a-realistic-articulated-3d-model-of-a-lamp_20260609_185910_557031_f8540f5f <- picture/Facade Element/Lamp1/001.png (wall lantern with conical/roofed body, bracket, hook/chain, swing joint). Covers Slot A=wall_hook_arm, Slot B=parent_lantern_body, Slot C=single_head.
- rec_build-a-realistic-articulated-3d-model-of-a-lamp_20260609_185914_197581_ece7fee7 <- picture/Facade Element/Lamp1/002.png (cylindrical wall lantern with cage/glass evidence). Covers cylindrical lantern body and alternate roof/bracket evidence.

Facade wall-lantern family with backplate/bracket, arm or chain suspension, lantern body/cage, cap,
and optional head multiplicity. Variants isolate support arm, lantern body/cap, suspension length,
and double-head layout.

## Slot 候选覆盖

### Slot A:arm / suspension
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| wall_hook_arm | parents | backplate, hook/arm, swing joint | inherited bracket/hook wall lantern support | converged |
| gooseneck_arm | rec_lamp1_var_gooseneck_arm | curved arm geometry | curved gooseneck wall arm | converged |
| chain_drop | rec_lamp1_var_chain_drop | chain links/suspension | longer chain-drop suspension | converged |

### Slot B:lantern body / cap
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| parent_lantern_body | parents | glass/cage/body visuals | inherited lantern body style | converged |
| caged_cylinder_lantern | rec_lamp1_var_caged_cylinder_lantern | cylinder glass, vertical bar loop | cylindrical caged lantern body | converged |
| conical_roof | rec_lamp1_var_conical_roof | conical roof cap, finial | conical metal roof cap over lantern body | converged |

### Slot C:head multiplicity
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| single_lantern | parents and single variants | lantern assembly | one lantern head on wall bracket | converged |
| double_lantern | rec_lamp1_var_double_lantern | `lantern_{i}` assemblies | two lantern heads on one bracket | converged |

## Multiplicity / Copy Logic
- count_param: `lantern_count`; local chain/bar counts are candidate-local.
- N 样本已覆盖: lantern_count {1, 2}.
- 模板建议 N_range: [1, 2] for wall-mounted Lamp1 evidence.
- copied object / naming / placement / joint policy: double heads should be emitted as `lantern_{i}` assemblies with mirrored placement on a shared bracket and identical swing/fixture policy.

## 组合数预审
Slot A(3) x Slot B(3) x Slot C(2) = 18 >= 10 ✓.

## 排除项(未来 compatibility matrix 素材)
- No blocked cells in this batch; all planned Lamp1 variants converged.
- double_lantern may require bracket widening; keep this change scoped to support/head layout, not body style.
- chain_drop and gooseneck_arm are mutually exclusive support-arm candidates unless a future hybrid is intentionally sampled.
