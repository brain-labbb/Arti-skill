# Facade Element / Gutter downchain - template source map

pattern: repeated hanging modules
parents:
- rec_build-a-realistic-articulated-3d-model-of-a-gutt_20260609_185904_296819_efa584a4 <- picture/Facade Element/Gutter downchain/001.png (round cup downchain: hanger bracket, repeated round cup modules with bail links, cup-to-cup swing joints). PRIMARY parent. Covers Slot A=parent_cup_count, Slot B=round_cups baseline, Slot C=cup_swing_chain. Forked by cup_count_3, cup_count_8, link_chain, lotus_cups.
- rec_build-a-realistic-articulated-3d-model-of-a-gutt_20260609_185907_280318_3dcec91a <- picture/Facade Element/Gutter downchain/002.png (square funnel downchain: hanger with square/funnel cup modules and swing joints). Covers Slot B=square/funnel_cups baseline. Forked by round_cups.

Rain-chain family with hanger, repeated cup or link modules, and swing joints between visible hanging
segments. Variants isolate cup count, cup shape, and chain-link-only topology.

## Slot 候选覆盖

### Slot A:cup / module count
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| parent_count | parents | `cup_{i}` loop, `swing_{i}` | inherited cup count and spacing | converged |
| cup_count_3 | rec_gutter_downchain_var_cup_count_3 | `cup_{i}` loop (N_CUPS=3), `swing_{i}` | three cup modules | converged |
| cup_count_8 | rec_gutter_downchain_var_cup_count_8 | `cup_{i}` loop (N_CUPS=8), `swing_{i}` | eight cup modules | converged |

### Slot B:cup / hanging shape
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| square_funnel_cups | parents | `cup_{i}` shell/rim, `cup_{i}_bail` | square/funnel cup form (square-funnel parent 3dcec91a) | converged |
| round_bowl_cups | rec_gutter_downchain_var_round_cups | `cup_{i}` lathe shell, `swing_{i}` | round bowl cups with circular rims (forked from square-funnel parent) | converged |
| lotus_cups | rec_gutter_downchain_var_lotus_cups | `cup_{i}_shell`, `cup_{i}_rim` (scalloped) | flared lotus-shaped cups with scalloped rims | converged |
| link_chain_only | rec_gutter_downchain_var_link_chain | `link_{i}_oval`, `swing_{i}` | chain-link-only hanging form without cups | converged |

### Slot C:hanger / swing policy
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| cup_swing_chain | parents and cup variants | `swing_{i}` revolute joints | repeated cup modules connected by swing joints | converged |
| link_swing_chain | rec_gutter_downchain_var_link_chain | `link_{i}` parts, `swing_{i}` joints | repeated chain links with hanging swing policy | converged |

## Multiplicity / Copy Logic
- count_param: `N_CUPS` (cup chains) / `N_LINKS` (link-only chains).
- N 样本已覆盖: {3, 8, parent}.
- 模板建议 N_range: [3, 8] for cup chains; link-only chains can use a separate link_count range.
- copied object / naming / placement / joint policy: repeated modules use `cup_{i}` or `link_{i}` with vertical spacing and one revolute `swing_{i}` joint per module.

## 组合数预审
Slot A(3) x Slot B(4) x Slot C(2) = 24 >= 10 ✓.

## 排除项(未来 compatibility matrix 素材)
- No blocked cells in this batch; all planned gutter downchain variants converged.
- cup_count is not directly meaningful for link_chain_only; use link_count if that candidate is selected.
- Round/lotus cup shapes require curved geometry generation rather than box placeholders.
