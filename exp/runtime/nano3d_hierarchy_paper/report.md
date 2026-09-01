# Table 3 matched-category hierarchy evaluation

This paper-facing panel evaluates five shared categories with six frozen samples per category and method: storage furniture/cabinet, table, refrigerator, dishwasher, and oven. Selection is fixed before compile or hierarchy evaluation; failures remain in the requested denominator and are never replaced.

| Method | Valid Tree | Has Hierarchy | Depth | Groups | Pivots |
|---|---:|---:|---:|---:|---:|
| PV-A | 27/30 | 27/30 | 2.222 [2,3], N=27 | 0.000, N=27 | 4.407, N=27 |
| LAM official release | 29/30 | 29/30 | 2.567 [2,5], N=30 | 0.000, N=30 | 3.733, N=30 |
| Articraft-10K official release | 30/30 | 30/30 | 2.833 [2,5], N=30 | 0.000, N=30 | 6.467, N=30 |
| Infinite Mobility | 30/30 | 30/30 | 3.500 [2,5], N=30 | 3.167, N=30 | 8.467, N=30 |

PV-A and Infinite Mobility use frozen seeds 0-5. LAM uses an identity-only SHA-256 rank over explicit official-release category allowlists. Articraft first applies the paper harness's released rating 4-5 retained-set definition, then uses an identity-only SHA-256 rank. LAM and Articraft are deterministic official-release re-evaluations, not new common-prompt generation runs; Articraft's selected records are freshly compiled with the pinned paper harness.

Parent-Child Edge F1, Hierarchy Exact Match, and Semantic Nesting Accuracy are N/A for every method because no independent hierarchy gold is available. Cross-seed signatures measure topology stability only, not semantic correctness.

## Extended structure diagnostics

| Method | Nodes | Edges | Leaves | Branch nodes | Movable / Fixed edges | Visual / Collision coverage |
|---|---:|---:|---:|---:|---:|---:|
| PV-A | 5.407 | 4.407 | 4.185 | 0.963 | 4.407 / 0.000 | 1.000 / 1.000 |
| LAM | 6.379 | 5.379 | 4.517 | 1.414 | 3.621 / 1.759 | 1.000 / 0.828 |
| Articraft-10K | 7.900 | 6.900 | 5.400 | 1.733 | 6.467 / 0.433 | 1.000 / 1.000 |
| Infinite Mobility | 15.500 | 14.500 | 11.800 | 1.200 | 8.467 / 6.033 | 0.768 / 0.000 |

All values above are per valid-tree asset. Root/component/cycle/malformed/multi-parent defect counts are zero except for the single LAM asset, which has one root defect and one component defect.

## Within-category canonical topology

| Method | Unique rate | Mode rate | Pairwise exact | Normalized entropy |
|---|---:|---:|---:|---:|
| PV-A | 0.667 | 0.433 | 0.200 | 0.690 |
| LAM | 0.833 | 0.307 | 0.080 | 0.861 |
| Articraft-10K | 0.933 | 0.233 | 0.027 | 0.948 |
| Infinite Mobility | 0.667 | 0.433 | 0.227 | 0.681 |

The canonical signature removes semantic names and sibling order while retaining rooted shape, exact joint type, and visual/group role. Comparisons are within category and macro-averaged over the five categories. Diversity and consistency are descriptive, not correctness scores.
