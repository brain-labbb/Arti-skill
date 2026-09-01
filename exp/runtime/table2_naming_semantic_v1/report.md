# Table 2 matched Naming semantic evaluation

Status: COMPLETE

Cohort: 4 methods x 5 categories x 7 assets = 140 assets; 1,107 renderable-link tasks.
Judging: three isolated method-blind Codex judge sessions; consensus requires at least two identical non-uncertain votes.
Field completeness: 72 anonymous items received an independent field-only re-review; 15/1,107 geometry-role fields that remained split were resolved by one fresh blind tie-break adjudicator with all prior votes hidden.

| Method | Precision micro | Recall asset-macro | Functional Richness asset-macro | Instance micro | Over-seg micro |
|---|---:|---:|---:|---:|---:|
| Ours | 1.000000 | 0.885714 | 0.885714 | 0.276596 | 0.017442 |
| LAM | 0.993080 | 0.871429 | 0.871429 | 0.966387 | 0.006920 |
| Articraft | 1.000000 | 0.833333 | 0.833333 | 0.504425 | 0.000000 |
| Infinite Mobility | 0.000000 | 0.000000 | 0.000000 | 0.000000 | 0.000000 |

Supplementary granularity-sensitive diagnostics:

| Method | Validated named-link density | Extra real parts / asset |
|---|---:|---:|
| Ours | 1.995238 | 1.828571 |
| LAM | 3.423810 | 5.771429 |
| Articraft | 2.490476 | 3.742857 |
| Infinite Mobility | 0.000000 | 0.000000 |

Verdict Fleiss kappa: 0.987141
Mean pairwise exact verdict agreement: 0.991569

The role gold was frozen without inspecting evaluated outputs. Optional-role absence is not penalized. Functional Richness follows the preexisting PV-A definition and does not reward extra real parts; named-link density and extra parts are supplementary. These are LLM-judge results, not human annotations. Cross-seed consistency remains a separate direct-output metric.
