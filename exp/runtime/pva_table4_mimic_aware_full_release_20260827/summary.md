# pva: Table 4 full-release evaluation

Status: **COMPLETE_WITH_RETAINED_FAILURES**

N_eval: 302440  \  J_eval: 1453516

| Metric | Result |
|---|---:|
| Rest All-pair CF | 3335 / 302440 |
| Rest Non-adjacent CF | 217613 / 302440 |
| Single-joint Sweep CF | 199037 / 302440 |
| Multi-joint Sobol CF | 197126 / 302440 |
| Collision-state Rate | 14574467 / 48113096 |
| AOR | N/E |
| Max Penetration | 0.8451219060259535 |
| Collision-free Range | 19187701 / 28454496 |
| Strict Collision Pass | 195017 / 302440 |

Collision-dependent metrics are N/E when native collision geometry is absent. Unexecuted states remain fail-closed in denominators; AOR is N/E because no exact overlap-volume backend is registered.
