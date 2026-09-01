# pva: Table 4 full-release evaluation

Status: **COMPLETE_WITH_RETAINED_FAILURES**

N_eval: 302440  \  J_eval: 1453516

| Metric | Result |
|---|---:|
| Rest All-pair CF | 15497 / 302440 |
| Rest Non-adjacent CF | 227963 / 302440 |
| Single-joint Sweep CF | 208637 / 302440 |
| Multi-joint Sobol CF | 206908 / 302440 |
| Collision-state Rate | 12736434 / 48006121 |
| AOR | N/E |
| Max Penetration | 0.9609308866413696 |
| Collision-free Range | 20322889 / 28347585 |
| Strict Collision Pass | 204468 / 302440 |

Collision-dependent metrics are N/E when native collision geometry is absent. Unexecuted states remain fail-closed in denominators; AOR is N/E because no exact overlap-volume backend is registered.
