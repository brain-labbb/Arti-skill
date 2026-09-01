# pva: Table 4 full-release evaluation

Status: **COMPLETE**

N_eval: 384  \  J_eval: 5719

| Metric | Result |
|---|---:|
| Rest All-pair CF | 0 / 384 |
| Rest Non-adjacent CF | 135 / 384 |
| Single-joint Sweep CF | 52 / 384 |
| Multi-joint Sobol CF | 30 / 384 |
| Collision-state Rate | 112451 / 145059 |
| AOR | N/E |
| Max Penetration | 0.26528117304242166 |
| Collision-free Range | 23558 / 120099 |
| Strict Collision Pass | 25 / 384 |

Collision-dependent metrics are N/E when native collision geometry is absent. Unexecuted states remain fail-closed in denominators; AOR is N/E because no exact overlap-volume backend is registered.
