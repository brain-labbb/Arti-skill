# pva: Table 4 full-release evaluation

Status: **COMPLETE**

N_eval: 2  \  J_eval: 3

| Metric | Result |
|---|---:|
| Rest All-pair CF | 0 / 2 |
| Rest Non-adjacent CF | 1 / 2 |
| Single-joint Sweep CF | 1 / 2 |
| Multi-joint Sobol CF | 1 / 2 |
| Collision-state Rate | 80 / 193 |
| AOR | N/E |
| Max Penetration | 0.13075753776530602 |
| Collision-free Range | 21 / 63 |
| Strict Collision Pass | 1 / 2 |

Collision-dependent metrics are N/E when native collision geometry is absent. Unexecuted states remain fail-closed in denominators; AOR is N/E because no exact overlap-volume backend is registered.
