# Structural Integrity Evaluation

The first six columns below contain the existing Core-200 pilot values. The
three collision columns must be filled by the new full-release protocol.

| Dataset | Rooted Assets (%) up | Joint Support macro (%) up | Joint Gap P95 (% diag.) down | Axis Rooted Assets (%) up | Axis Support macro (%) up | K=9 Axis Pose Support macro (%) up | Collision-Free Joint Motion Range (%) up | Premature Collision-Free Joints (%) up | Penetration Growth P95 (% diag.) down |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| PV-A | 21.5 | 52.47 | 0.949 | 53.5 | 78.21 | pending macro re-aggregation (pooled pilot: 57.87) | pending | pending | pending |
| Articraft | 17.0 | 43.68 | 0.716 | 44.0 | 67.05 | pending macro re-aggregation (pooled pilot: 37.86) | pending | pending | pending |

Do not mix the Core-200 pilot cells with full-release collision cells in a
publication table. The final table must be regenerated from one frozen full
manifest and one frozen protocol using `aggregate.py`.
