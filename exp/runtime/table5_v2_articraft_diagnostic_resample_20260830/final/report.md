# Table 5 revision-2 results

Run classification: **INCOMPLETE**.

## Table 5a: Genesis Simulation Readiness

| Dataset | N | Import Success (%) ↑ | DoF Coverage (%) ↑ | Stable Rollout (%) ↑ | Trajectory Coverage (%) ↑ | Tracking NRMSE P95 (%) ↓ | Limit Violation P95 (%) ↓ |
|---|---:|---:|---:|---:|---:|---:|---:|
| Articraft-10K | 200 | 98.00 | 99.49 | 97.50 | 100.00 | 76.9149 (415/415) | 24.9453 (415/415) |
| LAM released outputs | 200 | 0.00 | 0.00 | 0.00 | 0.00 | N/E (0/379) | N/E (0/379) |
| Artiverse | 200 | 0.00 | 0.00 | 0.00 | 0.00 | N/E (0/600) | N/E (0/600) |
| PartNet-Mobility | 200 | 0.00 | 0.00 | 0.00 | 0.00 | N/E (0/396) | N/E (0/396) |
| PhysX-Mobility | 200 | 0.00 | 0.00 | 0.00 | 0.00 | N/E (0/479) | N/E (0/479) |
| SketchMobility | 200 | 0.00 | 0.00 | 0.00 | 0.00 | N/E (0/292) | N/E (0/292) |
| Infinigen-Sim | 200 | 0.00 | 0.00 | 0.00 | 0.00 | N/E (0/813) | N/E (0/813) |
| Ours (PV-A) | 200 | 0.00 | 0.00 | 0.00 | 0.00 | N/E (0/628) | N/E (0/628) |

## Table 5b: Cross-Simulator Portability

| Dataset | N | Genesis Import Success (%) ↑ | Genesis DoF (%) ↑ | Genesis Stable Rollout (%) ↑ | PyBullet Import Success (%) ↑ | PyBullet DoF (%) ↑ | PyBullet Stable Rollout (%) ↑ | MuJoCo Import Success (%) ↑ | MuJoCo DoF (%) ↑ | MuJoCo Stable Rollout (%) ↑ |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Articraft-10K | 200 | 98.00 | 99.49 | 97.50 | 98.00 | 100.00 | 98.00 | 92.00 | 95.11 | 92.00 |
| LAM released outputs | 200 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 |
| Artiverse | 200 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 |
| PartNet-Mobility | 200 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 |
| PhysX-Mobility | 200 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 |
| SketchMobility | 200 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 |
| Infinigen-Sim | 200 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 |
| Ours (PV-A) | 200 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 |

**Table 5b, kinematic diagnostics**

| Dataset | Genesis FK Position P95 (% diag.) ↓ | PyBullet FK Position P95 (% diag.) ↓ | MuJoCo FK Position P95 (% diag.) ↓ | Genesis FK Rotation P95 (deg) ↓ | PyBullet FK Rotation P95 (deg) ↓ | MuJoCo FK Rotation P95 (deg) ↓ |
|---|---:|---:|---:|---:|---:|---:|
| Articraft-10K | 0.0000 (3295/3295) | 0.0000 (3295/3295) | 0.0000 (2980/3295) | 0.0000 (3295/3295) | 0.0000 (3295/3295) | 0.0000 (2980/3295) |
| LAM released outputs | N/E (0/3065) | N/E (0/3065) | N/E (0/3065) | N/E (0/3065) | N/E (0/3065) | N/E (0/3065) |
| Artiverse | N/E (0/5300) | N/E (0/5300) | N/E (0/5300) | N/E (0/5300) | N/E (0/5300) | N/E (0/5300) |
| PartNet-Mobility | N/E (0/0) | N/E (0/0) | N/E (0/0) | N/E (0/0) | N/E (0/0) | N/E (0/0) |
| PhysX-Mobility | N/E (0/5405) | N/E (0/5405) | N/E (0/5405) | N/E (0/5405) | N/E (0/5405) | N/E (0/5405) |
| SketchMobility | N/E (0/1415) | N/E (0/1415) | N/E (0/1415) | N/E (0/1415) | N/E (0/1415) | N/E (0/1415) |
| Infinigen-Sim | N/E (0/0) | N/E (0/0) | N/E (0/0) | N/E (0/0) | N/E (0/0) | N/E (0/0) |
| Ours (PV-A) | N/E (0/5115) | N/E (0/5115) | N/E (0/5115) | N/E (0/5115) | N/E (0/5115) | N/E (0/5115) |

Import Success records native asset-load acceptance only. Stable Rollout is an independent fixed-step zero-force passive finite-state test.
Continuous cells show `P95 (evaluated/candidate)`; N/E never counts as a successful continuous evaluation.
