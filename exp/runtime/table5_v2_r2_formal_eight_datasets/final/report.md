# Table 5 revision-2 results

Run classification: **INCOMPLETE**.

## Table 5a: Genesis Simulation Readiness

| Dataset | N | Import Success (%) ↑ | DoF Coverage (%) ↑ | Stable Rollout (%) ↑ | Trajectory Coverage (%) ↑ | Tracking NRMSE P95 (%) ↓ | Limit Violation P95 (%) ↓ |
|---|---:|---:|---:|---:|---:|---:|---:|
| Articraft-10K | 200 | 100.00 | 100.00 | 100.00 | 100.00 | 76.9127 (463/463) | 39.2613 (463/463) |
| LAM released outputs | 200 | 87.50 | 98.93 | 87.00 | 95.78 | 83.7739 (363/379) | 17.0934 (363/379) |
| Artiverse | 200 | 99.50 | 99.21 | 96.00 | 95.50 | 70.8101 (573/600) | 28.7315 (573/600) |
| PartNet-Mobility | 200 | 95.00 | 92.02 | 94.50 | 0.00 | N/E (0/396) | N/E (0/396) |
| PhysX-Mobility | 200 | 100.00 | 100.00 | 99.50 | 100.00 | 618.2454 (479/479) | 779.3137 (479/479) |
| SketchMobility | 200 | 99.50 | 99.02 | 99.50 | 80.14 | 1737.8529 (234/292) | 336.7446 (234/292) |
| Infinigen-Sim | 200 | 93.00 | 90.28 | 93.00 | 0.00 | N/E (0/813) | N/E (0/813) |
| Ours (PV-A) | 200 | 100.00 | 100.00 | 100.00 | 99.36 | 76.9378 (624/628) | 32.3548 (624/628) |

## Table 5b: Cross-Simulator Portability

| Dataset | N | Genesis Import Success (%) ↑ | Genesis DoF (%) ↑ | Genesis Stable Rollout (%) ↑ | PyBullet Import Success (%) ↑ | PyBullet DoF (%) ↑ | PyBullet Stable Rollout (%) ↑ | MuJoCo Import Success (%) ↑ | MuJoCo DoF (%) ↑ | MuJoCo Stable Rollout (%) ↑ |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Articraft-10K | 200 | 100.00 | 100.00 | 100.00 | 100.00 | 100.00 | 100.00 | 100.00 | 100.00 | 100.00 |
| LAM released outputs | 200 | 87.50 | 98.93 | 87.00 | 88.00 | 99.36 | 88.00 | 44.00 | 54.60 | 42.00 |
| Artiverse | 200 | 99.50 | 99.21 | 96.00 | 100.00 | 100.00 | 100.00 | 85.00 | 85.51 | 84.50 |
| PartNet-Mobility | 200 | 95.00 | 92.02 | 94.50 | 98.50 | 99.37 | 98.50 | 42.00 | 25.82 | 42.00 |
| PhysX-Mobility | 200 | 100.00 | 100.00 | 99.50 | 100.00 | 100.00 | 99.50 | 100.00 | 100.00 | 99.50 |
| SketchMobility | 200 | 99.50 | 99.02 | 99.50 | 55.00 | 68.05 | 55.00 | 85.50 | 74.88 | 85.50 |
| Infinigen-Sim | 200 | 93.00 | 90.28 | 93.00 | 100.00 | 100.00 | 100.00 | 99.50 | 99.51 | 99.50 |
| Ours (PV-A) | 200 | 100.00 | 100.00 | 100.00 | 100.00 | 100.00 | 100.00 | 100.00 | 99.76 | 99.50 |

**Table 5b, kinematic diagnostics**

| Dataset | Genesis FK Position P95 (% diag.) ↓ | PyBullet FK Position P95 (% diag.) ↓ | MuJoCo FK Position P95 (% diag.) ↓ | Genesis FK Rotation P95 (deg) ↓ | PyBullet FK Rotation P95 (deg) ↓ | MuJoCo FK Rotation P95 (deg) ↓ |
|---|---:|---:|---:|---:|---:|---:|
| Articraft-10K | 0.0000 (3595/3595) | 0.0000 (3595/3595) | 0.0000 (3465/3595) | 0.0000 (3595/3595) | 0.0000 (3595/3595) | 0.0000 (3465/3595) |
| LAM released outputs | 0.0000 (3050/3065) | 0.0000 (3050/3065) | 0.0000 (1395/3065) | 0.0000 (3050/3065) | 0.0000 (3050/3065) | 0.0000 (1395/3065) |
| Artiverse | 0.0000 (5215/5300) | 0.0000 (5300/5300) | 0.0000 (2893/5300) | 0.0000 (5215/5300) | 0.0000 (5300/5300) | 0.0000 (2893/5300) |
| PartNet-Mobility | N/E (0/0) | N/E (0/0) | N/E (0/0) | N/E (0/0) | N/E (0/0) | N/E (0/0) |
| PhysX-Mobility | 0.0000 (5405/5405) | 0.0000 (5405/5405) | 0.0000 (2655/5405) | 0.0000 (5405/5405) | 0.0000 (5405/5405) | 0.0000 (2655/5405) |
| SketchMobility | 0.0000 (1415/1415) | 0.0000 (825/1415) | 0.0000 (1225/1415) | 0.0000 (1415/1415) | 0.0000 (825/1415) | 0.0000 (1225/1415) |
| Infinigen-Sim | N/E (0/0) | N/E (0/0) | N/E (0/0) | N/E (0/0) | N/E (0/0) | N/E (0/0) |
| Ours (PV-A) | 0.0000 (5115/5115) | 0.0000 (5115/5115) | 0.0000 (4840/5115) | 0.0000 (5115/5115) | 0.0000 (5115/5115) | 0.0000 (4840/5115) |

Import Success records native asset-load acceptance only. Stable Rollout is an independent fixed-step zero-force passive finite-state test.
Continuous cells show `P95 (evaluated/candidate)`; N/E never counts as a successful continuous evaluation.
