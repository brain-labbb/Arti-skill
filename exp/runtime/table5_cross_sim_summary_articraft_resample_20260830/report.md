# Cross-simulator Table 5 summary

Articraft-10K = diagnostic resample re-run (20260830); other datasets = formal eight-dataset run.
Load means native simulator import success only; preserving every declared link or joint is not required.
Reset/Settling/Act/Limits/Drift/SimPass are legacy stress-test diagnostics, not the primary Table 5a metrics.

## All-3 metrics

| Dataset | N | All-3 Load (%) ↑ | All-3 Runtime (%) ↑ | Joint RMSE joints | Joint RMSE median ↓ | Joint RMSE P95 ↓ | Joint RMSE ≤0.1 pass (%) ↑ | Pose links | Pose trans/bbox P95 (%) ↓ | Pose rot P95 (deg) ↓ |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| articraft_10k | 200 | 92.00 | 0.00 | 415 | 0.3838 | 2.9302 | 28.92 | 659 | 62.07 | 154.075 |
| lam_released_outputs | 200 | 41.50 | 0.00 | 375 | 0.2733 | 1.5798 | 22.93 | 610 | 48.90 | 163.026 |
| artiverse | 200 | 85.00 | 0.00 | 587 | 0.6865 | 2.3192 | 12.78 | 1042 | 32.19 | 94.221 |
| partnet_mobility | 200 | 42.00 | 0.00 | 0 | N/E | N/E | N/E | 0 | N/E | N/E |
| physx_mobility | 200 | 100.00 | 0.00 | 479 | 0.0002 | 18.7585 | 73.07 | 1081 | 2.03 | 1.126 |
| sketchmobility | 200 | 42.00 | 0.00 | 231 | 0.5028 | 576.2782 | 22.51 | 280 | 1401.72 | 171.710 |
| infinigen_sim | 200 | 93.00 | 0.00 | 0 | N/E | N/E | N/E | 0 | N/E | N/E |
| pva | 200 | 100.00 | 0.00 | 628 | 0.6242 | 18.9282 | 14.33 | 1019 | 44.81 | 166.668 |

## Per-simulator legacy diagnostics (genesis), % of 200

| Dataset | Load | Reset | Settling | Act | Limits | Drift | SimPass |
|---|---:|---:|---:|---:|---:|---:|---:|
| articraft_10k | 98.00 | 97.50 | 38.50 | 33.00 | 6.50 | 57.00 | 0.50 |
| lam_released_outputs | 87.50 | 87.00 | 55.00 | 37.50 | 18.00 | 62.50 | 5.00 |
| artiverse | 99.50 | 96.00 | 79.50 | 72.50 | 5.00 | 89.50 | 0.00 |
| partnet_mobility | 95.00 | 95.00 | 95.00 | 0.00 | 0.00 | 0.00 | 0.00 |
| physx_mobility | 100.00 | 99.50 | 34.00 | 98.50 | 2.00 | 99.50 | 0.50 |
| sketchmobility | 99.50 | 99.50 | 65.00 | 53.50 | 4.00 | 66.00 | 1.50 |
| infinigen_sim | 93.00 | 93.00 | 93.00 | 0.00 | 0.00 | 0.00 | 0.00 |
| pva | 100.00 | 99.50 | 47.00 | 33.00 | 6.00 | 55.00 | 1.00 |

## Per-simulator legacy diagnostics (pybullet), % of 200

| Dataset | Load | Reset | Settling | Act | Limits | Drift | SimPass |
|---|---:|---:|---:|---:|---:|---:|---:|
| articraft_10k | 98.00 | 98.00 | 41.00 | 29.00 | 9.50 | 57.00 | 2.00 |
| lam_released_outputs | 88.00 | 87.50 | 55.00 | 28.00 | 31.00 | 62.50 | 11.50 |
| artiverse | 100.00 | 100.00 | 80.50 | 60.50 | 5.00 | 93.50 | 2.50 |
| partnet_mobility | 98.50 | 98.50 | 98.50 | 0.00 | 0.00 | 0.00 | 0.00 |
| physx_mobility | 100.00 | 99.50 | 34.00 | 98.50 | 2.00 | 99.50 | 0.00 |
| sketchmobility | 55.00 | 55.00 | 42.50 | 19.00 | 6.50 | 23.00 | 3.50 |
| infinigen_sim | 100.00 | 100.00 | 100.00 | 0.00 | 0.00 | 0.00 | 0.00 |
| pva | 100.00 | 100.00 | 40.00 | 28.50 | 7.50 | 55.50 | 1.00 |

## Per-simulator legacy diagnostics (mujoco), % of 200

| Dataset | Load | Reset | Settling | Act | Limits | Drift | SimPass |
|---|---:|---:|---:|---:|---:|---:|---:|
| articraft_10k | 92.00 | 92.00 | 32.50 | 16.50 | 9.00 | 12.00 | 0.00 |
| lam_released_outputs | 44.00 | 42.00 | 9.50 | 4.50 | 9.00 | 5.00 | 0.00 |
| artiverse | 85.00 | 84.00 | 11.50 | 20.50 | 18.00 | 7.50 | 0.00 |
| partnet_mobility | 42.00 | 42.00 | 42.00 | 0.00 | 0.00 | 0.00 | 0.00 |
| physx_mobility | 100.00 | 99.50 | 34.00 | 98.50 | 1.00 | 0.00 | 0.00 |
| sketchmobility | 85.50 | 85.50 | 42.00 | 47.00 | 14.50 | 9.50 | 0.00 |
| infinigen_sim | 99.50 | 99.50 | 99.50 | 0.00 | 0.00 | 0.00 | 0.00 |
| pva | 100.00 | 99.50 | 34.00 | 7.00 | 18.00 | 10.50 | 0.00 |

