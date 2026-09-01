# Articraft diagnostic resample vs formal Table 5

## Table 5a (genesis)

| Metric | Formal sample | Diagnostic sample |
|---|---:|---:|
| Import success (%) | 100.00 | 98.00 |
| Stable rollout (%) | 100.00 | 97.50 |
| DoF coverage (%) | 100.00 | 99.49 |
| Trajectory coverage (%) | 100.00 | 100.00 |
| Tracking NRMSE P95 (%) | 76.91 (463/463) | 76.91 (415/415) |
| Limit violation P95 (%) | 39.26 (463/463) | 24.95 (415/415) |

## Table 5b (per simulator)

| Simulator | Metric | Formal | Diagnostic |
|---|---|---:|---:|
| genesis | import % | 100.00 | 98.00 |
| genesis | stable % | 100.00 | 97.50 |
| pybullet | import % | 100.00 | 98.00 |
| pybullet | stable % | 100.00 | 98.00 |
| mujoco | import % | 100.00 | 92.00 |
| mujoco | stable % | 100.00 | 92.00 |

## Per-stratum breakdown (diagnostic sample)

### genesis

| Stratum | n | import % | stable % | actuated joints | act pass % | NRMSE median % | NRMSE P95 % | limit joints | limit pass % |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| F | 4 | 0.0 | 0.0 | 0 | N/E | N/E | N/E | 0 | N/E |
| C | 13 | 100.0 | 92.3 | 28 | 50.0 | 76.11 | 391.27 | 28 | 46.4 |
| I | 40 | 100.0 | 100.0 | 111 | 73.0 | 15.60 | 76.92 | 111 | 18.9 |
| R | 143 | 100.0 | 100.0 | 276 | 75.0 | 17.06 | 75.64 | 276 | 20.7 |

### pybullet

| Stratum | n | import % | stable % | actuated joints | act pass % | NRMSE median % | NRMSE P95 % | limit joints | limit pass % |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| F | 4 | 0.0 | 0.0 | 0 | N/E | N/E | N/E | 0 | N/E |
| C | 13 | 100.0 | 100.0 | 28 | 28.6 | 75.70 | 183.93 | 28 | 64.3 |
| I | 40 | 100.0 | 100.0 | 111 | 79.3 | 29.83 | 80.98 | 111 | 39.6 |
| R | 143 | 100.0 | 100.0 | 276 | 67.0 | 21.10 | 91.30 | 276 | 23.2 |

### mujoco

| Stratum | n | import % | stable % | actuated joints | act pass % | NRMSE median % | NRMSE P95 % | limit joints | limit pass % |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| F | 4 | 0.0 | 0.0 | 0 | N/E | N/E | N/E | 0 | N/E |
| C | 13 | 7.7 | 7.7 | 0 | N/E | N/E | N/E | 0 | N/E |
| I | 40 | 100.0 | 100.0 | 111 | 49.5 | 44.72 | 185.35 | 111 | 42.3 |
| R | 143 | 100.0 | 100.0 | 276 | 57.6 | 35.79 | 357.53 | 276 | 30.4 |

