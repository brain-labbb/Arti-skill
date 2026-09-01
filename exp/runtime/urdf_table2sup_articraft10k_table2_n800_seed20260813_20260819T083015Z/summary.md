# Articraft-10K Table 2 Supplementary Diagnostics

Run classification: **FORMAL**.

Exact Table 2 manifest cohort: N_eval=800 from N_release=9996, seed=20260813; J_eval=2865. Existing package order was preserved without resampling.

| Metric | Result |
|---|---:|
| visual_bearing_collision_coverage (asset) | 224 / 800 (28.00%) |
| visual_bearing_collision_coverage (link-micro) | 1222 / 3966 (30.81%) [link extraction coverage 800/800; COMPLETE] |
| joint_limit_portability | 2865 / 2865 (100.00%) |
| joint_dynamics_coverage | 79 / 2865 (2.76%) |
| placeholder_mass_incidence | N/E (placeholder registry is empty; no mass values are evaluable) |
| placeholder_mass_incidence complete-inertial coverage | 1365 / 3967 (34.41%) |

Category macro average over 222 observed categories (unweighted; joint metrics only over categories with >=1 declared movable joint):

| Metric | Category macro |
|---|---:|
| visual_bearing_collision_coverage | 29.78% (categories=222) |
| joint_limit_portability | 100.00% (categories=222) |
| joint_dynamics_coverage | 2.75% (categories=222) |
| placeholder_mass_incidence | N/E (registry empty) |

Declared-DoF bin breakdown (frozen bins 0 / 1 / 2-3 / 4-7 / >=8):

| Bin | Assets | Joints | VBCC asset pass | Portability | Dynamics |
|---|---:|---:|---:|---:|---:|
| 0 | 0 | 0 | 0 / 0 (N/A) | 0 / 0 (N/A) | 0 / 0 (N/A) |
| 1 | 163 | 163 | 31 / 163 (19.02%) | 163 / 163 (100.00%) | 5 / 163 (3.07%) |
| 2-3 | 407 | 1004 | 99 / 407 (24.32%) | 1004 / 1004 (100.00%) | 15 / 1004 (1.49%) |
| 4-7 | 177 | 874 | 70 / 177 (39.55%) | 874 / 874 (100.00%) | 46 / 874 (5.26%) |
| >=8 | 53 | 824 | 24 / 53 (45.28%) | 824 / 824 (100.00%) | 13 / 824 (1.58%) |

Link-count bin breakdown uses the same bin edges as the frozen DoF bins (declared total links per asset):

| Bin | Assets | VBCC asset pass |
|---|---:|---:|
| 0 | 0 | 0 / 0 (N/A) |
| 1 | 0 | 0 / 0 (N/A) |
| 2-3 | 317 | 64 / 317 (20.19%) |
| 4-7 | 390 | 116 / 390 (29.74%) |
| >=8 | 93 | 44 / 93 (47.31%) |

Joint-type breakdown:

| Joint type | Portability | Dynamics |
|---|---:|---:|
| continuous | 560 / 560 (100.00%) | 33 / 560 (5.89%) |
| prismatic | 981 / 981 (100.00%) | 12 / 981 (1.22%) |
| revolute | 1324 / 1324 (100.00%) | 34 / 1324 (2.57%) |

These are proposed Table 2 supplementary diagnostics; they do not retroactively change the frozen Table 2 Strict URDF Pass values.
