# Articraft-10K Table 2 Supplementary Diagnostics

Run classification: **NON_FORMAL_SMOKE**.

Exact Table 2 manifest cohort: N_eval=5 from N_release=0, seed=20260813; J_eval=16. Existing package order was preserved without resampling.

| Metric | Result |
|---|---:|
| visual_bearing_collision_coverage (asset) | 1 / 5 (20.00%) |
| visual_bearing_collision_coverage (link-micro) | 4 / 23 (17.39%) [link extraction coverage 5/5; COMPLETE] |
| joint_limit_portability | 16 / 16 (100.00%) |
| joint_dynamics_coverage | 0 / 16 (0.00%) |
| placeholder_mass_incidence | N/E (placeholder registry is empty; no mass values are evaluable) |
| placeholder_mass_incidence complete-inertial coverage | 9 / 23 (39.13%) |

Category macro average over 5 observed categories (unweighted; joint metrics only over categories with >=1 declared movable joint):

| Metric | Category macro |
|---|---:|
| visual_bearing_collision_coverage | 20.00% (categories=5) |
| joint_limit_portability | 100.00% (categories=5) |
| joint_dynamics_coverage | 0.00% (categories=5) |
| placeholder_mass_incidence | N/E (registry empty) |

Declared-DoF bin breakdown (frozen bins 0 / 1 / 2-3 / 4-7 / >=8):

| Bin | Assets | Joints | VBCC asset pass | Portability | Dynamics |
|---|---:|---:|---:|---:|---:|
| 0 | 0 | 0 | 0 / 0 (N/A) | 0 / 0 (N/A) | 0 / 0 (N/A) |
| 1 | 2 | 2 | 0 / 2 (0.00%) | 2 / 2 (100.00%) | 0 / 2 (0.00%) |
| 2-3 | 2 | 5 | 1 / 2 (50.00%) | 5 / 5 (100.00%) | 0 / 5 (0.00%) |
| 4-7 | 0 | 0 | 0 / 0 (N/A) | 0 / 0 (N/A) | 0 / 0 (N/A) |
| >=8 | 1 | 9 | 0 / 1 (0.00%) | 9 / 9 (100.00%) | 0 / 9 (0.00%) |

Link-count bin breakdown uses the same bin edges as the frozen DoF bins (declared total links per asset):

| Bin | Assets | VBCC asset pass |
|---|---:|---:|
| 0 | 0 | 0 / 0 (N/A) |
| 1 | 0 | 0 / 0 (N/A) |
| 2-3 | 2 | 0 / 2 (0.00%) |
| 4-7 | 2 | 1 / 2 (50.00%) |
| >=8 | 1 | 0 / 1 (0.00%) |

Joint-type breakdown:

| Joint type | Portability | Dynamics |
|---|---:|---:|
| continuous | 10 / 10 (100.00%) | 0 / 10 (0.00%) |
| prismatic | 4 / 4 (100.00%) | 0 / 4 (0.00%) |
| revolute | 2 / 2 (100.00%) | 0 / 2 (0.00%) |

These are proposed Table 2 supplementary diagnostics; they do not retroactively change the frozen Table 2 Strict URDF Pass values.
