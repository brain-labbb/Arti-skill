# Table 4a - Ours per-class N=5 (supplementary) (frozen cohort, N=18; Genesis contact-penetration oracle)

- Protocol ID: `table4a_ours_pva_per_class_n5_max_joints_v1` (engine `genesis_contact_penetration_v1`)
- Run directory: `/mnt/zsn/lyb/arti-skill/exp/runtime/urdf_table4a_ours_pva_per_class_n5_early_affinity_workers1_smoke_n18_20260824T102812Z`
- N_eval = 18, J_eval = 131
- Status: completed = 18, error = 0

| Dataset / Outputs | Joint-level Full-range CF ↑ | Executable CF DoF/Asset ↑ | Collision-safe DoF Retention ↑ | Normalized Clearance P5 ↑ | Limit Reachability ↑ |
|---|---:|---:|---:|---:|---:|
| Ours per-class N=5 (supplementary) | 32 / 131 (24.4275%) | mean 1.7778 / median 0.5 / P90 4.0 | 32 / 14968 (0.2138%) | N/E (COMPLETE; states 2751 / 2751) | 32 / 125 (25.6000%; continuous excluded: 6) |

## DoF bins (declared movable DoF; existing Strict Collision Pass from Table 4)

| DoF bin | N_eval | Joint-level Full-range CF ↑ | Collision-safe DoF Retention ↑ | Existing Strict Collision Pass ↑ |
|---|---:|---:|---:|---:|
| 0 | 0 | 0 / 0 (N/A%) | 0 / 0 (N/A%) | 0 / 0 (N/A%) |
| 1 | 1 | 1 / 1 (100.0%) | 1 / 1 (100.0%) | 1 / 1 (100.0%) |
| 2--3 | 9 | 8 / 19 (42.1053%) | 8 / 19 (42.1053%) | 3 / 9 (33.3333%) |
| 4--7 | 3 | 12 / 12 (100.0%) | 12 / 12 (100.0%) | 0 / 3 (0.0%) |
| >=8 | 5 | 11 / 99 (11.1111%) | 11 / 99 (11.1111%) | 0 / 5 (0.0%) |

