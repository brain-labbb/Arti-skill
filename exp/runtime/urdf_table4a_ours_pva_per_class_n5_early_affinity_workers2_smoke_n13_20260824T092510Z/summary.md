# Table 4a - Ours per-class N=5 (supplementary) (frozen cohort, N=13; Genesis contact-penetration oracle)

- Protocol ID: `table4a_ours_pva_per_class_n5_max_joints_v1` (engine `genesis_contact_penetration_v1`)
- Run directory: `/mnt/zsn/lyb/arti-skill/exp/runtime/urdf_table4a_ours_pva_per_class_n5_early_affinity_workers2_smoke_n13_20260824T092510Z`
- N_eval = 13, J_eval = 87
- Status: completed = 13, error = 0

| Dataset / Outputs | Joint-level Full-range CF ↑ | Executable CF DoF/Asset ↑ | Collision-safe DoF Retention ↑ | Normalized Clearance P5 ↑ | Limit Reachability ↑ |
|---|---:|---:|---:|---:|---:|
| Ours per-class N=5 (supplementary) | 15 / 87 (17.2414%) | mean 1.1538 / median 0.0 / P90 4.0 | 15 / 14968 (0.1002%) | N/E (COMPLETE; states 1827 / 1827) | 15 / 83 (18.0723%; continuous excluded: 4) |

## DoF bins (declared movable DoF; existing Strict Collision Pass from Table 4)

| DoF bin | N_eval | Joint-level Full-range CF ↑ | Collision-safe DoF Retention ↑ | Existing Strict Collision Pass ↑ |
|---|---:|---:|---:|---:|
| 0 | 0 | 0 / 0 (N/A%) | 0 / 0 (N/A%) | 0 / 0 (N/A%) |
| 1 | 1 | 1 / 1 (100.0%) | 1 / 1 (100.0%) | 1 / 1 (100.0%) |
| 2--3 | 6 | 2 / 13 (15.3846%) | 2 / 13 (15.3846%) | 0 / 6 (0.0%) |
| 4--7 | 3 | 12 / 12 (100.0%) | 12 / 12 (100.0%) | 0 / 3 (0.0%) |
| >=8 | 3 | 0 / 61 (0.0%) | 0 / 61 (0.0%) | 0 / 3 (0.0%) |

