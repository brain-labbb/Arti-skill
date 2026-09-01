# Table 4a - Ours per-class N=5 (supplementary) (frozen cohort, N=7; Genesis contact-penetration oracle)

- Protocol ID: `table4a_ours_pva_per_class_n5_max_joints_v1` (engine `genesis_contact_penetration_v1`)
- Run directory: `/mnt/zsn/lyb/arti-skill/exp/runtime/urdf_table4a_ours_pva_per_class_n5_early_affinity_smoke_n7_20260824T091057Z`
- N_eval = 7, J_eval = 18
- Status: completed = 7, error = 0

| Dataset / Outputs | Joint-level Full-range CF ↑ | Executable CF DoF/Asset ↑ | Collision-safe DoF Retention ↑ | Normalized Clearance P5 ↑ | Limit Reachability ↑ |
|---|---:|---:|---:|---:|---:|
| Ours per-class N=5 (supplementary) | 9 / 18 (50.0000%) | mean 1.2857 / median 0.0 / P90 4.0 | 9 / 14968 (0.0601%) | N/E (COMPLETE; states 378 / 378) | 9 / 18 (50.0000%; continuous excluded: 0) |

## DoF bins (declared movable DoF; existing Strict Collision Pass from Table 4)

| DoF bin | N_eval | Joint-level Full-range CF ↑ | Collision-safe DoF Retention ↑ | Existing Strict Collision Pass ↑ |
|---|---:|---:|---:|---:|
| 0 | 0 | 0 / 0 (N/A%) | 0 / 0 (N/A%) | 0 / 0 (N/A%) |
| 1 | 1 | 1 / 1 (100.0%) | 1 / 1 (100.0%) | 1 / 1 (100.0%) |
| 2--3 | 4 | 0 / 9 (0.0%) | 0 / 9 (0.0%) | 0 / 4 (0.0%) |
| 4--7 | 2 | 8 / 8 (100.0%) | 8 / 8 (100.0%) | 0 / 2 (0.0%) |
| >=8 | 0 | 0 / 0 (N/A%) | 0 / 0 (N/A%) | 0 / 0 (N/A%) |

