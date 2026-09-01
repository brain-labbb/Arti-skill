# Table 4a - Ours per-class N=5 (supplementary) (frozen cohort, N=3; Genesis contact-penetration oracle)

- Protocol ID: `table4a_ours_pva_per_class_n5_max_joints_v1` (engine `genesis_contact_penetration_v1`)
- Run directory: `/mnt/zsn/lyb/arti-skill/exp/runtime/urdf_table4a_ours_pva_per_class_n5_smoke_n3_20260824T074757Z`
- N_eval = 3, J_eval = 5
- Status: completed = 3, error = 0

| Dataset / Outputs | Joint-level Full-range CF ↑ | Executable CF DoF/Asset ↑ | Collision-safe DoF Retention ↑ | Normalized Clearance P5 ↑ | Limit Reachability ↑ |
|---|---:|---:|---:|---:|---:|
| Ours per-class N=5 (supplementary) | 1 / 5 (20.0000%) | mean 0.3333 / median 0.0 / P90 0.8 | 1 / 14968 (0.0067%) | N/E (COMPLETE; states 105 / 105) | 1 / 5 (20.0000%; continuous excluded: 0) |

## DoF bins (declared movable DoF; existing Strict Collision Pass from Table 4)

| DoF bin | N_eval | Joint-level Full-range CF ↑ | Collision-safe DoF Retention ↑ | Existing Strict Collision Pass ↑ |
|---|---:|---:|---:|---:|
| 0 | 0 | 0 / 0 (N/A%) | 0 / 0 (N/A%) | 0 / 0 (N/A%) |
| 1 | 1 | 1 / 1 (100.0%) | 1 / 1 (100.0%) | 1 / 1 (100.0%) |
| 2--3 | 2 | 0 / 4 (0.0%) | 0 / 4 (0.0%) | 0 / 2 (0.0%) |
| 4--7 | 0 | 0 / 0 (N/A%) | 0 / 0 (N/A%) | 0 / 0 (N/A%) |
| >=8 | 0 | 0 / 0 (N/A%) | 0 / 0 (N/A%) | 0 / 0 (N/A%) |

