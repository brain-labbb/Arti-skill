# Table 4a — Artiverse (frozen Table 4 Table-1 cohort, N=800; Genesis contact-penetration oracle)

- Protocol ID: `table4a_artiverse_table1cohort_n800_seed20260813_v1` (engine `genesis_contact_penetration_v1`)
- Run directory: `/mnt/zsn/lyb/arti-skill/exp/runtime/table4a_urdf_artiverse_smoke_n3_20260819T145245Z`
- N_eval = 3, J_eval = 5
- Status: completed = 2, error = 1

| Dataset / Outputs | Joint-level Full-range CF ↑ | Executable CF DoF/Asset ↑ | Collision-safe DoF Retention ↑ | Normalized Clearance P5 ↑ | Limit Reachability ↑ |
|---|---:|---:|---:|---:|---:|
| Artiverse | 4 / 5 (80.0000%) | mean 1.3333 / median 1.0 / P90 2.6 | 4 / 3875 (0.1032%) | N/E (PARTIAL; states 84 / 105) | 4 / 5 (80.0000%; continuous excluded: 0) |

## DoF bins (declared movable DoF; existing Strict Collision Pass from Table 4)

| DoF bin | N_eval | Joint-level Full-range CF ↑ | Collision-safe DoF Retention ↑ | Existing Strict Collision Pass ↑ |
|---|---:|---:|---:|---:|
| 0 | 0 | 0 / 0 (N/A%) | 0 / 0 (N/A%) | 0 / 0 (N/A%) |
| 1 | 2 | 1 / 2 (50.0%) | 1 / 2 (50.0%) | 2 / 2 (100.0%) |
| 2--3 | 1 | 3 / 3 (100.0%) | 3 / 3 (100.0%) | 0 / 1 (0.0%) |
| 4--7 | 0 | 0 / 0 (N/A%) | 0 / 0 (N/A%) | 0 / 0 (N/A%) |
| >=8 | 0 | 0 / 0 (N/A%) | 0 / 0 (N/A%) | 0 / 0 (N/A%) |

