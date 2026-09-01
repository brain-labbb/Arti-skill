# Table 4a — Articraft-10K (frozen Table 2 cohort, N=800; Genesis contact-penetration oracle)

- Protocol ID: `table4a_articraft10k_table2cohort_n800_v1` (engine `genesis_contact_penetration_v1`)
- Run directory: `/mnt/zsn/lyb/arti-skill/exp/runtime/table4a_urdf_articraft10k_smoke_n3_20260819T101632Z`
- N_eval = 3, J_eval = 12
- Status: completed = 3, error = 0

| Dataset / Outputs | Joint-level Full-range CF ↑ | Executable CF DoF/Asset ↑ | Collision-safe DoF Retention ↑ | Normalized Clearance P5 ↑ | Limit Reachability ↑ |
|---|---:|---:|---:|---:|---:|
| Articraft-10K | 12 / 12 (100.0000%) | mean 4.0 / median 2.0 / P90 7.6000000000000005 | 12 / 2865 (0.4188%) | N/E (COMPLETE; states 252 / 252) | 4 / 4 (100.0000%; continuous excluded: 8) |

## DoF bins (declared movable DoF; existing Strict Collision Pass from Table 4)

| DoF bin | N_eval | Joint-level Full-range CF ↑ | Collision-safe DoF Retention ↑ | Existing Strict Collision Pass ↑ |
|---|---:|---:|---:|---:|
| 0 | 0 | 0 / 0 (N/A%) | 0 / 0 (N/A%) | 0 / 0 (N/A%) |
| 1 | 1 | 1 / 1 (100.0%) | 1 / 1 (100.0%) | 0 / 1 (0.0%) |
| 2--3 | 1 | 2 / 2 (100.0%) | 2 / 2 (100.0%) | 0 / 1 (0.0%) |
| 4--7 | 0 | 0 / 0 (N/A%) | 0 / 0 (N/A%) | 0 / 0 (N/A%) |
| >=8 | 1 | 9 / 9 (100.0%) | 9 / 9 (100.0%) | 0 / 1 (0.0%) |

