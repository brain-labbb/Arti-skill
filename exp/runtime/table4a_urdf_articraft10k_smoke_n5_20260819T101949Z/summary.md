# Table 4a — Articraft-10K (frozen Table 2 cohort, N=800; Genesis contact-penetration oracle)

- Protocol ID: `table4a_articraft10k_table2cohort_n800_v1` (engine `genesis_contact_penetration_v1`)
- Run directory: `/mnt/zsn/lyb/arti-skill/exp/runtime/table4a_urdf_articraft10k_smoke_n5_20260819T101949Z`
- N_eval = 5, J_eval = 16
- Status: completed = 4, error = 1

| Dataset / Outputs | Joint-level Full-range CF ↑ | Executable CF DoF/Asset ↑ | Collision-safe DoF Retention ↑ | Normalized Clearance P5 ↑ | Limit Reachability ↑ |
|---|---:|---:|---:|---:|---:|
| Articraft-10K | 13 / 16 (81.2500%) | mean 2.6 / median 1.0 / P90 6.200000000000001 | 13 / 2865 (0.4538%) | N/E (PARTIAL; states 273 / 336) | 5 / 6 (83.3333%; continuous excluded: 10) |

## DoF bins (declared movable DoF; existing Strict Collision Pass from Table 4)

| DoF bin | N_eval | Joint-level Full-range CF ↑ | Collision-safe DoF Retention ↑ | Existing Strict Collision Pass ↑ |
|---|---:|---:|---:|---:|
| 0 | 0 | 0 / 0 (N/A%) | 0 / 0 (N/A%) | 0 / 0 (N/A%) |
| 1 | 2 | 2 / 2 (100.0%) | 2 / 2 (100.0%) | 0 / 2 (0.0%) |
| 2--3 | 2 | 2 / 5 (40.0%) | 2 / 5 (40.0%) | 1 / 2 (50.0%) |
| 4--7 | 0 | 0 / 0 (N/A%) | 0 / 0 (N/A%) | 0 / 0 (N/A%) |
| >=8 | 1 | 9 / 9 (100.0%) | 9 / 9 (100.0%) | 0 / 1 (0.0%) |

