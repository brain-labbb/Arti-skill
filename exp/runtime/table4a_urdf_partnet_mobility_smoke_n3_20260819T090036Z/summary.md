# Table 4a — PartNet-Mobility (frozen Table 4 cohort, N=800; Genesis contact-penetration oracle)

- Protocol ID: `table4a_partnet_mobility_table4cohort_n800_v1` (engine `genesis_contact_penetration_v1`)
- Run directory: `/mnt/zsn/lyb/arti-skill/exp/runtime/table4a_urdf_partnet_mobility_smoke_n3_20260819T090036Z`
- N_eval = 3, J_eval = 5
- Status: completed = 3, error = 0

| Dataset / Outputs | Joint-level Full-range CF ↑ | Executable CF DoF/Asset ↑ | Collision-safe DoF Retention ↑ | Normalized Clearance P5 ↑ | Limit Reachability ↑ |
|---|---:|---:|---:|---:|---:|
| PartNet-Mobility | 5 / 5 (100.0000%) | mean 1.6667 / median 2.0 / P90 2.0 | 5 / 4078 (0.1226%) | N/E (COMPLETE; states 105 / 105) | 2 / 2 (100.0000%; continuous excluded: 3) |

## DoF bins (declared movable DoF; existing Strict Collision Pass from Table 4)

| DoF bin | N_eval | Joint-level Full-range CF ↑ | Collision-safe DoF Retention ↑ | Existing Strict Collision Pass ↑ |
|---|---:|---:|---:|---:|
| 0 | 0 | 0 / 0 (N/A%) | 0 / 0 (N/A%) | 0 / 0 (N/A%) |
| 1 | 1 | 1 / 1 (100.0%) | 1 / 1 (100.0%) | 1 / 1 (100.0%) |
| 2--3 | 2 | 4 / 4 (100.0%) | 4 / 4 (100.0%) | 2 / 2 (100.0%) |
| 4--7 | 0 | 0 / 0 (N/A%) | 0 / 0 (N/A%) | 0 / 0 (N/A%) |
| >=8 | 0 | 0 / 0 (N/A%) | 0 / 0 (N/A%) | 0 / 0 (N/A%) |

