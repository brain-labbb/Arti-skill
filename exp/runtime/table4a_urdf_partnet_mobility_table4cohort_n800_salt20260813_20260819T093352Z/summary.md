# Table 4a — PartNet-Mobility (frozen Table 4 cohort, N=800; Genesis contact-penetration oracle)

- Protocol ID: `table4a_partnet_mobility_table4cohort_n800_v1` (engine `genesis_contact_penetration_v1`)
- Run directory: `/mnt/zsn/lyb/arti-skill/exp/runtime/table4a_urdf_partnet_mobility_table4cohort_n800_salt20260813_20260819T093352Z`
- N_eval = 800, J_eval = 4078
- Status: completed = 172, error = 628

| Dataset / Outputs | Joint-level Full-range CF ↑ | Executable CF DoF/Asset ↑ | Collision-safe DoF Retention ↑ | Normalized Clearance P5 ↑ | Limit Reachability ↑ |
|---|---:|---:|---:|---:|---:|
| PartNet-Mobility | 510 / 4078 (12.5061%) | mean 0.6375 / median 0.0 / P90 1.0 | 510 / 4078 (12.5061%) | N/E (PARTIAL; states 11247 / 85638) | 443 / 3416 (12.9684%; continuous excluded: 662) |

## DoF bins (declared movable DoF; existing Strict Collision Pass from Table 4)

| DoF bin | N_eval | Joint-level Full-range CF ↑ | Collision-safe DoF Retention ↑ | Existing Strict Collision Pass ↑ |
|---|---:|---:|---:|---:|
| 0 | 0 | 0 / 0 (N/A%) | 0 / 0 (N/A%) | 0 / 0 (N/A%) |
| 1 | 327 | 88 / 327 (26.9113%) | 88 / 327 (26.9113%) | 318 / 327 (97.2477%) |
| 2--3 | 288 | 122 / 654 (18.6544%) | 122 / 654 (18.6544%) | 152 / 288 (52.7778%) |
| 4--7 | 99 | 44 / 449 (9.7996%) | 44 / 449 (9.7996%) | 53 / 99 (53.5354%) |
| >=8 | 86 | 256 / 2648 (9.6677%) | 256 / 2648 (9.6677%) | 44 / 86 (51.1628%) |

