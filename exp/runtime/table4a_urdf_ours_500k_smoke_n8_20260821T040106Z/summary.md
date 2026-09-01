# Table 4a - Ours-500K (frozen cohort, N=8; Genesis contact-penetration oracle)

- Protocol ID: `table4a_ours_500k_table2cohort_n500_v1` (engine `genesis_contact_penetration_v1`)
- Run directory: `/mnt/zsn/lyb/arti-skill/exp/runtime/table4a_urdf_ours_500k_smoke_n8_20260821T040106Z`
- N_eval = 8, J_eval = 55
- Status: completed = 4, error = 4

| Dataset / Outputs | Joint-level Full-range CF ↑ | Executable CF DoF/Asset ↑ | Collision-safe DoF Retention ↑ | Normalized Clearance P5 ↑ | Limit Reachability ↑ |
|---|---:|---:|---:|---:|---:|
| Ours-500K | 6 / 55 (10.9091%) | mean 0.75 / median 0.5 / P90 2.0 | 6 / 2467 (0.2432%) | N/E (PARTIAL; states 126 / 1155) | 6 / 55 (10.9091%; continuous excluded: 0) |

## DoF bins (declared movable DoF; existing Strict Collision Pass from Table 4)

| DoF bin | N_eval | Joint-level Full-range CF ↑ | Collision-safe DoF Retention ↑ | Existing Strict Collision Pass ↑ |
|---|---:|---:|---:|---:|
| 0 | 0 | 0 / 0 (N/A%) | 0 / 0 (N/A%) | 0 / 0 (N/A%) |
| 1 | 2 | 2 / 2 (100.0%) | 2 / 2 (100.0%) | 2 / 2 (100.0%) |
| 2--3 | 2 | 4 / 4 (100.0%) | 4 / 4 (100.0%) | 2 / 2 (100.0%) |
| 4--7 | 0 | 0 / 0 (N/A%) | 0 / 0 (N/A%) | 0 / 0 (N/A%) |
| >=8 | 4 | 0 / 49 (0.0%) | 0 / 49 (0.0%) | 4 / 4 (100.0%) |

