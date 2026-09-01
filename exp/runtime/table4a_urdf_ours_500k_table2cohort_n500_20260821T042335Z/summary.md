# Table 4a - Ours-500K (frozen cohort, N=500; Genesis contact-penetration oracle)

- Protocol ID: `table4a_ours_500k_table2cohort_n500_v1` (engine `genesis_contact_penetration_v1`)
- Run directory: `/mnt/zsn/lyb/arti-skill/exp/runtime/table4a_urdf_ours_500k_table2cohort_n500_20260821T042335Z`
- N_eval = 500, J_eval = 2467
- Status: completed = 480, error = 20

| Dataset / Outputs | Joint-level Full-range CF ↑ | Executable CF DoF/Asset ↑ | Collision-safe DoF Retention ↑ | Normalized Clearance P5 ↑ | Limit Reachability ↑ |
|---|---:|---:|---:|---:|---:|
| Ours-500K | 2187 / 2467 (88.6502%) | mean 4.374 / median 4.0 / P90 7.0 | 2187 / 2467 (88.6502%) | N/E (PARTIAL; states 46326 / 51807) | 2003 / 2147 (93.2930%; continuous excluded: 320) |

## DoF bins (declared movable DoF; existing Strict Collision Pass from Table 4)

| DoF bin | N_eval | Joint-level Full-range CF ↑ | Collision-safe DoF Retention ↑ | Existing Strict Collision Pass ↑ |
|---|---:|---:|---:|---:|
| 0 | 0 | 0 / 0 (N/A%) | 0 / 0 (N/A%) | 0 / 0 (N/A%) |
| 1 | 39 | 39 / 39 (100.0%) | 39 / 39 (100.0%) | 39 / 39 (100.0%) |
| 2--3 | 173 | 440 / 440 (100.0%) | 440 / 440 (100.0%) | 169 / 173 (97.6879%) |
| 4--7 | 220 | 1107 / 1107 (100.0%) | 1107 / 1107 (100.0%) | 215 / 220 (97.7273%) |
| >=8 | 68 | 601 / 881 (68.2179%) | 601 / 881 (68.2179%) | 62 / 68 (91.1765%) |

