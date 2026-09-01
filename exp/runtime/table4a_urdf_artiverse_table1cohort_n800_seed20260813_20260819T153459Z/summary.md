# Table 4a — Artiverse (frozen Table 4 Table-1 cohort, N=800; Genesis contact-penetration oracle)

- Protocol ID: `table4a_artiverse_table1cohort_n800_seed20260813_v1` (engine `genesis_contact_penetration_v1`)
- Run directory: `/mnt/zsn/lyb/arti-skill/exp/runtime/table4a_urdf_artiverse_table1cohort_n800_seed20260813_20260819T153459Z`
- N_eval = 800, J_eval = 3875
- Status: completed = 595, error = 205

| Dataset / Outputs | Joint-level Full-range CF ↑ | Executable CF DoF/Asset ↑ | Collision-safe DoF Retention ↑ | Normalized Clearance P5 ↑ | Limit Reachability ↑ |
|---|---:|---:|---:|---:|---:|
| Artiverse | 2278 / 3875 (58.7871%) | mean 2.82 / median 1.0 / P90 5.0 | 2256 / 3875 (58.2194%) | N/E (PARTIAL; states 58548 / 81375) | 2217 / 3742 (59.2464%; continuous excluded: 133) |

## DoF bins (declared movable DoF; existing Strict Collision Pass from Table 4)

| DoF bin | N_eval | Joint-level Full-range CF ↑ | Collision-safe DoF Retention ↑ | Existing Strict Collision Pass ↑ |
|---|---:|---:|---:|---:|
| 0 | 0 | 0 / 0 (N/A%) | 0 / 0 (N/A%) | 0 / 0 (N/A%) |
| 1 | 170 | 109 / 170 (64.1176%) | 103 / 170 (60.5882%) | 108 / 170 (63.5294%) |
| 2--3 | 363 | 500 / 858 (58.2751%) | 492 / 858 (57.3427%) | 102 / 363 (28.0992%) |
| 4--7 | 196 | 522 / 979 (53.3197%) | 514 / 979 (52.5026%) | 37 / 196 (18.8776%) |
| >=8 | 71 | 1147 / 1868 (61.4026%) | 1147 / 1868 (61.4026%) | 7 / 71 (9.8592%) |

