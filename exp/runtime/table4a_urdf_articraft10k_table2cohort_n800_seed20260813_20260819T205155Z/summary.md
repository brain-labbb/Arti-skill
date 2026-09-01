# Table 4a — Articraft-10K (frozen Table 2 cohort, N=800; Genesis contact-penetration oracle)

- Protocol ID: `table4a_articraft10k_table2cohort_n800_v1` (engine `genesis_contact_penetration_v1`)
- Run directory: `/mnt/zsn/lyb/arti-skill/exp/runtime/table4a_urdf_articraft10k_table2cohort_n800_seed20260813_20260819T205155Z`
- N_eval = 800, J_eval = 2865
- Status: completed = 776, error = 24

| Dataset / Outputs | Joint-level Full-range CF ↑ | Executable CF DoF/Asset ↑ | Collision-safe DoF Retention ↑ | Normalized Clearance P5 ↑ | Limit Reachability ↑ |
|---|---:|---:|---:|---:|---:|
| Articraft-10K | 2642 / 2865 (92.2164%) | mean 3.2788 / median 2.0 / P90 6.0 | 2623 / 2865 (91.5532%) | N/E (PARTIAL; states 57708 / 60165) | 2118 / 2305 (91.8872%; continuous excluded: 560) |

## DoF bins (declared movable DoF; existing Strict Collision Pass from Table 4)

| DoF bin | N_eval | Joint-level Full-range CF ↑ | Collision-safe DoF Retention ↑ | Existing Strict Collision Pass ↑ |
|---|---:|---:|---:|---:|
| 0 | 0 | 0 / 0 (N/A%) | 0 / 0 (N/A%) | 0 / 0 (N/A%) |
| 1 | 163 | 162 / 163 (99.3865%) | 162 / 163 (99.3865%) | 29 / 163 (17.7914%) |
| 2--3 | 407 | 960 / 1004 (95.6175%) | 956 / 1004 (95.2191%) | 70 / 407 (17.199%) |
| 4--7 | 177 | 780 / 874 (89.2449%) | 773 / 874 (88.4439%) | 37 / 177 (20.904%) |
| >=8 | 53 | 740 / 824 (89.8058%) | 732 / 824 (88.835%) | 11 / 53 (20.7547%) |

