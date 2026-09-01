# Table 4a — Articraft-10K (frozen Table 2 cohort, N=800; Genesis contact-penetration oracle)

- Protocol ID: `table4a_articraft10k_table2cohort_n800_v1` (engine `genesis_contact_penetration_v1`)
- Run directory: `/mnt/zsn/lyb/arti-skill/exp/runtime/table4a_urdf_articraft10k_table2cohort_n800_seed20260813_20260820T040459Z`
- N_eval = 800, J_eval = 2865
- Status: completed = 781, error = 19

| Dataset / Outputs | Joint-level Full-range CF ↑ | Executable CF DoF/Asset ↑ | Collision-safe DoF Retention ↑ | Normalized Clearance P5 ↑ | Limit Reachability ↑ |
|---|---:|---:|---:|---:|---:|
| Articraft-10K | 2665 / 2865 (93.0192%) | mean 3.3075 / median 2.0 / P90 6.0 | 2646 / 2865 (92.3560%) | N/E (PARTIAL; states 58212 / 60165) | 2134 / 2305 (92.5813%; continuous excluded: 560) |

## DoF bins (declared movable DoF; existing Strict Collision Pass from Table 4)

| DoF bin | N_eval | Joint-level Full-range CF ↑ | Collision-safe DoF Retention ↑ | Existing Strict Collision Pass ↑ |
|---|---:|---:|---:|---:|
| 0 | 0 | 0 / 0 (N/A%) | 0 / 0 (N/A%) | 0 / 0 (N/A%) |
| 1 | 163 | 162 / 163 (99.3865%) | 162 / 163 (99.3865%) | 29 / 163 (17.7914%) |
| 2--3 | 407 | 966 / 1004 (96.2151%) | 962 / 1004 (95.8167%) | 70 / 407 (17.199%) |
| 4--7 | 177 | 784 / 874 (89.7025%) | 777 / 874 (88.9016%) | 37 / 177 (20.904%) |
| >=8 | 53 | 753 / 824 (91.3835%) | 745 / 824 (90.4126%) | 11 / 53 (20.7547%) |

