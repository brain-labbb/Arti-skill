# Table 4a — PartNet-Mobility (frozen Table 4 cohort, N=800; Genesis contact-penetration oracle)

- Protocol ID: `table4a_partnet_mobility_table4cohort_n800_v1` (engine `genesis_contact_penetration_v1`)
- Run directory: `/mnt/zsn/lyb/arti-skill/exp/runtime/table4a_urdf_partnet_mobility_table4cohort_n800_salt20260813_20260819T123437Z`
- N_eval = 800, J_eval = 4078
- Status: completed = 646, error = 154

| Dataset / Outputs | Joint-level Full-range CF ↑ | Executable CF DoF/Asset ↑ | Collision-safe DoF Retention ↑ | Normalized Clearance P5 ↑ | Limit Reachability ↑ |
|---|---:|---:|---:|---:|---:|
| PartNet-Mobility | 2604 / 4078 (63.8548%) | mean 3.255 / median 1.0 / P90 4.0 | 2604 / 4078 (63.8548%) | N/E (PARTIAL; states 62958 / 85638) | 2321 / 3416 (67.9450%; continuous excluded: 662) |

## DoF bins (declared movable DoF; existing Strict Collision Pass from Table 4)

| DoF bin | N_eval | Joint-level Full-range CF ↑ | Collision-safe DoF Retention ↑ | Existing Strict Collision Pass ↑ |
|---|---:|---:|---:|---:|
| 0 | 0 | 0 / 0 (N/A%) | 0 / 0 (N/A%) | 0 / 0 (N/A%) |
| 1 | 327 | 271 / 327 (82.8746%) | 271 / 327 (82.8746%) | 318 / 327 (97.2477%) |
| 2--3 | 288 | 429 / 654 (65.5963%) | 429 / 654 (65.5963%) | 152 / 288 (52.7778%) |
| 4--7 | 99 | 276 / 449 (61.4699%) | 276 / 449 (61.4699%) | 53 / 99 (53.5354%) |
| >=8 | 86 | 1628 / 2648 (61.4804%) | 1628 / 2648 (61.4804%) | 44 / 86 (51.1628%) |

