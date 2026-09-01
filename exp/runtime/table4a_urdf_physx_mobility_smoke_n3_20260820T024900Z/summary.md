# Table 4a — PhysX-Mobility (frozen Table 5 receipt-set cohort, N=800; Genesis contact-penetration oracle)

- Protocol ID: `table4a_physx_mobility_table5cohort_n800_v1` (engine `genesis_contact_penetration_v1`)
- Run directory: `/mnt/zsn/lyb/arti-skill/exp/runtime/table4a_urdf_physx_mobility_smoke_n3_20260820T024900Z`
- N_eval = 3, J_eval = 7
- Status: completed = 3, error = 0
- Claim boundary: official PhysX-Mobility URDFs declare zero collision elements; every asset has 0 eligible collision pairs, so all collision-free outcomes are vacuous.

| Dataset / Outputs | Joint-level Full-range CF ↑ | Executable CF DoF/Asset ↑ | Collision-safe DoF Retention ↑ | Normalized Clearance P5 ↑ | Limit Reachability ↑ |
|---|---:|---:|---:|---:|---:|
| PhysX-Mobility | 7 / 7 (100.0000%) | mean 2.3333 / median 3.0 / P90 3.0 | 7 / 3809 (0.1838%) | N/E (COMPLETE; states 147 / 147) | 7 / 7 (100.0000%; continuous excluded: 0) |

## DoF bins (declared movable DoF; existing Strict Collision Pass from PhysX Table 4)

| DoF bin | N_eval | Joint-level Full-range CF ↑ | Collision-safe DoF Retention ↑ | Existing Strict Collision Pass ↑ |
|---|---:|---:|---:|---:|
| 0 | 0 | 0 / 0 (N/A%) | 0 / 0 (N/A%) | 0 / 0 (N/A%) |
| 1 | 1 | 1 / 1 (100.0%) | 1 / 1 (100.0%) | 1 / 1 (100.0%) |
| 2--3 | 2 | 6 / 6 (100.0%) | 6 / 6 (100.0%) | 2 / 2 (100.0%) |
| 4--7 | 0 | 0 / 0 (N/A%) | 0 / 0 (N/A%) | 0 / 0 (N/A%) |
| >=8 | 0 | 0 / 0 (N/A%) | 0 / 0 (N/A%) | 0 / 0 (N/A%) |

