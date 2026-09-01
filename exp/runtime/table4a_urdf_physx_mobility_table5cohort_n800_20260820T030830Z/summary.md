# Table 4a — PhysX-Mobility (frozen Table 5 receipt-set cohort, N=800; Genesis contact-penetration oracle)

- Protocol ID: `table4a_physx_mobility_table5cohort_n800_v1` (engine `genesis_contact_penetration_v1`)
- Run directory: `/mnt/zsn/lyb/arti-skill/exp/runtime/table4a_urdf_physx_mobility_table5cohort_n800_20260820T030830Z`
- N_eval = 800, J_eval = 3809
- Status: completed = 799, error = 1
- Claim boundary: official PhysX-Mobility URDFs declare zero collision elements; every asset has 0 eligible collision pairs, so all collision-free outcomes are vacuous.

| Dataset / Outputs | Joint-level Full-range CF ↑ | Executable CF DoF/Asset ↑ | Collision-safe DoF Retention ↑ | Normalized Clearance P5 ↑ | Limit Reachability ↑ |
|---|---:|---:|---:|---:|---:|
| PhysX-Mobility | 3807 / 3809 (99.9475%) | mean 4.7575 / median 2.0 / P90 7.0 | 3806 / 3809 (99.9212%) | N/E (PARTIAL; states 79947 / 79989) | 3807 / 3809 (99.9475%; continuous excluded: 0) |

## DoF bins (declared movable DoF; existing Strict Collision Pass from PhysX Table 4)

| DoF bin | N_eval | Joint-level Full-range CF ↑ | Collision-safe DoF Retention ↑ | Existing Strict Collision Pass ↑ |
|---|---:|---:|---:|---:|
| 0 | 1 | 0 / 0 (N/A%) | 0 / 0 (N/A%) | 0 / 1 (0.0%) |
| 1 | 358 | 358 / 358 (100.0%) | 357 / 358 (99.7207%) | 358 / 358 (100.0%) |
| 2--3 | 285 | 637 / 639 (99.687%) | 637 / 639 (99.687%) | 284 / 285 (99.6491%) |
| 4--7 | 83 | 396 / 396 (100.0%) | 396 / 396 (100.0%) | 83 / 83 (100.0%) |
| >=8 | 73 | 2416 / 2416 (100.0%) | 2416 / 2416 (100.0%) | 61 / 73 (83.5616%) |

