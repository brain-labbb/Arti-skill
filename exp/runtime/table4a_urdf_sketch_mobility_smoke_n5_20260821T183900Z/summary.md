# Table 4a — PartNet-Mobility (frozen Table 4 cohort, N=800; Genesis contact-penetration oracle)

- Protocol ID: `table4a-sketchmobility-table1-cohort-n800-v1` (engine `genesis_contact_penetration_v1`)
- Run directory: `/mnt/zsn/lyb/arti-skill/exp/runtime/table4a_urdf_sketch_mobility_smoke_n5_20260821T183900Z`
- N_eval = 5, J_eval = 5
- Status: completed = 1, error = 4

| Dataset / Outputs | Joint-level Full-range CF ↑ | Executable CF DoF/Asset ↑ | Collision-safe DoF Retention ↑ | Normalized Clearance P5 ↑ | Limit Reachability ↑ |
|---|---:|---:|---:|---:|---:|
| PartNet-Mobility | 1 / 5 (20.0000%) | mean 0.2 / median 0.0 / P90 0.6000000000000001 | 1 / 5 (20.0000%) | N/E (PARTIAL; states 21 / 105) | 1 / 5 (20.0000%; continuous excluded: 0) |

## DoF bins (declared movable DoF; existing Strict Collision Pass from Table 4)

| DoF bin | N_eval | Joint-level Full-range CF ↑ | Collision-safe DoF Retention ↑ | Existing Strict Collision Pass ↑ |
|---|---:|---:|---:|---:|
| 0 | 0 | 0 / 0 (N/A%) | 0 / 0 (N/A%) | 0 / 0 (N/A%) |
| 1 | 5 | 1 / 5 (20.0%) | 1 / 5 (20.0%) | 1 / 5 (20.0%) |
| 2--3 | 0 | 0 / 0 (N/A%) | 0 / 0 (N/A%) | 0 / 0 (N/A%) |
| 4--7 | 0 | 0 / 0 (N/A%) | 0 / 0 (N/A%) | 0 / 0 (N/A%) |
| >=8 | 0 | 0 / 0 (N/A%) | 0 / 0 (N/A%) | 0 / 0 (N/A%) |

