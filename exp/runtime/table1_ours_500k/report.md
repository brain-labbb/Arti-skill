# Ours-500K Table 1: Dataset Scale and Structural Diversity

## Frozen cohort

- Acquired release archive: `/mnt/zsn/lyb/arti-skill/exp/Brain/arti_cabinet_drawer_geometry_500_20260813.zip`.
- Archive SHA-256: `ffedf5bd90ae5eb96a061d0e127b700915ed6c221eeb7c5afe282b7249bfbd66` (matches published sidecar).
- Roster hash (ours500k-table1-roster-v1): `ed70ebeb97f9ad8a655288e2afce96b0c3a8e26f50653e50dbbdc00238cfea3b`.
- `N_release`: 500 assets across 12 raw categories.
- `N_eval`: 500 assets (full acquired roster; below the N=800 sample size used for larger releases; no subsampling).
- Cohort type: FULL_ACQUIRED_RELEASE_SAMPLE_NO_SUBSAMPLING; this is not the shared-category balanced cohort.

## Table 1 result

| Dataset / Outputs | Paper-reported Assets | N_release | N_eval | #Categories (release / eval) | Links/Asset (mean / median / P90) | Movable Joints/Asset (mean / median / P90) | Multi-joint Assets | Unique Topologies | Exact Duplicate Rate |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Ours-500K | 10K | 500 | 500 | 12 / 12 | 6.25 / 5.0 / 13 | 4.93 / 4.0 / 12 | 92.20% | 13.40% (n=500) | 0.00% (n=500) |

## Diagnostics

- XML parse coverage: 500 / 500 (100.00%).
- Category macro over 12 raw categories: multi-joint 90.05%; unique topologies 35.55% over 12 evaluable categories; exact duplicate rate 0.00% over 12 evaluable categories.
- Unique topologies: 67 unique hashes; coverage 500 / 500 (100.00%).
- Exact duplicates: excess 0; assets in duplicate clusters 0 / 500 (0.00%); 0 clusters, maximum size 1; coverage 500 / 500 (100.00%).
- Declared joint type counts: {"continuous": 320, "fixed": 156, "prismatic": 1339, "revolute": 808}.
- Status counts: {"EVALUATED": 500}.
- Movable-joint counts include all declared XML joints except literal `fixed`, including exporter extension types; this is not a runtime-valid DoF count.
- Unique-topology rate is defined over valid rooted trees only; coverage against `N_eval` is reported separately.
- Exact duplicate rate uses canonicalized URDF plus the recursively resolved simulation resource closure; incomplete closures are not treated as unique.
