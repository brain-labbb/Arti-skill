# Artiverse Table 1: Dataset Scale and Structural Diversity

## Frozen cohort

- Release snapshot: pre-release manifest `8fa6468254a1f74c58f0c25699598bf88f622fabdaf74f0cd9268ee5663c5586`.
- `N_release`: 3544 assets across 84 raw categories.
- `N_eval`: 5 globally sampled assets across 5 raw categories.
- Selection: deterministic salted SHA-256 rank, seed `20260813`; no replacement or outcome filtering.
- This is not the shared-category balanced cohort.

## Table 1 result

| Dataset / Outputs | Paper-reported Assets | N_release | N_eval | #Categories (release / eval) | Links/Asset (mean / median / P90) | Movable Joints/Asset (mean / median / P90) | Multi-joint Assets | Unique Topologies | Exact Duplicate Rate |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Artiverse | 5,402 | 3544 | 5 | 84 / 5 | 6.200 / 6.000 / 11 (n=5) | 2.200 / 1.000 / 5 (n=5) | 2 / 5 (40.00%) | 5 / 5 (100.00%); coverage 5 / 5 (100.00%) | 0 / 5 (0.00%); coverage 5 / 5 (100.00%) |

## Diagnostics

- XML parse coverage: 5 / 5 (100.00%).
- Category macro over 5 sampled raw categories: multi-joint 40.00%; unique topologies 100.00% over 5 evaluable categories; exact duplicate rate 0.00% over 5 evaluable categories.
- Assets in duplicate clusters: 0 / 5 (0.00%); 0 clusters, maximum size 1.
- Topology hashes describe URDF representation structure, not semantic joint correctness.
- Exact duplicate rate uses canonicalized URDF plus the recursively resolved simulation resource closure; incomplete closures are not treated as unique.
