# Artiverse Table 1: Dataset Scale and Structural Diversity

## Frozen cohort

- Release snapshot: pre-release manifest `8fa6468254a1f74c58f0c25699598bf88f622fabdaf74f0cd9268ee5663c5586`.
- `N_release`: 3544 assets across 84 raw categories.
- `N_eval`: 800 globally sampled assets across 67 raw categories.
- Selection: deterministic salted SHA-256 rank, seed `20260813`; no replacement or outcome filtering.
- This is not the shared-category balanced cohort.

## Table 1 result

| Dataset / Outputs | Paper-reported Assets | N_release | N_eval | #Categories (release / eval) | Links/Asset (mean / median / P90) | Movable Joints/Asset (mean / median / P90) | Multi-joint Assets | Unique Topologies | Exact Duplicate Rate |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Artiverse | 5,402 | 3544 | 800 | 84 / 67 | 8.592 / 5.000 / 16 (n=800) | 4.844 / 2.000 / 7 (n=800) | 630 / 800 (78.75%) | 286 / 797 (35.88%); coverage 797 / 800 (99.62%) | 0 / 797 (0.00%); coverage 797 / 800 (99.62%) |

## Diagnostics

- XML parse coverage: 800 / 800 (100.00%).
- Category macro over 67 sampled raw categories: multi-joint 69.08%; unique topologies 68.82% over 67 evaluable categories; exact duplicate rate 0.00% over 67 evaluable categories.
- Assets in duplicate clusters: 0 / 797 (0.00%); 0 clusters, maximum size 1.
- Topology hashes describe URDF representation structure, not semantic joint correctness.
- Exact duplicate rate uses canonicalized URDF plus the recursively resolved simulation resource closure; incomplete closures are not treated as unique.
