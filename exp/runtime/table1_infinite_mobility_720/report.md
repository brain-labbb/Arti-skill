# Infinite Mobility Table 1: Dataset Scale and Structural Diversity

## Frozen cohort

- This is a supplementary full generated cohort, not an official finite release.
- Identity policy: all obtained factory/seed identities are retained; recovery is an immutable pre-freeze overlay for original TIMEOUT cases; no post-freeze reselection.
- `N_release`: 720; `N_eval`: 720; raw factories: 20 / 20.
- Cohort manifest SHA-256: `cfd9c06ea35dcec57c53d44dbf52903ecba6f33321075495c97c58fe30d23c08`.

## Table 1 result

| Dataset | Cohort | N_release | N_eval | Factories (release / eval) | Links/Asset (mean / median / P90) | Movable Joints/Asset (mean / median / P90) | Multi-joint Assets | Unique Topologies | Exact Duplicate Rate |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Infinite Mobility | supplementary full generated | 720 | 720 | 20 / 20 | 15.042 / 8.000 / 41 (n=720) | 6.560 / 3.000 / 16 (n=720) | 550 / 720 (76.39%) | 157 / 720 (21.81%); coverage 720 / 720 (100.00%) | 0 / 720 (0.00%); coverage 720 / 720 (100.00%) |

## Diagnostics

- XML parse coverage: 720 / 720 (100.00%).
- Declared joint types: `{"continuous": 1307, "fixed": 5387, "prismatic": 2103, "revolute": 1313}`.
- Category macro across 20 factories: multi-joint 76.39%; unique topology 22.08%; duplicate 0.00%.
- Status counts: `{"COMPLETED": 720}`.

## Provenance

- Original primary PASS rows: 713.
- Original TIMEOUT recovery overlays: 7.
- Topology and duplicate rates retain their shared evaluator denominators and coverage definitions; failed assets remain in `N_eval`.
